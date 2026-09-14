#include <std_msgs/String.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "lidar_object_tracker/ros_utils.h"

namespace lot {
class TrackerNode {
  ros::NodeHandle nh_, private_{"~"};
  ObjectTracker tracker_;
  std::string frame_;
  ros::Subscriber cloud_sub_, odom_sub_;
  ros::Publisher dynamic_, background_, uncertain_, markers_, tracks_;
  ros::WallTimer watchdog_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_;
  bool stopped_ = false;
  std::map<uint64_t, sensor_msgs::PointCloud2ConstPtr> clouds_;
  std::map<uint64_t, nav_msgs::OdometryConstPtr> odoms_;
  uint64_t last_received_ = 0, processed_ = 0;
  std::atomic<uint64_t> dropped_{0};
  std::atomic<double> last_work_{ros::WallTime::now().toSec()};
  template <typename Map>
  void bound(Map& map) {
    while (map.size() > 60) {
      map.erase(map.begin());
      ++dropped_;
    }
  }
  void cloud(const sensor_msgs::PointCloud2ConstPtr& m) {
    std::lock_guard<std::mutex> lock(mutex_);
    uint64_t key = m->header.stamp.toNSec();
    if (last_received_ && key < last_received_) {
      clouds_.clear();
      odoms_.clear();
    }
    last_received_ = key;
    clouds_[key] = m;
    bound(clouds_);
    cv_.notify_one();
  }
  void odom(const nav_msgs::OdometryConstPtr& m) {
    std::lock_guard<std::mutex> lock(mutex_);
    odoms_[m->header.stamp.toNSec()] = m;
    bound(odoms_);
    cv_.notify_one();
  }
  void work() {
    while (ros::ok()) {
      sensor_msgs::PointCloud2ConstPtr cloud;
      nav_msgs::OdometryConstPtr odom;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stopped_)
          return;
        uint64_t latest = 0;
        size_t count = 0;
        for (const auto& kv : clouds_)
          if (odoms_.count(kv.first)) {
            latest = kv.first;
            ++count;
          }
        if (!count) {
          cv_.wait_for(lock, std::chrono::milliseconds(200));
          continue;
        }
        cloud = clouds_.at(latest);
        odom = odoms_.at(latest);
        dropped_ += count - 1;
        clouds_.erase(clouds_.begin(), clouds_.upper_bound(latest));
        odoms_.erase(odoms_.begin(), odoms_.upper_bound(latest));
      }
      try {
        if (uint64_t(cloud->width) * cloud->height > tracker_.config().at("max_input_points"))
          throw std::invalid_argument("cloud exceeds max_input_points");
        auto input = worldCloud(*cloud, *odom, frame_);
        auto result = tracker_.process(input.first, input.second, cloud->header.stamp.toSec());
        std_msgs::Header header = cloud->header;
        header.frame_id = frame_;
        Cloud dynamic, background, uncertain;
        for (size_t i = 0; i < input.first.size(); ++i) {
          if (result.dynamic[i])
            dynamic.push_back(input.first[i]);
          else
            background.push_back(input.first[i]);
          if (result.uncertain[i])
            uncertain.push_back(input.first[i]);
        }
        dynamic_.publish(cloudMessage(dynamic, header));
        background_.publish(cloudMessage(background, header));
        uncertain_.publish(cloudMessage(uncertain, header));
        markers_.publish(markers(result.objects, header));
        auto j = report(result, header);
        double age = (ros::Time::now() - header.stamp).toSec();
        j["input_age_seconds"] = age;
        j["processed"] = Json::UInt64(++processed_);
        j["skipped_pairs"] = Json::UInt64(dropped_.load());
        std_msgs::String msg;
        msg.data = jsonString(j);
        tracks_.publish(msg);
        last_work_ = ros::WallTime::now().toSec();
        ROS_INFO_THROTTLE(5, "tracker: objects=%u time=%.1fms age=%.3fs skipped=%lu",
                          result.objects.size(), result.processing_ms, age,
                          static_cast<unsigned long>(dropped_.load()));
        if (!result.ground_valid)
          ROS_WARN_THROTTLE(5, "No reliable ground estimate: no new object classification.");
      } catch (const std::exception& e) {
        ROS_WARN_THROTTLE(2, "Object tracker rejected frame: %s", e.what());
      }
    }
  }

 public:
  TrackerNode() : tracker_(loadConfig(private_)) {
    private_.param<std::string>("world_frame", frame_, "map");
    std::string cloud_topic, odom_topic;
    private_.param<std::string>("cloud_topic", cloud_topic,
                                "/lidar_object_tracker/input/cloud_body");
    private_.param<std::string>("odom_topic", odom_topic, "/lidar_object_tracker/input/odom");
    dynamic_ = private_.advertise<sensor_msgs::PointCloud2>("dynamic", 1);
    background_ = private_.advertise<sensor_msgs::PointCloud2>("background", 1);
    uncertain_ = private_.advertise<sensor_msgs::PointCloud2>("uncertain", 1);
    markers_ = private_.advertise<visualization_msgs::MarkerArray>("markers", 1);
    tracks_ = private_.advertise<std_msgs::String>("tracks", 1);
    cloud_sub_ = nh_.subscribe(cloud_topic, 4, &TrackerNode::cloud, this);
    odom_sub_ = nh_.subscribe(odom_topic, 60, &TrackerNode::odom, this);
    watchdog_ = nh_.createWallTimer(ros::WallDuration(1), [this](const ros::WallTimerEvent&) {
      if (ros::WallTime::now().toSec() - last_work_ > 3)
        ROS_WARN_THROTTLE(5, "No fresh paired input: keep the input adapter running.");
    });
    worker_ = std::thread(&TrackerNode::work, this);
  }
  ~TrackerNode() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopped_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable())
      worker_.join();
  }
};
}  // namespace lot
int main(int argc, char** argv) {
  ros::init(argc, argv, "object_tracker");
  try {
    lot::TrackerNode node;
    ros::spin();
    return 0;
  } catch (const std::exception& e) {
    ROS_FATAL("%s", e.what());
    return 1;
  }
}
