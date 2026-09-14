#include "lidar_object_tracker/livox_adapter.h"

#include <algorithm>
#include <stdexcept>
namespace lot {
namespace {
class Adapter {
  ros::NodeHandle nh_, p_{"~"};
  bool real_;
  Extrinsic extrinsic_;
  std::string world_, body_, lidar_frame_;
  double slop_, duration_, gap_, wait_, age_, min_range_;
  int max_points_;
  int64_t offset_ = 0;
  ros::Subscriber scan_sub_, odom_sub_;
  ros::Publisher cloud_pub_, odom_pub_;
  ros::WallTimer worker_, reporter_;
  uint64_t scans_ = 0, pairs_ = 0, rejected_ = 0;
  int64_t last_scan_ = 0, last_output_ = 0;
  std::deque<nav_msgs::Odometry> poses_;
  std::deque<sensor_msgs::PointCloud2> sim_clouds_;
  struct Pending {
    ros::WallTime received;
    livox_ros_driver2::CustomMsgConstPtr scan;
    std::vector<int64_t> times;
  };
  std::deque<Pending> pending_;
  void reject(const std::string& error) {
    ++rejected_;
    ROS_WARN_THROTTLE(5, "adapter rejected: %s", error.c_str());
  }
  void odom(const nav_msgs::OdometryConstPtr& m) {
    if (real_) {
      try {
        readPose(m->pose.pose);
        if (frameName(m->header.frame_id) != frameName(world_) ||
            frameName(m->child_frame_id) != frameName(body_) || m->header.stamp.isZero())
          throw std::invalid_argument("invalid MAVROS pose/frames");
        if (!poses_.empty() && m->header.stamp <= poses_.back().header.stamp)
          throw std::invalid_argument("non-monotonic MAVROS time: restart after clock reset");
      } catch (const std::exception& e) {
        ROS_WARN_THROTTLE(5, "%s", e.what());
        return;
      }
    }
    poses_.push_back(*m);
    while (poses_.size() > (real_ ? 400 : 60))
      poses_.pop_front();
    if (!real_)
      pairSimulation();
  }
  void scan(const livox_ros_driver2::CustomMsgConstPtr& m) {
    ++scans_;
    try {
      if (!real_) {
        sim_clouds_.push_back(convertLivox(*m, extrinsic_, body_, max_points_, min_range_));
        while (sim_clouds_.size() > 60)
          sim_clouds_.pop_front();
        pairSimulation();
        return;
      }
      if (frameName(m->header.frame_id) != frameName(lidar_frame_))
        throw std::invalid_argument("unexpected lidar frame; verify extrinsics");
      if (m->points.empty() || m->points.size() != m->point_num ||
          m->points.size() > size_t(max_points_))
        throw std::invalid_argument("empty scan, point count mismatch or max_points exceeded");
      std::vector<int64_t> offsets;
      for (const auto& p : m->points)
        offsets.push_back(p.offset_time);
      auto times = scanTimes(m->timebase, m->header.stamp.toNSec(), offsets, duration_, offset_);
      auto mm = std::minmax_element(times.begin(), times.end());
      int64_t now = ros::Time::now().toNSec();
      if (*mm.first < now - age_ * 1e9 || *mm.second > now + gap_ * 1e9)
        throw std::invalid_argument("LiDAR/ROS clocks not aligned, or stale/future scan");
      if (*mm.second <= last_scan_)
        throw std::invalid_argument("non-monotonic scan time; restart after clock reset");
      last_scan_ = *mm.second;
      if (pending_.size() >= 3) {
        pending_.pop_front();
        reject("pending queue full; dropped oldest scan");
      }
      pending_.push_back({ros::WallTime::now(), m, std::move(times)});
    } catch (const std::exception& e) {
      reject(e.what());
    }
  }
  void pairSimulation() {
    // Bounded one-to-one nearest-time pairing; no pose interpolation in simulation.
    while (!sim_clouds_.empty() && !poses_.empty()) {
      double best = slop_;
      size_t ci = sim_clouds_.size(), oi = 0;
      for (size_t i = 0; i < sim_clouds_.size(); ++i)
        for (size_t j = 0; j < poses_.size(); ++j) {
          double dt = std::abs((sim_clouds_[i].header.stamp - poses_[j].header.stamp).toSec());
          if (dt < best) {
            best = dt;
            ci = i;
            oi = j;
          }
        }
      if (ci == sim_clouds_.size())
        return;
      auto cloud = sim_clouds_[ci];
      auto pose = poses_[oi];
      sim_clouds_.erase(sim_clouds_.begin() + ci);
      poses_.erase(poses_.begin() + oi);
      if (!cloud_pub_.getNumSubscribers() || !odom_pub_.getNumSubscribers())
        continue;
      try {
        int64_t stamp = cloud.header.stamp.toNSec();
        if (stamp <= 0 || pose.header.stamp.isZero() || stamp <= last_output_)
          throw std::invalid_argument("invalid or non-monotonic timestamp; restart full launch");
        auto odom = pairedOdometry(cloud, pose, world_, body_);
        cloud_pub_.publish(cloud);
        odom_pub_.publish(odom);
        last_output_ = stamp;
        ++pairs_;
      } catch (const std::exception& e) {
        reject(e.what());
      }
    }
  }
  void process(const ros::WallTimerEvent&) {
    if (!real_)
      return;
    while (!pending_.empty()) {
      const auto& first = pending_.front();
      if ((ros::WallTime::now() - first.received).toSec() > wait_) {
        pending_.pop_front();
        reject("timed out waiting for scan odometry");
        continue;
      }
      if (poses_.size() < 2 || int64_t(poses_.back().header.stamp.toNSec()) <
                                   *std::max_element(first.times.begin(), first.times.end()))
        return;
      auto pending = std::move(pending_.front());
      pending_.pop_front();
      try {
        publish(pending);
      } catch (const std::exception& e) {
        reject(e.what());
      }
    }
  }
  void publish(const Pending& pending) {
    std::vector<TimedPose> samples;
    for (const auto& o : poses_)
      samples.push_back({int64_t(o.header.stamp.toNSec()), readPose(o.pose.pose)});
    Cloud xyz;
    std::vector<bool> valid;
    for (const auto& p : pending.scan->points) {
      Vec v(p.x, p.y, p.z);
      valid.push_back(v.allFinite() && v.norm() > min_range_);
      for (int i = 0; i < 3; ++i)
        if (!std::isfinite(v[i]))
          v[i] = 0;
      xyz.push_back(v);
    }
    if (std::none_of(valid.begin(), valid.end(), [](bool x) { return x; }))
      throw std::invalid_argument("scan has no valid returns");
    auto result = deskew(xyz, pending.times, samples, extrinsic_, gap_);
    if (result.stamp <= last_output_)
      throw std::invalid_argument("non-monotonic output timestamp");
    Cloud points;
    std::vector<float> intensity;
    for (size_t i = 0; i < valid.size(); ++i)
      if (valid[i]) {
        points.push_back(result.points[i]);
        intensity.push_back(pending.scan->points[i].reflectivity);
      }
    std_msgs::Header header;
    header.stamp.fromNSec(result.stamp);
    header.frame_id = body_;
    auto cloud = cloudMessage(points, header, &intensity);
    auto it = std::lower_bound(samples.begin(), samples.end(), result.stamp,
                               [](const TimedPose& p, int64_t t) { return p.stamp < t; });
    size_t right = std::clamp<size_t>(it - samples.begin(), 1, samples.size() - 1);
    const auto& a = poses_[right - 1];
    const auto& b = poses_[right];
    double weight = double(result.stamp - samples[right - 1].stamp) /
                    (samples[right].stamp - samples[right - 1].stamp);
    auto odom = a;
    odom.header = header;
    odom.header.frame_id = world_;
    odom.child_frame_id = body_;
    writePose(result.pose, odom.pose.pose);
    for (size_t i = 0; i < 36; ++i) {
      odom.pose.covariance[i] = (1 - weight) * a.pose.covariance[i] + weight * b.pose.covariance[i];
      odom.twist.covariance[i] =
          (1 - weight) * a.twist.covariance[i] + weight * b.twist.covariance[i];
    }
    auto lerp = [&](const geometry_msgs::Vector3& x, const geometry_msgs::Vector3& y) {
      geometry_msgs::Vector3 v;
      v.x = (1 - weight) * x.x + weight * y.x;
      v.y = (1 - weight) * x.y + weight * y.y;
      v.z = (1 - weight) * x.z + weight * y.z;
      return v;
    };
    odom.twist.twist.linear = lerp(a.twist.twist.linear, b.twist.twist.linear);
    odom.twist.twist.angular = lerp(a.twist.twist.angular, b.twist.twist.angular);
    cloud_pub_.publish(cloud);
    odom_pub_.publish(odom);
    last_output_ = result.stamp;
    ++pairs_;
  }

 public:
  explicit Adapter(bool real) : real_(real) {
    p_.param<std::string>("world_frame", world_, "map");
    p_.param<std::string>("body_frame", body_, "base_link");
    p_.param<std::string>("lidar_frame", lidar_frame_, "livox_frame");
    p_.param("max_dt", slop_, .05);
    p_.param("max_scan_duration", duration_, .15);
    p_.param("max_odom_gap", gap_, .1);
    p_.param("max_wait", wait_, .5);
    p_.param("max_input_age", age_, 1.);
    p_.param("min_range", min_range_, .3);
    p_.param("max_points", max_points_, 30000);
    double offset;
    p_.param("lidar_time_offset", offset, 0.);
    for (double value : {slop_, duration_, gap_, wait_, age_})
      if (!std::isfinite(value) || value <= 0)
        throw std::invalid_argument("invalid adapter timing limit");
    if (world_.empty() || body_.empty() || lidar_frame_.empty() || !std::isfinite(min_range_) ||
        min_range_ < 0 || !std::isfinite(offset) || std::abs(offset) > 9e9 || max_points_ < 1 ||
        max_points_ > 1000000)
      throw std::invalid_argument("invalid adapter limits/frames");
    offset_ = std::llround(offset * 1e9);
    std::vector<double> xyz, xyzw;
    if (!p_.getParam("lidar_translation", xyz) && !real_)
      xyz = {0, 0, .235077};
    if (!p_.getParam("lidar_quaternion", xyzw) && !real_)
      xyzw = {0, 0, 0, 1};
    extrinsic_ = extrinsic(xyz, xyzw);
    std::string lidar_topic, odom_topic;
    p_.param<std::string>("lidar_topic", lidar_topic, "/livox/lidar");
    p_.param<std::string>("odom_topic", odom_topic, "/mavros/local_position/odom");
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("input/cloud_body", real_ ? 2 : 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("input/odom", real_ ? 2 : 10);
    scan_sub_ = nh_.subscribe(lidar_topic, 3, &Adapter::scan, this);
    odom_sub_ = nh_.subscribe(odom_topic, real_ ? 100 : 60, &Adapter::odom, this);
    worker_ = nh_.createWallTimer(ros::WallDuration(.02), &Adapter::process, this);
    reporter_ = nh_.createWallTimer(ros::WallDuration(5), [this](const ros::WallTimerEvent&) {
      ROS_INFO("%s adapter: scans=%lu pairs=%lu rejected=%lu pending=%zu odom=%zu",
               real_ ? "real" : "simulation", scans_, pairs_, rejected_, pending_.size(),
               poses_.size());
    });
    ROS_INFO("%s Livox adapter: world=%s body=%s",
             real_ ? "per-point deskew" : "instantaneous scan", world_.c_str(), body_.c_str());
  }
};
}  // namespace
int runAdapter(int argc, char** argv, bool real) {
  ros::init(argc, argv, real ? "livox_real_to_body" : "livox_to_body");
  try {
    Adapter adapter(real);
    ros::spin();
    return 0;
  } catch (const std::exception& e) {
    ROS_FATAL("%s", e.what());
    return 1;
  }
}
}  // namespace lot
