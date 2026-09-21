#include <gtest/gtest.h>
#include <std_msgs/String.h>

#include <map>
#include <set>

#include "lidar_object_tracker/livox_adapter.h"
#include "scenes.h"
using namespace lot;
TEST(Pipeline, RawScanThroughAdapterAndTracker) {
  ros::NodeHandle nh, priv("~");
  bool real = false;
  priv.param("real", real, false);
  std::vector<Json::Value> reports;
  std::map<uint64_t, sensor_msgs::PointCloud2> clouds;
  std::map<uint64_t, nav_msgs::Odometry> poses;
  std::map<uint64_t, size_t> full_counts, background_counts, dynamic_counts;
  std::set<uint64_t> dynamic_stamps;
  size_t moving_points = 0;
  auto sub = nh.subscribe<std_msgs::String>(
      "/lidar_object_tracker/tracks", 30,
      [&](const std_msgs::StringConstPtr& m) { reports.push_back(parseJson(m->data)); });
  auto cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "/lidar_object_tracker/input/cloud_body", 30,
      [&](const sensor_msgs::PointCloud2ConstPtr& m) { clouds[m->header.stamp.toNSec()] = *m; });
  auto pose_sub = nh.subscribe<nav_msgs::Odometry>(
      "/lidar_object_tracker/input/odom", 30,
      [&](const nav_msgs::OdometryConstPtr& m) { poses[m->header.stamp.toNSec()] = *m; });
  auto dynamic_sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "/lidar_object_tracker/dynamic", 30, [&](const sensor_msgs::PointCloud2ConstPtr& m) {
        EXPECT_EQ("map", m->header.frame_id);
        dynamic_stamps.insert(m->header.stamp.toNSec());
        moving_points += m->width;
      });
  auto full_sub = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_object_tracker/full", 30,
      [&](const sensor_msgs::PointCloud2ConstPtr& m) {
        EXPECT_EQ("map", m->header.frame_id);
        full_counts[m->header.stamp.toNSec()] = m->width * m->height;
      });
  auto background_sub = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_object_tracker/background", 30,
      [&](const sensor_msgs::PointCloud2ConstPtr& m) { background_counts[m->header.stamp.toNSec()] = m->width * m->height; });
  auto dynamic_count_sub = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_object_tracker/dynamic", 30,
      [&](const sensor_msgs::PointCloud2ConstPtr& m) { dynamic_counts[m->header.stamp.toNSec()] = m->width * m->height; });
  auto raw = nh.advertise<livox_ros_driver2::CustomMsg>("/migration/raw", 3);
  auto odom = nh.advertise<nav_msgs::Odometry>("/migration/pose", 30);
  auto spinFor = [](double seconds) {
    auto until = ros::WallTime::now() + ros::WallDuration(seconds);
    while (ros::WallTime::now() < until) {
      ros::spinOnce();
      ros::WallDuration(.005).sleep();
    }
  };
  auto deadline = ros::WallTime::now() + ros::WallDuration(10);
  while ((!raw.getNumSubscribers() || !odom.getNumSubscribers() || !sub.getNumPublishers()) &&
         ros::WallTime::now() < deadline)
    spinFor(.05);
  ASSERT_GT(raw.getNumSubscribers(), 0u);
  ASSERT_GT(odom.getNumSubscribers(), 0u);
  spinFor(.3);
  double base = ros::Time::now().toSec() - .2;
  for (int frame = 0; frame < 24; ++frame) {
    double stamp = base + frame * .1;
    for (double dt : {-.04, 0., .04}) {
      nav_msgs::Odometry o;
      o.header.stamp.fromSec(stamp + dt);
      o.header.frame_id = "map";
      o.child_frame_id = "base_link";
      o.pose.pose.orientation.w = 1;
      o.twist.twist.linear.x = 2;
      o.pose.covariance[0] = .25;
      odom.publish(o);
    }
    auto xyz = test::scene(frame * .08);
    livox_ros_driver2::CustomMsg msg;
    msg.header.stamp.fromSec(stamp);
    msg.header.frame_id = "livox_frame";
    msg.timebase = msg.header.stamp.toNSec();
    for (size_t i = 0; i < xyz.size(); ++i) {
      livox_ros_driver2::CustomPoint p;
      p.x = xyz[i].x();
      p.y = xyz[i].y();
      p.z = xyz[i].z();
      p.reflectivity = 80;
      p.offset_time = real ? uint32_t(double(i) / (xyz.size() - 1) * 20000000) : 0;
      msg.points.push_back(p);
    }
    msg.point_num = msg.points.size();
    raw.publish(msg);
    spinFor(.1);
  }
  spinFor(.5);
  ASSERT_GE(reports.size(), 15u);
  EXPECT_GT(moving_points, 120u);
  bool moving = false;
  for (const auto& r : reports) {
    EXPECT_EQ("map", r["frame_id"].asString());
    ASSERT_TRUE(r["stamp_ns"].isUInt64());
    ASSERT_TRUE(r["session_id"].isString());
    EXPECT_FALSE(r["session_id"].asString().empty());
    uint64_t stamp_ns = r["stamp_ns"].asUInt64();
    ASSERT_TRUE(full_counts.count(stamp_ns));
    ASSERT_TRUE(background_counts.count(stamp_ns));
    ASSERT_TRUE(dynamic_counts.count(stamp_ns));
    EXPECT_EQ(full_counts.at(stamp_ns), background_counts.at(stamp_ns) + dynamic_counts.at(stamp_ns));
    for (const auto& o : r["objects"])
      moving = moving || o["state"].asString() == "MOVING";
  }
  EXPECT_TRUE(moving);
  ASSERT_GE(clouds.size(), 15u);
  for (const auto& kv : clouds) {
    ASSERT_TRUE(poses.count(kv.first));
    const auto& o = poses.at(kv.first);
    EXPECT_EQ(kv.second.header.stamp, o.header.stamp);
    EXPECT_EQ("base_link", kv.second.header.frame_id);
    EXPECT_NEAR(.25, o.pose.covariance[0], 1e-12);
    EXPECT_NEAR(2, o.twist.twist.linear.x, 1e-12);
    EXPECT_TRUE(dynamic_stamps.count(kv.first));
  }
  ROS_INFO("%s pipeline: %zu reports, moving points=%zu", real ? "real" : "simulation",
           reports.size(), moving_points);
}
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "pipeline_check");
  return RUN_ALL_TESTS();
}
