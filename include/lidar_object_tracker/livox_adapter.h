#pragma once
#include <livox_ros_driver2/CustomMsg.h>

#include <deque>

#include "lidar_object_tracker/ros_utils.h"
namespace lot {
struct TimedPose {
  int64_t stamp;
  Pose pose;
};
struct Extrinsic {
  Vec translation = Vec::Zero();
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
};
Extrinsic extrinsic(const std::vector<double>& xyz, const std::vector<double>& xyzw);
std::vector<int64_t> scanTimes(uint64_t timebase, int64_t header,
                               const std::vector<int64_t>& offsets, double max_duration,
                               int64_t time_offset = 0);
Pose interpolate(const std::vector<TimedPose>& poses, int64_t time);
struct Deskewed {
  Cloud points;
  int64_t stamp;
  Pose pose;
};
Deskewed deskew(const Cloud& points, const std::vector<int64_t>& times,
                const std::vector<TimedPose>& poses, const Extrinsic& ext, double max_gap);
sensor_msgs::PointCloud2 convertLivox(const livox_ros_driver2::CustomMsg& msg, const Extrinsic& ext,
                                      const std::string& body, int max_points, double min_range);
nav_msgs::Odometry pairedOdometry(const sensor_msgs::PointCloud2& cloud,
                                  const nav_msgs::Odometry& odom, const std::string& world,
                                  const std::string& body);
// Implementation shared by simulation and real-scanner executable entrypoints.
int runAdapter(int argc, char** argv, bool real);
}  // namespace lot
