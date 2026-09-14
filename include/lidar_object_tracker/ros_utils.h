#pragma once
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include "lidar_object_tracker/tracking_core.h"
namespace lot {
std::string frameName(std::string name);
Eigen::Quaterniond checkedQuaternion(const Eigen::Quaterniond& q);
struct Pose {
  Vec position = Vec::Zero();
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
};
Pose readPose(const geometry_msgs::Pose& pose);
void writePose(const Pose& pose, geometry_msgs::Pose& message);
Cloud readXYZ(const sensor_msgs::PointCloud2& cloud);
sensor_msgs::PointCloud2 cloudMessage(const Cloud& points, const std_msgs::Header& header,
                                      const std::vector<float>* reflectivity = nullptr);
std::pair<Cloud, Vec> worldCloud(const sensor_msgs::PointCloud2&, const nav_msgs::Odometry&,
                                 const std::string& frame);
Config loadConfig(const ros::NodeHandle& private_node);
Config loadYaml(const std::string& path);
visualization_msgs::MarkerArray markers(const Json::Value& objects, const std_msgs::Header& header);
Json::Value report(const Result& result, const std_msgs::Header& header);
std::string jsonString(const Json::Value& value, bool pretty = false);
Json::Value parseJson(const std::string& text);
}  // namespace lot
