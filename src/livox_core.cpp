#include <algorithm>
#include <limits>
#include <stdexcept>

#include "lidar_object_tracker/livox_adapter.h"
namespace lot {
Extrinsic extrinsic(const std::vector<double>& xyz, const std::vector<double>& xyzw) {
  if (xyz.size() != 3 || xyzw.size() != 4)
    throw std::invalid_argument("provide measured lidar_translation xyz and lidar_quaternion xyzw");
  Extrinsic e;
  e.translation = Vec(xyz[0], xyz[1], xyz[2]);
  if (!e.translation.allFinite())
    throw std::invalid_argument("invalid extrinsic translation");
  e.rotation = checkedQuaternion(Eigen::Quaterniond(xyzw[3], xyzw[0], xyzw[1], xyzw[2]));
  return e;
}
std::vector<int64_t> scanTimes(uint64_t base, int64_t header, const std::vector<int64_t>& offsets,
                               double duration, int64_t shift) {
  if (!base || base > uint64_t(INT64_MAX) || header <= 0 ||
      std::llabs(int64_t(base) - header) > 1000000)
    throw std::invalid_argument("timebase/header disagree: fix driver timestamp generation");
  if (offsets.empty() || !std::isfinite(duration) || duration <= 0)
    throw std::invalid_argument("invalid scan duration/offsets");
  std::vector<int64_t> times;
  for (auto offset : offsets) {
    if (offset < 0 || offset > duration * 1e9)
      throw std::invalid_argument("invalid point offsets: fix driver timebase");
    __int128 t = __int128(base) + offset + shift;
    if (t <= 0 || t > INT64_MAX)
      throw std::invalid_argument("point time overflow");
    times.push_back(int64_t(t));
  }
  return times;
}
Pose interpolate(const std::vector<TimedPose>& samples, int64_t t) {
  if (samples.size() < 2 || t < samples.front().stamp || t > samples.back().stamp)
    throw std::invalid_argument("odometry does not cover scan; no extrapolation");
  auto it = std::lower_bound(samples.begin(), samples.end(), t,
                             [](const TimedPose& p, int64_t time) { return p.stamp < time; });
  size_t right = std::clamp<size_t>(it - samples.begin(), 1, samples.size() - 1);
  const auto& a = samples[right - 1];
  const auto& b = samples[right];
  if (b.stamp <= a.stamp)
    throw std::invalid_argument("non-monotonic odometry");
  double weight = double(t - a.stamp) / double(b.stamp - a.stamp);
  Pose p;
  p.position = (1 - weight) * a.pose.position + weight * b.pose.position;
  p.rotation = a.pose.rotation.slerp(weight, b.pose.rotation).normalized();
  return p;
}
Deskewed deskew(const Cloud& p, const std::vector<int64_t>& times,
                const std::vector<TimedPose>& samples, const Extrinsic& e, double gap) {
  if (p.size() != times.size() || p.empty() || samples.size() < 2 || !std::isfinite(gap) ||
      gap <= 0)
    throw std::invalid_argument("invalid deskew input");
  auto mm = std::minmax_element(times.begin(), times.end());
  int64_t first = *mm.first, last = *mm.second;
  std::vector<TimedPose> poses = samples;
  for (size_t i = 0; i < poses.size(); ++i) {
    if (!poses[i].pose.position.allFinite())
      throw std::invalid_argument("invalid odometry position");
    poses[i].pose.rotation = checkedQuaternion(poses[i].pose.rotation);
    if (i) {
      auto a = poses[i - 1].stamp, b = poses[i].stamp;
      if (b <= a)
        throw std::invalid_argument("non-monotonic odometry");
      if (a < last && b > first && b - a > gap * 1e9)
        throw std::invalid_argument("odometry gap exceeds max_odom_gap");
    }
  }
  if (first < poses.front().stamp || last > poses.back().stamp)
    throw std::invalid_argument("odometry does not cover entire scan; no extrapolation");
  Deskewed out;
  out.stamp = last;
  out.pose = interpolate(poses, last);
  out.points.reserve(p.size());
  for (size_t i = 0; i < p.size(); ++i) {
    auto pose = interpolate(poses, times[i]);
    Vec world = pose.rotation * (e.rotation * p[i] + e.translation) + pose.position;
    out.points.push_back(out.pose.rotation.conjugate() * (world - out.pose.position));
  }
  return out;
}
sensor_msgs::PointCloud2 convertLivox(const livox_ros_driver2::CustomMsg& msg, const Extrinsic& e,
                                      const std::string& body, int max_points, double min_range) {
  if (msg.point_num != msg.points.size() || msg.points.empty())
    throw std::invalid_argument("empty scan or point count mismatch");
  Cloud points;
  std::vector<float> reflectivity;
  for (const auto& p : msg.points) {
    if (p.offset_time)
      throw std::invalid_argument("nonzero point offsets: simulation adapter has no scan deskew");
    Vec v(p.x, p.y, p.z);
    if (v.allFinite() && v.norm() > min_range) {
      points.push_back(e.rotation * v + e.translation);
      reflectivity.push_back(p.reflectivity);
    }
  }
  if (points.empty() || points.size() > size_t(max_points))
    throw std::invalid_argument("no valid returns or max_points exceeded");
  std_msgs::Header h = msg.header;
  h.frame_id = body;
  return cloudMessage(points, h, &reflectivity);
}
nav_msgs::Odometry pairedOdometry(const sensor_msgs::PointCloud2& cloud,
                                  const nav_msgs::Odometry& odom, const std::string& world,
                                  const std::string& body) {
  if (frameName(odom.header.frame_id) != frameName(world) ||
      frameName(odom.child_frame_id) != frameName(body))
    throw std::invalid_argument("MAVROS world/body frames mismatch");
  auto pose = readPose(odom.pose.pose);
  auto out = odom;
  out.header.stamp = cloud.header.stamp;
  writePose(pose, out.pose.pose);
  return out;
}
}  // namespace lot
