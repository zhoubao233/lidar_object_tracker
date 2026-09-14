#include <gtest/gtest.h>

#include <cstring>

#include "lidar_object_tracker/livox_adapter.h"
using namespace lot;
namespace {
livox_ros_driver2::CustomMsg scan() {
  livox_ros_driver2::CustomMsg m;
  m.header.stamp = ros::Time(10);
  m.header.frame_id = "livox_frame";
  m.timebase = m.header.stamp.toNSec();
  livox_ros_driver2::CustomPoint p;
  p.x = 1;
  p.reflectivity = 80;
  m.points = {p, livox_ros_driver2::CustomPoint()};
  m.point_num = 2;
  return m;
}
}  // namespace
TEST(Livox, ExtrinsicIntensityAndZeroReturns) {
  auto e = extrinsic({0, 0, .235077}, {0, 0, std::sqrt(.5), std::sqrt(.5)});
  auto m = convertLivox(scan(), e, "base_link", 30000, .3);
  auto p = readXYZ(m);
  ASSERT_EQ(1u, p.size());
  EXPECT_NEAR(0, p[0].x(), 1e-6);
  EXPECT_NEAR(1, p[0].y(), 1e-6);
  EXPECT_NEAR(.235077, p[0].z(), 1e-6);
  EXPECT_EQ(32u, m.point_step);
  float intensity;
  std::memcpy(&intensity, m.data.data() + 12, 4);
  EXPECT_FLOAT_EQ(80, intensity);
}
TEST(Livox, PairPreservesPoseAndCovariance) {
  auto m = convertLivox(scan(), Extrinsic(), "base_link", 30000, .3);
  nav_msgs::Odometry o;
  o.header.frame_id = "map";
  o.header.stamp = ros::Time(10, .01 * 1e9);
  o.child_frame_id = "base_link";
  o.pose.pose.orientation.w = 2;
  o.pose.pose.position.x = 3;
  o.twist.twist.linear.x = 2;
  o.pose.covariance[0] = .25;
  auto paired = pairedOdometry(m, o, "map", "base_link");
  EXPECT_EQ(m.header.stamp, paired.header.stamp);
  EXPECT_DOUBLE_EQ(1, paired.pose.pose.orientation.w);
  EXPECT_DOUBLE_EQ(3, paired.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(2, paired.twist.twist.linear.x);
  EXPECT_DOUBLE_EQ(.25, paired.pose.covariance[0]);
}
TEST(Livox, RejectsSweepsAndOversizedValidScans) {
  auto m = scan();
  m.points[0].offset_time = 1;
  EXPECT_THROW(convertLivox(m, Extrinsic(), "base_link", 30000, .3), std::invalid_argument);
  m.points[0].offset_time = 0;
  m.points[1] = m.points[0];
  EXPECT_THROW(convertLivox(m, Extrinsic(), "base_link", 1, .3), std::invalid_argument);
  m.point_num = 7;
  EXPECT_THROW(convertLivox(m, Extrinsic(), "base_link", 30000, .3), std::invalid_argument);
}
TEST(Deskew, TranslationRotationAndExtrinsic) {
  int64_t epoch = 1789030700000000000LL;
  std::vector<TimedPose> poses;
  for (int i = 0; i < 3; ++i) {
    Pose p;
    p.position = Vec(i * .05, 0, 0);
    p.rotation = Eigen::AngleAxisd(i * .1, Vec::UnitZ());
    poses.push_back({epoch + i * 50000000, p});
  }
  Extrinsic e;
  e.translation = Vec(.2, -.1, .3);
  e.rotation = Eigen::AngleAxisd(.3, Vec::UnitX());
  Cloud points;
  std::vector<int64_t> times;
  Vec world(5, 2, 1);
  for (int i = 0; i <= 100; ++i) {
    double dt = i * .001;
    Eigen::Quaterniond q(Eigen::AngleAxisd(dt * 2, Vec::UnitZ()));
    points.push_back(e.rotation.conjugate() *
                     (q.conjugate() * (world - Vec(dt, 0, 0)) - e.translation));
    times.push_back(epoch + i * 1000000);
  }
  auto result = deskew(points, times, poses, e, .06);
  Vec expected = poses.back().pose.rotation.conjugate() * (world - poses.back().pose.position);
  for (const auto& p : result.points)
    EXPECT_LT((p - expected).norm(), 1e-9);
  EXPECT_EQ(epoch + 100000000, result.stamp);
  EXPECT_LT((result.pose.position - Vec(.1, 0, 0)).norm(), 1e-12);
}
TEST(Deskew, NanosecondTimePreserved) {
  int64_t base = 1789030700000000123LL;
  auto times = scanTimes(base, base + 20, {0, 90000000, 1}, .15);
  EXPECT_EQ(base, times[0]);
  EXPECT_EQ(base + 1, times[2]);
  EXPECT_EQ(base - 100, scanTimes(base, base, {0}, .15, -100)[0]);
}
TEST(Deskew, BrokenDriverTimesRejected) {
  for (const auto& offsets : std::vector<std::vector<int64_t>>{{4194986170LL, 31}, {-1}, {}})
    EXPECT_THROW(scanTimes(1000000000, 1000000000, offsets, .15), std::invalid_argument);
  EXPECT_THROW(scanTimes(1000000000, 2000000000, {0}, .15), std::invalid_argument);
}
TEST(Deskew, NoExtrapolationOrLargeGap) {
  std::vector<TimedPose> poses{{0, Pose()}, {100000000, Pose()}};
  Cloud points(2, Vec::Ones());
  EXPECT_THROW(deskew(points, {-1, 50000000}, poses, Extrinsic(), 1), std::invalid_argument);
  EXPECT_THROW(deskew(points, {50000000, 110000000}, poses, Extrinsic(), 1), std::invalid_argument);
  EXPECT_THROW(deskew(points, {0, 100000000}, poses, Extrinsic(), .05), std::invalid_argument);
  poses[1].stamp = 0;
  EXPECT_THROW(deskew(points, {0, 1}, poses, Extrinsic(), 1), std::invalid_argument);
}
TEST(Deskew, ExplicitFiniteExtrinsic) {
  EXPECT_THROW(extrinsic({}, {}), std::invalid_argument);
  EXPECT_THROW(extrinsic({0, 0, 0}, {0, 0, 0, 0}), std::invalid_argument);
  EXPECT_THROW(extrinsic({NAN, 0, 0}, {0, 0, 0, 1}), std::invalid_argument);
}
