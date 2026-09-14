#include <gtest/gtest.h>

#include <cstring>
#include <set>

#include "lidar_object_tracker/livox_adapter.h"
#include "scenes.h"
using namespace lot;
using lot::test::count;
using lot::test::scene;
TEST(Tracking, MovingSurfaceKeepsIdAndStops) {
  ObjectTracker t;
  std::set<int> ids;
  Result r;
  for (int i = 0; i < 35; ++i) {
    r = t.process(scene(i * .08), Vec(0, 0, .3), 10 + i * .1);
    if (i >= 15) {
      ASSERT_EQ(1u, r.objects.size());
      EXPECT_EQ("MOVING", r.objects[0]["state"].asString());
      ids.insert(r.objects[0]["id"].asInt());
      EXPECT_GT(count(r.dynamic), 120u);
      EXPECT_NEAR(.8, r.objects[0]["speed"].asDouble(), .12);
    }
  }
  EXPECT_EQ(1u, ids.size());
  for (int i = 35; i < 65; ++i)
    r = t.process(scene(34 * .08), Vec(0, 0, .3), 10 + i * .1);
  ASSERT_EQ(1u, r.objects.size());
  EXPECT_EQ("STATIC", r.objects[0]["state"].asString());
  EXPECT_EQ(0u, count(r.dynamic));
}
TEST(Tracking, PartialViewIsNotMotion) {
  ObjectTracker t;
  for (int i = 0; i < 45; ++i) {
    auto r = t.process(scene(0, .25 * (1 + std::sin(i * .17))), Vec(0, 0, .3), 20 + i * .1);
    EXPECT_EQ(0u, count(r.dynamic));
  }
}
TEST(Tracking, PoseCompensation) {
  ObjectTracker t;
  auto fixed = scene();
  Result r;
  for (int i = 0; i < 35; ++i) {
    Pose pose;
    pose.rotation = Eigen::AngleAxisd(i * .015, Vec::UnitZ());
    pose.position = Vec(i * .02, -.2, .3);
    Cloud body;
    for (const auto& p : fixed)
      body.push_back(pose.rotation.conjugate() * (p - pose.position));
    std_msgs::Header h;
    h.frame_id = "base_link";
    h.stamp.fromSec(30 + i * .1);
    nav_msgs::Odometry o;
    o.header = h;
    o.header.frame_id = "map";
    o.child_frame_id = "base_link";
    writePose(pose, o.pose.pose);
    auto input = worldCloud(cloudMessage(body, h), o, "map");
    r = t.process(input.first, input.second, h.stamp.toSec());
    EXPECT_EQ(0u, count(r.dynamic));
  }
  ASSERT_EQ(1u, r.objects.size());
  EXPECT_EQ("STATIC", r.objects[0]["state"].asString());
}
TEST(Tracking, LossPredictsWithoutPointsThenExpires) {
  ObjectTracker t;
  for (int i = 0; i < 20; ++i)
    t.process(scene(i * .08), Vec(0, 0, .3), 40 + i * .1);
  auto ground = scene();
  ground.resize(1225);
  auto r = t.process(ground, Vec(0, 0, .3), 42);
  ASSERT_EQ(1u, r.objects.size());
  EXPECT_EQ("PREDICTED", r.objects[0]["state"].asString());
  EXPECT_EQ(0u, count(r.dynamic));
  EXPECT_EQ(0u, t.process(ground, Vec(0, 0, .3), 42.5).objects.size());
}
TEST(Tracking, TimeResetKeepsIdsUnique) {
  ObjectTracker t;
  Result r;
  for (int i = 0; i < 20; ++i)
    r = t.process(scene(i * .08), Vec(0, 0, .3), 50 + i * .1);
  int id = r.objects[0]["id"].asInt();
  r = t.process(scene(), Vec(0, 0, .3), 2);
  EXPECT_TRUE(r.reset);
  ASSERT_EQ(1u, r.objects.size());
  EXPECT_NE(id, r.objects[0]["id"].asInt());
  EXPECT_EQ("UNKNOWN", r.objects[0]["state"].asString());
}
TEST(Tracking, PoseJumpResetsEvidence) {
  ObjectTracker t;
  for (int i = 0; i < 20; ++i)
    t.process(scene(), Vec(0, 0, .3), 60 + i * .1);
  auto p = scene();
  for (auto& x : p)
    x += Vec(5, 0, 0);
  auto r = t.process(p, Vec(5, 0, .3), 62);
  EXPECT_TRUE(r.reset);
  EXPECT_EQ(0u, count(r.dynamic));
}
TEST(Tracking, MergedShapeCannotInheritMoving) {
  ObjectTracker t;
  for (int i = 0; i < 25; ++i)
    t.process(scene(i * .08), Vec(0, 0, .3), 70 + i * .1);
  auto p = scene(24 * .08);
  for (double d : {-.8, .8}) {
    auto extra = scene(24 * .08 + d);
    p.insert(p.end(), extra.begin() + 1225, extra.end());
  }
  EXPECT_EQ(0u, count(t.process(p, Vec(0, 0, .3), 72.5).dynamic));
}
TEST(Tracking, EmptyCloudRoundtrip) {
  std_msgs::Header h;
  EXPECT_TRUE(readXYZ(cloudMessage({}, h)).empty());
}
TEST(Tracking, MismatchedFramesAndTimesRejected) {
  std_msgs::Header h;
  h.stamp = ros::Time(1);
  h.frame_id = "base_link";
  auto cloud = cloudMessage(scene(), h);
  nav_msgs::Odometry o;
  o.header.stamp = ros::Time(2);
  o.header.frame_id = "map";
  o.child_frame_id = "base_link";
  o.pose.pose.orientation.w = 1;
  EXPECT_THROW(worldCloud(cloud, o, "map"), std::invalid_argument);
  o.header.stamp = h.stamp;
  o.child_frame_id = "other";
  EXPECT_THROW(worldCloud(cloud, o, "map"), std::invalid_argument);
}
TEST(Tracking, ConfigurationAndMissingGround) {
  EXPECT_THROW(ObjectTracker({{"typo", 1}}), std::invalid_argument);
  EXPECT_THROW(ObjectTracker({{"static_speed", 1}}), std::invalid_argument);
  ObjectTracker t;
  Cloud sparse{Vec(1, 2, 3)};
  auto r = t.process(sparse, Vec::Zero(), 1);
  EXPECT_FALSE(r.ground_valid);
  EXPECT_TRUE(r.objects.empty());
}
TEST(Tracking, GlobalAssignmentNotGreedy) {
  auto p = assign({{1, 2}, {1.1, 100}});
  ASSERT_EQ(2u, p.size());
  EXPECT_EQ(1, p[0].second);
  EXPECT_EQ(0, p[1].second);
  EXPECT_EQ(1u, assign({{2}, {1}, {3}}).size());
}
TEST(Tracking, TwoObjectsKeepDifferentIds) {
  ObjectTracker t;
  Result r;
  for (int i = 0; i < 25; ++i) {
    auto p = scene(i * .08);
    auto second = scene(-2 + i * .08);
    p.insert(p.end(), second.begin() + 1225, second.end());
    r = t.process(p, Vec(0, 0, .3), 10 + i * .1);
  }
  ASSERT_EQ(2u, r.objects.size());
  EXPECT_NE(r.objects[0]["id"], r.objects[1]["id"]);
  for (const auto& o : r.objects)
    EXPECT_EQ("MOVING", o["state"].asString());
}
TEST(Cloud, BigEndianPaddedRowsAndFloat64) {
  sensor_msgs::PointCloud2 m;
  m.height = 2;
  m.width = 1;
  m.is_bigendian = true;
  m.point_step = 24;
  m.row_step = 32;
  m.data.resize(64);
  for (int k = 0; k < 3; ++k) {
    sensor_msgs::PointField f;
    f.name = std::vector<std::string>{"x", "y", "z"}[k];
    f.datatype = 8;
    f.count = 1;
    f.offset = k * 8;
    m.fields.push_back(f);
  }
  for (int row = 0; row < 2; ++row)
    for (int k = 0; k < 3; ++k) {
      double x = row * 3 + k + .5;
      uint64_t bits;
      std::memcpy(&bits, &x, 8);
      for (int b = 0; b < 8; ++b)
        m.data[row * 32 + k * 8 + b] = (bits >> (56 - 8 * b)) & 255;
    }
  auto p = readXYZ(m);
  ASSERT_EQ(2u, p.size());
  EXPECT_NEAR(5.5, p[1].z(), 1e-12);
  m.data.resize(40);
  EXPECT_THROW(readXYZ(m), std::invalid_argument);
}
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
