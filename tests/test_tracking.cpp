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

TEST(Tracking, GroundlessTracksMotionAndRetainsAllMapPoints) {
  ObjectTracker t({{"allow_groundless", 1}});
  Result r;
  for (int i=0;i<45;++i) {
    auto cloud=scene(i*.06);
    // Simulate ascent: ground returns disappear after initial acquisition.
    if (i>=10) cloud.erase(cloud.begin(),cloud.begin()+1225);
    r=t.process(cloud,Vec(0,0,.3+std::min(i,20)*.24),100+i*.1);
    ASSERT_TRUE(r.segmentation_valid);
    if (i>=20) {
      EXPECT_FALSE(r.ground_valid);
      EXPECT_EQ("unfiltered",r.segmentation_mode);
      ASSERT_EQ(1u,r.objects.size());
      EXPECT_EQ("MOVING",r.objects[0]["state"].asString());
      EXPECT_NEAR(.6,r.objects[0]["speed"].asDouble(),.12);
      EXPECT_EQ(0u,count(r.dynamic));
      EXPECT_GT(std::count_if(r.moving_owner.begin(),r.moving_owner.end(),[](int32_t id){return id>0;}),0);
      for(auto id:r.moving_owner) EXPECT_TRUE(id==0 || id==r.objects[0]["id"].asInt());
    }
  }
  EXPECT_FALSE(t.process({},Vec(0,0,5.1),104.5).segmentation_valid);
}
TEST(Tracking, GroundlessColdStartStaticAndSparseInput) {
  ObjectTracker t({{"allow_groundless",1}});
  auto cloud=scene(); cloud.erase(cloud.begin(),cloud.begin()+1225);
  Result r;
  for(int i=0;i<30;++i) {
    r=t.process(cloud,Vec(0,0,5),200+i*.1);
    EXPECT_TRUE(r.segmentation_valid); EXPECT_FALSE(r.ground_valid);
    EXPECT_EQ(0u,count(r.dynamic));
  }
  ASSERT_EQ(1u,r.objects.size()); EXPECT_EQ("STATIC",r.objects[0]["state"].asString());
  EXPECT_FALSE(ObjectTracker().process(cloud,Vec(0,0,5),1).segmentation_valid);
  EXPECT_TRUE(t.process({Vec(30,0,5)},Vec(0,0,5),203).segmentation_valid);
  EXPECT_FALSE(t.process({Vec(NAN,NAN,NAN)},Vec(0,0,5),203.1).segmentation_valid);
  EXPECT_THROW(ObjectTracker({{"allow_groundless",2}}),std::invalid_argument);
}
TEST(Tracking, GroundReturnsRestoreFiltering) {
  ObjectTracker t({{"allow_groundless",1}});
  auto cloud=scene(); cloud.erase(cloud.begin(),cloud.begin()+1225);
  EXPECT_EQ("unfiltered",t.process(cloud,Vec(0,0,5),1).segmentation_mode);
  auto r=t.process(scene(),Vec(0,0,5),1.1);
  EXPECT_TRUE(r.ground_valid); EXPECT_EQ("ground_filtered",r.segmentation_mode);
}
namespace {
// 地面 + 一根竖直柱体（宽 2*half_width，高到 height），z_step 控制竖直采样密度。
Cloud tallColumn(double height,double z_step,double half_width=.5,double y_shift=0.){
  Cloud p;
  for(int j=0;j<35;++j)for(int i=0;i<35;++i)
    p.emplace_back(-3.+12.*i/34.,-6.+12.*j/34.,0);
  for(double z=.17;z<=height+1e-9;z+=z_step)
    for(double y=-half_width;y<=half_width+1e-9;y+=.05)
      p.emplace_back(4.5,y+y_shift,z);
  return p;
}
}  // namespace

// 20 m 高柱体的水平尺寸只有约 1 m：不能再因为"高"把整个簇丢掉。
TEST(Tracking, TallObjectPassesHorizontalSizeGate) {
  ObjectTracker t;
  ASSERT_DOUBLE_EQ(5.,t.config().at("max_cluster_size"));  // 不需要为高度放宽该参数
  auto r=t.process(tallColumn(20,.05),Vec(0,0,.3),1);
  ASSERT_EQ(1u,r.objects.size());
  EXPECT_GT(r.objects[0]["size"][2].asDouble(),18.);
  EXPECT_LT(r.objects[0]["size"][1].asDouble(),2.);
}

// 竖直采样稀疏时同一根柱体被各向同性邻域切成多段；column_distance 把它们连回一个目标。
TEST(Tracking, ColumnDistanceMergesVerticallySparseFragments) {
  ObjectTracker off;
  EXPECT_GT(off.process(tallColumn(20,.5),Vec(0,0,.3),1).objects.size(),10u);
  ObjectTracker on({{"column_distance",2.}});
  Result r;
  for(int i=0;i<40;++i)
    r=on.process(tallColumn(20,.5),Vec(0,0,.3),10+i*.1);
  ASSERT_EQ(1u,r.objects.size());
  EXPECT_GT(r.objects[0]["size"][2].asDouble(),18.);
  EXPECT_LT(r.objects[0]["size"][2].asDouble(),21.);
  EXPECT_GT(r.objects[0]["point_count"].asUInt64(),500u);
}

// 柱状邻域只放宽竖直方向：水平错开的两个物体不能被连成一个。
TEST(Tracking, ColumnDistanceKeepsHorizontallySeparatedObjectsApart) {
  ObjectTracker t({{"column_distance",2.}});
  auto cloud=tallColumn(20,.5,.5),second=tallColumn(20,.5,.5,1.5);
  cloud.insert(cloud.end(),second.begin()+1225,second.end());
  auto r=t.process(cloud,Vec(0,0,.3),1);
  ASSERT_EQ(2u,r.objects.size());
  for(const auto& o:r.objects) EXPECT_GT(o["size"][2].asDouble(),18.);
}

namespace {
// 地面 + 一个"两面箱"：正面法向沿 y（点密），侧面法向沿 x（点稀），侧面可整面消失/出现。
Cloud twoFaceBox(double x0, bool side) {
  Cloud p;
  for (int j = 0; j < 35; ++j)
    for (int i = 0; i < 35; ++i)
      p.emplace_back(-3. + 12. * i / 34, -6. + 12. * j / 34, 0);
  for (int j = 0; j < 24; ++j)
    for (int i = 0; i < 24; ++i)
      p.emplace_back(x0 - .5 + 1. * i / 23, 4.5, .17 + .83 * j / 23);
  if (side)
    for (int j = 0; j < 12; ++j)
      for (int i = 0; i < 12; ++i)
        p.emplace_back(x0 + .5, 4.5 + 1. * i / 11, .17 + .83 * j / 11);
  return p;
}
}  // namespace

// 物体只沿 x 平移，但侧面反复进出视野：包围盒中点会横向跳变，不能把它算成速度。
TEST(Tracking, SideFaceFlickerDoesNotAddLateralVelocity) {
  ObjectTracker t;
  double worst_vy = 0, worst_jump = 0, speed = 0, last_cy = 0;
  int n = 0;
  for (int i = 0; i < 60; ++i) {
    auto r = t.process(twoFaceBox(i * .08, ((i / 6) % 2) == 1), Vec(0, 0, .3), 40 + i * .1);
    if (i < 10 || r.objects.empty())
      continue;
    const auto& o = r.objects[0];
    double cy = o["center"][1].asDouble();
    if (n)
      worst_jump = std::max(worst_jump, std::abs(cy - last_cy));
    last_cy = cy;
    worst_vy = std::max(worst_vy, std::abs(o["velocity"][1].asDouble()));
    speed += o["speed"].asDouble();
    ++n;
  }
  ASSERT_GT(n, 30);
  EXPECT_GT(worst_jump, .3);          // 侧面进出视野确实让包围盒中点横跳
  EXPECT_LT(worst_vy, .2);            // 但报告的速度不跟着跳（旧实现这里是 2.5 m/s）
  EXPECT_NEAR(.8, speed / n, .15);    // 真实平移仍被正确估计
}

namespace {
// 地面 + 一根竖直面：沿 y 平移，同时可见高度在两个平台间切换（模拟只看到物体的一部分）。
Cloud flickerBox(double y, double top) {
  Cloud p;
  for (int j = 0; j < 35; ++j)
    for (int i = 0; i < 35; ++i)
      p.emplace_back(-3. + 12. * i / 34, -6. + 12. * j / 34, 0);
  for (int j = 0; j < 16; ++j)
    for (int i = 0; i < 12; ++i)
      p.emplace_back(4.5, -.5 + y + i / 11., .17 + (top - .17) * j / 15.);
  return p;
}
}  // namespace

// 可见高度来回切换时，簇质心的 z 会整体漂移；不能把它读成纵向速度（实测可造出 ±3 m/s）。
TEST(Tracking, VisibleHeightFlickerDoesNotCreateVerticalVelocity) {
  auto run = [](const Config& c, double& worst_vz, double& mean_vy) {
    ObjectTracker t(c);
    double mz = 0, sy = 0;
    int n = 0;
    for (int i = 0; i < 64; ++i) {
      // 平台 + 每帧 0.5 m 的渐进过渡（和实测一致：单帧尺寸变化不能触发关联门限 shape<0.85）
      int ph = i % 16;
      double top = ph < 4 ? 2.0 : (ph < 8 ? 2.0 + .5 * (ph - 3) : (ph < 12 ? 4.0 : 4.0 - .5 * (ph - 11)));
      auto r = t.process(flickerBox(i * .06, top), Vec(0, 0, .3), 60 + i * .1);
      if (i < 12 || r.objects.empty())
        continue;
      const auto& o = r.objects[0];
      mz = std::max(mz, std::abs(o["velocity"][2].asDouble()));
      sy += o["velocity"][1].asDouble();
      ++n;
    }
    worst_vz = mz;
    mean_vy = sy / n;
  };
  double off_vz = 0, off_vy = 0, on_vz = 0, on_vy = 0;
  run({}, off_vz, off_vy);
  run({{"motion_vertical_tolerance", .3}}, on_vz, on_vy);
  EXPECT_GT(off_vz, .8);            // 不过滤：可见高度切换造出纵向假速度
  EXPECT_LT(on_vz, .25);            // 过滤后不再有
  EXPECT_NEAR(.6, off_vy, .15);     // 真实平移（y 方向 0.6 m/s）仍被正确估计
  EXPECT_NEAR(.6, on_vy, .15);
}

// 只看到物体的一部分（可见高度剧烈波动）时，形状判据若含竖直尺寸就会把 MOVING 判没。
TEST(Tracking, HorizontalOnlyShapeKeepsTallObjectMoving) {
  auto run = [](const Config& c) {
    ObjectTracker t(c);
    double moving = 0;
    int n = 0;
    for (int i = 0; i < 64; ++i) {
      int ph = i % 16;
      double top = ph < 4 ? 2.0 : (ph < 8 ? 2.0 + .5 * (ph - 3) : (ph < 12 ? 4.0 : 4.0 - .5 * (ph - 11)));
      auto r = t.process(flickerBox(i * .06, top), Vec(0, 0, .3), 80 + i * .1);
      if (i < 16) continue;
      ++n;
      if (!r.objects.empty() && r.objects[0]["state"].asString() == "MOVING") ++moving;
    }
    return moving / n;
  };
  const double old_ratio = run({});
  const double new_ratio = run({{"shape_horizontal_only", 1}});
  EXPECT_LT(old_ratio, .2);    // 旧行为：可见高度一变就掉成 UNKNOWN
  EXPECT_GT(new_ratio, .7);    // 只用水平尺寸后能维持 MOVING
}
