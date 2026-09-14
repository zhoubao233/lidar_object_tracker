// Offline bag scoring only. Gazebo ground truth never enters the live tracker.
#include <gazebo_msgs/ModelStates.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>

#include "lidar_object_tracker/ros_utils.h"

namespace lot {
namespace {
bool endsWith(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}
struct GroundTruth {
  double stamp;
  std::map<std::string, geometry_msgs::Pose> models;
};
struct Match {
  std::vector<uint8_t> mask;
  size_t unmatched = 0;
};
Match matchPoints(const Cloud& input, const Cloud& output, bool strict) {
  Match out;
  out.mask.resize(input.size());
  if (output.empty())
    return out;
  if (input.empty()) {
    if (strict)
      throw std::invalid_argument("dynamic output with empty input");
    out.unmatched = output.size();
    return out;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<size_t> indices;
  for (size_t i = 0; i < input.size(); ++i)
    if (input[i].allFinite()) {
      cloud->push_back(pcl::PointXYZ(input[i].x(), input[i].y(), input[i].z()));
      indices.push_back(i);
    }
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  if (!cloud->empty())
    tree.setInputCloud(cloud);
  for (const auto& p : output) {
    std::vector<int> id(1);
    std::vector<float> d(1);
    if (!p.allFinite() || cloud->empty() ||
        tree.nearestKSearch(pcl::PointXYZ(p.x(), p.y(), p.z()), 1, id, d) != 1 ||
        (p - input[indices[id[0]]]).norm() >= .001) {
      ++out.unmatched;
      if (strict)
        throw std::invalid_argument("recorded dynamic points mismatch current input");
    } else
      out.mask[indices[id[0]]] = 1;
  }
  return out;
}
Json::Value qs(const std::vector<Json::Value>& rows, const std::string& name) {
  std::vector<double> v;
  for (const auto& r : rows)
    v.push_back(r[name].asDouble());
  Json::Value a(Json::arrayValue);
  for (double q : {.5, .95, 1.})
    a.append(quantile(v, q));
  return a;
}
double sum(const std::vector<Json::Value>& rows, const std::string& name) {
  double s = 0;
  for (const auto& r : rows)
    s += r[name].asDouble();
  return s;
}
std::string csvValue(const Json::Value& value) {
  std::string s = value.isString() ? value.asString() : jsonString(value);
  if (s.find_first_of(",\"\n") != std::string::npos) {
    std::string escaped = "\"";
    for (char c : s) {
      escaped += c;
      if (c == '\"')
        escaped += '\"';
    }
    return escaped + '\"';
  }
  return s;
}
}  // namespace
void evaluateBag(const std::string& bag_path, const Config& config, const std::string& output,
                 bool recorded) {
  std::map<uint64_t, sensor_msgs::PointCloud2> clouds, dynamic_clouds, original_clouds;
  std::map<uint64_t, nav_msgs::Odometry> odoms;
  // Microseconds mitigate JSON floating-point epoch serialization loss.
  std::map<int64_t, Json::Value> reports;
  std::vector<GroundTruth> states;
  rosbag::Bag bag(bag_path, rosbag::bagmode::Read);
  for (const auto& item : rosbag::View(bag)) {
    const auto topic = item.getTopic();
    if (endsWith(topic, "/input/cloud_body")) {
      auto m = item.instantiate<sensor_msgs::PointCloud2>();
      if (m)
        clouds[m->header.stamp.toNSec()] = *m;
    } else if (endsWith(topic, "/input/odom")) {
      auto m = item.instantiate<nav_msgs::Odometry>();
      if (m)
        odoms[m->header.stamp.toNSec()] = *m;
    } else if (topic == "/gazebo/model_states") {
      auto m = item.instantiate<gazebo_msgs::ModelStates>();
      double t = item.getTime().toSec();
      if (m && (states.empty() || t - states.back().stamp > .009)) {
        GroundTruth state;
        state.stamp = t;
        for (size_t i = 0; i < std::min(m->name.size(), m->pose.size()); ++i)
          state.models[m->name[i]] = m->pose[i];
        states.push_back(state);
      }
    } else if (topic == "/object_tracker_test/tracks" || topic == "/lidar_object_tracker/tracks") {
      auto m = item.instantiate<std_msgs::String>();
      if (m) {
        auto j = parseJson(m->data);
        reports[std::llround(j["stamp"].asDouble() * 1e6)] = j;
      }
    } else if (topic == "/object_tracker_test/dynamic" ||
               topic == "/lidar_object_tracker/dynamic") {
      auto m = item.instantiate<sensor_msgs::PointCloud2>();
      if (m)
        dynamic_clouds[m->header.stamp.toNSec()] = *m;
    } else if (topic == "/m_detector_livox_test/frame_out") {
      auto m = item.instantiate<sensor_msgs::PointCloud2>();
      if (m)
        original_clouds[m->header.stamp.toNSec()] = *m;
    }
  }
  bag.close();
  auto nearest = [&](double stamp) {
    auto it = std::lower_bound(states.begin(), states.end(), stamp,
                               [](const GroundTruth& s, double t) { return s.stamp < t; });
    size_t hi = std::min<size_t>(it - states.begin(), states.size() - 1),
           lo = it == states.begin() ? 0 : size_t(it - states.begin() - 1);
    return std::abs(states[lo].stamp - stamp) <= std::abs(states[hi].stamp - stamp) ? lo : hi;
  };
  ObjectTracker tracker(config);
  std::vector<Json::Value> rows;
  for (const auto& kv : clouds) {
    uint64_t key = kv.first;
    if (!odoms.count(key))
      continue;
    int64_t micro = std::llround(double(key) / 1000);
    if (recorded && (!reports.count(micro) || !dynamic_clouds.count(key)))
      continue;
    double stamp = double(key) / 1e9;
    const auto& odom = odoms.at(key);
    auto input = worldCloud(kv.second, odom, odom.header.frame_id);
    const auto& points = input.first;
    Vec origin = input.second;
    Result result;
    Json::Value result_json;
    if (recorded) {
      result_json = reports.at(micro);
      result.objects = result_json["objects"];
      result.processing_ms = result_json["processing_ms"].asDouble();
      result.ground_valid = result_json["ground_valid"].asBool();
      result.dynamic = matchPoints(points, readXYZ(dynamic_clouds.at(key)), true).mask;
    } else
      result = tracker.process(points, origin, stamp);
    Json::Value row(Json::objectValue);
    row["stamp"] = stamp;
    row["processing_ms"] = result.processing_ms;
    row["ground_valid"] = result.ground_valid;
    row["objects"] = result.objects.size();
    int moving = 0;
    for (const auto& o : result.objects)
      if (o["state"].asString() == "MOVING")
        ++moving;
    row["moving"] = moving;
    if (recorded) {
      row["input_age_seconds"] = result_json["input_age_seconds"];
      row["skipped_pairs"] = result_json["skipped_pairs"];
    }
    if (!states.empty()) {
      const auto& state = states[nearest(stamp)].models;
      if (!state.count("iris") || !state.count("unit_box"))
        throw std::invalid_argument("Gazebo scoring requires iris and unit_box ground truth");
      auto base = readPose(state.at("iris"));
      base.position += base.rotation * Vec(0, 0, .194923);
      auto box = readPose(state.at("unit_box"));
      auto observed_pose = readPose(odom.pose.pose);
      std::vector<uint8_t> upper(points.size()), lower(points.size()), loose(points.size());
      int input_upper = 0, input_lower = 0, dynamic_upper = 0, dynamic_lower = 0, outside = 0;
      for (size_t i = 0; i < points.size(); ++i) {
        Vec body = observed_pose.rotation.conjugate() * (points[i] - origin);
        Vec gazebo = base.rotation * body + base.position;
        Vec local = box.rotation.conjugate() * (gazebo - box.position);
        bool in = local.cwiseAbs().maxCoeff() < .54 && local.z() > -.35;
        upper[i] = in && local.z() >= 0;
        lower[i] = in && local.z() < 0;
        loose[i] = local.cwiseAbs().maxCoeff() < .65;
        input_upper += upper[i];
        input_lower += lower[i];
        dynamic_upper += upper[i] && result.dynamic[i];
        dynamic_lower += lower[i] && result.dynamic[i];
        outside += !loose[i] && result.dynamic[i];
      }
      Vec box_world =
          observed_pose.rotation * (base.rotation.conjugate() * (box.position - base.position)) +
          origin;
      Json::Value object;
      double distance = .9;
      for (const auto& o : result.objects) {
        double d = (fromJson(o["center"]) - box_world).norm();
        if (o["observed"].asBool() && d < distance) {
          object = o;
          distance = d;
        }
      }
      auto lo = nearest(stamp - .15), hi = nearest(stamp + .15);
      double speed = (readPose(states[hi].models.at("unit_box")).position -
                      readPose(states[lo].models.at("unit_box")).position)
                         .norm() /
                     std::max(1e-6, states[hi].stamp - states[lo].stamp);
      row["box_id"] = object.isNull() ? Json::Value(-1) : object["id"];
      row["box_state"] = object.isNull() ? Json::Value("MISSING") : object["state"];
      row["true_speed"] = speed;
      row["estimated_speed"] = object.isNull() ? Json::Value(0.) : object["speed"];
      row["input_upper"] = input_upper;
      row["input_lower"] = input_lower;
      row["dynamic_upper"] = dynamic_upper;
      row["dynamic_lower"] = dynamic_lower;
      row["outside_dynamic"] = outside;
      row["box_geometry_ok"] =
          !object.isNull() && object["evidence"].get("geometry_ok", false).asBool();
      row["original_present"] = false;
      row["original_upper"] = 0;
      row["original_lower"] = 0;
      row["original_unmatched"] = 0;
      if (original_clouds.count(key)) {
        auto matched = matchPoints(points, readXYZ(original_clouds.at(key)), false);
        int u = 0, l = 0;
        for (size_t i = 0; i < points.size(); ++i) {
          u += matched.mask[i] && upper[i];
          l += matched.mask[i] && lower[i];
        }
        row["original_present"] = matched.unmatched == 0;
        row["original_upper"] = u;
        row["original_lower"] = l;
        row["original_unmatched"] = Json::UInt64(matched.unmatched);
      }
    }
    rows.push_back(row);
  }
  if (rows.empty())
    throw std::invalid_argument("no complete input/output pairs in recording");
  std::vector<Json::Value> scored;
  for (const auto& r : rows)
    if (r["stamp"].asDouble() >= rows[0]["stamp"].asDouble() + 1.5)
      scored.push_back(r);
  Json::Value summary(Json::objectValue);
  summary["bag"] = bag_path;
  summary["recorded"] = recorded;
  summary["frames"] = Json::UInt64(rows.size());
  summary["scored_frames"] = Json::UInt64(scored.size());
  summary["processing_ms_quantiles"] = qs(rows, "processing_ms");
  int missing = 0;
  for (const auto& r : rows)
    missing += !r["ground_valid"].asBool();
  summary["ground_missing"] = missing;
  if (recorded) {
    summary["input_age_seconds_quantiles"] = qs(rows, "input_age_seconds");
    double max = 0;
    for (const auto& r : rows)
      max = std::max(max, r["skipped_pairs"].asDouble());
    summary["skipped_pairs_max"] = max;
  }
  if (!states.empty()) {
    for (const std::string s : {"MOVING", "STATIC", "UNKNOWN", "MISSING"}) {
      int count = 0;
      for (const auto& r : scored)
        count += r["box_state"].asString() == s;
      summary["box_states"][s] = count;
    }
    std::set<int> ids;
    for (const auto& r : scored)
      ids.insert(r["box_id"].asInt());
    summary["box_ids"] = Json::Value(Json::arrayValue);
    for (int id : ids)
      summary["box_ids"].append(id);
    summary["upper_coverage"] =
        sum(scored, "dynamic_upper") / std::max(1., sum(scored, "input_upper"));
    summary["lower_coverage"] =
        sum(scored, "dynamic_lower") / std::max(1., sum(scored, "input_lower"));
    summary["outside_dynamic_mean"] =
        scored.empty() ? Json::Value()
                       : Json::Value(sum(scored, "outside_dynamic") / scored.size());
    std::vector<Json::Value> common;
    for (const auto& r : scored)
      if (r["original_present"].asBool())
        common.push_back(r);
    if (!common.empty()) {
      auto& c = summary["same_frame_comparison"];
      c["frames"] = Json::UInt64(common.size());
      for (const std::string half : {"upper", "lower"}) {
        c["original_" + half + "_coverage"] =
            sum(common, "original_" + half) / std::max(1., sum(common, "input_" + half));
        c["tracker_" + half + "_coverage"] =
            sum(common, "dynamic_" + half) / std::max(1., sum(common, "input_" + half));
      }
    }
  }
  std::filesystem::create_directories(output);
  std::ofstream json(std::filesystem::path(output) / "summary.json"),
      csv(std::filesystem::path(output) / "frames.csv");
  if (!json || !csv)
    throw std::runtime_error("cannot open evaluation output files");
  json << jsonString(summary, true) << '\n';
  auto fields = rows[0].getMemberNames();
  for (size_t i = 0; i < fields.size(); ++i)
    csv << (i ? "," : "") << fields[i];
  csv << '\n';
  for (const auto& r : rows) {
    for (size_t i = 0; i < fields.size(); ++i)
      csv << (i ? "," : "") << csvValue(r[fields[i]]);
    csv << '\n';
  }
  std::cout << jsonString(summary, true) << std::endl;
}
}  // namespace lot
int main(int argc, char** argv) {
  try {
    ros::Time::init();
    std::string bag, output, config;
    bool recorded = false;
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--help" || arg == "-h") {
        std::cout << "evaluate_bag BAG --output DIRECTORY [--config tracking.yaml] [--recorded]\n";
        return 0;
      }
      if (arg == "--recorded")
        recorded = true;
      else if (arg == "--output" || arg == "--config") {
        if (++i >= argc)
          throw std::invalid_argument("missing option value");
        (arg == "--output" ? output : config) = argv[i];
      } else if (!arg.empty() && arg[0] == '-')
        throw std::invalid_argument("unknown option: " + arg);
      else if (bag.empty())
        bag = arg;
      else
        throw std::invalid_argument("too many bag arguments");
    }
    if (bag.empty() || output.empty())
      throw std::invalid_argument("usage: evaluate_bag BAG --output DIRECTORY [--recorded]");
    if (config.empty())
      config = ros::package::getPath("lidar_object_tracker") + "/config/tracking.yaml";
    lot::evaluateBag(bag, lot::loadYaml(config), output, recorded);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "evaluate_bag: " << e.what() << '\n';
    return 1;
  }
}
