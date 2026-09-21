#include "lidar_object_tracker/ros_utils.h"

#include <sensor_msgs/PointField.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
namespace lot {
std::string frameName(std::string s) {
  s.erase(0, s.find_first_not_of('/'));
  return s;
}
Eigen::Quaterniond checkedQuaternion(const Eigen::Quaterniond& q) {
  if (!q.coeffs().allFinite() || q.norm() < 1e-8)
    throw std::invalid_argument("invalid quaternion");
  return q.normalized();
}
Pose readPose(const geometry_msgs::Pose& p) {
  Pose out;
  out.position = Vec(p.position.x, p.position.y, p.position.z);
  if (!out.position.allFinite())
    throw std::invalid_argument("invalid position");
  out.rotation = checkedQuaternion(
      Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z));
  return out;
}
void writePose(const Pose& p, geometry_msgs::Pose& m) {
  m.position.x = p.position.x();
  m.position.y = p.position.y();
  m.position.z = p.position.z();
  m.orientation.x = p.rotation.x();
  m.orientation.y = p.rotation.y();
  m.orientation.z = p.rotation.z();
  m.orientation.w = p.rotation.w();
}
namespace {
bool bigEndian() {
  uint16_t x = 1;
  return reinterpret_cast<uint8_t*>(&x)[0] == 0;
}
template <typename T>
T readNumber(const uint8_t* p, bool swap) {
  uint8_t bytes[sizeof(T)];
  std::copy(p, p + sizeof(T), bytes);
  if (swap)
    std::reverse(bytes, bytes + sizeof(T));
  T x;
  std::memcpy(&x, bytes, sizeof(T));
  return x;
}
void putFloat(uint8_t* p, float value) {
  uint8_t bytes[4];
  std::memcpy(bytes, &value, 4);
  if (bigEndian())
    std::reverse(bytes, bytes + 4);
  std::copy(bytes, bytes + 4, p);
}
geometry_msgs::Point point(const Vec& p) {
  geometry_msgs::Point out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  return out;
}
}  // namespace
Cloud readXYZ(const sensor_msgs::PointCloud2& msg) {
  if (msg.data.size() < uint64_t(msg.row_step) * msg.height)
    throw std::invalid_argument("truncated PointCloud2");
  if (!msg.width || !msg.height)
    return {};
  sensor_msgs::PointField f[3];
  int k = 0;
  for (const std::string name : {"x", "y", "z"}) {
    auto it = std::find_if(msg.fields.begin(), msg.fields.end(),
                           [&](const auto& x) { return x.name == name; });
    if (it == msg.fields.end() || it->count != 1 || (it->datatype != 7 && it->datatype != 8))
      throw std::invalid_argument("XYZ must be scalar float fields");
    if (uint64_t(it->offset) + (it->datatype == 7 ? 4 : 8) > msg.point_step ||
        uint64_t(msg.width) * msg.point_step > msg.row_step)
      throw std::invalid_argument("invalid PointCloud2 strides");
    f[k++] = *it;
  }
  Cloud out;
  out.reserve(uint64_t(msg.width) * msg.height);
  for (uint32_t row = 0; row < msg.height; ++row)
    for (uint32_t col = 0; col < msg.width; ++col) {
      Vec p;
      for (int i = 0; i < 3; ++i) {
        const auto* b = msg.data.data() + uint64_t(row) * msg.row_step +
                        uint64_t(col) * msg.point_step + f[i].offset;
        p[i] = f[i].datatype == 7 ? readNumber<float>(b, msg.is_bigendian != bigEndian())
                                  : readNumber<double>(b, msg.is_bigendian != bigEndian());
      }
      out.push_back(p);
    }
  return out;
}
sensor_msgs::PointCloud2 cloudMessage(const Cloud& p, const std_msgs::Header& h,
                                      const std::vector<float>* intensity) {
  if (intensity && intensity->size() != p.size())
    throw std::invalid_argument("intensity size mismatch");
  sensor_msgs::PointCloud2 out;
  out.header = h;
  out.height = 1;
  out.width = p.size();
  out.is_bigendian = false;
  out.is_dense = true;
  const std::vector<std::string> names = {"x",        "y",        "z",        "intensity",
                                          "normal_x", "normal_y", "normal_z", "curvature"};
  for (int i = 0; i < (intensity ? 8 : 3); ++i) {
    sensor_msgs::PointField f;
    f.name = names[i];
    f.offset = i * 4;
    f.count = 1;
    f.datatype = 7;
    out.fields.push_back(f);
  }
  out.point_step = intensity ? 32 : 12;
  out.row_step = out.width * out.point_step;
  out.data.resize(out.row_step, 0);
  for (size_t i = 0; i < p.size(); ++i) {
    for (int k = 0; k < 3; ++k)
      putFloat(out.data.data() + i * out.point_step + k * 4, p[i][k]);
    if (!p[i].allFinite())
      out.is_dense = false;
    if (intensity)
      putFloat(out.data.data() + i * out.point_step + 12, (*intensity)[i]);
  }
  return out;
}
std::pair<Cloud, Vec> worldCloud(const sensor_msgs::PointCloud2& cloud,
                                 const nav_msgs::Odometry& odom, const std::string& frame) {
  if (cloud.header.stamp != odom.header.stamp)
    throw std::invalid_argument("cloud and odometry timestamps differ");
  if (frameName(odom.header.frame_id) != frameName(frame))
    throw std::invalid_argument("odometry world frame mismatch");
  if (frameName(cloud.header.frame_id) != frameName(odom.child_frame_id))
    throw std::invalid_argument("cloud/body frame mismatch");
  auto pose = readPose(odom.pose.pose);
  auto points = readXYZ(cloud);
  for (auto& p : points)
    p = pose.rotation * p + pose.position;
  return {points, pose.position};
}
Config loadConfig(const ros::NodeHandle& nh) {
  XmlRpc::XmlRpcValue params;
  Config c;
  if (nh.getParam("tracking", params)) {
    if (params.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      throw std::invalid_argument("tracking must be a dictionary");
    for (auto it = params.begin(); it != params.end(); ++it) {
      auto& v = it->second;
      if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
        c[it->first] = static_cast<int>(v);
      else if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        c[it->first] = static_cast<double>(v);
      else
        throw std::invalid_argument("non-numeric tracking parameter");
    }
  }
  return validatedConfig(c);
}
Config loadYaml(const std::string& file) {
  auto node = YAML::LoadFile(file)["tracking"];
  if (!node.IsMap())
    throw std::invalid_argument("config requires tracking map");
  Config c;
  for (const auto& kv : node)
    c[kv.first.as<std::string>()] = kv.second.as<double>();
  return validatedConfig(c);
}
visualization_msgs::MarkerArray markers(const Json::Value& objects, const std_msgs::Header& h) {
  using M = visualization_msgs::Marker;
  visualization_msgs::MarkerArray out;
  M clear;
  clear.header = h;
  clear.action = M::DELETEALL;
  out.markers.push_back(clear);
  for (const auto& o : objects) {
    Vec center = fromJson(o["center"]), size = fromJson(o["size"]).cwiseMax(.06);
    std::string state = o["state"].asString();
    Vec color = state == "MOVING"      ? Vec(1, .12, .12)
                : state == "STATIC"    ? Vec(.15, .85, .35)
                : state == "PREDICTED" ? Vec(1, .5, .08)
                                       : Vec(1, .85, .1);
    M box;
    box.header = h;
    box.ns = "objects";
    box.id = o["id"].asInt();
    box.type = M::LINE_LIST;
    box.action = M::ADD;
    box.pose.orientation.w = 1;
    box.scale.x = .035;
    box.color.r = color.x();
    box.color.g = color.y();
    box.color.b = color.z();
    box.color.a = o["observed"].asBool() ? 1 : .55;
    box.lifetime = ros::Duration(.35);
    Vec corners[8];
    for (int i = 0; i < 8; ++i)
      corners[i] =
          center + .5 * size.cwiseProduct(Vec(i & 4 ? 1 : -1, i & 2 ? 1 : -1, i & 1 ? 1 : -1));
    for (int i = 0; i < 8; ++i)
      for (int bit : {1, 2, 4})
        if (i < (i ^ bit)) {
          box.points.push_back(point(corners[i]));
          box.points.push_back(point(corners[i ^ bit]));
        }
    out.markers.push_back(box);
    M label = box;
    label.ns = "labels";
    label.type = M::TEXT_VIEW_FACING;
    label.points.clear();
    label.pose.position = point(center + Vec(0, 0, size.z() / 2 + .25));
    label.scale.x = 0;
    label.scale.z = .2;
    label.color.a = 1;
    std::ostringstream text;
    text << "#" << box.id << " " << state << "\n"
         << std::fixed << std::setprecision(2) << o["speed"].asDouble() << " m/s";
    label.text = text.str();
    out.markers.push_back(label);
    if (o["observed"].asBool() && state == "MOVING" && o["speed"].asDouble() > .05) {
      M arrow = box;
      arrow.ns = "velocity";
      arrow.type = M::ARROW;
      arrow.points = {point(center), point(center + .5 * fromJson(o["velocity"]))};
      arrow.scale.x = .05;
      arrow.scale.y = .12;
      arrow.scale.z = .15;
      arrow.color.r = 1;
      arrow.color.g = .25;
      arrow.color.b = .15;
      arrow.color.a = 1;
      out.markers.push_back(arrow);
    }
  }
  return out;
}
Json::Value report(const Result& r, const std_msgs::Header& h) {
  Json::Value j(Json::objectValue);
  j["stamp"] = h.stamp.toSec();
  j["stamp_ns"] = Json::UInt64(h.stamp.toNSec());
  j["frame_id"] = h.frame_id;
  j["objects"] = r.objects;
  j["ground_valid"] = r.ground_valid;
  j["segmentation_valid"] = r.segmentation_valid;
  j["segmentation_mode"] = r.segmentation_mode;
  j["reset"] = r.reset;
  j["processing_ms"] = r.processing_ms;
  return j;
}
std::string jsonString(const Json::Value& j, bool pretty) {
  Json::StreamWriterBuilder b;
  b["indentation"] = pretty ? "  " : "";
  b["precision"] = 17;
  return Json::writeString(b, j);
}
Json::Value parseJson(const std::string& text) {
  Json::CharReaderBuilder b;
  Json::Value j;
  std::string error;
  std::istringstream in(text);
  if (!Json::parseFromStream(b, in, &j, &error))
    throw std::invalid_argument("invalid JSON: " + error);
  return j;
}
}  // namespace lot
