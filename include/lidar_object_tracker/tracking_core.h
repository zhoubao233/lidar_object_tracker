#pragma once
#include <jsoncpp/json/json.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace lot {
using Vec = Eigen::Vector3d;
using Cloud = std::vector<Vec>;
using Config = std::map<std::string, double>;
Config defaultConfig();
Config validatedConfig(const Config& overrides = {});
double quantile(std::vector<double> values, double fraction);
Vec quantile(const Cloud& points, double fraction);
Json::Value toJson(const Vec& v);
Vec fromJson(const Json::Value& v);
struct VoxelCloud {
  Cloud points;
  std::vector<int> inverse;
};
VoxelCloud voxelize(const Cloud& points, double size);
Cloud sampleCloud(const Cloud& points, size_t limit);
// Hungarian assignment: globally minimum rectangular, one-to-one assignment.
std::vector<std::pair<int, int>> assign(const std::vector<std::vector<double>>& costs);
Json::Value alignmentEvidence(const Cloud& old, const Cloud& now, const Vec& displacement,
                              double distance);
struct Detection {
  std::vector<size_t> indices;
  Cloud points;
  Vec center = Vec::Zero(), size = Vec::Zero(), lower = Vec::Zero(), upper = Vec::Zero();
  // 簇内原始点的质心。包围盒中点会随"看到哪几个面"整体平移，质心对这种可见性变化
  // 敏感度低得多，因此速度估计用它，包围盒中点仍然用于关联和对外报告。
  Vec centroid = Vec::Zero();
};
struct Observation {
  double stamp;
  Vec center, centroid;
  Cloud points;
  Vec size;
};
struct Track {
  int id;
  Detection detection;
  double last_seen;
  std::deque<Observation> history;
  Vec velocity = Vec::Zero();
  std::string state = "UNKNOWN";
  int moving_votes = 0;
  double quiet_since = std::numeric_limits<double>::quiet_NaN();
  double last_motion = -std::numeric_limits<double>::infinity();
  Json::Value evidence = Json::Value(Json::objectValue);
};
struct Result {
  Json::Value objects = Json::Value(Json::arrayValue);
  std::vector<uint8_t> dynamic, uncertain;
  std::vector<int32_t> moving_owner; // 0 = retain as environment; otherwise observed MOVING track ID
  bool ground_valid = false, reset = false;
  bool segmentation_valid = false;
  std::string segmentation_mode = "unavailable";
  double processing_ms = 0;
};
// World-space input; this class neither uses ROS nor reads simulator ground truth.
class ObjectTracker {
 public:
  explicit ObjectTracker(const Config& overrides = {});
  void reset();
  Result process(const Cloud& points, const Vec& origin, double stamp);
  const Config& config() const {
    return c_;
  }

 private:
  Config c_;
  int next_id_ = 1;
  std::vector<Track> tracks_;
  Vec ground_normal_ = Vec::UnitZ(), previous_origin_ = Vec::Zero();
  double ground_offset_ = 0, ground_stamp_, previous_stamp_;
  bool estimateGround(const Cloud&, const Vec&, double);
  std::pair<std::vector<Detection>, bool> segment(const Cloud&, const Vec&, double, Result&);
  void update(Track&, const Detection&, double);
};
}  // namespace lot
