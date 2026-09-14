#include "lidar_object_tracker/tracking_core.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <numeric>
#include <random>
#include <stdexcept>
#include <tuple>

namespace lot {
Config defaultConfig() {
  return {{"max_range", 25},
          {"voxel_size", .1},
          {"cluster_distance", .32},
          {"min_cluster_points", 18},
          {"max_cluster_size", 5},
          {"max_objects", 80},
          {"ground_distance", .1},
          {"object_min_height", .13},
          {"ground_min_points", 60},
          {"ground_trials", 64},
          {"ground_cache_seconds", .5},
          {"history_seconds", 1.4},
          {"motion_window", .55},
          {"min_observation_time", .4},
          {"moving_speed", .25},
          {"static_speed", .12},
          {"moving_confirm", 2},
          {"static_confirm_seconds", .7},
          {"unknown_after_seconds", .8},
          {"association_distance", .65},
          {"max_speed", 5},
          {"max_missed_seconds", .5},
          {"max_frame_gap", 1},
          {"registration_distance", .18},
          {"registration_overlap", .65},
          {"registration_gain", .018},
          {"registration_points", 350},
          {"max_input_points", 30000}};
}
Config validatedConfig(const Config& overrides) {
  auto c = defaultConfig();
  for (const auto& kv : overrides) {
    if (!c.count(kv.first))
      throw std::invalid_argument("unknown tracker parameter: " + kv.first);
    c[kv.first] = kv.second;
  }
  for (const auto& kv : c)
    if (!std::isfinite(kv.second) || kv.second <= 0)
      throw std::invalid_argument("parameter must be positive and finite: " + kv.first);
  if (c.at("static_speed") >= c.at("moving_speed"))
    throw std::invalid_argument("static_speed must be below moving_speed");
  for (const auto& key :
       {"max_objects", "ground_trials", "registration_points", "max_input_points"})
    if (c.at(key) < 1 || c.at(key) > 10000000)
      throw std::invalid_argument(std::string("invalid count: ") + key);
  return c;
}
double quantile(std::vector<double> v, double q) {
  if (v.empty())
    throw std::invalid_argument("empty quantile");
  std::sort(v.begin(), v.end());
  double x = (v.size() - 1) * q;
  size_t i = static_cast<size_t>(x), j = std::min(i + 1, v.size() - 1);
  return v[i] + (v[j] - v[i]) * (x - i);
}
Vec quantile(const Cloud& p, double q) {
  Vec out;
  for (int k = 0; k < 3; ++k) {
    std::vector<double> v;
    for (const auto& x : p)
      v.push_back(x[k]);
    out[k] = quantile(v, q);
  }
  return out;
}
Json::Value toJson(const Vec& v) {
  Json::Value a(Json::arrayValue);
  for (int i = 0; i < 3; ++i)
    a.append(v[i]);
  return a;
}
Vec fromJson(const Json::Value& v) {
  return Vec(v[0].asDouble(), v[1].asDouble(), v[2].asDouble());
}
VoxelCloud voxelize(const Cloud& p, double size) {
  using Key = std::array<int64_t, 3>;
  struct Acc {
    Vec sum = Vec::Zero();
    size_t count = 0;
    int id = 0;
  };
  std::map<Key, Acc> grid;
  std::vector<Key> keys;
  for (const auto& x : p) {
    Key key;
    for (int k = 0; k < 3; ++k) {
      double v = std::floor(x[k] / size);
      if (!std::isfinite(v) || std::abs(v) > 9e18)
        throw std::invalid_argument("invalid voxel coordinate");
      key[k] = static_cast<int64_t>(v);
    }
    auto& a = grid[key];
    a.sum += x;
    ++a.count;
    keys.push_back(key);
  }
  VoxelCloud out;
  // Ordered keys match numpy.unique(axis=0) so sampling retains spatial order.
  for (auto& kv : grid) {
    kv.second.id = out.points.size();
    out.points.push_back(kv.second.sum / kv.second.count);
  }
  for (const auto& k : keys)
    out.inverse.push_back(grid.at(k).id);
  return out;
}
Cloud sampleCloud(const Cloud& p, size_t n) {
  if (p.size() <= n)
    return p;
  Cloud out;
  for (size_t i = 0; i < n; ++i)
    out.push_back(p[n == 1 ? 0 : static_cast<size_t>(double(i) * (p.size() - 1) / (n - 1))]);
  return out;
}
namespace {
class Neighbors {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_{new pcl::PointCloud<pcl::PointXYZ>};
  pcl::KdTreeFLANN<pcl::PointXYZ> tree_;

 public:
  explicit Neighbors(const Cloud& p) {
    for (const auto& v : p)
      cloud_->push_back(pcl::PointXYZ(v.x(), v.y(), v.z()));
    tree_.setInputCloud(cloud_);
  }
  int nearest(const Vec& v) const {
    std::vector<int> ids(1);
    std::vector<float> d(1);
    if (tree_.nearestKSearch(pcl::PointXYZ(v.x(), v.y(), v.z()), 1, ids, d) != 1)
      throw std::runtime_error("nearest neighbor failed");
    return ids[0];
  }
  std::vector<int> radius(const Vec& v, double radius) const {
    std::vector<int> ids;
    std::vector<float> d;
    tree_.radiusSearch(pcl::PointXYZ(v.x(), v.y(), v.z()), radius, ids, d);
    return ids;
  }
};
std::vector<double> distances(const Cloud& a, const Cloud& b, const Neighbors& tree) {
  std::vector<double> d;
  for (const auto& p : a)
    d.push_back((p - b[tree.nearest(p)]).norm());
  return d;
}
Vec mean(const Cloud& p) {
  Vec m = Vec::Zero();
  for (const auto& v : p)
    m += v;
  return m / p.size();
}
// Legacy numpy RandomState uses MT19937 and a masked-rejection integer draw.
uint32_t interval(std::mt19937& rng, uint32_t max) {
  uint32_t mask = max;
  mask |= mask >> 1;
  mask |= mask >> 2;
  mask |= mask >> 4;
  mask |= mask >> 8;
  mask |= mask >> 16;
  uint32_t v;
  do {
    v = rng() & mask;
  } while (v > max);
  return v;
}
}  // namespace
Json::Value alignmentEvidence(const Cloud& old, const Cloud& now, const Vec& delta,
                              double threshold) {
  Neighbors target(now), source(old);
  auto stat = distances(old, now, target);
  auto reverse = distances(now, old, source);
  stat.insert(stat.end(), reverse.begin(), reverse.end());
  Cloud transformed;
  for (const auto& p : old)
    transformed.push_back(p + delta);
  for (int iter = 0; iter < 5; ++iter) {
    Cloud a, b;
    for (const auto& p : transformed) {
      auto j = target.nearest(p);
      if ((p - now[j]).norm() < threshold) {
        a.push_back(p);
        b.push_back(now[j]);
      }
    }
    if (a.size() < 12)
      break;
    Vec ac = mean(a), bc = mean(b);
    Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < a.size(); ++i)
      h += (a[i] - ac) * (b[i] - bc).transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(h, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d v = svd.matrixV(), u = svd.matrixU(), rotation = v * u.transpose();
    if (rotation.determinant() < 0) {
      v.col(2) *= -1;
      rotation = v * u.transpose();
    }
    Vec translation = bc - rotation * ac;
    Cloud candidate;
    for (const auto& p : transformed)
      candidate.push_back(rotation * p + translation);
    if ((mean(candidate) - mean(old) - delta).norm() > .18)
      break;
    transformed = std::move(candidate);
  }
  Neighbors moved(transformed);
  auto aligned = distances(transformed, now, target);
  reverse = distances(now, transformed, moved);
  aligned.insert(aligned.end(), reverse.begin(), reverse.end());
  Json::Value e(Json::objectValue);
  size_t count =
      std::count_if(aligned.begin(), aligned.end(), [&](double d) { return d < threshold; });
  e["overlap"] = double(count) / aligned.size();
  e["error"] = quantile(aligned, .8);
  e["static_error"] = quantile(stat, .8);
  e["gain"] = e["static_error"].asDouble() - e["error"].asDouble();
  return e;
}
std::vector<std::pair<int, int>> assign(const std::vector<std::vector<double>>& a) {
  if (a.empty() || a[0].empty())
    return {};
  int n = a.size(), real_m = a[0].size(), m = std::max(n, real_m);
  std::vector<double> u(n + 1), v(m + 1);
  std::vector<int> p(m + 1), way(m + 1);
  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> mins(m + 1, std::numeric_limits<double>::infinity());
    std::vector<bool> used(m + 1);
    do {
      used[j0] = true;
      int i0 = p[j0], j1 = 0;
      double delta = std::numeric_limits<double>::infinity();
      for (int j = 1; j <= m; ++j)
        if (!used[j]) {
          double cur = (j <= real_m ? a[i0 - 1][j - 1] : 0) - u[i0] - v[j];
          if (cur < mins[j]) {
            mins[j] = cur;
            way[j] = j0;
          }
          if (mins[j] < delta) {
            delta = mins[j];
            j1 = j;
          }
        }
      for (int j = 0; j <= m; ++j)
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else
          mins[j] -= delta;
      j0 = j1;
    } while (p[j0]);
    do {
      int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }
  std::vector<std::pair<int, int>> out;
  for (int j = 1; j <= real_m; ++j)
    if (p[j])
      out.emplace_back(p[j] - 1, j - 1);
  std::sort(out.begin(), out.end());
  return out;
}
ObjectTracker::ObjectTracker(const Config& overrides) : c_(validatedConfig(overrides)) {
  reset();
}
void ObjectTracker::reset() {
  tracks_.clear();
  ground_stamp_ = -INFINITY;
  previous_stamp_ = NAN;
}
bool ObjectTracker::estimateGround(const Cloud& p, const Vec& origin, double stamp) {
  std::vector<double> z;
  for (const auto& x : p)
    z.push_back(x.z());
  double limit = quantile(z, .6);
  Cloud candidates;
  for (const auto& x : p)
    if (x.z() <= limit)
      candidates.push_back(x);
  size_t best_count = 0;
  Cloud best;
  if (candidates.size() >= std::max(3., c_.at("ground_min_points"))) {
    std::mt19937 rng(7);
    for (int trial = 0; trial < int(c_.at("ground_trials")); ++trial) {
      std::vector<int> order(candidates.size());
      std::iota(order.begin(), order.end(), 0);
      for (size_t i = order.size() - 1; i > 0; --i)
        std::swap(order[i], order[interval(rng, i)]);
      Vec a = candidates[order[0]],
          normal = (candidates[order[1]] - a).cross(candidates[order[2]] - a);
      double len = normal.norm();
      if (len < 1e-7)
        continue;
      normal /= len;
      if (normal.z() < 0)
        normal = -normal;
      if (normal.z() < .94)
        continue;
      double offset = -normal.dot(a),
             floor_z = -(offset + normal.head<2>().dot(origin.head<2>())) / normal.z();
      if (floor_z > origin.z() + .1 || origin.z() - floor_z > 10)
        continue;
      Cloud inliers;
      for (const auto& x : candidates)
        if (std::abs(normal.dot(x) + offset) < c_.at("ground_distance"))
          inliers.push_back(x);
      if (inliers.size() > best_count) {
        best_count = inliers.size();
        best = std::move(inliers);
      }
    }
    if (best_count >= c_.at("ground_min_points")) {
      Vec center = mean(best);
      Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
      for (const auto& x : best)
        covariance += (x - center) * (x - center).transpose();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
      Vec normal = solver.eigenvectors().col(0);
      if (normal.z() < 0)
        normal = -normal;
      if (normal.z() >= .94) {
        ground_normal_ = normal;
        ground_offset_ = -normal.dot(center);
        ground_stamp_ = stamp;
      }
    }
  }
  return stamp - ground_stamp_ <= c_.at("ground_cache_seconds");
}
std::pair<std::vector<Detection>, bool> ObjectTracker::segment(const Cloud& p, const Vec& origin,
                                                               double stamp) {
  Cloud valid;
  std::vector<size_t> raw;
  for (size_t i = 0; i < p.size(); ++i)
    if (p[i].allFinite() && (p[i] - origin).norm() <= c_.at("max_range")) {
      valid.push_back(p[i]);
      raw.push_back(i);
    }
  if (valid.size() < c_.at("ground_min_points"))
    return {{}, false};
  auto vox = voxelize(valid, c_.at("voxel_size"));
  if (!estimateGround(vox.points, origin, stamp))
    return {{}, false};
  Cloud obs;
  std::vector<int> foreground;
  for (size_t i = 0; i < vox.points.size(); ++i)
    if (ground_normal_.dot(vox.points[i]) + ground_offset_ > c_.at("object_min_height")) {
      obs.push_back(vox.points[i]);
      foreground.push_back(i);
    }
  if (obs.empty())
    return {{}, true};
  Neighbors tree(obs);
  std::vector<int> labels(obs.size(), -1);
  int count = 0;
  for (size_t i = 0; i < obs.size(); ++i)
    if (labels[i] < 0) {
      std::vector<int> queue{int(i)};
      labels[i] = count;
      for (size_t head = 0; head < queue.size(); ++head)
        for (int j : tree.radius(obs[queue[head]], c_.at("cluster_distance")))
          if (labels[j] < 0) {
            labels[j] = count;
            queue.push_back(j);
          }
      ++count;
    }
  std::vector<int> voxel_labels(vox.points.size(), -1);
  for (size_t i = 0; i < obs.size(); ++i)
    voxel_labels[foreground[i]] = labels[i];
  std::vector<Detection> grouped(count);
  for (size_t i = 0; i < valid.size(); ++i) {
    int label = voxel_labels[vox.inverse[i]];
    if (label >= 0)
      grouped[label].indices.push_back(raw[i]);
  }
  for (size_t i = 0; i < obs.size(); ++i)
    grouped[labels[i]].points.push_back(obs[i]);
  std::vector<Detection> out;
  for (auto& d : grouped) {
    if (d.indices.size() < c_.at("min_cluster_points"))
      continue;
    d.lower = quantile(d.points, .02);
    d.upper = quantile(d.points, .98);
    d.size = d.upper - d.lower;
    if (d.size.maxCoeff() > c_.at("max_cluster_size") || d.size.norm() < .15)
      continue;
    d.center = (d.lower + d.upper) / 2;
    d.points = sampleCloud(d.points, c_.at("registration_points"));
    out.push_back(std::move(d));
  }
  std::stable_sort(out.begin(), out.end(), [&](const Detection& a, const Detection& b) {
    return (a.center - origin).norm() < (b.center - origin).norm();
  });
  if (out.size() > c_.at("max_objects"))
    out.resize(c_.at("max_objects"));
  return {out, true};
}
void ObjectTracker::update(Track& t, const Detection& d, double stamp) {
  t.detection = d;
  t.last_seen = stamp;
  t.history.push_back({stamp, d.center, d.points, d.size});
  while (stamp - t.history.front().stamp > c_.at("history_seconds"))
    t.history.pop_front();
  std::vector<const Observation*> window;
  for (const auto& h : t.history)
    if (stamp - h.stamp <= c_.at("motion_window") + .11)
      window.push_back(&h);
  if (window.size() < 4 || stamp - window.front()->stamp < c_.at("min_observation_time")) {
    t.state = "UNKNOWN";
    return;
  }
  Cloud slopes;
  for (size_t i = 0; i < window.size(); ++i)
    for (size_t j = i + 1; j < window.size(); ++j) {
      double dt = window[j]->stamp - window[i]->stamp;
      if (dt >= .2)
        slopes.push_back((window[j]->center - window[i]->center) / dt);
    }
  if (slopes.empty()) {
    t.state = "UNKNOWN";
    return;
  }
  t.velocity = quantile(slopes, .5);
  auto& anchor = *window.front();
  auto e = alignmentEvidence(anchor.points, d.points, d.center - anchor.center,
                             c_.at("registration_distance"));
  double shape = (d.size - anchor.size).norm(), speed = t.velocity.norm();
  bool geometry = e["overlap"].asDouble() >= c_.at("registration_overlap") &&
                  e["error"].asDouble() < c_.at("registration_distance") && shape < .65;
  bool moving = geometry && speed >= c_.at("moving_speed") && speed <= c_.at("max_speed") &&
                e["gain"].asDouble() >= c_.at("registration_gain");
  bool quiet = geometry && speed < c_.at("static_speed") &&
               e["static_error"].asDouble() < c_.at("registration_distance");
  e["speed"] = speed;
  e["size_change"] = shape;
  e["geometry_ok"] = geometry;
  e["motion_supported"] = moving;
  if (moving) {
    ++t.moving_votes;
    t.quiet_since = NAN;
    t.last_motion = stamp;
    if (t.moving_votes >= c_.at("moving_confirm"))
      t.state = "MOVING";
  } else {
    t.moving_votes = 0;
    if (quiet) {
      if (std::isnan(t.quiet_since))
        t.quiet_since = stamp;
      if (stamp - t.quiet_since >= c_.at("static_confirm_seconds"))
        t.state = "STATIC";
    } else
      t.quiet_since = NAN;
    if (t.state == "MOVING" && stamp - t.last_motion > c_.at("unknown_after_seconds"))
      t.state = "UNKNOWN";
    if (t.state == "STATIC" && !quiet)
      t.state = "UNKNOWN";
  }
  e["motion_held"] = t.state == "MOVING" && !moving;
  e["motion_evidence_age"] =
      std::isfinite(t.last_motion) ? Json::Value(stamp - t.last_motion) : Json::Value();
  t.evidence = e;
}
Result ObjectTracker::process(const Cloud& p, const Vec& origin, double stamp) {
  auto start = std::chrono::steady_clock::now();
  if (p.size() > c_.at("max_input_points") || !origin.allFinite() || !std::isfinite(stamp))
    throw std::invalid_argument("invalid input size/time/pose");
  Result out;
  if (std::isfinite(previous_stamp_)) {
    double dt = stamp - previous_stamp_;
    if (dt <= 0 || dt > c_.at("max_frame_gap") ||
        (origin - previous_origin_).norm() > std::max(2., 15 * dt)) {
      reset();
      out.reset = true;
    }
  }
  previous_stamp_ = stamp;
  previous_origin_ = origin;
  auto segmented = segment(p, origin, stamp);
  auto& detections = segmented.first;
  out.ground_valid = segmented.second;
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [&](const Track& t) {
                                 return stamp - t.last_seen > c_.at("max_missed_seconds");
                               }),
                tracks_.end());
  std::vector<std::vector<double>> costs(tracks_.size(),
                                         std::vector<double>(detections.size(), 1e6));
  for (size_t i = 0; i < tracks_.size(); ++i) {
    const auto& t = tracks_[i];
    double dt = stamp - t.last_seen;
    Vec predicted = t.detection.center + t.velocity * dt;
    for (size_t j = 0; j < detections.size(); ++j) {
      double distance = (predicted - detections[j].center).norm(),
             shape = (t.detection.size - detections[j].size).norm();
      if (distance < c_.at("association_distance") + .5 * c_.at("max_speed") * dt && shape < .85)
        costs[i][j] = distance + .35 * shape;
    }
  }
  std::vector<bool> matched(detections.size());
  for (auto pair : assign(costs))
    if (costs[pair.first][pair.second] < 1e5) {
      update(tracks_[pair.first], detections[pair.second], stamp);
      matched[pair.second] = true;
    }
  for (size_t j = 0; j < detections.size(); ++j)
    if (!matched[j]) {
      Track t;
      t.id = next_id_++;
      update(t, detections[j], stamp);
      tracks_.push_back(std::move(t));
    }
  out.dynamic.resize(p.size());
  out.uncertain.resize(p.size());
  for (const auto& t : tracks_) {
    bool observed = t.last_seen == stamp;
    if (observed)
      for (auto i : t.detection.indices) {
        if (t.state == "MOVING")
          out.dynamic[i] = 1;
        else if (t.state == "UNKNOWN")
          out.uncertain[i] = 1;
      }
    Json::Value o(Json::objectValue);
    o["id"] = t.id;
    o["state"] = observed ? t.state : "PREDICTED";
    o["previous_state"] = t.state;
    o["observed"] = observed;
    o["center"] = toJson(t.detection.center + (observed ? 0 : stamp - t.last_seen) * t.velocity);
    o["size"] = toJson(t.detection.size);
    o["velocity"] = toJson(t.velocity);
    o["speed"] = t.velocity.norm();
    o["point_count"] = Json::UInt64(observed ? t.detection.indices.size() : 0);
    o["observation_age"] = stamp - t.last_seen;
    o["evidence"] = t.evidence;
    out.objects.append(o);
  }
  out.processing_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  return out;
}
}  // namespace lot
