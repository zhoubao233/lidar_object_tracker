#pragma once
#include <algorithm>

#include "lidar_object_tracker/tracking_core.h"
namespace lot::test {
inline Cloud scene(double y = 0, double crop = 0) {
  Cloud p;
  for (int j = 0; j < 35; ++j)
    for (int i = 0; i < 35; ++i)
      p.emplace_back(-3. + 12. * i / 34, -6. + 12. * j / 34, 0);
  for (int j = 0; j < 12; ++j)
    for (int i = 0; i < 12; ++i)
      p.emplace_back(4.5, -.5 + crop + (1 - crop) * i / 11 + y, .17 + .83 * j / 11);
  return p;
}
inline size_t count(const std::vector<uint8_t>& mask) {
  return std::count(mask.begin(), mask.end(), uint8_t(1));
}
}  // namespace lot::test
