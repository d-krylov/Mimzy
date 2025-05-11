#include "bounding_box.h"
#include <algorithm>
#include <limits>

namespace Mimzy {

BoundingBox::BoundingBox() {
  auto min = std::numeric_limits<Float>::lowest();
  auto max = std::numeric_limits<Float>::max();
  min_ = Point3f(max);
  max_ = Point3f(min);
}

BoundingBox::BoundingBox(const Point3f &p1, const Point3f &p2) {
  min_ = Yoth::Min(p1, p2);
  max_ = Yoth::Max(p1, p2);
}

void BoundingBox::Expand(const Point3f &p) {
  min_ = Yoth::Min(min_, p);
  max_ = Yoth::Max(max_, p);
}

void BoundingBox::Expand(const BoundingBox &other) {
  min_ = Yoth::Min(min_, other.min_);
  max_ = Yoth::Max(max_, other.max_);
}

Float BoundingBox::GetSurfaceArea() const {
  auto extent = GetExtent();
  return 2 * (extent.x * extent.y + extent.x * extent.z + extent.y * extent.z);
}

Vector3f BoundingBox::GetExtent() const { return max_ - min_; }

uint32_t BoundingBox::MaximumExtent() const {
  auto extent = GetExtent();
  if (extent.x > extent.y && extent.x > extent.z) return 0;
  if (extent.y > extent.z) return 1;
  return 2;
}

Float BoundingBox::GetVolume() const {
  auto extent = GetExtent();
  return extent.x * extent.y * extent.z;
}

Float BoundingBox::Intersect(const Ray &ray) const {
  auto t0 = (min_ - ray.origin_) / ray.direction_;
  auto t1 = (max_ - ray.origin_) / ray.direction_;
  auto min_vec = Min(t0, t1);
  auto max_vec = Max(t0, t1);

  auto t_min = std::max({min_vec.x, min_vec.y, min_vec.z});
  auto t_max = std::min({max_vec.x, max_vec.y, max_vec.z});

  return (t_max >= t_min && t_max > 0) ? t_min : MIMZY_INFINITY;
}

// clang-format off
std::vector<Point3f> GetBoxVertices(const Point3f &min, const Point3f max) {
  return std::vector<Point3f>{
    {min.x, min.y, max.z}, {max.x, min.y, max.z}, {max.x, max.y, max.z}, {min.x, max.y, max.z}, 
    {min.x, min.y, min.z}, {max.x, min.y, min.z}, {max.x, max.y, min.z}, {min.x, max.y, min.z}};
}
// clang-format on

std::vector<Point3f> BoundingBox::GetBox() const {}

} // namespace Mimzy