#include "triangle.h"

namespace Mimzy {

Triangle::Triangle(Point3f p0, Point3f p1, Point3f p2) : p0_(p0), p1_(p1), p2_(p2) {
}

Point3f Triangle::GetCentroid() const {
  return (p0_ + p1_ + p2_) / 3;
}

BoundingBox Triangle::GetBoundingBox() const {
  auto min = Min(p0_, Min(p1_, p2_));
  auto max = Max(p0_, Max(p1_, p2_));
  return BoundingBox(min, max);
}

Normal3f Triangle::GetNormal() const {
  auto normal = Yoth::Cross(p1_ - p0_, p2_ - p0_);
  normal = Yoth::Normalize(normal);
  return Normal3f(normal.x, normal.y, normal.z);
}

std::optional<Float> Triangle::Intersect(const Ray &ray) const {
  auto edge1 = p1_ - p0_;
  auto edge2 = p2_ - p0_;
  auto h = Cross(ray.direction_, edge2);
  auto a = Dot(edge1, h);
  if (a > -MIMZY_EPSILON && a < MIMZY_EPSILON) return std::nullopt;
  auto f = 1 / a;
  auto s = ray.origin_ - p0_;
  auto u = f * Dot(s, h);
  if (u < 0 || u > 1) return std::nullopt;
  auto q = Cross(s, edge1);
  auto v = f * Dot(ray.direction_, q);
  if (v < 0 || u + v > 1) return std::nullopt;
  return f * Dot(edge2, q);
}

} // namespace Mimzy