#ifndef MIMZY_BOUNDING_BOX_H
#define MIMZY_BOUNDING_BOX_H

#include "ray.h"
#include <optional>
#include <vector>

namespace Mimzy {

class BoundingBox {
public:
  BoundingBox();
  BoundingBox(const Point3f &p1, const Point3f &p2);

  void Expand(const Point3f &p);
  void Expand(const BoundingBox &other);

  Vector3f GetExtent() const;
  uint32_t MaximumExtent() const;
  Float GetSurfaceArea() const;
  Float GetVolume() const;

  std::pair<Float, Float> Intersect(const Ray &ray) const;
  std::pair<Float, Float> GetSurfaceArea(Float offset, uint32_t axis) const;

  std::vector<Point3f> GetBox() const;

public:
  Point3f min_;
  Point3f max_;
};

} // namespace Mimzy

#endif // MIMZY_BOUNDING_BOX_H