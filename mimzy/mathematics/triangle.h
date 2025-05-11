#ifndef MIMZY_TRIANGLE_H
#define MIMZY_TRIANGLE_H

#include "bounding_box.h"

namespace Mimzy {

class Triangle {
public:
  Triangle(Point3f p0, Point3f p1, Point3f p2) : p0_(p0), p1_(p1), p2_(p2) {}

  BoundingBox GetBoundingBox() const;
  Point3f GetCentroid() const;
  Normal3f GetNormal() const;

  std::optional<Float> Intersect(const Ray &ray) const;

public:
  Point3f p0_;
  Point3f p1_;
  Point3f p2_;
};

} // namespace Mimzy

#endif // MIMZY_TRIANGLE_H