#ifndef MIMZY_RAY_H
#define MIMZY_RAY_H

#include "mathematics_types.h"

namespace Mimzy {

struct Hit {
  Point3f position_;
  Normal3f normal_;
  uint32_t index_;
};

class Ray {
public:
  Point3f operator()(Float t) { return origin_ + direction_ * t; }

public:
  Point3f origin_;
  Vector3f direction_;
};

} // namespace Mimzy

#endif // MIMZY_RAY_H