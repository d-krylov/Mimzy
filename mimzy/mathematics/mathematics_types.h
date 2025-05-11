#ifndef MIMZY_MATHEMATICS_TYPES_H
#define MIMZY_MATHEMATICS_TYPES_H

#include "yoth/common/yoth.h"

namespace Mimzy {

using Float = double;

using Vector3f = Yoth::Vector3<Float>;
using Point2f = Yoth::Point2<Float>;
using Point3f = Yoth::Point3<Float>;
using Normal3f = Yoth::Normal3<Float>;

constexpr Float MIMZY_INFINITY = std::numeric_limits<Float>::infinity();
constexpr Float MIMZY_EPSILON = 1e-12;

} // namespace Mimzy

#endif // MIMZY_MATHEMATICS_TYPES_H