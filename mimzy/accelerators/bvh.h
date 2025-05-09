#ifndef MIMZY_BVH_H
#define MIMZY_BVH_H

#include "mimzy/mathematics/triangle.h"
#include <span>
#include <vector>

namespace Mimzy {

enum class SplitMethod { SAH, HLBVH, MIDDLE };

class BVH {
public:
  BVH(std::span<const Triangle> triangles);

  void Build();

  void RecursiveBuild(uint32_t node_index);

  bool Intersect(Ray &ray);

  void Print();

protected:
  struct BVHNode {
    bool IsLeaf() const { return primitive_count_ > 0; }
    BoundingBox bounding_box_;
    uint32_t primitive_count_{0};
    union {
      uint32_t left_node_index_{0};
      uint32_t primitive_start_;
    };
  };

  struct Bin {
    BoundingBox bounding_box_;
    uint32_t primitive_count_{0};
  };

  bool Intersect(Ray &ray, uint32_t node_index);
  void UpdateNodeBounds(uint32_t node_index);
  float FindBestSplitPlane(BVHNode &node, int &axis, FloatType &split_position);

private:
  std::vector<Triangle> triangles_;
  std::vector<uint32_t> triangles_indices_;
  std::vector<BVHNode> nodes_;
  uint32_t nodes_used_{1};
  uint32_t bin_count_{8};
};

} // namespace Mimzy

#endif // MIMZY_BVH_H