#ifndef MIMZY_BVH_H
#define MIMZY_BVH_H

#include "mimzy/mathematics/triangle.h"
#include <span>
#include <vector>

namespace Mimzy {

enum class SplitMethod {
  SAH,
  HLBVH,
  MIDDLE
};

class BVH {
public:
  BVH(std::span<const Triangle> triangles);

  void Build();

  void RecursiveBuild(uint32_t node_index);

  std::optional<Hit> Intersect(const Ray &ray);

protected:
  struct alignas(32) BVHNode {
    auto IsLeaf() const {
      return primitive_count_ > 0;
    }
    BoundingBox bounding_box_;
    uint32_t primitive_count_{0};
    uint32_t primitive_start_{0}; // Or left node index if it's not leaf
  };

  struct Bin {
    BoundingBox bounding_box_;
    uint32_t primitive_count_{0};
  };

  void UpdateNodeBounds(uint32_t node_index);
  Float FindBestSplitPlane(BVHNode &node, int &axis, Float &split_position);
  BoundingBox GetBounds(const BVHNode &node) const;
  std::vector<Bin> GetBins(const BVHNode &node, const Yoth::Vector3i bin_indices) const;

private:
  std::vector<Triangle> triangles_;
  std::vector<uint32_t> triangles_indices_;
  std::vector<BVHNode> nodes_;
  uint32_t nodes_used_{1};
  uint32_t bin_count_{8};
};

} // namespace Mimzy

#endif // MIMZY_BVH_H