#ifndef MIMZY_KD_TREE_H
#define MIMZY_KD_TREE_H

#include "mimzy/mathematics/triangle.h"
#include <span>
#include <vector>

namespace Mimzy {

struct BoundEdge;

struct KDOptions {
  uint32_t intersection_cost_{5};
  uint32_t traversal_cost_{1};
  uint32_t maximum_depth_{8};
};

class KDTree {
public:
  KDTree(std::span<const Triangle> primitives);

  void Build(uint32_t depth);

  std::optional<Hit> Intersect(const Ray &ray);

protected:
  // clang-format off
  struct KDTreeNode {
    void InitLeaf(std::span<const uint32_t> primitives_numbers, std::vector<uint32_t> &primitives_indices);
    void InitInterior(uint32_t axis, uint32_t above_child, Float split_position);
    auto IsLeaf() const { return (mask_ & 3) == 3; }
    auto SplitAxis() const { return mask_ & 3; }
    auto SplitPosition() const { return split_position_; }
    auto NumberPrimitives() const { return mask_ >> 2; }
    auto AboveChild() const { return mask_ >> 2; }

    union {
      Float split_position_;
      uint32_t one_primitive_index_;
      uint32_t primitives_indices_offset_;
    };

  private:
    uint32_t mask_;
  };
  // clang-format on

  struct KDNodeToVisit {
    const KDTreeNode *node;
    Float t_min;
    Float t_max;
  };

protected:
  void GetSortedEdges(std::span<const uint32_t> primitives_indices, std::span<BoundEdge> edges_out, uint32_t axis);

  void BuildRecursive(uint32_t node_index,
                      uint32_t depth,
                      const BoundingBox &bounding_box,
                      std::array<std::vector<BoundEdge>, 3> &edges,
                      std::span<const uint32_t> primitive_indices,
                      std::span<uint32_t> primitives_0,
                      std::span<uint32_t> primitives_1,
                      uint32_t bad_refines);

  std::tuple<Float, int32_t, int32_t> FindBestSplitPlane(std::span<const BoundEdge> edges,
                                                         uint32_t primitives_numbers_size,
                                                         uint32_t axis,
                                                         const BoundingBox &node_bounds);

  std::pair<uint32_t, uint32_t> СlassifyPrimitives(std::span<const BoundEdge> edges,
                                                   uint32_t best_offset,
                                                   uint32_t primitives_numbers_size,
                                                   std::span<uint32_t> primitives_0,
                                                   std::span<uint32_t> primitives_1);

private:
  std::vector<Triangle> primitives_;
  std::vector<uint32_t> primitives_indices_;
  std::vector<KDTreeNode> nodes_;
  BoundingBox bounding_box_;
  KDOptions options_;
  uint32_t next_free_node_{0};
  uint32_t maximum_primitives_{1};
};

} // namespace Mimzy

#endif // MIMZY_KD_TREE_H