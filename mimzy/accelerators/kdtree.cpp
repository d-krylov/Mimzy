#include "kdtree.h"
#include <cassert>
#include <iostream>
#include <ranges>

namespace Mimzy {

enum class EdgeType {
  BEGIN = 0,
  END
};

struct BoundEdge {
  Float position_;
  EdgeType type_;
  uint32_t primitive_index_;
};

KDTree::KDTree(std::span<const Triangle> primitives) : primitives_(primitives.begin(), primitives.end()) {
}

void KDTree::KDTreeNode::InitLeaf(std::span<const uint32_t> primitives_numbers,
                                  std::vector<uint32_t> &primitives_indices) {
  mask_ = 3 | (primitives_numbers.size() << 2);
  if (primitives_numbers.size() == 0) {
    one_primitive_index_ = 0;
  } else if (primitives_numbers.size() == 1) {
    one_primitive_index_ = primitives_numbers[0];
  } else {
    primitives_indices_offset_ = primitives_indices.size();
    primitives_indices.insert(primitives_indices.end(), primitives_numbers.begin(), primitives_numbers.end());
  }
}

void KDTree::KDTreeNode::InitInterior(uint32_t axis, uint32_t above_child, Float split_position) {
  split_position_ = split_position;
  mask_ = axis | (above_child << 2);
}

void KDTree::GetSortedEdges(std::span<const uint32_t> primitives_numbers,
                            std::span<BoundEdge> edges_out,
                            uint32_t axis) {
  for (const auto &[index, primitive_number] : std::views::enumerate(primitives_numbers)) {
    auto primitive = primitives_[primitive_number];
    auto bounding_box = primitive.GetBoundingBox();
    edges_out[2 * index + 0] = BoundEdge(bounding_box.min_[axis], EdgeType::BEGIN, primitive_number);
    edges_out[2 * index + 1] = BoundEdge(bounding_box.max_[axis], EdgeType::END, primitive_number);
  }

  auto predicate = [](const BoundEdge &e0, const BoundEdge &e1) {
    return std::tie(e0.position_, e0.type_) < std::tie(e1.position_, e1.type_);
  };

  std::sort(edges_out.begin(), edges_out.begin() + 2 * primitives_numbers.size(), predicate);
}

void KDTree::Build(uint32_t depth) {
  auto n = primitives_.size();

  std::vector<uint32_t> primitives_numbers(n);
  std::ranges::iota(primitives_numbers, 0);

  for (auto &primitive : primitives_) {
    bounding_box_.Expand(primitive.GetBoundingBox());
  }

  std::array<std::vector<BoundEdge>, 3> edges;
  edges[0].resize(2 * n);
  edges[1].resize(2 * n);
  edges[2].resize(2 * n);

  std::vector<uint32_t> primitives_0(n);
  std::vector<uint32_t> primitives_1((depth + 1) * n);

  BuildRecursive(0, depth, bounding_box_, edges, primitives_numbers, primitives_0, primitives_1, 0);
}

void KDTree::BuildRecursive(uint32_t node_index,
                            uint32_t depth,
                            const BoundingBox &bounding_box,
                            std::array<std::vector<BoundEdge>, 3> &edges,
                            std::span<const uint32_t> primitives_numbers,
                            std::span<uint32_t> primitives_0,
                            std::span<uint32_t> primitives_1,
                            uint32_t bad_refines) {

  if (next_free_node_ == nodes_.size()) {
    auto number_allocated_nodes = std::max(2 * nodes_.size(), 512ul);
    nodes_.resize(number_allocated_nodes);
  }
  ++next_free_node_;

  if (primitives_numbers.size() <= maximum_primitives_ || depth == 0) {
    nodes_[node_index].InitLeaf(primitives_numbers, primitives_indices_);
    return;
  }

  auto best_axis = -1, best_offset = -1;
  auto best_cost = FLOAT_MAX;
  for (auto axis = 0; axis < 3; axis++) {
    GetSortedEdges(primitives_numbers, edges[axis], axis);
    std::tie(best_cost, best_axis, best_offset) =
      FindBestSplitPlane(edges[axis], primitives_numbers.size(), axis, bounding_box);
    if (best_axis != -1) break;
  }

  auto leaf_cost = options_.intersection_cost_ * primitives_numbers.size();

  if (best_cost > leaf_cost) {
    ++bad_refines;
  }

  if ((best_cost > 4 * leaf_cost && primitives_numbers.size() < 16) || best_axis == -1 || bad_refines == 3) {
    nodes_[node_index].InitLeaf(primitives_numbers, primitives_indices_);
    return;
  }

  auto [n0, n1] =
    СlassifyPrimitives(edges[best_axis], best_offset, primitives_numbers.size(), primitives_0, primitives_1);

  auto split_position = edges[best_axis][best_offset].position_;

  auto bounding_box_0 = bounding_box;
  auto bounding_box_1 = bounding_box;

  bounding_box_0.max_[best_axis] = split_position;
  bounding_box_1.min_[best_axis] = split_position;

  BuildRecursive(node_index + 1, depth - 1, bounding_box_0, edges, primitives_0.subspan(0, n0), primitives_0,
                 primitives_1.subspan(n1), bad_refines);

  nodes_[node_index].InitInterior(best_axis, next_free_node_, split_position);

  BuildRecursive(next_free_node_, depth - 1, bounding_box_1, edges, primitives_1.subspan(0, n1), primitives_0,
                 primitives_1.subspan(n1), bad_refines);
}

std::pair<uint32_t, uint32_t> KDTree::СlassifyPrimitives(std::span<const BoundEdge> edges,
                                                         uint32_t best_offset,
                                                         uint32_t primitives_numbers_size,
                                                         std::span<uint32_t> primitives_0,
                                                         std::span<uint32_t> primitives_1) {
  auto n0 = 0, n1 = 0;

  for (auto i = 0; i < best_offset; ++i) {
    if (edges[i].type_ == EdgeType::BEGIN) primitives_0[n0++] = edges[i].primitive_index_;
  }

  for (auto i = best_offset + 1; i < 2 * primitives_numbers_size; ++i) {
    if (edges[i].type_ == EdgeType::END) primitives_1[n1++] = edges[i].primitive_index_;
  }

  return std::make_pair(n0, n1);
}

std::tuple<Float, int32_t, int32_t> KDTree::FindBestSplitPlane(std::span<const BoundEdge> edges,
                                                               uint32_t primitives_numbers_size,
                                                               uint32_t axis,
                                                               const BoundingBox &node_bounds) {
  auto best_axis = -1, best_offset = -1;
  auto best_cost = FLOAT_MAX;
  auto number_below = 0;
  auto number_above = primitives_numbers_size;
  for (auto i = 0; i < 2 * primitives_numbers_size; ++i) {
    if (edges[i].type_ == EdgeType::END) number_above--;

    auto edge_position = edges[i].position_;

    if (edge_position > node_bounds.min_[axis] && edge_position < node_bounds.max_[axis]) {
      auto [below_area, above_area] = node_bounds.GetSurfaceArea(edge_position, axis);

      // std::cout << below_area << " " << above_area << std::endl;
      auto cost = options_.traversal_cost_ + options_.intersection_cost_ *
                                               (below_area * number_below + above_area * number_above) /
                                               node_bounds.GetSurfaceArea();

      if (cost < best_cost) {
        best_cost = cost;
        best_axis = axis;
        best_offset = i;
      }
    }
    if (edges[i].type_ == EdgeType::BEGIN) number_below++;
  }

  return {best_cost, best_axis, best_offset};
}

std::optional<Hit> KDTree::Intersect(const Ray &ray) {
  auto [t_min, t_max] = bounding_box_.Intersect(ray);

  if (t_min == FLOAT_INFINITY) return std::nullopt;

  auto time = FLOAT_INFINITY;
  auto index = 0;
  std::array<KDNodeToVisit, 64> to_visit_nodes;
  auto to_visit_index = 0;
  auto nodes_visited = 0;

  const auto *current_node = &nodes_[0];

  auto invert_direction = Vector3f(1.0f) / ray.direction_;

  while (current_node) {

    ++nodes_visited;

    if (current_node->IsLeaf() == false) {

      auto axis = current_node->SplitAxis();
      auto t_split = (current_node->SplitPosition() - ray.origin_[axis]) * invert_direction[axis];

      auto below_first = (ray.origin_[axis] < current_node->SplitPosition()) ||
                         (ray.origin_[axis] == current_node->SplitPosition() && ray.direction_[axis] <= 0);

      auto first_child = below_first ? current_node + 1 : &nodes_[current_node->AboveChild()];
      auto second_child = below_first ? &nodes_[current_node->AboveChild()] : current_node + 1;

      if (t_split > t_max || t_split <= 0) {
        current_node = first_child;
      } else if (t_split < t_min) {
        current_node = second_child;
      } else {
        to_visit_nodes[to_visit_index++] = {second_child, t_split, t_max};
        current_node = first_child;
        t_max = t_split;
      }
    } else {
      auto number_primitives = current_node->NumberPrimitives();
      if (number_primitives == 1) {
        const auto &primitive = primitives_[current_node->one_primitive_index_];
        auto t = primitive.Intersect(ray);
        if (t.has_value() && t.value() < time) {
          time = std::min(time, t.value());
          index = current_node->one_primitive_index_;
        }
      } else {
        for (auto i = 0; i < number_primitives; ++i) {
          auto primitive_index = primitives_indices_[current_node->primitives_indices_offset_ + i];
          const auto &primitive = primitives_[primitive_index];
          auto t = primitive.Intersect(ray);
          if (t.has_value() && t.value() < time) {
            time = std::min(time, t.value());
            index = primitive_index;
          }
        }
      }

      if (to_visit_index > 0) {
        --to_visit_index;
        current_node = to_visit_nodes[to_visit_index].node;
        t_min = to_visit_nodes[to_visit_index].t_min;
        t_max = to_visit_nodes[to_visit_index].t_max;
      } else {
        break;
      }
    }
  }
  if (time == FLOAT_INFINITY) return std::nullopt;

  auto &triangle = primitives_[index];

  return Hit(ray(time), triangle.GetNormal());
}

} // namespace Mimzy