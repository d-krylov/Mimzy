#include "bvh.h"
#include "mimzy/core/stack.h"
#include <algorithm>
#include <iostream>
#include <numeric>

namespace Mimzy {

constexpr auto ROOT_INDEX = 0;

BVH::BVH(std::span<const Triangle> triangles) : triangles_(triangles.begin(), triangles.end()) {
  auto n = triangles_.size();
  triangles_indices_.resize(n);
  nodes_.resize(2 * n - 1);
  std::ranges::iota(triangles_indices_, 0);
}

void BVH::Build() {
  auto &root = nodes_[ROOT_INDEX];
  root.primitive_start_ = ROOT_INDEX;
  root.primitive_count_ = triangles_.size();
  UpdateNodeBounds(ROOT_INDEX);
  RecursiveBuild(ROOT_INDEX);
}

void BVH::UpdateNodeBounds(uint32_t node_index) {
  auto &node = nodes_[node_index];
  for (auto i = 0; i < node.primitive_count_; i++) {
    auto triangle_index = triangles_indices_[node.primitive_start_ + i];
    auto &triangle = triangles_[triangle_index];
    node.bounding_box_.Expand(triangle.GetBoundingBox());
  }
}

void BVH::RecursiveBuild(uint32_t node_index) {
  auto &node = nodes_[node_index];
  if (node.primitive_count_ <= 10) return;
  auto extent = node.bounding_box_.GetExtent();
  int32_t max_axis;
  Float split_position;
  FindBestSplitPlane(node, max_axis, split_position);
  // auto max_axis = node.bounding_box_.MaximumExtent();
  // auto split_position = node.bounding_box_.min_[max_axis] + extent[max_axis] * 0.5f;
  auto begin = triangles_indices_.begin() + node.primitive_start_;
  auto end = begin + node.primitive_count_;
  auto middle = std::partition(begin, end, [&](auto index) {
    auto centroid = triangles_[index].GetCentroid();
    return centroid[max_axis] < split_position;
  });
  auto left_count = std::distance(begin, middle);
  if (left_count == 0 || left_count == node.primitive_count_) return;

  auto left_child_index = nodes_used_++;
  auto right_child_index = nodes_used_++;

  nodes_[left_child_index].primitive_start_ = node.primitive_start_;
  nodes_[left_child_index].primitive_count_ = left_count;
  nodes_[right_child_index].primitive_start_ = std::distance(triangles_indices_.begin(), middle);
  nodes_[right_child_index].primitive_count_ = node.primitive_count_ - left_count;

  node.primitive_start_ = left_child_index;
  node.primitive_count_ = 0;

  UpdateNodeBounds(left_child_index);
  UpdateNodeBounds(right_child_index);

  RecursiveBuild(left_child_index);
  RecursiveBuild(right_child_index);
}

BoundingBox BVH::GetBounds(const BVHNode &node) const {
  BoundingBox bounding_box;
  for (auto i = 0; i < node.primitive_count_; i++) {
    auto triangle_index = triangles_indices_[node.primitive_start_ + i];
    auto centroid = triangles_[triangle_index].GetCentroid();
    bounding_box.Expand(centroid);
  }
  return bounding_box;
}

std::vector<BVH::Bin> BVH::GetBins(const BVHNode &node, const Yoth::Vector3i bounds_min) const {
}

Float BVH::FindBestSplitPlane(BVHNode &node, int &axis, Float &split_position) {
  auto best_cost = FLOAT_MAX;

  auto bounds = GetBounds(node);

  for (auto a = 0; a < 3; a++) {
    auto bounds_min = bounds.min_[a];
    auto bounds_max = bounds.max_[a];

    if (bounds_min == bounds_max) continue;

    std::vector<Bin> bins(bin_count_);
    auto scale = bin_count_ / (bounds_max - bounds_min);

    for (auto i = 0; i < node.primitive_count_; i++) {
      auto triangle_index = triangles_indices_[node.primitive_start_ + i];
      auto &triangle = triangles_[triangle_index];
      auto centroid = triangle.GetCentroid();
      auto bin_index = std::min(bin_count_ - 1, (uint32_t)((centroid[a] - bounds_min) * scale));

      bins[bin_index].primitive_count_++;
      bins[bin_index].bounding_box_.Expand(triangle.GetBoundingBox());
    }

    std::vector<Float> left_area(bin_count_ - 1);
    std::vector<Float> right_area(bin_count_ - 1);
    std::vector<uint32_t> left_count(bin_count_ - 1);
    std::vector<uint32_t> right_count(bin_count_ - 1);

    BoundingBox left_box;
    BoundingBox right_box;

    auto left_sum = 0, right_sum = 0;

    for (auto i = 0; i < bin_count_ - 1; i++) {
      left_sum += bins[i].primitive_count_;
      left_count[i] = left_sum;
      left_box.Expand(bins[i].bounding_box_);
      left_area[i] = left_box.GetSurfaceArea();

      right_sum += bins[bin_count_ - 1 - i].primitive_count_;
      right_count[bin_count_ - 2 - i] = right_sum;
      right_box.Expand(bins[bin_count_ - 1 - i].bounding_box_);
      right_area[bin_count_ - 2 - i] = right_box.GetSurfaceArea();
    }

    scale = (bounds_max - bounds_min) / bin_count_;
    for (auto i = 0; i < bin_count_ - 1; i++) {
      auto plane_cost = left_count[i] * left_area[i] + right_count[i] * right_area[i];
      if (plane_cost < best_cost) {
        axis = a;
        split_position = bounds_min + scale * (i + 1);
        best_cost = plane_cost;
      }
    }
  }
  return best_cost;
}

std::optional<Hit> BVH::Intersect(const Ray &ray) {
  Stack<BVHNode *, 64> stack;
  auto time = FLOAT_INFINITY;
  auto index = 0;
  auto node = &nodes_[ROOT_INDEX];
  while (true) {
    if (node->IsLeaf()) {
      for (auto i = 0; i < node->primitive_count_; i++) {
        const auto &triangle = triangles_[triangles_indices_[node->primitive_start_ + i]];
        auto t = triangle.Intersect(ray);
        if (t.has_value() && t.value() < time) {
          time = std::min(time, t.value());
          index = node->primitive_start_ + i;
        }
      }
      if (stack.Empty()) break;
      node = stack.Pop();
      continue;
    }
    auto c1 = &nodes_[node->primitive_start_ + 0];
    auto c2 = &nodes_[node->primitive_start_ + 1];

    auto d1 = c1->bounding_box_.Intersect(ray);
    auto d2 = c2->bounding_box_.Intersect(ray);

    if (d1 > d2) {
      std::swap(d1, d2);
      std::swap(c1, c2);
    }

    if (d1.first == FLOAT_INFINITY) {
      if (stack.Empty()) break;
      node = stack.Pop();
    } else {
      node = c1;
      if (d2.first != FLOAT_INFINITY) {
        stack.Push(c2);
      }
    }
  }

  if (time == FLOAT_INFINITY) return std::nullopt;

  auto &triangle = triangles_[triangles_indices_[index]];

  return Hit(ray(time), triangle.GetNormal());
}

} // namespace Mimzy
