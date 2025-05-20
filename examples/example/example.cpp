#define TINYOBJLOADER_IMPLEMENTATION
#include "mimzy/common/mimzy.h"
#include "tiny_obj_loader.h"
#include "tools/tools.h"
#include <filesystem>
#include <fstream>
#include <iostream>

struct Vertex {
  Mimzy::Point3f position_;
  Mimzy::Vector3f normal_;
};

std::vector<Vertex> LoadWavefront(std::span<const tinyobj::shape_t> shapes, const tinyobj::attrib_t &attributes) {
  std::vector<Vertex> vertices;
  for (const auto &shape : shapes) {
    for (const auto &index : shape.mesh.indices) {
      auto &new_vertex = vertices.emplace_back();
      new_vertex.position_.x = attributes.vertices[3 * index.vertex_index + 0];
      new_vertex.position_.y = attributes.vertices[3 * index.vertex_index + 1];
      new_vertex.position_.z = attributes.vertices[3 * index.vertex_index + 2];
      if (attributes.normals.empty() == false) {
        new_vertex.normal_.x = attributes.normals[3 * index.normal_index + 0];
        new_vertex.normal_.y = attributes.normals[3 * index.normal_index + 1];
        new_vertex.normal_.z = attributes.normals[3 * index.normal_index + 2];
      }
    }
  }
  return vertices;
}

std::vector<Vertex> LoadWavefront(const std::filesystem::path &path) {
  tinyobj::ObjReader reader;
  tinyobj::ObjReaderConfig reader_config;
  auto status = reader.ParseFromFile(path.string(), reader_config);
  auto &shapes = reader.GetShapes();
  auto &attributes = reader.GetAttrib();
  auto vertices = LoadWavefront(shapes, attributes);
  return vertices;
}

void WriteImage(uint32_t width, uint32_t height, std::span<const uint32_t> data) {
  std::ofstream out("output.ppm");
  out << "P3\n" << width << " " << height << "\n255\n";
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x) {
      auto color = data[y * width + x];
      int r = (color >> 24) & 0xff;
      int g = (color >> 16) & 0xff;
      int b = (color >> 8) & 0xff;
      out << r << " " << g << " " << b << "\n";
    }
}

int main(int argc, char **argv) {

  if (argc == 1) return 0;

  std::filesystem::path model_path = argv[1];

  auto vertces = LoadWavefront(model_path);

  std::vector<Mimzy::Triangle> triangles;

  for (auto i = 0; i < vertces.size(); i += 3) {
    auto p0 = vertces[i + 0].position_;
    auto p1 = vertces[i + 1].position_;
    auto p2 = vertces[i + 2].position_;
    triangles.emplace_back(p0, p1, p2);
  }

  // Mimzy::BVH bvh(triangles);

  // bvh.Build();

  Mimzy::KDTree kdtree(triangles);

  kdtree.Build(8);

  auto width = 800;
  auto height = 600;

  std::vector<uint32_t> image(width * height);

  Mimzy::Ray ray;
  ray.origin_ = Mimzy::Point3f(0.0f, 0.0f, 9.0f);

  for (auto x = 0; x < width; x++) {
    for (auto y = 0; y < height; y++) {
      Mimzy::Point2f position(x, y);
      Mimzy::Point2f resolution(width, height);
      auto uv = 2.0f * position / resolution - Mimzy::Point2f(1.0f);
      ray.direction_ = Yoth::Normalize(Mimzy::Vector3f(uv.x, -uv.y, -1.0));
      auto b = kdtree.Intersect(ray);
      if (b.has_value()) {

        auto cos = Dot(b.value().normal_, Yoth::Normalize(Mimzy::Vector3f(10.0, 1.0, 10.0)));
        auto color = Mimzy::Vector3f(0.2) + std::max(cos, 0.0) * Mimzy::Vector3f(0.5);

        uint8_t r = static_cast<uint8_t>(color.x * 255.0);
        uint8_t g = static_cast<uint8_t>(color.y * 255.0);
        uint8_t b = static_cast<uint8_t>(color.z * 255.0);

        image[y * width + x] = (r << 24) | (g << 16) | (b << 8) | 0xff;
      }
    }
  }

  Renderer renderer(800, 600);

  while (renderer.ShouldClose() == false) {
    renderer.PollEvent();

    renderer.Clear();
    renderer.DrawImage(image);
    renderer.Present();
  }

  return 0;
}