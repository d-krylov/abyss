#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include <glm/glm.hpp>
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"
#include "polyscope/surface_mesh.h"
#include <vector>
#include <random>
#include <ranges>
#include <algorithm>
#include <filesystem>

using Vector3f = glm::vec3;
using Vector3u = glm::uvec3;

struct BoundingBox {

  BoundingBox() {
    auto min = std::numeric_limits<float>::lowest();
    auto max = std::numeric_limits<float>::max();
    min_ = Vector3f(max);
    max_ = Vector3f(min);
  }

  BoundingBox(const Vector3f &p1, const Vector3f &p2) {
    min_ = glm::min(p1, p2);
    max_ = glm::max(p1, p2);
  }

  void Expand(const Vector3f &point) {
    min_ = glm::min(min_, point);
    max_ = glm::max(max_, point);
  }

  auto GetVertices() const {
    return std::vector<Vector3f>{
      {min_.x, min_.y, max_.z}, // LEFT DOWN FRONT
      {max_.x, min_.y, max_.z}, // RIGHT DOWN FRONT
      {max_.x, max_.y, max_.z}, // RIGHT UP FRONT
      {min_.x, max_.y, max_.z}, // LEFT UP FRONT
      {min_.x, min_.y, min_.z}, // LEFT DOWN BACK
      {max_.x, min_.y, min_.z}, // RIGHT DOWN BACK
      {max_.x, max_.y, min_.z}, // RIGHT UP BACK
      {min_.x, max_.y, min_.z}, // LEFT UP BACK
    };
  }

  auto GetEdges() const {
    return std::vector<std::array<uint32_t, 2>>{
      {0, 1}, {1, 2}, {2, 3}, {3, 0}, // FRONT
      {4, 5}, {5, 6}, {6, 7}, {7, 4}, // BACK
      {0, 4}, {1, 5}, {2, 6}, {3, 7}  // FRONT-BACK
    };
  }

  auto GetFaces() const {
    return std::vector<std::array<uint32_t, 4>>{
      {0, 1, 2, 3}, // FRONT
      {1, 5, 6, 2}, // RIGHT
      {5, 4, 7, 6}, // BACK
      {4, 0, 3, 7}, // LEFT
      {3, 2, 6, 7}, // TOP
      {4, 5, 1, 0}  // BOTTOM
    };
  }

  auto GetSeperationVertices(uint32_t axis, float position) const {
    std::vector<Vector3f> vertices(4);
    auto u = (axis + 1) % 3;
    auto v = (axis + 2) % 3;
    for (auto i = 0; i < 4; ++i) {
      vertices[i][axis] = position;
      vertices[i][u] = (i & 1) ? max_[u] : min_[u];
      vertices[i][v] = (i & 2) ? max_[v] : min_[v];
    }
    return vertices;
  }

  auto GetSeperationIndices() {
    return std::array<std::array<uint32_t, 4>, 1>{{{0, 1, 3, 2}}};
  }

  Vector3f min_;
  Vector3f max_;
};

struct KDTreeNode {

  KDTreeNode(std::span<uint32_t> indices, std::span<const Vector3f> points) : indices_(indices) {
    for (const auto &index : indices_) {
      bounding_box_.Expand(points[index]);
    }
  }

  auto IsLeaf() const {
    return indices_.size() != 0;
  }

  std::span<uint32_t> indices_;
  BoundingBox bounding_box_;
  uint32_t axis_;
  float position_;
};

struct KDTree {

  KDTree(std::span<const Vector3f> points, uint32_t points_maximum, uint32_t depth_maximim)
    : points_(points), points_maximum_(points_maximum), depth_maximim_(depth_maximim) {
    indices_.resize(points.size());
    nodes_.reserve(2 * points.size());
    std::ranges::iota(indices_, 0);
  }

  void Build() {
    auto &root = nodes_.emplace_back(indices_, points_);
    RecursiveBuild(root, 0);
  }

  void RecursiveBuild(KDTreeNode &node, uint32_t depth) {
    if (node.indices_.size() <= points_maximum_ || depth == depth_maximim_) return;

    auto axis = depth % 3;
    std::ranges::sort(node.indices_, [&](auto a, auto b) { return points_[a][axis] < points_[b][axis]; });

    auto median = node.indices_.size() / 2;
    auto position = points_[node.indices_[median]][axis];

    auto &node_L = nodes_.emplace_back(node.indices_.first(median), points_);
    auto &node_R = nodes_.emplace_back(node.indices_.subspan(median), points_);

    node.axis_ = axis;
    node.position_ = position;
    node.indices_ = {};

    RecursiveBuild(node_L, depth + 1);
    RecursiveBuild(node_R, depth + 1);
  }

  std::span<const Vector3f> points_;
  std::vector<uint32_t> indices_;
  std::vector<KDTreeNode> nodes_;
  uint32_t points_maximum_;
  uint32_t depth_maximim_;
};

auto GeneratePoints(std::size_t count, float min, float max) {
  std::vector<Vector3f> points(count);
  std::random_device random_device;
  std::mt19937 generator(random_device());
  std::uniform_real_distribution<float> distribution(min, max);

  for (auto &point : points) {
    auto x = distribution(generator);
    auto y = distribution(generator);
    auto z = distribution(generator);
    point = Vector3f(x, y, z);
  }

  return points;
}

std::vector<Vector3f> GetMeshVertices(const tinyobj::ObjReader &reader) {
  std::vector<Vector3f> out;
  const auto &attributes = reader.GetAttrib();
  for (auto v = 0; v < attributes.vertices.size(); v += 3) {
    auto x = attributes.vertices[v + 0];
    auto y = attributes.vertices[v + 1];
    auto z = attributes.vertices[v + 2];
    out.emplace_back(x, y, z);
  }
  return out;
}

std::vector<Vector3u> GetMeshFaces(const tinyobj::ObjReader &reader) {
  std::vector<Vector3u> out;
  const auto &shapes = reader.GetShapes();
  for (const auto &shape : shapes) {
    for (auto i = 0; i < shape.mesh.indices.size(); i += 3) {
      auto i0 = shape.mesh.indices[i + 0];
      auto i1 = shape.mesh.indices[i + 1];
      auto i2 = shape.mesh.indices[i + 2];
      out.emplace_back(i0.vertex_index, i1.vertex_index, i2.vertex_index);
    }
  }
  return out;
}

int main(int argc, char **argv) {

  if (argc == 1) return 0;

  std::filesystem::path mesh_path = argv[1];

  tinyobj::ObjReader reader;
  tinyobj::ObjReaderConfig reader_config;

  auto status = reader.ParseFromFile(mesh_path.string(), reader_config);

  auto vertices = GetMeshVertices(reader);
  auto faces = GetMeshFaces(reader);

  std::vector<Vector3f> points;

  for (const auto &face : faces) {
    auto p0 = vertices[face.x];
    auto p1 = vertices[face.y];
    auto p2 = vertices[face.z];
    auto center = (p0 + p1 + p2) / 3.0f;
    points.emplace_back(center);
  }

  KDTree tree(points, 10, 100);

  tree.Build();

  polyscope::init();

  for (const auto &[node_index, node] : std::views::enumerate(tree.nodes_)) {
    auto aabb = node.bounding_box_;
    auto name = "aabb" + std::to_string(node_index);
    if (node.IsLeaf()) {
      auto network = polyscope::registerSurfaceMesh(name, aabb.GetVertices(), aabb.GetFaces());
    }
  }

  polyscope::show();

  return 0;
}