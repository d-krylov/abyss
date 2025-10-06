#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include "polyscope/curve_network.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include <filesystem>
#include <glm/glm.hpp>
#include <ranges>
#include <span>
#include <vector>

using Vector3f = glm::vec3;
using Face = std::array<int32_t, 3>;

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

  void Expand(const BoundingBox &other) {
    min_ = glm::min(min_, other.min_);
    max_ = glm::max(max_, other.max_);
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

  auto GetExtent() const {
    return max_ - min_;
  }

  uint32_t MaximumExtent() const {
    auto extent = GetExtent();
    if (extent.x > extent.y && extent.x > extent.z) return 0;
    if (extent.y > extent.z) return 1;
    return 2;
  }

  Vector3f min_;
  Vector3f max_;
};

struct Triangle {

  auto GetBoundingBox() const {
    auto min = glm::min(p0_, glm::min(p1_, p2_));
    auto max = glm::max(p0_, glm::max(p1_, p2_));
    return BoundingBox(min, max);
  }

  auto GetCenter() const {
    return (p0_ + p1_ + p2_) / 3.0f;
  }

  Vector3f p0_;
  Vector3f p1_;
  Vector3f p2_;
};

struct BVHNode {

  BVHNode(std::span<Face> primitives, std::span<const Vector3f> vertices) : primitives_(primitives) {
    for (const auto &primitive : primitives_) {
      auto p0 = vertices[primitive[0]];
      auto p1 = vertices[primitive[1]];
      auto p2 = vertices[primitive[2]];
      bounding_box_.Expand(Triangle(p0, p1, p2).GetBoundingBox());
    }
  }

  auto IsLeaf() const {
    return primitives_.size() != 0;
  }

  std::span<Face> primitives_;
  BoundingBox bounding_box_;
};

struct BVH {

  BVH(std::span<const Vector3f> vertices, std::span<const Face> faces, uint32_t primitives_maximum)
    : vertices_(vertices), faces_(faces.begin(), faces.end()), primitives_maximum_(primitives_maximum) {
    nodes_.reserve(2 * faces_.size() - 1); // To prevent references from invalidation when emplace_back occurs
  }

  Triangle GetTriangle(const Face &face) const {
    auto p0 = vertices_[face[0]];
    auto p1 = vertices_[face[1]];
    auto p2 = vertices_[face[2]];
    return Triangle(p0, p1, p2);
  }

  void Build() {
    auto &root = nodes_.emplace_back(faces_, vertices_);
    RecursiveBuild(root);
  }

  void RecursiveBuild(BVHNode &node) {
    if (node.primitives_.size() <= primitives_maximum_) return;

    auto extent = node.bounding_box_.GetExtent();
    auto max_axis = node.bounding_box_.MaximumExtent();
    auto position = node.bounding_box_.min_[max_axis] + extent[max_axis] * 0.5f; // naive split position

    auto middle = std::ranges::partition(node.primitives_, [&](auto index) { return GetTriangle(index).GetCenter()[max_axis] < position; });
    auto offset = std::distance(node.primitives_.begin(), middle.begin());

    if (offset == 0 || offset == node.primitives_.size()) return;

    auto &node_L = nodes_.emplace_back(node.primitives_.first(offset), vertices_);
    auto &node_R = nodes_.emplace_back(node.primitives_.subspan(offset), vertices_);

    node.primitives_ = {};

    RecursiveBuild(node_L);
    RecursiveBuild(node_R);
  }

  std::vector<BVHNode> nodes_;
  std::vector<Face> faces_;
  std::span<const Vector3f> vertices_;
  uint32_t primitives_maximum_{1};
};

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

std::vector<Face> GetMeshFaces(const tinyobj::ObjReader &reader) {
  std::vector<Face> out;
  const auto &shapes = reader.GetShapes();
  for (const auto &shape : shapes) {
    for (auto i = 0; i < shape.mesh.indices.size(); i += 3) {
      auto i0 = shape.mesh.indices[i + 0];
      auto i1 = shape.mesh.indices[i + 1];
      auto i2 = shape.mesh.indices[i + 2];
      out.emplace_back(Face{i0.vertex_index, i1.vertex_index, i2.vertex_index});
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

  BVH bvh(vertices, faces, 20);

  bvh.Build();

  polyscope::init();

  polyscope::registerSurfaceMesh("mesh", vertices, faces);

  for (const auto &[node_index, node] : std::views::enumerate(bvh.nodes_)) {
    if (node.IsLeaf()) {
      auto aabb = node.bounding_box_;
      auto name = "aabb" + std::to_string(node_index);
      auto network = polyscope::registerCurveNetwork(name, aabb.GetVertices(), aabb.GetEdges());
      network->setRadius(0.002f, true);
    }
  }

  polyscope::show();
}