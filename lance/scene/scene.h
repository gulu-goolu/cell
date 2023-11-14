#pragma once

#include "lance/core/linalg.h"
#include "lance/core/object.h"
#include "lance/rendering/device.h"

namespace lance {
namespace scene {
class Material : public core::Inherit<Material, core::Object> {
 public:
};

class Transform : public core::Inherit<Transform, core::Object> {
 public:
  const core::Matrix4x4& local_to_world() const { return local_to_world_; }

  const core::Matrix4x4& world_to_local() const { return world_to_local_; }

  //
  core::RefCountPtr<Transform> transform(const Transform* t);

 private:
  core::Matrix4x4 local_to_world_;

  core::Matrix4x4 world_to_local_;
};

class Geometry : public core::Inherit<Geometry, core::Object> {
 public:
  // retrieve bounding box of this Geometry
  virtual const core::BoundingBox* bounding_box() const { return nullptr; }
};

class Camera : public core::Inherit<Camera, core::Object> {
 public:
  virtual core::Matrix4x4 projection_matrix() const = 0;
};

class SceneNode : public core::Inherit<SceneNode, core::Object> {
 public:
  core::RefCountPtr<Transform> transform;

  core::RefCountPtr<Geometry> geometry;

  // children nodes
  std::vector<core::RefCountPtr<SceneNode>> children;
};

class Scene : public core::Inherit<Scene, core::Object> {};
}  // namespace scene
}  // namespace lance
