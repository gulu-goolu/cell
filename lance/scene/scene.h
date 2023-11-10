#pragma once

#include "lance/core/linalg.h"
#include "lance/core/object.h"

namespace lance {
namespace scene {
class Material : public core::Inherit<Material, core::Object> {
 public:
};

class Transform : public core::Inherit<Transform, core::Object> {
 public:
  const core::matrix4x4& local_to_world() const { return local_to_world_; }

  const core::matrix4x4& world_to_local() const { return world_to_local_; }

 private:
  core::matrix4x4 local_to_world_;

  core::matrix4x4 world_to_local_;
};

class Geometry : public core::Inherit<Geometry, core::Object> {
 public:
};

class Camera : public core::Inherit<Camera, core::Object> {};

class SceneNode : public core::Inherit<SceneNode, core::Object> {
 public:
  core::RefCountPtr<Transform> transform;

  core::RefCountPtr<Geometry> geometry;
};

}  // namespace scene
}  // namespace lance
