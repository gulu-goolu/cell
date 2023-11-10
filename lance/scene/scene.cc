#include "scene.h"

namespace lance {
namespace scene {
core::RefCountPtr<Transform> Transform::transform(const Transform* t) {
  auto result = core::make_refcounted<Transform>();

  result->local_to_world_ = local_to_world_ * t->local_to_world_;
  result->world_to_local_ = world_to_local_ * t->world_to_local_;

  return result;
}
}  // namespace scene
}  // namespace lance
