#pragma once

#include "lance/scene/scene.h"

namespace lance {
namespace scene {
class VertexBuffer : public core::Inherit<VertexBuffer, core::Object> {};

class IndexBuffer : public core::Inherit<IndexBuffer, core::Object> {};

class Mesh : public core::Inherit<Mesh, Geometry> {
 public:
  core::RefCountPtr<VertexBuffer> vertex_buffer_;
};
}  // namespace scene
}  // namespace lance
