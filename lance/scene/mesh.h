#pragma once

#include "lance/rendering/device.h"
#include "lance/scene/scene.h"

namespace lance {
namespace scene {
class VertexBuffer : public core::Inherit<VertexBuffer, core::Object> {
 public:
 private:
  core::RefCountPtr<rendering::Buffer> buffer_;
};

class IndexBuffer : public core::Inherit<IndexBuffer, core::Object> {};

class Mesh : public core::Inherit<Mesh, Geometry> {
 public:
  static core::RefCountPtr<Mesh> create_box();

 private:
  core::RefCountPtr<VertexBuffer> vertex_buffer_;

  core::RefCountPtr<IndexBuffer> index_buffer_;
};
}  // namespace scene
}  // namespace lance
