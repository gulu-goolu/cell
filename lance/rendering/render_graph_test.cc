#include "render_graph.h"

#include "device.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "shader_compiler.h"
#include "util.h"

namespace lance {
namespace rendering {
namespace {
core::RefCountPtr<Device>& test_device() {
  static auto instance = Instance::create_for_3d().value();

  static auto device = instance->create_device_for_graphics().value();
  return device;
}

}  // namespace
TEST(render_graph, compute) {
  auto& device = test_device();

  uint32_t compute_queue_family_index =
      device->find_queue_family_index(VK_QUEUE_COMPUTE_BIT).value();
  LOG(INFO) << "compute_queue_family_index: " << compute_queue_family_index;

  auto command_pool = CommandPool::create(device, compute_queue_family_index).value();

  VkDescriptorSetLayoutBinding bindings = {};
  bindings.binding = 0;
  bindings.descriptorCount = 1;
  bindings.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
  bindings.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  auto descriptor_set_layout = DescriptorSetLayout::create(device, {bindings}).value();

  auto rg = create_render_graph(device).value();

  const char* add_shader = R"glsl(
#version 450 core

layout(local_size_x=512, local_size_y=1, local_size_z=1) in;

void main() {

}
)glsl";

  auto blob = compile_glsl_shader(add_shader, glslang_stage_t::GLSLANG_STAGE_COMPUTE).value();
  auto shader_module = device->create_shader_module(blob.get()).value();

  auto st_v1 = rg->add_compute_pass(
      "Compute",
      [&shader_module, &descriptor_set_layout](ComputePassBuilder* builder) -> absl::Status {
        // LANCE_RETURN_IF_FAILED(builder->set_descriptor_set_layout(0, descriptor_set_layout));

        LANCE_RETURN_IF_FAILED(builder->set_compute_shader(shader_module));

        return absl::OkStatus();
      },
      [=](Context* ctx) -> absl::Status {
        // VkApi::get()->vkCmdBindDescriptorSets(ctx->vk_command_buffer(),
        //                                       VK_PIPELINE_BIND_POINT_COMPUTE,
        //                                       ctx->vk_pipeline_layout(), 0, 0, nullptr, 0,
        //                                       nullptr);

        ctx->dispatch(1024, 1024, 1024);

        return absl::OkStatus();
      });
  ASSERT_TRUE(st_v1.ok());

  LANCE_THROW_IF_FAILED(rg->compile());

  auto command_buffer =
      command_pool->allocate_command_buffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY).value();

  LANCE_THROW_IF_FAILED(command_buffer->begin());

  LANCE_THROW_IF_FAILED(rg->execute(command_buffer->vk_command_buffer(), {}));

  LANCE_THROW_IF_FAILED(command_buffer->end());

  render_doc_begin_capture();

  auto st_v4 = device->submit(compute_queue_family_index, {command_buffer->vk_command_buffer()});
  ASSERT_TRUE(st_v4.ok());

  render_doc_end_capture();
}

TEST(render_graph, graphics) {
  const uint32_t graphics_queue_family_index =
      test_device()->find_queue_family_index(VK_QUEUE_GRAPHICS_BIT).value();

  auto rg = create_render_graph(test_device()).value();

  auto color0 = rg->create_texture2d("color0", VK_FORMAT_R8G8B8A8_UNORM, {640, 480}).value();

  LANCE_THROW_IF_FAILED(rg->add_graphics_pass(
      "clear",
      [&](GraphicsPassBuilder* builder) -> absl::Status {
        // input
        builder
            ->set_vertex_binding(0, VK_VERTEX_INPUT_RATE_VERTEX, 12,
                                 {VertexInputAttribute(0, 0, VK_FORMAT_R32G32B32_SFLOAT)})
            ->set_shader_by_glsl(VK_SHADER_STAGE_VERTEX_BIT, R"glsl(
#version 450

layout(location = 0) in vec3 inPosition;

void main() {
  gl_Position = vec4(inPosition, 1);
}
)glsl");

        builder->set_shader_by_glsl(VK_SHADER_STAGE_FRAGMENT_BIT, R"glsl(
#version 450

layout(location=0) out vec4 outColor;

void main() {
  outColor = vec4(1);
}
        )glsl");

        builder->add_color_attachment(color0, 0,
                                      AttachmentDescription()
                                          .set_format(VK_FORMAT_R32G32B32_SFLOAT)
                                          .clear_to({0.f, 0.f, 0.f, 1.f})
                                          .set_store_op(VK_ATTACHMENT_STORE_OP_DONT_CARE));

        return absl::OkStatus();
      },
      [=](Context* ctx) -> absl::Status { return absl::OkStatus(); }));

  LANCE_THROW_IF_FAILED(rg->compile());
}

TEST(render_graph, depth_test) {
  auto rg = create_render_graph(test_device()).value();

  auto depth_buffer =
      rg->create_texture2d("depth-buffer", VK_FORMAT_D32_SFLOAT, {640, 480}).value();

  LANCE_THROW_IF_FAILED(rg->add_graphics_pass(
      "DepthPass",
      [depth_buffer](GraphicsPassBuilder* builder) -> absl::Status {
        builder->set_depth_stencil_attachment(
            depth_buffer, AttachmentDescription().set_format(VK_FORMAT_R32_SFLOAT));

        return absl::OkStatus();
      },
      [](Context* ctx) -> absl::Status { return absl::OkStatus(); }));
}
}  // namespace rendering
}  // namespace lance
