#include "render_graph.h"

#include "device.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "shader_compiler.h"
#include "util.h"
#include "vk_api.h"

namespace lance {
namespace rendering {
namespace {}
TEST(render_graph, compute) {
  auto instance = Instance::create_for_3d().value();

  auto device = instance->create_device_for_graphics().value();

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


void main() {

}
)glsl";

  auto blob = compile_glsl_shader(add_shader, glslang_stage_t::GLSLANG_STAGE_COMPUTE).value();
  auto shader_module = device->create_shader_module(blob.get()).value();

  auto st_v1 = rg->add_pass(
      "Compute",
      [&shader_module, &descriptor_set_layout](PassBuilder* builder) -> absl::Status {
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

  auto st_v2 = rg->compile();
  ASSERT_TRUE(st_v2.ok());

  auto command_buffer =
      command_pool->allocate_command_buffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY).value();

  LANCE_THROW_IF_FAILED(command_buffer->begin());

  auto st_v3 = rg->execute(command_buffer->vk_command_buffer(), {});
  ASSERT_TRUE(st_v3.ok());

  LANCE_THROW_IF_FAILED(command_buffer->end());

  render_doc_begin_capture();

  auto st_v4 = device->submit(compute_queue_family_index, {command_buffer->vk_command_buffer()});
  ASSERT_TRUE(st_v4.ok());

  render_doc_end_capture();
}
}  // namespace rendering
}  // namespace lance
