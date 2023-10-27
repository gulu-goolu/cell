#include "render_graph.h"

#include "device.h"
#include "gtest/gtest.h"
#include "shader_compiler.h"
#include "vk_api.h"

namespace lance {
namespace rendering {
namespace {}
TEST(render_graph, compute) {
  auto instance = Instance::create_for_3d().value();

  auto device = instance->create_device_for_graphics().value();

  VkDescriptorSetLayoutBinding bindings = {};
  bindings.binding = 0;
  bindings.descriptorCount = 1;
  bindings.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
  bindings.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  auto descriptor_set_layout = DescriptorSetLayout::create(device, {bindings}).value();

  auto rg = create_render_graph(device).value();

  const char* add_shader = R"glsl(
#version 450 core

layout(set=0, binding=0) uniform Arguments {
    vec4 v[];
};

void main() {

}
)glsl";

  auto blob = compile_glsl_shader(add_shader, glslang_stage_t::GLSLANG_STAGE_COMPUTE).value();
  auto shader_module = device->create_shader_module(blob.get()).value();

  auto st_v1 = rg->add_pass(
      "Compute",
      [&shader_module, &descriptor_set_layout](PassBuilder* builder) -> absl::Status {
        LANCE_RETURN_IF_FAILED(builder->set_descriptor_set_layout(0, descriptor_set_layout));

        LANCE_RETURN_IF_FAILED(builder->set_compute_shader(shader_module));

        return absl::OkStatus();
      },
      [=](Context* ctx) -> absl::Status {
        VkApi::get()->vkCmdBindDescriptorSets(ctx->vk_command_buffer(),
                                              VK_PIPELINE_BIND_POINT_COMPUTE,
                                              ctx->vk_pipeline_layout(), 0, 0, nullptr, 0, nullptr);

        VkApi::get()->vkCmdDispatch(ctx->vk_command_buffer(), 0, 0, 0);
        return absl::OkStatus();
      });
  ASSERT_TRUE(st_v1.ok());

  auto st_v2 = rg->compile();
  ASSERT_TRUE(st_v2.ok());
}
}  // namespace rendering
}  // namespace lance
