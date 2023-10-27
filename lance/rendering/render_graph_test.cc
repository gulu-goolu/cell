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
      [shader_module](PassBuilder* builder) -> absl::Status {
        LANCE_RETURN_IF_FAILED(builder->set_compute_shader(shader_module));

        return absl::OkStatus();
      },
      [=](VkCommandBuffer cmd) -> absl::Status {
        VkApi::get()->vkCmdDispatch(cmd, 0, 0, 0);
        return absl::OkStatus();
      });
  ASSERT_TRUE(st_v1.ok());

  auto st_v2 = rg->compile();
  ASSERT_TRUE(st_v2.ok());
}
}  // namespace rendering
}  // namespace lance
