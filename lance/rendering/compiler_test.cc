#include "gtest/gtest.h"
#include "shader_compiler.h"

namespace lance {
namespace rendering {
TEST(rendering, glsl) {
  const char* hlsl = R"hlsl(
#version 450 core

void main() {
    gl_Position = vec4(0,0,0,1);
}
)hlsl";

  auto spirv = compile_glsl_shader(hlsl, glslang_stage_t::GLSLANG_STAGE_VERTEX);
  ASSERT_TRUE(spirv.ok()) << spirv.status().ToString();
}
}  // namespace rendering
}  // namespace lance
