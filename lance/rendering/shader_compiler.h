#pragma once

#include "absl/status/statusor.h"
#include "glslang/Include/glslang_c_shader_types.h"

namespace lance {
namespace rendering {
absl::StatusOr<std::string> compile_glsl_shader(const char* source, glslang_stage_t stage);
}
}  // namespace lance
