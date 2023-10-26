#pragma once

#include "absl/status/statusor.h"
#include "glslang/Include/glslang_c_shader_types.h"
#include "lance/core/object.h"

namespace lance {
namespace rendering {
absl::StatusOr<core::RefCountPtr<core::Blob>> compile_glsl_shader(const char* source,
                                                                  glslang_stage_t stage);
}
}  // namespace lance
