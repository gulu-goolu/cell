#include "shader_compiler.h"

#include "absl/strings/str_format.h"
#include "glog/logging.h"
#include "glslang/Public/resource_limits_c.h"
#include "lance/core/util.h"

namespace lance {
namespace rendering {
void make_sure_glslang_ready() {
  struct EnsureGlslangReady {
    EnsureGlslangReady() { CHECK(glslang_initialize_process()); }

    ~EnsureGlslangReady() { glslang_finalize_process(); }
  };

  static EnsureGlslangReady glslang_ready;
}

absl::StatusOr<core::RefCountPtr<core::Blob>> compile_glsl_shader(const char* source,
                                                                  glslang_stage_t stage) {
  make_sure_glslang_ready();

  glslang_input_t input = {};
  input.language = GLSLANG_SOURCE_GLSL;
  input.stage = stage;
  input.client = GLSLANG_CLIENT_VULKAN;
  input.client_version = GLSLANG_TARGET_VULKAN_1_2;
  input.target_language = GLSLANG_TARGET_SPV;
  input.target_language_version = GLSLANG_TARGET_SPV_1_5;
  input.code = source;
  input.default_version = 100;
  input.default_profile = GLSLANG_NO_PROFILE;
  input.force_default_version_and_profile = false;
  input.forward_compatible = false;
  input.messages = GLSLANG_MSG_DEFAULT_BIT;
  input.resource = glslang_default_resource();

  auto shader = glslang_shader_create(&input);
  CHECK(shader != nullptr);

  LANCE_ON_SCOPE_EXIT([&]() {
    if (shader) glslang_shader_delete(shader);
  });

  if (!glslang_shader_preprocess(shader, &input)) {
    return absl::UnknownError(absl::StrFormat("preprocessing failed, %s\n, %s\n",
                                              glslang_shader_get_info_log(shader),
                                              glslang_shader_get_info_debug_log(shader)));
  }

  if (!glslang_shader_parse(shader, &input)) {
    return absl::UnknownError(absl::StrFormat(
        "parsing failed, %s\n%s\n%s\n", glslang_shader_get_info_log(shader),
        glslang_shader_get_info_debug_log(shader), glslang_shader_get_preprocessed_code(shader)));
  }

  glslang_program_t* program = glslang_program_create();
  CHECK(program != nullptr);

  LANCE_ON_SCOPE_EXIT([&]() {
    if (program) glslang_program_delete(program);
  });

  glslang_program_add_shader(program, shader);

  if (!glslang_program_link(program, GLSLANG_MSG_SPV_RULES_BIT | GLSLANG_MSG_VULKAN_RULES_BIT)) {
    return absl::UnknownError(absl::StrFormat("GLSL linking failed, %s\n%s\n",
                                              glslang_program_get_info_log(program),
                                              glslang_program_get_info_debug_log(program)));
  }

  glslang_program_SPIRV_generate(program, stage);

  return core::Blob::create(glslang_program_SPIRV_get_ptr(program),
                            glslang_program_SPIRV_get_size(program) * 4);
}
}  // namespace rendering
}  // namespace lance
