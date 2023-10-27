#pragma once

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "device.h"
#include "lance/core/object.h"

namespace lance {
namespace rendering {
class Resource : public core::Inherit<Resource, core::Object> {
 public:
};

class PassBuilder {
 public:
  virtual ~PassBuilder() = default;

  virtual absl::Status set_color_attachment(std::string_view id) = 0;
  virtual absl::Status set_depth_attachment(std::string_view id) = 0;
  virtual absl::Status set_shader(VkShaderStageFlagBits stage,
                                  core::RefCountPtr<ShaderModule> shader_module) = 0;
  virtual absl::Status set_descriptor_set_layout(
      uint32_t set, core::RefCountPtr<DescriptorSetLayout> descriptor_set_layout) = 0;
  virtual absl::Status add_descriptor_binding(uint32_t set,
                                              VkDescriptorSetLayoutBinding binding) = 0;

  absl::Status set_vertex_shader(core::RefCountPtr<ShaderModule> shader_module) {
    return set_shader(VK_SHADER_STAGE_VERTEX_BIT, shader_module);
  }

  absl::Status set_compute_shader(core::RefCountPtr<ShaderModule> shader_module) {
    return set_shader(VK_SHADER_STAGE_COMPUTE_BIT, shader_module);
  }
};

class Context {
 public:
  virtual ~Context() = default;

  virtual VkCommandBuffer vk_command_buffer() const = 0;
  virtual VkPipeline vk_pipeline() const = 0;
  virtual VkPipelineLayout vk_pipeline_layout() const = 0;
};

class RenderGraph : public core::Inherit<RenderGraph, core::Object> {
 public:
  absl::StatusOr<std::string> add_resource(absl::Span<const std::string> deps);

  virtual absl::Status add_pass(std::string name,
                                std::function<absl::Status(PassBuilder*)> setup_fn,
                                std::function<absl::Status(Context* ctx)> execute_fn) = 0;

  virtual absl::Status compile() = 0;

  virtual absl::Status execute(
      VkCommandBuffer command_buffer,
      absl::Span<const std::pair<std::string, core::RefCountPtr<Resource>>> inputs) = 0;
};

absl::StatusOr<core::RefCountPtr<RenderGraph>> create_render_graph(
    core::RefCountPtr<Device> device);

}  // namespace rendering
}  // namespace lance
