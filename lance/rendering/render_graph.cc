#include "render_graph.h"

#include "absl/strings/str_format.h"

namespace lance {
namespace rendering {
namespace {
class PassBuilderImpl : public PassBuilder {
 public:
  absl::Status set_color_attachment(std::string_view id) override { return absl::OkStatus(); }

  absl::Status set_depth_attachment(std::string_view id) override { return absl::OkStatus(); }

  absl::Status set_shader(VkShaderStageFlagBits stage,
                          core::RefCountPtr<ShaderModule> shader_module) override {
    if (shader_modules_.find(stage) != shader_modules_.end()) {
      return absl::AlreadyExistsError(absl::StrFormat("stage: %d", stage));
    }

    shader_modules_[stage] = shader_module;

    return absl::OkStatus();
  }

 private:
  std::unordered_map<VkShaderStageFlags, core::RefCountPtr<ShaderModule>> shader_modules_;
};

class RenderGraphImpl : public core::Inherit<RenderGraphImpl, RenderGraph> {
 public:
  explicit RenderGraphImpl(core::RefCountPtr<Device> device) {}

  absl::Status add_pass(std::string name, std::function<absl::Status(PassBuilder*)> setup_fn,
                        std::function<absl::Status(VkCommandBuffer)> execute_fn) override {
    return absl::OkStatus();
  }

  absl::Status compile() override { return absl::OkStatus(); }

  absl::Status execute(
      VkCommandBuffer command_buffer,
      absl::Span<const std::pair<std::string, core::RefCountPtr<Resource>>> inputs) override {
    return absl::OkStatus();
  }

 private:
  std::vector<std::unique_ptr<PassBuilderImpl>> pass_builders_;
};

}  // namespace

absl::StatusOr<core::RefCountPtr<RenderGraph>> create_render_graph(
    core::RefCountPtr<Device> device) {
  return core::make_refcounted<RenderGraphImpl>(device);
}

}  // namespace rendering
}  // namespace lance
