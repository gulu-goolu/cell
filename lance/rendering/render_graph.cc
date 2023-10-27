#include "render_graph.h"

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "lance/rendering/vk_api.h"

namespace lance {
namespace rendering {
namespace {
class PassBuilderImpl : public PassBuilder {
 public:
  PassBuilderImpl(core::RefCountPtr<Device> device) : device_(device) {}

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

  bool is_compute_pass() const {
    return shader_modules_.find(VK_SHADER_STAGE_COMPUTE_BIT) != shader_modules_.end();
  }

  absl::StatusOr<VkPipelineLayout> create_pipeline_layout() const { return absl::OkStatus(); }

  absl::StatusOr<VkPipeline> create_compute_pipeline(VkPipelineLayout vk_pipeline_layout) const {
    auto it = shader_modules_.find(VK_SHADER_STAGE_COMPUTE_BIT);
    if (it == shader_modules_.end()) {
      return absl::NotFoundError("compute shader not found");
    }

    VkComputePipelineCreateInfo pipeline_create_info = {};
    pipeline_create_info.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipeline_create_info.basePipelineIndex = -1;
    pipeline_create_info.basePipelineHandle = nullptr;
    pipeline_create_info.layout = vk_pipeline_layout;
    pipeline_create_info.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    pipeline_create_info.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    pipeline_create_info.stage.module = it->second->vk_shader_module();
    pipeline_create_info.stage.pName = "main";

    VkPipeline vk_pipeline;
    VK_RETURN_IF_FAILED(VkApi::get()->vkCreateComputePipelines(
        device_->vk_device(), VK_NULL_HANDLE, 1, &pipeline_create_info, nullptr, &vk_pipeline));

    return vk_pipeline;
  }

 private:
  core::RefCountPtr<Device> device_;
  std::unordered_map<VkShaderStageFlagBits, core::RefCountPtr<ShaderModule>> shader_modules_;
};

class Pass {
 public:
  virtual ~Pass() = default;

  virtual absl::Status execute(VkCommandBuffer cmd) = 0;
};

class ComputePass : public Pass {
 public:
  ComputePass(std::function<absl::Status(VkCommandBuffer)> execute_fn, VkPipeline vk_pipeline)
      : execute_fn_(execute_fn), vk_pipeline_(vk_pipeline) {}

  absl::Status execute(VkCommandBuffer cmd) override {
    VkApi::get()->vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, vk_pipeline_);

    return execute_fn_(cmd);
  }

 private:
  const std::function<absl::Status(VkCommandBuffer)> execute_fn_;
  VkPipeline vk_pipeline_{VK_NULL_HANDLE};
};

class RenderGraphImpl : public core::Inherit<RenderGraphImpl, RenderGraph> {
 public:
  explicit RenderGraphImpl(core::RefCountPtr<Device> device) : device_(device) {}

  absl::Status add_pass(std::string name, std::function<absl::Status(PassBuilder *)> setup_fn,
                        std::function<absl::Status(VkCommandBuffer)> execute_fn) override {
    PassBuilderImpl builder(device_);
    LANCE_RETURN_IF_FAILED(setup_fn(&builder));

    if (builder.is_compute_pass()) {
      LANCE_ASSIGN_OR_RETURN(pipeline_layout, builder.create_pipeline_layout());

      LANCE_ASSIGN_OR_RETURN(pipeline, builder.create_compute_pipeline(pipeline_layout));

      passes_.push_back(std::make_unique<ComputePass>(execute_fn, pipeline));
    }

    return absl::OkStatus();
  }

  absl::Status compile() override { return absl::OkStatus(); }

  absl::Status execute(
      VkCommandBuffer command_buffer,
      absl::Span<const std::pair<std::string, core::RefCountPtr<Resource>>> inputs) override {
    for (auto &pass : passes_) {
      LANCE_RETURN_IF_FAILED(pass->execute(command_buffer));
    }

    return absl::OkStatus();
  }

 private:
  core::RefCountPtr<Device> device_;

  std::vector<std::unique_ptr<Pass>> passes_;
  std::vector<std::unique_ptr<PassBuilderImpl>> pass_builders_;
};

}  // namespace

absl::StatusOr<core::RefCountPtr<RenderGraph>> create_render_graph(
    core::RefCountPtr<Device> device) {
  return core::make_refcounted<RenderGraphImpl>(device);
}

}  // namespace rendering
}  // namespace lance
