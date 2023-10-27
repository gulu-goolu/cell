#include "render_graph.h"

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "lance/rendering/vk_api.h"

namespace lance {
namespace rendering {
namespace {
class Pass {
 public:
  virtual ~Pass() = default;

  virtual absl::Status execute(VkCommandBuffer cmd) = 0;
};

class ComputePass : public Pass {
 public:
  ComputePass(std::function<absl::Status(Context *)> execute_fn,
              VkPipelineLayout vk_pipeline_layout, core::RefCountPtr<Pipeline> pipeline)
      : execute_fn_(execute_fn), vk_pipeline_layout_(vk_pipeline_layout), pipeline_(pipeline) {}

  absl::Status execute(VkCommandBuffer cmd) override {
    VkApi::get()->vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_->vk_pipeline());

    class ContextImpl : public Context {
     public:
      ContextImpl(ComputePass *pass, VkCommandBuffer vk_command_buffer)
          : pass_(pass), vk_command_buffer_(vk_command_buffer) {}

      VkCommandBuffer vk_command_buffer() const override { return vk_command_buffer_; }
      VkPipeline vk_pipeline() const override { return pass_->pipeline_->vk_pipeline(); }
      VkPipelineLayout vk_pipeline_layout() const override { return pass_->vk_pipeline_layout_; }

     private:
      ComputePass *pass_ = nullptr;
      VkCommandBuffer vk_command_buffer_{VK_NULL_HANDLE};
    };

    ContextImpl ctx(this, cmd);
    return execute_fn_(&ctx);
  }

 private:
  const std::function<absl::Status(Context *)> execute_fn_;
  VkPipelineLayout vk_pipeline_layout_{VK_NULL_HANDLE};
  core::RefCountPtr<Pipeline> pipeline_;
};

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

  absl::Status set_descriptor_set_layout(
      uint32_t set, core::RefCountPtr<DescriptorSetLayout> descriptor_set_layout) override {
    return absl::OkStatus();
  }

  absl::Status add_descriptor_binding(uint32_t set, VkDescriptorSetLayoutBinding binding) override {
    auto &descriptor_set = buffer_descriptors_[set];
    if (descriptor_set.find(binding.binding) != descriptor_set.end()) {
      return absl::AlreadyExistsError(
          absl::StrFormat("binding already exits, binding: %d", binding.binding));
    }

    descriptor_set[binding.binding] = binding;

    return absl::OkStatus();
  }

  bool is_compute_pass() const {
    return shader_modules_.find(VK_SHADER_STAGE_COMPUTE_BIT) != shader_modules_.end();
  }

  absl::StatusOr<VkPipelineLayout> create_pipeline_layout() const {
    std::vector<VkDescriptorSetLayout> set_layouts;
    set_layouts.resize(descriptor_set_layouts_.size(), VK_NULL_HANDLE);
    for (const auto &pair : descriptor_set_layouts_) {
      if (pair.first >= descriptor_set_layouts_.size()) {
        return absl::InvalidArgumentError(absl::StrFormat("out of bound, set: %d", pair.first));
      }

      set_layouts[pair.first] = pair.second->vk_descriptor_set_layout();
    }

    VkPipelineLayoutCreateInfo pipeline_layout_create_info = {};
    pipeline_layout_create_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipeline_layout_create_info.setLayoutCount = set_layouts.size();
    pipeline_layout_create_info.pSetLayouts = set_layouts.data();

    VkPipelineLayout vk_pipeline_layout;
    VK_RETURN_IF_FAILED(VkApi::get()->vkCreatePipelineLayout(
        device_->vk_device(), &pipeline_layout_create_info, nullptr, &vk_pipeline_layout));

    return vk_pipeline_layout;
  }

  absl::StatusOr<core::RefCountPtr<Pipeline>> create_compute_pipeline(
      VkPipelineLayout vk_pipeline_layout) const {
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

    return core::make_refcounted<Pipeline>(device_, vk_pipeline);
  }

  absl::StatusOr<std::unique_ptr<Pass>> create_pass(
      std::function<absl::Status(Context *)> execute_fn) const {
    if (is_compute_pass()) {
      LANCE_ASSIGN_OR_RETURN(pipeline_layout, create_pipeline_layout());

      LANCE_ASSIGN_OR_RETURN(pipeline, create_compute_pipeline(pipeline_layout));

      return std::make_unique<ComputePass>(std::move(execute_fn), pipeline_layout, pipeline);
    } else {
      return nullptr;
    }
  }

 private:
  core::RefCountPtr<Device> device_;
  std::unordered_map<VkShaderStageFlagBits, core::RefCountPtr<ShaderModule>> shader_modules_;

  std::unordered_map<uint32_t, core::RefCountPtr<DescriptorSetLayout>> descriptor_set_layouts_;

  std::unordered_map<uint32_t, std::unordered_map<uint32_t, VkDescriptorSetLayoutBinding>>
      buffer_descriptors_;
};

class RenderGraphImpl : public core::Inherit<RenderGraphImpl, RenderGraph> {
 public:
  explicit RenderGraphImpl(core::RefCountPtr<Device> device) : device_(device) {}

  absl::Status add_pass(std::string name, std::function<absl::Status(PassBuilder *)> setup_fn,
                        std::function<absl::Status(Context *)> execute_fn) override {
    PassBuilderImpl builder(device_);
    LANCE_RETURN_IF_FAILED(setup_fn(&builder));

    LANCE_ASSIGN_OR_RETURN(pass, builder.create_pass(std::move(execute_fn)));

    passes_.push_back(std::move(pass));

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
};

}  // namespace

void Context::push_constants(VkShaderStageFlags stage, uint32_t offset, uint32_t size,
                             const void *values) {
  VkApi::get()->vkCmdPushConstants(vk_command_buffer(), vk_pipeline_layout(), stage, offset, size,
                                   values);
}

void Context::dispatch(uint32_t group_count_x, uint32_t group_count_y, uint32_t group_count_z) {
  VkApi::get()->vkCmdDispatch(vk_command_buffer(), group_count_x, group_count_y, group_count_z);
}

absl::StatusOr<core::RefCountPtr<RenderGraph>> create_render_graph(
    core::RefCountPtr<Device> device) {
  return core::make_refcounted<RenderGraphImpl>(device);
}

}  // namespace rendering
}  // namespace lance
