#include "render_graph.h"

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "glog/logging.h"
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
              const core::RefCountPtr<PipelineLayout> &pipeline_layout,
              core::RefCountPtr<Pipeline> pipeline)
      : execute_fn_(execute_fn), pipeline_layout_(pipeline_layout), pipeline_(pipeline) {}

  absl::Status execute(VkCommandBuffer cmd) override {
    VkApi::get()->vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_->vk_pipeline());

    class ContextImpl : public Context {
     public:
      ContextImpl(ComputePass *pass, VkCommandBuffer vk_command_buffer)
          : pass_(pass), vk_command_buffer_(vk_command_buffer) {}

      VkCommandBuffer vk_command_buffer() const override { return vk_command_buffer_; }
      VkPipeline vk_pipeline() const override { return pass_->pipeline_->vk_pipeline(); }
      VkPipelineLayout vk_pipeline_layout() const override {
        return pass_->pipeline_layout_->vk_pipeline_layout();
      }

     private:
      ComputePass *pass_ = nullptr;
      VkCommandBuffer vk_command_buffer_{VK_NULL_HANDLE};
    };

    ContextImpl ctx(this, cmd);
    return execute_fn_(&ctx);
  }

 private:
  const std::function<absl::Status(Context *)> execute_fn_;
  core::RefCountPtr<PipelineLayout> pipeline_layout_;
  core::RefCountPtr<Pipeline> pipeline_;
};

class GraphicsPassBuilderImpl : public GraphicsPassBuilder {
 public:
  GraphicsPassBuilderImpl(const core::RefCountPtr<Device> &d) : device(d) {}

  GraphicsPassBuilder *set_vertex_binding(uint32_t binding, VkVertexInputRate input_rate,
                                          uint32_t stride,
                                          absl::Span<const VertexInputAttribute> attrs) override {
    VkVertexInputBindingDescription vertex_binding;
    vertex_binding.binding = binding;
    vertex_binding.inputRate = input_rate;
    vertex_binding.stride = stride;
    vertex_input_bindings.push_back(vertex_binding);

    for (const auto &attr : attrs) {
      VkVertexInputAttributeDescription vertex_attribute;
      vertex_attribute.binding = binding;
      vertex_attribute.format = attr.format;
      vertex_attribute.location = attr.location;
      vertex_attribute.offset = attr.offset;
      vertex_input_attributes.push_back(vertex_attribute);
    }

    return this;
  }

  GraphicsPassBuilder *set_topology(VkPrimitiveTopology _topology) override {
    topology = _topology;

    return this;
  }

  GraphicsPassBuilder *set_color_attachments(absl::Span<const std::string> ids) override {
    CHECK(color_attachment_ids.empty());

    for (const auto &id : ids) {
      color_attachment_ids.push_back(id);
    }

    return this;
  }

  GraphicsPassBuilder *set_depth_stencil_attachment(const std::string &id) override {
    depth_stencil_attachment_id = id;

    return this;
  }

  GraphicsPassBuilder *set_viewport(float x, float y, float width, float height, float min_depth,
                                    float max_depth) override {
    viewport = std::make_unique<VkViewport>(VkViewport{x, y, width, height, min_depth, max_depth});

    return this;
  }

  GraphicsPassBuilder *add_descriptor_set(uint32_t set,
                                          core::RefCountPtr<DescriptorSetLayout> layout) override {
    CHECK(descriptor_set_layouts.find(set) == descriptor_set_layouts.end());

    descriptor_set_layouts[set] = layout;

    return this;
  }

  GraphicsPassBuilder *add_push_constants(VkShaderStageFlags stage_flags, uint32_t offset,
                                          uint32_t size) override {
    push_constants.push_back(VkPushConstantRange{
        stage_flags,
        offset,
        size,
    });

    return this;
  }

  GraphicsPassBuilder *set_shader(VkShaderStageFlagBits stage,
                                  const core::RefCountPtr<ShaderModule> &shader_module) override {
    CHECK(shader_modules.find(stage) == shader_modules.end());

    shader_modules[stage] = shader_module;

    return this;
  }

  absl::StatusOr<core::RefCountPtr<PipelineLayout>> create_pipeline_layout() const {
    std::vector<VkDescriptorSetLayout> set_layouts;
    set_layouts.resize(descriptor_set_layouts.size(), VK_NULL_HANDLE);
    for (const auto &pair : descriptor_set_layouts) {
      CHECK_EQ(set_layouts[pair.first], VK_NULL_HANDLE);
      CHECK_LT(pair.first, set_layouts.size());

      set_layouts[pair.first] = pair.second->vk_descriptor_set_layout();
    }

    VkPipelineLayoutCreateInfo pipeline_create_info = {};
    pipeline_create_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipeline_create_info.pushConstantRangeCount = push_constants.size();
    pipeline_create_info.pPushConstantRanges = push_constants.data();
    pipeline_create_info.setLayoutCount = set_layouts.size();
    pipeline_create_info.pSetLayouts = set_layouts.data();

    VkPipelineLayout vk_pipeline_layout{VK_NULL_HANDLE};
    VK_RETURN_IF_FAILED(VkApi::get()->vkCreatePipelineLayout(
        device->vk_device(), &pipeline_create_info, nullptr, &vk_pipeline_layout));

    return core::make_refcounted<PipelineLayout>(device, vk_pipeline_layout);
  }

  absl::StatusOr<core::RefCountPtr<Pipeline>> create_pipeline(
      const core::RefCountPtr<PipelineLayout> &pipeline_layout,
      const core::RefCountPtr<RenderPass> &render_pass, uint32_t subpass) const {
    std::vector<VkPipelineShaderStageCreateInfo> shader_stage_create_infos = {};
    for (const auto &pair : shader_modules) {
      VkPipelineShaderStageCreateInfo shader_stage_create_info = {};
      shader_stage_create_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
      shader_stage_create_info.stage = pair.first;
      shader_stage_create_info.module = pair.second->vk_shader_module();
      shader_stage_create_info.pName = "main";
      shader_stage_create_infos.push_back(shader_stage_create_info);
    }

    VkGraphicsPipelineCreateInfo graphics_pipeline_create_info = {};
    graphics_pipeline_create_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;

    graphics_pipeline_create_info.stageCount = shader_stage_create_infos.size();
    graphics_pipeline_create_info.pStages = shader_stage_create_infos.data();

    VkPipelineVertexInputStateCreateInfo vertex_input_state;
    vertex_input_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertex_input_state.vertexBindingDescriptionCount = vertex_input_bindings.size();
    vertex_input_state.pVertexBindingDescriptions = vertex_input_bindings.data();
    vertex_input_state.vertexAttributeDescriptionCount = vertex_input_attributes.size();
    vertex_input_state.pVertexAttributeDescriptions = vertex_input_attributes.data();

    graphics_pipeline_create_info.pVertexInputState = &vertex_input_state;

    VkPipelineInputAssemblyStateCreateInfo input_assembly_state = {};
    input_assembly_state.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    input_assembly_state.topology = topology;
    input_assembly_state.primitiveRestartEnable = VK_FALSE;

    graphics_pipeline_create_info.pInputAssemblyState = &input_assembly_state;

    graphics_pipeline_create_info.renderPass = render_pass->vk_render_pass();
    graphics_pipeline_create_info.subpass = subpass;

    return nullptr;
  }

  class GraphicsPipeline : public core::Inherit<GraphicsPipeline, Pipeline> {
   public:
    GraphicsPipeline(const core::RefCountPtr<Device> &device, VkPipeline vk_pipeline,
                     const core::RefCountPtr<PipelineLayout> &pipeline_layout,
                     const core::RefCountPtr<RenderPass> &render_pass)
        : core::Inherit<GraphicsPipeline, Pipeline>(device, vk_pipeline, pipeline_layout),
          render_pass_(render_pass) {}

   private:
    core::RefCountPtr<RenderPass> render_pass_;
  };

  const core::RefCountPtr<Device> device;

  std::vector<VkVertexInputBindingDescription> vertex_input_bindings;
  std::vector<VkVertexInputAttributeDescription> vertex_input_attributes;

  VkPrimitiveTopology topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

  std::vector<std::string> color_attachment_ids;

  std::string depth_stencil_attachment_id;

  std::unique_ptr<VkViewport> viewport;

  std::unordered_map<uint32_t, core::RefCountPtr<DescriptorSetLayout>> descriptor_set_layouts;
  std::vector<VkPushConstantRange> push_constants;
  std::unordered_map<VkShaderStageFlagBits, core::RefCountPtr<ShaderModule>> shader_modules;
};

class PassBuilderImpl : public ComputePassBuilder {
 public:
  PassBuilderImpl(core::RefCountPtr<Device> device) : device_(device) {}

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

  absl::StatusOr<core::RefCountPtr<PipelineLayout>> create_pipeline_layout() const {
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

    return core::make_refcounted<PipelineLayout>(device_, vk_pipeline_layout);
  }

  absl::StatusOr<core::RefCountPtr<Pipeline>> create_compute_pipeline(
      const core::RefCountPtr<PipelineLayout> &pipeline_layout) const {
    auto it = shader_modules_.find(VK_SHADER_STAGE_COMPUTE_BIT);
    if (it == shader_modules_.end()) {
      return absl::NotFoundError("compute shader not found");
    }

    VkComputePipelineCreateInfo pipeline_create_info = {};
    pipeline_create_info.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipeline_create_info.basePipelineIndex = -1;
    pipeline_create_info.basePipelineHandle = nullptr;
    pipeline_create_info.layout = pipeline_layout->vk_pipeline_layout();
    pipeline_create_info.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    pipeline_create_info.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    pipeline_create_info.stage.module = it->second->vk_shader_module();
    pipeline_create_info.stage.pName = "main";

    VkPipeline vk_pipeline;
    VK_RETURN_IF_FAILED(VkApi::get()->vkCreateComputePipelines(
        device_->vk_device(), VK_NULL_HANDLE, 1, &pipeline_create_info, nullptr, &vk_pipeline));

    return core::make_refcounted<Pipeline>(device_, vk_pipeline, pipeline_layout);
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

  VkCullModeFlags cull_mode_ = VK_CULL_MODE_NONE;
  VkFrontFace front_face_ = VK_FRONT_FACE_CLOCKWISE;
};

class RenderGraphImpl : public core::Inherit<RenderGraphImpl, RenderGraph> {
 public:
  explicit RenderGraphImpl(core::RefCountPtr<Device> device) : device_(device) {}

  absl::StatusOr<std::string> import_resource(core::RefCountPtr<Resource> resource) override {
    return absl::OkStatus();
  }

  absl::StatusOr<std::string> create_attachment(VkImageType image_type, VkFormat format,
                                                VkImageUsageFlags usage,
                                                VkExtent3D extent) override {
    VkImageCreateInfo image_create_info = {};
    image_create_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image_create_info.imageType = image_type;
    image_create_info.format = format;
    image_create_info.extent = extent;
    image_create_info.mipLevels = 1;
    image_create_info.arrayLayers = 1;
    image_create_info.samples = VK_SAMPLE_COUNT_1_BIT;
    image_create_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_create_info.usage = usage;
    image_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    image_create_info.queueFamilyIndexCount = 0;
    image_create_info.pQueueFamilyIndices = nullptr;
    image_create_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    return absl::OkStatus();
  }

  absl::Status add_compute_pass(std::string name,
                                std::function<absl::Status(ComputePassBuilder *)> setup_fn,
                                std::function<absl::Status(Context *)> execute_fn) override {
    PassBuilderImpl builder(device_);
    LANCE_RETURN_IF_FAILED(setup_fn(&builder));

    LANCE_ASSIGN_OR_RETURN(pass, builder.create_pass(std::move(execute_fn)));

    passes_.push_back(std::move(pass));

    return absl::OkStatus();
  }

  absl::Status add_graphics_pass(std::string_view name,
                                 std::function<absl::Status(GraphicsPassBuilder *)> setup_fn,
                                 std::function<absl::Status(Context *)> execute_fn) override {
    GraphicsPassBuilderImpl builder(device_);

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

void Context::draw(uint32_t vertex_count, uint32_t instance_count, uint32_t first_vertex,
                   uint32_t first_instance) {
  VkApi::get()->vkCmdDraw(vk_command_buffer(), vertex_count, instance_count, first_vertex,
                          first_instance);
}

absl::StatusOr<core::RefCountPtr<RenderGraph>> create_render_graph(
    core::RefCountPtr<Device> device) {
  return core::make_refcounted<RenderGraphImpl>(device);
}

}  // namespace rendering
}  // namespace lance
