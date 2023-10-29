#include "render_graph.h"

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "glog/logging.h"
#include "lance/rendering/vk_api.h"

namespace lance {
namespace rendering {

AttachmentDescription &AttachmentDescription::clear_to(absl::Span<const float> values) {
  description.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;

  CHECK_EQ(4, values.size());
  for (size_t i = 0; i < 4; ++i) {
    clear_value.color.float32[i] = values[i];
  }

  return *this;
}

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

  GraphicsPassBuilder *add_color_attachment(int32_t resource_id, uint32_t location,
                                            AttachmentDescription builder) override {
    CHECK(color_attachments.find(location) == color_attachments.end());

    color_attachments[location].resource_id = resource_id;
    color_attachments[location].description = builder;

    return this;
  }

  GraphicsPassBuilder *set_depth_stencil_attachment(int32_t id, bool depth_test_enable) override {
    depth_stencil_attachment = std::make_unique<DepthStencilAttachment>();
    depth_stencil_attachment->id = id;
    depth_stencil_attachment->depth_test_enable = depth_test_enable;

    return this;
  }

  GraphicsPassBuilder *set_viewport(float x, float y, float width, float height, float min_depth,
                                    float max_depth) override {
    viewport = std::make_unique<VkViewport>(VkViewport{x, y, width, height, min_depth, max_depth});

    return this;
  }

  GraphicsPassBuilder *set_cull_mode(VkCullModeFlagBits _cull_mode) override {
    cull_mode = _cull_mode;

    return this;
  }

  GraphicsPassBuilder *set_front_face(VkFrontFace _front_face) override {
    front_face = _front_face;

    return this;
  }

  GraphicsPassBuilder *set_polygon_mode(VkPolygonMode mode) override {
    polygon_mode = mode;

    return this;
  }

  GraphicsPassBuilder *set_blend_constants(absl::Span<const float> values) override {
    CHECK_EQ(values.size(), 4);

    for (size_t i = 0; i < values.size(); ++i) {
      blend_constants[i] = values[i];
    }

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

  GraphicsPassBuilder *set_shader_by_glsl(VkShaderStageFlagBits stage,
                                          const char *source) override {
    auto shader = device->create_shader_from_source(stage, source);
    CHECK(shader.ok()) << "err_msg: " << shader.status().message();

    return set_shader(stage, shader.value());
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

  absl::StatusOr<std::vector<VkPipelineColorBlendAttachmentState>>
  create_color_blend_attachment_states() const {
    std::vector<VkPipelineColorBlendAttachmentState> states;
    return states;
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

    std::vector<VkDynamicState> dynamic_states;

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

    //
    graphics_pipeline_create_info.pTessellationState = nullptr;

    //
    VkPipelineViewportStateCreateInfo viewport_state = {};
    viewport_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;

    std::vector<VkViewport> viewports;
    if (viewport) {
      viewports.push_back(*viewport);
    } else {
      dynamic_states.push_back(VK_DYNAMIC_STATE_VIEWPORT);
    }
    viewport_state.viewportCount = viewports.size();
    viewport_state.pViewports = viewports.data();

    std::vector<VkRect2D> scissors;
    if (scissor) {
      scissors.push_back(*scissor);
    } else {
      dynamic_states.push_back(VK_DYNAMIC_STATE_SCISSOR);
    }
    viewport_state.scissorCount = scissors.size();
    viewport_state.pScissors = scissors.data();

    graphics_pipeline_create_info.pViewportState = &viewport_state;

    VkPipelineRasterizationStateCreateInfo rasterization_state = {};
    rasterization_state.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rasterization_state.depthClampEnable = VK_FALSE;
    rasterization_state.rasterizerDiscardEnable = VK_FALSE;
    rasterization_state.cullMode = cull_mode;
    rasterization_state.polygonMode = polygon_mode;
    rasterization_state.frontFace = front_face;
    rasterization_state.depthBiasEnable = VK_FALSE;
    rasterization_state.lineWidth = 1.0f;

    graphics_pipeline_create_info.pRasterizationState = &rasterization_state;

    VkPipelineMultisampleStateCreateInfo multisample_state = {};
    multisample_state.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    multisample_state.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    multisample_state.pSampleMask = nullptr;
    multisample_state.sampleShadingEnable = VK_FALSE;
    multisample_state.alphaToCoverageEnable = VK_FALSE;
    multisample_state.alphaToOneEnable = VK_FALSE;

    graphics_pipeline_create_info.pMultisampleState = &multisample_state;

    VkPipelineDepthStencilStateCreateInfo depth_stencil_state = {};
    depth_stencil_state.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    depth_stencil_state.depthTestEnable = VK_FALSE;
    depth_stencil_state.depthWriteEnable = VK_TRUE;
    depth_stencil_state.depthCompareOp = VK_COMPARE_OP_ALWAYS;
    depth_stencil_state.depthBoundsTestEnable = VK_FALSE;
    depth_stencil_state.stencilTestEnable = VK_FALSE;
    depth_stencil_state.minDepthBounds = 0.f;
    depth_stencil_state.maxDepthBounds = 1.f;

    if (depth_stencil_attachment) {
      depth_stencil_state.depthTestEnable = depth_stencil_attachment->depth_test_enable;
    }

    graphics_pipeline_create_info.pDepthStencilState = &depth_stencil_state;

    LANCE_ASSIGN_OR_RETURN(blend_attachment_states, create_color_blend_attachment_states());

    VkPipelineColorBlendStateCreateInfo color_blend_state = {};
    color_blend_state.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    color_blend_state.logicOpEnable = VK_FALSE;
    color_blend_state.attachmentCount = blend_attachment_states.size();
    color_blend_state.pAttachments = blend_attachment_states.data();
    for (size_t i = 0; i < 4; ++i) color_blend_state.blendConstants[i] = blend_constants[i];

    graphics_pipeline_create_info.pColorBlendState = &color_blend_state;

    // dynamic state
    VkPipelineDynamicStateCreateInfo dynamic_state_create_info = {};
    dynamic_state_create_info.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    dynamic_state_create_info.dynamicStateCount = dynamic_states.size();
    dynamic_state_create_info.pDynamicStates = dynamic_states.data();

    graphics_pipeline_create_info.pDynamicState = &dynamic_state_create_info;

    graphics_pipeline_create_info.layout = pipeline_layout->vk_pipeline_layout();

    graphics_pipeline_create_info.renderPass = render_pass->vk_render_pass();
    graphics_pipeline_create_info.subpass = subpass;

    VkPipeline vk_pipeline{VK_NULL_HANDLE};
    VK_RETURN_IF_FAILED(VkApi::get()->vkCreateGraphicsPipelines(device->vk_device(), VK_NULL_HANDLE,
                                                                1, &graphics_pipeline_create_info,
                                                                nullptr, &vk_pipeline));

    return core::make_refcounted<GraphicsPipeline>(device, vk_pipeline, pipeline_layout,
                                                   render_pass);
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

  struct ColorAttachment {
    int32_t resource_id;

    AttachmentDescription description;
  };
  std::unordered_map<int32_t, ColorAttachment> color_attachments;

  struct DepthStencilAttachment {
    int32_t id = -1;

    bool depth_test_enable = false;
  };
  std::unique_ptr<DepthStencilAttachment> depth_stencil_attachment;

  std::unique_ptr<VkViewport> viewport;
  std::unique_ptr<VkRect2D> scissor;

  VkCullModeFlagBits cull_mode = VK_CULL_MODE_NONE;
  VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;
  VkFrontFace front_face = VK_FRONT_FACE_COUNTER_CLOCKWISE;
  std::array<float, 4> blend_constants = {1, 1, 1, 1};

  std::unordered_map<uint32_t, core::RefCountPtr<DescriptorSetLayout>> descriptor_set_layouts;
  std::vector<VkPushConstantRange> push_constants;
  std::unordered_map<VkShaderStageFlagBits, core::RefCountPtr<ShaderModule>> shader_modules;
};

class GraphicsPass : public Pass {
 public:
  GraphicsPass(std::function<absl::Status(Context *)> execute_fn)
      : execute_fn_(std::move(execute_fn)) {}

  absl::Status execute(VkCommandBuffer vk_command_buffer) override {
    class GraphicsContext : public Context {
     public:
      GraphicsContext(GraphicsPass *pass, VkCommandBuffer vk_command_buffer)
          : pass_(pass), vk_command_buffer_(vk_command_buffer) {}

      VkCommandBuffer vk_command_buffer() const override { return vk_command_buffer_; }
      VkPipeline vk_pipeline() const override { return pass_->pipeline_->vk_pipeline(); }
      VkPipelineLayout vk_pipeline_layout() const override {
        return pass_->pipeline_layout_->vk_pipeline_layout();
      }

     private:
      GraphicsPass *pass_ = nullptr;
      VkCommandBuffer vk_command_buffer_{VK_NULL_HANDLE};
    };

    GraphicsContext ctx(this, vk_command_buffer);

    // begin render pass
    VkRenderPassBeginInfo render_pass_begin_info = {};
    render_pass_begin_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    render_pass_begin_info.framebuffer = framebuffer_->vk_framebuffer();

    LANCE_RETURN_IF_FAILED(execute_fn_(&ctx));

    VkApi::get()->vkCmdEndRenderPass(vk_command_buffer);

    return absl::OkStatus();
  }

 private:
  std::function<absl::Status(Context *)> execute_fn_;
  core::RefCountPtr<RenderPass> render_pass_;
  core::RefCountPtr<Pipeline> pipeline_;
  core::RefCountPtr<PipelineLayout> pipeline_layout_;

  core::RefCountPtr<Framebuffer> framebuffer_;
};

class RenderGraphImpl : public core::Inherit<RenderGraphImpl, RenderGraph> {
 public:
  explicit RenderGraphImpl(core::RefCountPtr<Device> device) : device_(device) {}

  absl::StatusOr<int32_t> import_resource(
      const std::string &name, const core::RefCountPtr<RenderGraphResource> &resource) override {
    return absl::OkStatus();
  }

  absl::StatusOr<int32_t> create_resource(const std::string &name) override {
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
    auto builder = std::make_unique<GraphicsPassBuilderImpl>(device_);
    LANCE_RETURN_IF_FAILED(setup_fn(builder.get()));

    return absl::OkStatus();
  }

  absl::Status compile() override { return absl::OkStatus(); }

  absl::Status execute(
      VkCommandBuffer command_buffer,
      absl::Span<const std::pair<std::string, core::RefCountPtr<RenderGraphResource>>> inputs)
      override {
    for (auto &pass : passes_) {
      LANCE_RETURN_IF_FAILED(pass->execute(command_buffer));
    }

    return absl::OkStatus();
  }

 private:
  int64_t resource_id_alloc_{0};

  std::unordered_map<int64_t, std::string> resources_;

  // device for create resource
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

void Context::set_viewport(uint32_t first_viewport, absl::Span<const VkViewport> viewports) {
  VkApi::get()->vkCmdSetViewport(vk_command_buffer(), first_viewport, viewports.size(),
                                 viewports.data());
}

void Context::set_scissors(uint32_t first_scissor, absl::Span<const VkRect2D> scissors) {
  VkApi::get()->vkCmdSetScissor(vk_command_buffer(), first_scissor, scissors.size(),
                                scissors.data());
}

absl::StatusOr<core::RefCountPtr<RenderGraph>> create_render_graph(
    core::RefCountPtr<Device> device) {
  return core::make_refcounted<RenderGraphImpl>(device);
}

}  // namespace rendering
}  // namespace lance
