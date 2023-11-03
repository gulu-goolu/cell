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
class RenderGraphTexture2D : public core::Inherit<RenderGraphTexture2D, RenderGraphImage> {
 public:
  RenderGraphTexture2D(int32_t id, VkFormat format, VkExtent2D extent)
      : id_(id), format_(format), extent_(extent) {}

  ~RenderGraphTexture2D() override {
    if (vk_image_view_) {
      VkApi::get()->vkDestroyImageView(device_->vk_device(), vk_image_view_, nullptr);
    }
    if (vk_image_) {
      VkApi::get()->vkDestroyImage(device_->vk_device(), vk_image_, nullptr);
    }
  }

  absl::Status append_image_usage(VkImageUsageFlags flags) override {
    usage_flags_ = usage_flags_ | flags;

    return absl::OkStatus();
  }

  VkImageView image_view() const override { return vk_image_view_; }

  VkExtent3D image_extent() const override {
    VkExtent3D result;
    result.width = extent_.width;
    result.height = extent_.height;
    result.depth = 1;
    return result;
  }

  VkFormat format() const override { return format_; }

  absl::Status initialize(Device *device) override {
    device_.reset(device);

    VkImageCreateInfo image_create_info = {};
    image_create_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image_create_info.imageType = VK_IMAGE_TYPE_2D;
    image_create_info.format = format_;
    image_create_info.extent.width = extent_.width;
    image_create_info.extent.height = extent_.height;
    image_create_info.extent.depth = 1;
    image_create_info.arrayLayers = 1;
    image_create_info.mipLevels = 1;
    image_create_info.samples = VK_SAMPLE_COUNT_1_BIT;
    image_create_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_create_info.usage = usage_flags_;
    image_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    image_create_info.queueFamilyIndexCount = 0;
    image_create_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    VK_RETURN_IF_FAILED(
        VkApi::get()->vkCreateImage(device->vk_device(), &image_create_info, nullptr, &vk_image_));

    VkMemoryRequirements mem_reqs;
    VkApi::get()->vkGetImageMemoryRequirements(device->vk_device(), vk_image_, &mem_reqs);

    LANCE_ASSIGN_OR_RETURN(memory_type_index,
                           device->find_memory_type_index(mem_reqs.memoryTypeBits,
                                                          VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT));

    LANCE_ASSIGN_OR_RETURN(memory, DeviceMemory::create(device_, memory_type_index, mem_reqs.size));
    device_memory_ = memory;

    VK_RETURN_IF_FAILED(VkApi::get()->vkBindImageMemory(device->vk_device(), vk_image_,
                                                        device_memory_->vk_device_memory(), 0));

    VkImageViewCreateInfo image_view_create_info = {};
    image_view_create_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    image_view_create_info.image = vk_image_;
    image_view_create_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    image_view_create_info.format = format_;
    image_view_create_info.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
    image_view_create_info.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
    image_view_create_info.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
    image_view_create_info.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
    image_view_create_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    image_view_create_info.subresourceRange.baseArrayLayer = 0;
    image_view_create_info.subresourceRange.layerCount = 1;
    image_view_create_info.subresourceRange.baseMipLevel = 0;
    image_view_create_info.subresourceRange.levelCount = 1;
    VK_RETURN_IF_FAILED(VkApi::get()->vkCreateImageView(
        device->vk_device(), &image_view_create_info, nullptr, &vk_image_view_));

    return absl::OkStatus();
  }

  int32_t id() const override { return id_; }

 private:
  const int32_t id_;
  const VkFormat format_;
  const VkExtent2D extent_;

  VkImageUsageFlags usage_flags_{0};

  core::RefCountPtr<Device> device_;

  VkImage vk_image_{VK_NULL_HANDLE};
  core::RefCountPtr<DeviceMemory> device_memory_;
  VkImageView vk_image_view_{VK_NULL_HANDLE};
};

class Pass {
 public:
  virtual ~Pass() = default;

  virtual std::string_view name() const = 0;

  virtual absl::Status compile(Device *device) = 0;

  virtual absl::Status execute(CommandBuffer *command_buffer) = 0;
};

class ComputePass : public Pass {
 public:
  ComputePass(std::function<absl::Status(Context *)> execute_fn,
              const core::RefCountPtr<PipelineLayout> &pipeline_layout,
              core::RefCountPtr<Pipeline> pipeline)
      : execute_fn_(execute_fn), pipeline_layout_(pipeline_layout), pipeline_(pipeline) {}

  absl::Status compile(Device *device) override { return absl::OkStatus(); }

  absl::Status execute(CommandBuffer *command_buffer) override {
    VkApi::get()->vkCmdBindPipeline(command_buffer->vk_command_buffer(),
                                    VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_->vk_pipeline());

    class ContextImpl : public Context {
     public:
      ContextImpl(ComputePass *pass, CommandBuffer *command_buffer)
          : pass_(pass), command_buffer_(command_buffer) {}

      CommandBuffer *command_buffer() const override { return command_buffer_; }
      VkPipeline vk_pipeline() const override { return pass_->pipeline_->vk_pipeline(); }
      VkPipelineLayout vk_pipeline_layout() const override {
        return pass_->pipeline_layout_->vk_pipeline_layout();
      }

     private:
      ComputePass *pass_ = nullptr;
      CommandBuffer *command_buffer_{VK_NULL_HANDLE};
    };

    ContextImpl ctx(this, command_buffer);
    return execute_fn_(&ctx);
  }

  std::string_view name() const override { return "ComputePass"; }

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

class RenderGraphImpl;

class GraphicsPassBuilderImpl : public GraphicsPassBuilder {
 public:
  GraphicsPassBuilderImpl(RenderGraph *rg, const core::RefCountPtr<Device> &d)
      : render_graph(rg), device(d) {}

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

  GraphicsPassBuilder *add_color_attachment(core::RefCountPtr<RenderGraphImage> image,
                                            uint32_t location, AttachmentDescription builder,
                                            const VkRect2D *render_area) override {
    CHECK(color_attachments.find(location) == color_attachments.end());

    auto st = image->append_image_usage(VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT);
    CHECK(st.ok()) << "err_msg: " << st.ToString();

    color_attachments[location].image = image;
    color_attachments[location].description = builder;

    if (render_area) {
      color_attachments[location].render_area = std::make_unique<VkRect2D>(*render_area);
    }

    LOG(INFO) << "[GraphicsBuilder::add_color_attachment] resource_id: " << image->id();

    return this;
  }

  GraphicsPassBuilder *set_depth_stencil_attachment(core::RefCountPtr<RenderGraphImage> image,
                                                    AttachmentDescription description) override {
    depth_stencil_attachment = std::make_unique<DepthStencilAttachment>();
    depth_stencil_attachment->id = image->id();
    depth_stencil_attachment->description = description;

    auto st = image->append_image_usage(VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT);
    CHECK(st.ok()) << "err_msg: " << st.ToString();

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

  GraphicsPassBuilder *set_depth_stencil_state(DepthStencilState state) override { return this; }

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

    VLOG(4) << "[GraphicsBuilder::set_shader] stage: " << stage;

    return this;
  }

  GraphicsPassBuilder *set_shader_by_glsl(VkShaderStageFlagBits stage,
                                          const char *source) override {
    auto shader = device->create_shader_from_source(stage, source);
    CHECK(shader.ok()) << "err_msg: " << shader.status().message();

    VLOG(1) << "[GraphicsBuilder::set_shader_by_glsl] stage: " << stage;

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
    for (const auto &color : color_attachments) {
      VkPipelineColorBlendAttachmentState state = {};
      state.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
      state.dstColorBlendFactor = VK_BLEND_FACTOR_ZERO;
      state.blendEnable = VK_FALSE;
      state.colorWriteMask = 0x0f;

      states.push_back(state);
    }
    return states;
  }

  absl::StatusOr<core::RefCountPtr<Pipeline>> create_pipeline(
      const core::RefCountPtr<PipelineLayout> &pipeline_layout,
      const core::RefCountPtr<RenderPass> &render_pass, uint32_t subpass) const {
    VLOG(10) << "[create_pipeline]";

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

    VkPipelineVertexInputStateCreateInfo vertex_input_state = {};
    vertex_input_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertex_input_state.vertexBindingDescriptionCount = vertex_input_bindings.size();
    vertex_input_state.pVertexBindingDescriptions = vertex_input_bindings.data();
    vertex_input_state.vertexAttributeDescriptionCount = vertex_input_attributes.size();
    vertex_input_state.pVertexAttributeDescriptions = vertex_input_attributes.data();
    VLOG(10) << "vertex_input_bindings: " << vertex_input_bindings.size()
             << ", vertex_attribute: " << vertex_input_attributes.size();

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
    viewport_state.viewportCount = viewports.empty() ? 1 : viewports.size();
    viewport_state.pViewports = viewports.data();

    std::vector<VkRect2D> scissors;
    if (scissor) {
      scissors.push_back(*scissor);
    } else {
      dynamic_states.push_back(VK_DYNAMIC_STATE_SCISSOR);
    }
    viewport_state.scissorCount = scissors.empty() ? 1 : scissors.size();
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

    VkPipelineDepthStencilStateCreateInfo depth_stencil_state_info = {};
    depth_stencil_state_info.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    depth_stencil_state_info.depthTestEnable =
        depth_stencil_state ? depth_stencil_state->depth_test_enable : VK_FALSE;
    depth_stencil_state_info.depthWriteEnable =
        depth_stencil_state ? depth_stencil_state->depth_write_enable : VK_FALSE;
    depth_stencil_state_info.depthCompareOp =
        depth_stencil_state ? depth_stencil_state->depth_test_op : VK_COMPARE_OP_ALWAYS;
    depth_stencil_state_info.depthBoundsTestEnable =
        depth_stencil_state ? depth_stencil_state->depth_bounds_test_enable : VK_FALSE;
    depth_stencil_state_info.stencilTestEnable =
        depth_stencil_state ? depth_stencil_state->stencil_test_enable : VK_FALSE;
    depth_stencil_state_info.minDepthBounds =
        depth_stencil_state ? depth_stencil_state->min_depth_bounds : 0.f;
    depth_stencil_state_info.maxDepthBounds =
        depth_stencil_state ? depth_stencil_state->max_depth_bounds : 1.f;
    if (depth_stencil_attachment) {
      graphics_pipeline_create_info.pDepthStencilState = &depth_stencil_state_info;
    }

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

    graphics_pipeline_create_info.basePipelineIndex = -1;

    VkPipeline vk_pipeline{VK_NULL_HANDLE};
    VK_RETURN_IF_FAILED(VkApi::get()->vkCreateGraphicsPipelines(device->vk_device(), VK_NULL_HANDLE,
                                                                1, &graphics_pipeline_create_info,
                                                                nullptr, &vk_pipeline));

    VLOG(10) << "[create_pipeline] complete";

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

  RenderGraph *render_graph;

  const core::RefCountPtr<Device> device;

  std::vector<VkVertexInputBindingDescription> vertex_input_bindings;
  std::vector<VkVertexInputAttributeDescription> vertex_input_attributes;

  VkPrimitiveTopology topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

  struct ColorAttachment {
    core::RefCountPtr<RenderGraphImage> image;

    AttachmentDescription description;

    std::unique_ptr<VkRect2D> render_area;
  };
  std::unordered_map<int32_t, ColorAttachment> color_attachments;

  struct DepthStencilAttachment {
    int32_t id = -1;

    AttachmentDescription description;

    std::unique_ptr<VkRect2D> render_area;
  };
  std::unique_ptr<DepthStencilAttachment> depth_stencil_attachment;

  std::unique_ptr<VkViewport> viewport;
  std::unique_ptr<VkRect2D> scissor;

  VkCullModeFlagBits cull_mode = VK_CULL_MODE_NONE;
  VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;
  VkFrontFace front_face = VK_FRONT_FACE_COUNTER_CLOCKWISE;

  std::unique_ptr<DepthStencilState> depth_stencil_state;

  std::array<float, 4> blend_constants = {1, 1, 1, 1};

  std::unordered_map<uint32_t, core::RefCountPtr<DescriptorSetLayout>> descriptor_set_layouts;
  std::vector<VkPushConstantRange> push_constants;
  std::unordered_map<VkShaderStageFlagBits, core::RefCountPtr<ShaderModule>> shader_modules;
};

class GraphicsPass : public Pass {
 public:
  GraphicsPass(std::unique_ptr<GraphicsPassBuilderImpl> builder,
               std::function<absl::Status(Context *)> execute_fn)
      : builder_(std::move(builder)), execute_fn_(std::move(execute_fn)) {}

  absl::Status compile(Device *device) override {
    device_.reset(device);

    LANCE_RETURN_IF_FAILED(create_render_pass());

    LANCE_RETURN_IF_FAILED(compute_render_area());

    LANCE_ASSIGN_OR_RETURN(pipeline_layout, builder_->create_pipeline_layout());
    pipeline_layout_ = pipeline_layout;

    LANCE_ASSIGN_OR_RETURN(pipeline, builder_->create_pipeline(pipeline_layout, render_pass_, 0));
    pipeline_ = pipeline;

    return absl::OkStatus();
  }

  absl::Status execute(CommandBuffer *command_buffer) override {
    class GraphicsContext : public Context {
     public:
      GraphicsContext(GraphicsPass *pass, CommandBuffer *command_buffer)
          : pass_(pass), command_buffer_(command_buffer) {}

      CommandBuffer *command_buffer() const override { return command_buffer_; }
      VkPipeline vk_pipeline() const override { return pass_->pipeline_->vk_pipeline(); }
      VkPipelineLayout vk_pipeline_layout() const override {
        return pass_->pipeline_layout_->vk_pipeline_layout();
      }

     private:
      GraphicsPass *pass_ = nullptr;
      CommandBuffer *command_buffer_ = nullptr;
    };

    GraphicsContext ctx(this, command_buffer);

    LANCE_RETURN_IF_FAILED(begin_render_pass(command_buffer->vk_command_buffer()));

    LANCE_RETURN_IF_FAILED(execute_fn_(&ctx));

    VkApi::get()->vkCmdEndRenderPass(command_buffer->vk_command_buffer());

    return absl::OkStatus();
  }

  std::string_view name() const override { return "GraphicsPass"; }

 private:
  absl::Status create_render_pass() {
    attachment_count_ = builder_->color_attachments.size();
    if (builder_->depth_stencil_attachment) {
      attachment_count_ += 1;
    }

    // create render pass
    //
    // prepare attachment descriptions
    std::vector<VkAttachmentDescription> attachment_descriptions;
    attachment_descriptions.resize(attachment_count_);

    for (const auto &pair : builder_->color_attachments) {
      attachment_descriptions[pair.first] = pair.second.description.description;
    }
    if (builder_->depth_stencil_attachment) {
      attachment_descriptions.back() = builder_->depth_stencil_attachment->description.description;
    }

    // prepare color attachment references
    std::vector<VkAttachmentReference> color_attachment_references;

    for (const auto &pair : builder_->color_attachments) {
      VkAttachmentReference t;
      t.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
      t.attachment = pair.first;

      color_attachment_references.push_back(t);
    }

    VkSubpassDescription subpass_description = {};
    subpass_description.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass_description.inputAttachmentCount = 0;
    subpass_description.colorAttachmentCount = color_attachment_references.size();
    subpass_description.pColorAttachments = color_attachment_references.data();
    subpass_description.pResolveAttachments = nullptr;

    VkAttachmentReference depth_stencil_attachment_reference = {};
    if (builder_->depth_stencil_attachment) {
      depth_stencil_attachment_reference.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
      depth_stencil_attachment_reference.attachment = attachment_count_ - 1;
      subpass_description.pDepthStencilAttachment = &depth_stencil_attachment_reference;
    }
    subpass_description.preserveAttachmentCount = 0;

    std::vector<VkSubpassDependency> subpass_dependencies;

    VkRenderPassCreateInfo render_pass_create_info = {};
    render_pass_create_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    render_pass_create_info.attachmentCount = attachment_descriptions.size();
    render_pass_create_info.pAttachments = attachment_descriptions.data();
    render_pass_create_info.subpassCount = 1;
    render_pass_create_info.pSubpasses = &subpass_description;
    render_pass_create_info.dependencyCount = subpass_dependencies.size();
    render_pass_create_info.pDependencies = subpass_dependencies.data();

    VkRenderPass vk_render_pass;
    VK_RETURN_IF_FAILED(VkApi::get()->vkCreateRenderPass(
        device_->vk_device(), &render_pass_create_info, nullptr, &vk_render_pass));

    render_pass_ = core::make_refcounted<RenderPass>(device_, vk_render_pass);

    return absl::OkStatus();
  }

  absl::Status compute_render_area() {
    // 使用到的 images
    std::unique_ptr<VkRect2D> render_area;
    for (const auto &pair : builder_->color_attachments) {
      VkRect2D t;

      // get current color attachment's extent
      if (pair.second.render_area) {
        t = *pair.second.render_area;
      } else {
        // if render area is not config, set it by image's extent
        t.offset.x = 0;
        t.offset.y = 0;

        auto image_extent = pair.second.image->image_extent();
        t.extent.width = image_extent.width;
        t.extent.height = image_extent.height;
      }

      if (render_area) {
        CHECK_EQ(render_area->offset.x, t.offset.x);
        CHECK_EQ(render_area->offset.y, t.offset.y);
        CHECK(render_area->extent.width == t.extent.width);
        CHECK(render_area->extent.height == t.extent.height);
      } else {
        render_area = std::make_unique<VkRect2D>(t);
      }
    }

    CHECK(render_area != nullptr);

    render_area_ = *render_area;

    VLOG(10) << "[compute_render_area] render_area: " << absl::StrFormat("%v", render_area_);

    return absl::OkStatus();
  }

  absl::Status begin_render_pass(VkCommandBuffer vk_command_buffer) {
    std::vector<VkClearValue> clear_values;
    clear_values.resize(attachment_count_);

    for (const auto &pair : builder_->color_attachments) {
      clear_values[pair.first] = pair.second.description.clear_value;
    }

    VLOG(10) << "[begin_render_pass] clear_values: ";

    VkRenderPassBeginInfo render_pass_begin_info = {};
    render_pass_begin_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    render_pass_begin_info.renderPass = render_pass_->vk_render_pass();
    render_pass_begin_info.framebuffer = framebuffer_->vk_framebuffer();
    render_pass_begin_info.renderArea = render_area_;
    render_pass_begin_info.clearValueCount = clear_values.size();
    render_pass_begin_info.pClearValues = clear_values.data();

    VkApi::get()->vkCmdBeginRenderPass(vk_command_buffer, &render_pass_begin_info,
                                       VK_SUBPASS_CONTENTS_INLINE);

    return absl::OkStatus();
  }

  std::unique_ptr<GraphicsPassBuilderImpl> builder_;
  std::function<absl::Status(Context *)> execute_fn_;

  core::RefCountPtr<Device> device_;
  core::RefCountPtr<RenderPass> render_pass_;
  core::RefCountPtr<Framebuffer> framebuffer_;

  int32_t attachment_count_ = 0;

  VkRect2D render_area_;

  core::RefCountPtr<Pipeline> pipeline_;
  core::RefCountPtr<PipelineLayout> pipeline_layout_;
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

  absl::StatusOr<core::RefCountPtr<RenderGraphImage>> create_texture2d(const std::string &name,
                                                                       VkFormat format,
                                                                       VkExtent2D extent) override {
    auto texture2d = core::make_refcounted<RenderGraphTexture2D>(
        static_cast<int32_t>(resources_.size()), format, extent);

    resources_[texture2d->id()] = texture2d;

    VLOG(10) << "create texture2d, name: " << name << ", extent: (" << extent.width << ","
             << extent.height << ")";

    return texture2d;
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
    auto builder = std::make_unique<GraphicsPassBuilderImpl>(this, device_);
    LANCE_RETURN_IF_FAILED(setup_fn(builder.get()));

    auto graphics_pass = std::make_unique<GraphicsPass>(std::move(builder), std::move(execute_fn));

    passes_.push_back(std::move(graphics_pass));

    return absl::OkStatus();
  }

  absl::Status add_pass(const std::string &name, VkPipelineBindPoint bind_point,
                        core::RefCountPtr<IPass> pass) override {
    if (bind_point == VK_PIPELINE_BIND_POINT_GRAPHICS) {
      LANCE_RETURN_IF_FAILED(add_graphics_pass(
          name,
          [pass](GraphicsPassBuilder *builder) -> absl::Status { return pass->setup(builder); },
          [pass](Context *ctx) -> absl::Status { return pass->execute(ctx); }));

    } else if (bind_point == VK_PIPELINE_BIND_POINT_COMPUTE) {
      LANCE_RETURN_IF_FAILED(add_compute_pass(
          name,
          [pass](ComputePassBuilder *builder) -> absl::Status { return pass->setup(builder); },
          [pass](Context *ctx) -> absl::Status { return pass->execute(ctx); }));
    }

    return absl::InvalidArgumentError(absl::StrFormat("unsupported bind point, %d", bind_point));
  }

  absl::Status compile(const CompileOptions *options) override {
    // setup resource
    for (auto &pair : resources_) {
      LANCE_RETURN_IF_FAILED(pair.second->initialize(device_.get()));
    }

    // compile passes
    for (const auto &pass : passes_) {
      LANCE_RETURN_IF_FAILED(pass->compile(device_.get()));
    }

    return absl::OkStatus();
  }

  absl::Status execute(
      CommandBuffer *command_buffer,
      absl::Span<const std::pair<std::string, core::RefCountPtr<RenderGraphResource>>> inputs)
      override {
    for (auto &pass : passes_) {
      VLOG(1) << "[execute] pass: " << pass->name();

      LANCE_RETURN_IF_FAILED(pass->execute(command_buffer));
    }

    return absl::OkStatus();
  }

 private:
  std::unordered_map<int64_t, core::RefCountPtr<RenderGraphResource>> resources_;

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
