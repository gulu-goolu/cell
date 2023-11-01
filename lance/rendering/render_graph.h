#pragma once

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "device.h"
#include "lance/core/object.h"

namespace lance {
namespace rendering {
class RenderGraphResource : public core::Inherit<RenderGraphResource, core::Object> {
 public:
  virtual absl::Status initialize(Device* device) = 0;

  // unique id of resource
  virtual int32_t id() const = 0;
};

class RenderGraphImage : public core::Inherit<RenderGraphImage, RenderGraphResource> {
 public:
  virtual absl::Status append_image_usage(VkImageUsageFlags flags) = 0;

  virtual VkImageView image_view() const = 0;

  virtual VkExtent3D image_extent() const = 0;
};

class PassBuilder {
 public:
  virtual ~PassBuilder() = default;
};

class ComputePassBuilder : public PassBuilder {
 public:
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

struct VertexInputAttribute {
  uint32_t offset;
  uint32_t location;
  VkFormat format;

  VertexInputAttribute() = default;

  VertexInputAttribute(uint32_t _offset, uint32_t _location, VkFormat _format)
      : offset(_offset), location(_location), format(_format) {}
};

struct AttachmentDescription {
  AttachmentDescription() {
    description = VkAttachmentDescription{
        0,
        VK_FORMAT_UNDEFINED,
        VK_SAMPLE_COUNT_1_BIT,
        VK_ATTACHMENT_LOAD_OP_LOAD,
        VK_ATTACHMENT_STORE_OP_STORE,
        VK_ATTACHMENT_LOAD_OP_DONT_CARE,
        VK_ATTACHMENT_STORE_OP_DONT_CARE,
        VK_IMAGE_LAYOUT_UNDEFINED,
        VK_IMAGE_LAYOUT_UNDEFINED,
    };

    clear_value = {};
  }

  AttachmentDescription& set_format(VkFormat f) {
    description.format = f;

    return *this;
  }

  AttachmentDescription& set_load_op(VkAttachmentLoadOp op) {
    description.loadOp = op;

    return *this;
  }

  AttachmentDescription& clear_to(absl::Span<const float> values);

  AttachmentDescription& set_store_op(VkAttachmentStoreOp op) {
    description.storeOp = op;

    return *this;
  }

  AttachmentDescription& set_initial_layout(VkImageLayout layout) {
    description.initialLayout = layout;

    return *this;
  }

  AttachmentDescription& set_final_layout(VkImageLayout layout) {
    description.finalLayout = layout;

    return *this;
  }

  VkAttachmentDescription description;
  VkClearValue clear_value;
};

struct DepthStencilState {
  VkBool32 depth_test_enable = VK_FALSE;

  VkBool32 depth_write_enable = VK_FALSE;

  VkCompareOp depth_test_op = VK_COMPARE_OP_ALWAYS;

  VkBool32 depth_bounds_test_enable = VK_FALSE;

  VkBool32 stencil_test_enable = VK_FALSE;
  float min_depth_bounds = 0.f;
  float max_depth_bounds = 1.f;

  DepthStencilState& enable_depth_test() {
    depth_test_enable = VK_TRUE;

    return *this;
  }

  DepthStencilState& enable_depth_write() {
    depth_write_enable = VK_TRUE;

    return *this;
  }

  DepthStencilState& set_depth_test_op(VkCompareOp op) {
    depth_test_op = op;

    return *this;
  }

  DepthStencilState& enable_depth_bounds_test() {
    depth_bounds_test_enable = VK_TRUE;

    return *this;
  }

  DepthStencilState& enable_stencil_test(float min_depth_bounds, float max_depth_bounds) {
    stencil_test_enable = VK_TRUE;
    min_depth_bounds = min_depth_bounds;
    max_depth_bounds = max_depth_bounds;

    return *this;
  }
};

class GraphicsPassBuilder : public PassBuilder {
 public:
  virtual GraphicsPassBuilder* set_vertex_binding(uint32_t binding, VkVertexInputRate input_rate,
                                                  uint32_t stride,
                                                  absl::Span<const VertexInputAttribute> attrs) = 0;

  // default: VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST
  virtual GraphicsPassBuilder* set_topology(VkPrimitiveTopology topology) = 0;

  virtual GraphicsPassBuilder* add_color_attachment(core::RefCountPtr<RenderGraphImage> image,
                                                    uint32_t location,
                                                    AttachmentDescription description,
                                                    const VkRect2D* render_arena = nullptr) = 0;

  virtual GraphicsPassBuilder* set_depth_stencil_attachment(
      core::RefCountPtr<RenderGraphImage> image, AttachmentDescription description) = 0;

  virtual GraphicsPassBuilder* set_viewport(float x, float y, float width, float height,
                                            float min_depth = 0, float max_depth = 1) = 0;

  virtual GraphicsPassBuilder* set_cull_mode(VkCullModeFlagBits cull_mode) = 0;
  virtual GraphicsPassBuilder* set_front_face(VkFrontFace front_face) = 0;
  virtual GraphicsPassBuilder* set_polygon_mode(VkPolygonMode mode) = 0;

  virtual GraphicsPassBuilder* set_blend_constants(absl::Span<const float> values) = 0;

  // default is disable
  virtual GraphicsPassBuilder* set_depth_stencil_state(DepthStencilState state) = 0;

  // resources
  virtual GraphicsPassBuilder* add_descriptor_set(
      uint32_t set, core::RefCountPtr<DescriptorSetLayout> layout) = 0;

  virtual GraphicsPassBuilder* add_push_constants(VkShaderStageFlags stage_flags, uint32_t offset,
                                                  uint32_t size) = 0;

  virtual GraphicsPassBuilder* set_shader(VkShaderStageFlagBits stage,
                                          const core::RefCountPtr<ShaderModule>& shader_module) = 0;

  virtual GraphicsPassBuilder* set_shader_by_glsl(VkShaderStageFlagBits stage,
                                                  const char* source) = 0;
};

class Context {
 public:
  virtual ~Context() = default;

  virtual VkCommandBuffer vk_command_buffer() const = 0;
  virtual VkPipeline vk_pipeline() const = 0;
  virtual VkPipelineLayout vk_pipeline_layout() const = 0;

  void push_constants(VkShaderStageFlags stage, uint32_t offset, uint32_t size, const void* values);

  void dispatch(uint32_t group_count_x, uint32_t group_count_y, uint32_t group_count_z);

  void draw(uint32_t vertex_count, uint32_t instance_count, uint32_t first_vertex,
            uint32_t first_instance);

  void set_viewport(uint32_t first_viewport, absl::Span<const VkViewport> viewports);

  void set_scissors(uint32_t first_scissor, absl::Span<const VkRect2D> scissors);
};

class IPass : public core::Inherit<IPass, core::Object> {
 public:
  virtual absl::Status setup(PassBuilder* builder) = 0;

  virtual absl::Status execute(Context* ctx) = 0;
};

class RenderGraph : public core::Inherit<RenderGraph, core::Object> {
 public:
  virtual absl::StatusOr<int32_t> import_resource(
      const std::string& name, const core::RefCountPtr<RenderGraphResource>& resource) = 0;

  virtual absl::StatusOr<int32_t> create_resource(const std::string& name) = 0;
  virtual absl::StatusOr<core::RefCountPtr<RenderGraphImage>> create_texture2d(
      const std::string& name, VkFormat format, VkExtent2D extent) = 0;

  virtual absl::StatusOr<std::string> create_attachment(VkImageType image_type, VkFormat format,
                                                        VkImageUsageFlags usage,
                                                        VkExtent3D extent) = 0;

  virtual absl::Status add_compute_pass(std::string name,
                                        std::function<absl::Status(ComputePassBuilder*)> setup_fn,
                                        std::function<absl::Status(Context* ctx)> execute_fn) = 0;

  virtual absl::Status add_graphics_pass(std::string_view name,
                                         std::function<absl::Status(GraphicsPassBuilder*)> setup_fn,
                                         std::function<absl::Status(Context*)> execute_fn) = 0;

  virtual absl::Status add_pass(const std::string& name, VkPipelineBindPoint bind_point,
                                core::RefCountPtr<IPass> pass) = 0;

  struct CompileOptions {
    bool enable_pass_fusion = true;
  };

  virtual absl::Status compile(const CompileOptions* options = nullptr) = 0;

  virtual absl::Status execute(
      VkCommandBuffer command_buffer,
      absl::Span<const std::pair<std::string, core::RefCountPtr<RenderGraphResource>>> inputs) = 0;
};

absl::StatusOr<core::RefCountPtr<RenderGraph>> create_render_graph(
    core::RefCountPtr<Device> device);

}  // namespace rendering
}  // namespace lance
