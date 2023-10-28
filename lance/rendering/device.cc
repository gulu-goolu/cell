#include "device.h"

#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "glog/logging.h"
#include "lance/core/util.h"
#include "shader_compiler.h"
#include "vk_api.h"

namespace lance {
namespace rendering {
absl::StatusOr<core::RefCountPtr<Instance>> Instance::create(absl::Span<const char *> layers,
                                                             absl::Span<const char *> extensions) {
  VkApplicationInfo application_info = {};
  application_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  application_info.apiVersion = VK_API_VERSION_1_3;

  VkInstanceCreateInfo instance_create_info = {};
  instance_create_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  instance_create_info.enabledLayerCount = layers.size();
  instance_create_info.ppEnabledLayerNames = layers.data();
  instance_create_info.enabledExtensionCount = extensions.size();
  instance_create_info.ppEnabledExtensionNames = extensions.data();
  instance_create_info.pApplicationInfo = &application_info;

  VkInstance vk_instance;
  VkResult ret_code = VkApi::get()->vkCreateInstance(&instance_create_info, nullptr, &vk_instance);
  if (ret_code != VK_SUCCESS) {
    return absl::UnknownError(
        absl::StrFormat("failed to create instance, ret_code: %s", VkResult_name(ret_code)));
  }

  auto result = core::make_refcounted<Instance>();
  result->vk_instance_ = vk_instance;

  return result;
}

absl::StatusOr<core::RefCountPtr<Instance>> Instance::create_for_3d() {
  const char *enabled_extensions[] = {
    "VK_KHR_surface",

// for windows platform
#if defined(_WIN64)
    "VK_KHR_win32_surface"
#endif
  };

  const char *enabled_layers[] = {
      "VK_LAYER_KHRONOS_validation",
  };

  return create(enabled_layers, enabled_extensions);
}

absl::StatusOr<std::vector<VkPhysicalDevice>> Instance::enumerate_physical_devices() const {
  uint32_t num_physical_device = 0;
  VK_RETURN_IF_FAILED(
      VkApi::get()->vkEnumeratePhysicalDevices(vk_instance_, &num_physical_device, nullptr));

  std::vector<VkPhysicalDevice> physical_devices;
  physical_devices.resize(num_physical_device);

  VK_RETURN_IF_FAILED(VkApi::get()->vkEnumeratePhysicalDevices(vk_instance_, &num_physical_device,
                                                               physical_devices.data()));

  return physical_devices;
}

absl::StatusOr<core::RefCountPtr<Device>> Instance::create_device(
    absl::Span<const char *> extensions) {
  return absl::UnknownError("failed to create device");
}

absl::StatusOr<core::RefCountPtr<Device>> Instance::create_device_for_graphics() {
  LANCE_ASSIGN_OR_RETURN(physical_devices, enumerate_physical_devices());

  std::sort(physical_devices.begin(), physical_devices.end(),
            [](VkPhysicalDevice l, VkPhysicalDevice r) {
              VkPhysicalDeviceProperties props1, props2;
              VkApi::get()->vkGetPhysicalDeviceProperties(l, &props1);
              VkApi::get()->vkGetPhysicalDeviceProperties(r, &props2);

              const static std::unordered_map<VkPhysicalDeviceType, int32_t> m = {
                  {VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU, 1},
                  {VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU, 2},
              };
              return m.at(props1.deviceType) > m.at(props2.deviceType);
            });

  VkPhysicalDevice target_device = VK_NULL_HANDLE;
  uint32_t graphics_queue_famil_index = UINT32_MAX;
  for (auto physical_device : physical_devices) {
    LANCE_ASSIGN_OR_RETURN(
        queue_family_props,
        VkApi::get()->get_physical_device_queue_family_properties(physical_device));

    for (uint32_t idx = 0; idx < queue_family_props.size(); ++idx) {
      if (queue_family_props[idx].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
        graphics_queue_famil_index = idx;
        break;
      }
    }

    if (graphics_queue_famil_index != UINT32_MAX) {
      target_device = physical_device;
      break;
    }
  }

  const char *extensions[] = {
      VK_KHR_SWAPCHAIN_EXTENSION_NAME,
  };

  const float queue_priorities[] = {1};

  VkDeviceQueueCreateInfo queue_create_info = {};
  queue_create_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
  queue_create_info.queueFamilyIndex = graphics_queue_famil_index;
  queue_create_info.queueCount = 1;
  queue_create_info.pQueuePriorities = queue_priorities;

  VkDeviceCreateInfo device_create_info = {};
  device_create_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  device_create_info.enabledExtensionCount = std::size(extensions);
  device_create_info.ppEnabledExtensionNames = extensions;
  device_create_info.queueCreateInfoCount = 1;
  device_create_info.pQueueCreateInfos = &queue_create_info;

  VkDevice logic_device = VK_NULL_HANDLE;
  VkResult ret_code =
      VkApi::get()->vkCreateDevice(target_device, &device_create_info, nullptr, &logic_device);
  if (ret_code != VK_SUCCESS) {
    return absl::UnknownError(
        absl::StrFormat("failed to create logic device, ret_code: %s", VkResult_name(ret_code)));
  }

  return core::make_refcounted<Device>(this, target_device, logic_device,
                                       absl::MakeConstSpan(&graphics_queue_famil_index, 1));
}

Device::Device(core::RefCountPtr<Instance> instance, VkPhysicalDevice vk_physical_device,
               VkDevice vk_device, absl::Span<const uint32_t> queue_family_indices)
    : instance_(instance),
      vk_physical_device_(vk_physical_device),
      vk_device_(vk_device),
      queue_family_indices_(queue_family_indices.begin(), queue_family_indices.end()) {
  VkPhysicalDeviceProperties properties;
  VkApi::get()->vkGetPhysicalDeviceProperties(vk_physical_device, &properties);

  LOG(INFO) << "queue_family_indices: [" << absl::StrJoin(queue_family_indices, ",")
            << "], device_name: " << properties.deviceName;
}

Device::~Device() {
  if (vk_device_) {
    VkApi::get()->vkDestroyDevice(vk_device_, nullptr);
  }
}

absl::StatusOr<core::RefCountPtr<ShaderModule>> Device::create_shader_module(
    const core::Blob *blob) {
  VkShaderModuleCreateInfo shader_module_create_info = {};
  shader_module_create_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  shader_module_create_info.codeSize = blob->size();
  shader_module_create_info.pCode = reinterpret_cast<const uint32_t *>(blob->data());

  VkShaderModule vk_shader_module{VK_NULL_HANDLE};
  VK_RETURN_IF_FAILED(VkApi::get()->vkCreateShaderModule(vk_device_, &shader_module_create_info,
                                                         nullptr, &vk_shader_module));

  return core::make_refcounted<ShaderModule>(this, vk_shader_module);
}

absl::StatusOr<core::RefCountPtr<ShaderModule>> Device::create_shader_from_source(
    VkShaderStageFlagBits stage, const char *source) {
  static const std::unordered_map<VkShaderStageFlagBits, glslang_stage_t> m = {
      {VK_SHADER_STAGE_VERTEX_BIT, glslang_stage_t::GLSLANG_STAGE_VERTEX},
  };
  LANCE_ASSIGN_OR_RETURN(blob, compile_glsl_shader(source, m.at(stage)));

  return create_shader_module(blob.get());
}

absl::StatusOr<uint32_t> Device::find_queue_family_index(VkQueueFlags flags) const {
  LANCE_ASSIGN_OR_RETURN(
      queue_family_props,
      VkApi::get()->get_physical_device_queue_family_properties(vk_physical_device_));

  for (uint32_t i : queue_family_indices_) {
    if ((queue_family_props[i].queueFlags & flags) == flags) {
      return i;
    }
  }

  return absl::NotFoundError("no suitable queue found");
}

absl::Status Device::submit(uint32_t queue_family_index,
                            absl::Span<const VkCommandBuffer> vk_command_buffers) {
  VkFenceCreateInfo fence_create_info = {};
  fence_create_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;

  VkFence vk_fence{VK_NULL_HANDLE};
  VK_RETURN_IF_FAILED(
      VkApi::get()->vkCreateFence(vk_device_, &fence_create_info, nullptr, &vk_fence));

  LANCE_ON_SCOPE_EXIT([&]() { VkApi::get()->vkDestroyFence(vk_device_, vk_fence, nullptr); });

  VkQueue vk_queue{VK_NULL_HANDLE};
  VkApi::get()->vkGetDeviceQueue(vk_device_, queue_family_index, 0, &vk_queue);

  VkSubmitInfo submit_info = {};
  submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submit_info.commandBufferCount = vk_command_buffers.size();
  submit_info.pCommandBuffers = vk_command_buffers.data();
  VK_RETURN_IF_FAILED(VkApi::get()->vkQueueSubmit(vk_queue, 1, &submit_info, vk_fence));

  VK_RETURN_IF_FAILED(VkApi::get()->vkWaitForFences(vk_device_, 1, &vk_fence, VK_TRUE, UINT64_MAX));

  return absl::OkStatus();
}

absl::StatusOr<uint32_t> Device::find_memory_type_index(uint32_t type_bits,
                                                        VkMemoryPropertyFlags flags) const {
  VkPhysicalDeviceMemoryProperties memory_properties;
  VkApi::get()->vkGetPhysicalDeviceMemoryProperties(vk_physical_device_, &memory_properties);

  for (uint32_t i = 0; i < memory_properties.memoryTypeCount; ++i) {
    if (((1 << i) & type_bits) &&
        ((memory_properties.memoryTypes[i].propertyFlags & flags) == flags)) {
      return i;
    }
  }

  return absl::NotFoundError(absl::StrFormat("no suitable memory found, flags: %d", type_bits));
}

absl::StatusOr<core::RefCountPtr<DeviceMemory>> DeviceMemory::create(
    const core::RefCountPtr<Device> &device, uint32_t memory_type_index, size_t allocation_size) {
  VkDeviceMemory vk_device_memory{VK_NULL_HANDLE};

  VkMemoryAllocateInfo memory_allocate_info = {};
  memory_allocate_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  memory_allocate_info.allocationSize = allocation_size;
  memory_allocate_info.memoryTypeIndex = memory_type_index;
  VK_RETURN_IF_FAILED(VkApi::get()->vkAllocateMemory(device->vk_device(), &memory_allocate_info,
                                                     nullptr, &vk_device_memory));

  return core::make_refcounted<DeviceMemory>(device, vk_device_memory);
}

DeviceMemory::~DeviceMemory() {
  if (vk_device_memory_) {
    VkApi::get()->vkFreeMemory(device_->vk_device(), vk_device_memory_, nullptr);
  }
}

absl::StatusOr<void *> DeviceMemory::map(size_t offset, size_t size) {
  void *addr = nullptr;
  VK_RETURN_IF_FAILED(
      VkApi::get()->vkMapMemory(device_->vk_device(), vk_device_memory_, offset, size, 0, &addr));

  return addr;
}

absl::Status DeviceMemory::unmap() {
  VkApi::get()->vkUnmapMemory(device_->vk_device(), vk_device_memory_);

  return absl::OkStatus();
}

Buffer::~Buffer() {
  if (vk_buffer_) {
    VkApi::get()->vkDestroyBuffer(device_->vk_device(), vk_buffer_, nullptr);
  }
}

VkMemoryRequirements Buffer::memory_requirements() const {
  VkMemoryRequirements memory_requirements;
  VkApi::get()->vkGetBufferMemoryRequirements(device_->vk_device(), vk_buffer_,
                                              &memory_requirements);

  return memory_requirements;
}

Image::~Image() {
  if (vk_image_) {
    VkApi::get()->vkDestroyImage(device_->vk_device(), vk_image_, nullptr);
  }
}

VkMemoryRequirements Image::memory_requirements() const {
  VkMemoryRequirements memory_requirements;
  VkApi::get()->vkGetImageMemoryRequirements(device_->vk_device(), vk_image_, &memory_requirements);

  return memory_requirements;
}

ImageView::~ImageView() {
  if (vk_image_view_) {
    VkApi::get()->vkDestroyImageView(device_->vk_device(), vk_image_view_, nullptr);
  }
}

ShaderModule::~ShaderModule() {
  if (vk_shader_module_) {
    VkApi::get()->vkDestroyShaderModule(device_->vk_device(), vk_shader_module_, nullptr);
  }
}

absl::StatusOr<core::RefCountPtr<DescriptorSetLayout>> DescriptorSetLayout::create(
    const core::RefCountPtr<Device> &device,
    absl::Span<const VkDescriptorSetLayoutBinding> bindings) {
  VkDescriptorSetLayoutCreateInfo descriptor_set_layout_create_info = {};
  descriptor_set_layout_create_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptor_set_layout_create_info.bindingCount = bindings.size();
  descriptor_set_layout_create_info.pBindings = bindings.data();

  VkDescriptorSetLayout vk_descriptor_set_layout;
  VK_RETURN_IF_FAILED(VkApi::get()->vkCreateDescriptorSetLayout(
      device->vk_device(), &descriptor_set_layout_create_info, nullptr, &vk_descriptor_set_layout));

  return core::make_refcounted<DescriptorSetLayout>(device, vk_descriptor_set_layout);
}

absl::StatusOr<core::Ref<DescriptorSetLayout>> DescriptorSetLayout::create_for_single_descriptor(
    const core::Ref<Device> &device, VkDescriptorType type, VkShaderStageFlags stage) {
  VkDescriptorSetLayoutBinding binding = {};
  binding.binding = 0;
  binding.descriptorCount = 1;
  binding.descriptorType = type;
  binding.stageFlags = stage;
  return create(device, {binding});
}

DescriptorSetLayout::~DescriptorSetLayout() {
  if (vk_descriptor_set_layout_) {
    VkApi::get()->vkDestroyDescriptorSetLayout(device_->vk_device(), vk_descriptor_set_layout_,
                                               nullptr);
  }
}

PipelineLayout::~PipelineLayout() {
  if (vk_pipeline_layout_) {
    VkApi::get()->vkDestroyPipelineLayout(device_->vk_device(), vk_pipeline_layout_, nullptr);
  }
}

Pipeline::~Pipeline() {
  if (vk_pipeline_) {
    VkApi::get()->vkDestroyPipeline(device_->vk_device(), vk_pipeline_, nullptr);
  }
}

absl::StatusOr<core::RefCountPtr<CommandPool>> CommandPool::create(
    const core::RefCountPtr<Device> &device, uint32_t queue_family_index) {
  VkCommandPoolCreateInfo command_pool_create_info = {};
  command_pool_create_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
  command_pool_create_info.queueFamilyIndex = queue_family_index;

  VkCommandPool vk_command_pool{VK_NULL_HANDLE};
  VK_RETURN_IF_FAILED(VkApi::get()->vkCreateCommandPool(
      device->vk_device(), &command_pool_create_info, nullptr, &vk_command_pool));

  return core::make_refcounted<CommandPool>(device, vk_command_pool);
}

CommandPool::~CommandPool() {
  if (vk_command_pool_) {
    VkApi::get()->vkDestroyCommandPool(device_->vk_device(), vk_command_pool_, nullptr);
  }
}

absl::StatusOr<core::RefCountPtr<CommandBuffer>> CommandPool::allocate_command_buffer(
    VkCommandBufferLevel level) {
  VkCommandBufferAllocateInfo command_buffer_allocate_info = {};
  command_buffer_allocate_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  command_buffer_allocate_info.commandPool = vk_command_pool_;
  command_buffer_allocate_info.commandBufferCount = 1;
  command_buffer_allocate_info.level = level;

  VkCommandBuffer vk_command_buffer{VK_NULL_HANDLE};
  VK_RETURN_IF_FAILED(VkApi::get()->vkAllocateCommandBuffers(
      device_->vk_device(), &command_buffer_allocate_info, &vk_command_buffer));

  return core::make_refcounted<CommandBuffer>(this, vk_command_buffer);
}

CommandBuffer::~CommandBuffer() {
  if (vk_command_buffer_) {
    VkApi::get()->vkFreeCommandBuffers(command_pool_->device()->vk_device(),
                                       command_pool_->vk_command_pool(), 1, &vk_command_buffer_);
  }
}

absl::Status CommandBuffer::begin() {
  VkCommandBufferBeginInfo command_buffer_begin_info = {};
  command_buffer_begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  VK_RETURN_IF_FAILED(
      VkApi::get()->vkBeginCommandBuffer(vk_command_buffer_, &command_buffer_begin_info));

  return absl::OkStatus();
}

absl::Status CommandBuffer::end() {
  VK_RETURN_IF_FAILED(VkApi::get()->vkEndCommandBuffer(vk_command_buffer_));

  return absl::OkStatus();
}

Framebuffer::~Framebuffer() {
  if (vk_framebuffer_) {
    VkApi::get()->vkDestroyFramebuffer(device_->vk_device(), vk_framebuffer_, nullptr);
  }
}

RenderPass::~RenderPass() {
  if (vk_render_pass_) {
    VkApi::get()->vkDestroyRenderPass(device_->vk_device(), vk_render_pass_, nullptr);
  }
}
}  // namespace rendering
}  // namespace lance
