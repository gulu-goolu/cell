#include "device.h"

#include "absl/strings/str_format.h"
#include "lance/core/util.h"
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
  return create({}, enabled_extensions);
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

absl::StatusOr<core::RefCountPtr<Device>> Instance::create_for_graphics() {
  LANCE_ASSIGN_OR_RETURN(physical_devices, enumerate_physical_devices());

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
  queue_create_info.queueFamilyIndex = 1;
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

  return core::make_refcounted<Device>(this, logic_device);
}

Device::Device(core::RefCountPtr<Instance> instance, VkDevice vk_device)
    : instance_(instance), vk_device_(vk_device) {}

Device::~Device() {
  if (vk_device_) {
    VkApi::get()->vkDestroyDevice(vk_device_, nullptr);
  }
}

Image::~Image() {
  if (vk_image_) {
    VkApi::get()->vkDestroyImage(device_->vk_device(), vk_image_, nullptr);
  }
}

ShaderModule::~ShaderModule() {
  if (vk_shader_module_) {
    VkApi::get()->vkDestroyShaderModule(device_->vk_device(), vk_shader_module_, nullptr);
  }
}

}  // namespace rendering
}  // namespace lance
