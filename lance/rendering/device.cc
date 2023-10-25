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

absl::StatusOr<core::RefCountPtr<Device>> Instance::create_for_graphics() { return nullptr; }

Device::Device(VkDevice vk_device, core::RefCountPtr<Instance> instance)
    : vk_device_(vk_device), instance_(instance) {}

}  // namespace rendering
}  // namespace lance
