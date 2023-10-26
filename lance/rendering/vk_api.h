#pragma once

#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "vulkan/vulkan_core.h"

namespace lance {
namespace rendering {
class VkApi {
 public:
  static const VkApi* get();

#define VK_API_DEFINE(API) decltype(::API)* API = nullptr;

  VK_API_DEFINE(vkEnumerateInstanceLayerProperties);
  VK_API_DEFINE(vkEnumerateInstanceExtensionProperties);

  VK_API_DEFINE(vkCreateInstance);
  VK_API_DEFINE(vkCreateDevice);
  VK_API_DEFINE(vkDestroyDevice);
  VK_API_DEFINE(vkAllocateMemory);
  VK_API_DEFINE(vkCreateBuffer);
  VK_API_DEFINE(vkCreateBufferView);
  VK_API_DEFINE(vkCreateImage);
  VK_API_DEFINE(vkDestroyImage);
  VK_API_DEFINE(vkCreateImageView);
  VK_API_DEFINE(vkEnumeratePhysicalDevices);
  VK_API_DEFINE(vkGetPhysicalDeviceProperties);
  VK_API_DEFINE(vkGetPhysicalDeviceQueueFamilyProperties);
  VK_API_DEFINE(vkCreateShaderModule);
  VK_API_DEFINE(vkDestroyShaderModule);

#undef VK_API_DEFINE

  absl::StatusOr<std::vector<VkQueueFamilyProperties>> get_physical_device_queue_family_properties(
      VkPhysicalDevice physical_device) const;

 private:
  VkApi();

  ~VkApi();

  void* shared_library_handle_ = nullptr;
};

std::string VkResult_name(VkResult ret_code);
}  // namespace rendering
}  // namespace lance

#define VK_RETURN_IF_FAILED(EXPR)                                                                \
  do {                                                                                           \
    VkResult ret = (EXPR);                                                                       \
    if (ret != VK_SUCCESS) {                                                                     \
      return absl::UnknownError(                                                                 \
          absl::StrFormat("%s failed, ret: %s", #EXPR, ::lance::rendering::VkResult_name(ret))); \
    }                                                                                            \
  } while (false);
