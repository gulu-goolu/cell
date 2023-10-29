#include "vk_api.h"

#include "absl/strings/str_format.h"
#include "glog/logging.h"

#if defined(__linux__)
#include <dlfcn.h>
#elif defined(_WIN64)
#include <Windows.h>
#endif

namespace lance {
namespace rendering {
const VkApi* VkApi::get() {
  static const VkApi api;
  return &api;
}

absl::StatusOr<std::vector<VkQueueFamilyProperties>>
VkApi::get_physical_device_queue_family_properties(VkPhysicalDevice physical_device) const {
  uint32_t queue_family_count = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(physical_device, &queue_family_count, nullptr);

  std::vector<VkQueueFamilyProperties> props;
  props.resize(queue_family_count);

  vkGetPhysicalDeviceQueueFamilyProperties(physical_device, &queue_family_count, props.data());

  return props;
}

VkApi::VkApi() {
#if defined(__linux__)
  shared_library_handle_ = dlopen("libvulkan.so.1", RTLD_NOW | RTLD_LOCAL);
  CHECK(shared_library_handle_ != nullptr) << "err_msg: " << dlerror();

  const auto get_symbol_by_name = [this](const char* name) {
    return dlsym(shared_library_handle_, name);
  };

#elif defined(_WIN64)
  shared_library_handle_ = LoadLibrary(TEXT("vulkan-1.dll"));
  CHECK(shared_library_handle_ != nullptr);

  const auto get_symbol_by_name = [this](const char* name) -> void* {
    return reinterpret_cast<void*>(
        GetProcAddress(reinterpret_cast<HMODULE>(shared_library_handle_), name));
  };
#else
  const auto get_symbol_by_name = [this](const char*) -> void* { return nullptr; };
#endif

#define VK_API_LOAD(API)                                                \
  do {                                                                  \
    API = reinterpret_cast<decltype(::API)*>(get_symbol_by_name(#API)); \
    CHECK(API != nullptr);                                              \
  } while (false)

  VK_API_LOAD(vkEnumerateInstanceLayerProperties);
  VK_API_LOAD(vkEnumerateInstanceExtensionProperties);

  VK_API_LOAD(vkCreateInstance);
  VK_API_LOAD(vkCreateDevice);
  VK_API_LOAD(vkDestroyDevice);
  VK_API_LOAD(vkAllocateMemory);
  VK_API_LOAD(vkFreeMemory);
  VK_API_LOAD(vkMapMemory);
  VK_API_LOAD(vkUnmapMemory);
  VK_API_LOAD(vkCreateBuffer);
  VK_API_LOAD(vkDestroyBuffer);
  VK_API_LOAD(vkCreateBufferView);
  VK_API_LOAD(vkBindBufferMemory);
  VK_API_LOAD(vkGetBufferMemoryRequirements);
  VK_API_LOAD(vkCreateImage);
  VK_API_LOAD(vkDestroyImage);
  VK_API_LOAD(vkGetImageMemoryRequirements);
  VK_API_LOAD(vkCreateImageView);
  VK_API_LOAD(vkDestroyImageView);
  VK_API_LOAD(vkEnumeratePhysicalDevices);
  VK_API_LOAD(vkGetPhysicalDeviceProperties);
  VK_API_LOAD(vkGetPhysicalDeviceMemoryProperties);
  VK_API_LOAD(vkGetPhysicalDeviceQueueFamilyProperties);
  VK_API_LOAD(vkCreateShaderModule);
  VK_API_LOAD(vkDestroyShaderModule);
  VK_API_LOAD(vkCmdDispatch);
  VK_API_LOAD(vkCmdDraw);
  VK_API_LOAD(vkCmdDrawIndexed);
  VK_API_LOAD(vkCmdDrawIndirect);
  VK_API_LOAD(vkCmdDrawIndexedIndirect);
  VK_API_LOAD(vkCmdDispatchBase);
  VK_API_LOAD(vkCmdDispatchIndirect);
  VK_API_LOAD(vkCmdBindDescriptorSets);
  VK_API_LOAD(vkCmdBindIndexBuffer);
  VK_API_LOAD(vkCmdBindVertexBuffers);
  VK_API_LOAD(vkCmdBindPipeline);
  VK_API_LOAD(vkCreateComputePipelines);
  VK_API_LOAD(vkCreateGraphicsPipelines);
  VK_API_LOAD(vkCreatePipelineLayout);
  VK_API_LOAD(vkCreateDescriptorSetLayout);
  VK_API_LOAD(vkDestroyDescriptorSetLayout);
  VK_API_LOAD(vkCmdPushConstants);
  VK_API_LOAD(vkDestroyPipeline);
  VK_API_LOAD(vkDestroyPipelineLayout);
  VK_API_LOAD(vkCreateCommandPool);
  VK_API_LOAD(vkDestroyCommandPool);
  VK_API_LOAD(vkAllocateCommandBuffers);
  VK_API_LOAD(vkFreeCommandBuffers);
  VK_API_LOAD(vkBeginCommandBuffer);
  VK_API_LOAD(vkEndCommandBuffer);
  VK_API_LOAD(vkGetDeviceQueue);
  VK_API_LOAD(vkQueueSubmit);
  VK_API_LOAD(vkCreateFence);
  VK_API_LOAD(vkDestroyFence);
  VK_API_LOAD(vkWaitForFences);
  VK_API_LOAD(vkCreateFramebuffer);
  VK_API_LOAD(vkDestroyFramebuffer);
  VK_API_LOAD(vkCreateRenderPass);
  VK_API_LOAD(vkDestroyRenderPass);
  VK_API_LOAD(vkCmdBeginRenderPass);
  VK_API_LOAD(vkCmdEndRenderPass);
  VK_API_LOAD(vkCmdNextSubpass);
  VK_API_LOAD(vkCmdSetViewport);
  VK_API_LOAD(vkCmdSetScissor);
  VK_API_LOAD(vkCmdSetBlendConstants);
  VK_API_LOAD(vkCmdClearAttachments);

#undef VK_API_LOAD
}

VkApi::~VkApi() {
#if defined(__linux__)
  if (shared_library_handle_) {
    dlclose(shared_library_handle_);
  }
#elif defined(_WIN64)
  // windows
  if (shared_library_handle_) {
    CloseHandle(shared_library_handle_);
  }
#else
#endif
}

std::string VkResult_name(VkResult ret_code) {
  switch (ret_code) {
#define CASE_TO_STRING(NAME) \
  case NAME: {               \
    return #NAME;            \
  } break

    CASE_TO_STRING(VK_SUCCESS);
    CASE_TO_STRING(VK_NOT_READY);
    CASE_TO_STRING(VK_EVENT_SET);
    CASE_TO_STRING(VK_EVENT_RESET);
    CASE_TO_STRING(VK_INCOMPLETE);
    CASE_TO_STRING(VK_ERROR_OUT_OF_HOST_MEMORY);
    CASE_TO_STRING(VK_ERROR_OUT_OF_DEVICE_MEMORY);
    CASE_TO_STRING(VK_ERROR_INITIALIZATION_FAILED);
    CASE_TO_STRING(VK_ERROR_DEVICE_LOST);
    CASE_TO_STRING(VK_ERROR_MEMORY_MAP_FAILED);
    CASE_TO_STRING(VK_ERROR_LAYER_NOT_PRESENT);
    CASE_TO_STRING(VK_ERROR_EXTENSION_NOT_PRESENT);
    CASE_TO_STRING(VK_ERROR_FEATURE_NOT_PRESENT);
    CASE_TO_STRING(VK_ERROR_INCOMPATIBLE_DRIVER);
    CASE_TO_STRING(VK_ERROR_OUT_OF_POOL_MEMORY);

#undef CASE_TO_STRING

    default: {
      return absl::StrFormat("UnknownError, ret_code: %d", static_cast<int32_t>(ret_code));
    }
  }
}
}  // namespace rendering
}  // namespace lance
