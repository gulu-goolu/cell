#pragma once

#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
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
  VK_API_DEFINE(vkFreeMemory);
  VK_API_DEFINE(vkMapMemory);
  VK_API_DEFINE(vkUnmapMemory);
  VK_API_DEFINE(vkCreateBuffer);
  VK_API_DEFINE(vkDestroyBuffer);
  VK_API_DEFINE(vkCreateBufferView);
  VK_API_DEFINE(vkBindBufferMemory);
  VK_API_DEFINE(vkGetBufferMemoryRequirements);
  VK_API_DEFINE(vkCreateImage);
  VK_API_DEFINE(vkDestroyImage);
  VK_API_DEFINE(vkGetImageMemoryRequirements);
  VK_API_DEFINE(vkCreateImageView);
  VK_API_DEFINE(vkDestroyImageView);
  VK_API_DEFINE(vkEnumeratePhysicalDevices);
  VK_API_DEFINE(vkGetPhysicalDeviceProperties);
  VK_API_DEFINE(vkGetPhysicalDeviceMemoryProperties);
  VK_API_DEFINE(vkGetPhysicalDeviceQueueFamilyProperties);
  VK_API_DEFINE(vkCreateShaderModule);
  VK_API_DEFINE(vkDestroyShaderModule);
  VK_API_DEFINE(vkCmdDispatch);
  VK_API_DEFINE(vkCmdDraw);
  VK_API_DEFINE(vkCmdDrawIndexed);
  VK_API_DEFINE(vkCmdDrawIndirect);
  VK_API_DEFINE(vkCmdDrawIndexedIndirect);
  VK_API_DEFINE(vkCmdDispatchBase);
  VK_API_DEFINE(vkCmdDispatchIndirect);
  VK_API_DEFINE(vkCmdBindDescriptorSets);
  VK_API_DEFINE(vkCmdBindIndexBuffer);
  VK_API_DEFINE(vkCmdBindVertexBuffers);
  VK_API_DEFINE(vkCmdBindPipeline);
  VK_API_DEFINE(vkCreateComputePipelines);
  VK_API_DEFINE(vkCreateGraphicsPipelines);
  VK_API_DEFINE(vkCreatePipelineLayout);
  VK_API_DEFINE(vkCreateDescriptorSetLayout);
  VK_API_DEFINE(vkDestroyDescriptorSetLayout);
  VK_API_DEFINE(vkCmdPushConstants);
  VK_API_DEFINE(vkDestroyPipeline);
  VK_API_DEFINE(vkDestroyPipelineLayout);
  VK_API_DEFINE(vkCreateCommandPool);
  VK_API_DEFINE(vkDestroyCommandPool);
  VK_API_DEFINE(vkAllocateCommandBuffers);
  VK_API_DEFINE(vkFreeCommandBuffers);
  VK_API_DEFINE(vkBeginCommandBuffer);
  VK_API_DEFINE(vkEndCommandBuffer);
  VK_API_DEFINE(vkGetDeviceQueue);
  VK_API_DEFINE(vkQueueSubmit);
  VK_API_DEFINE(vkCreateFence);
  VK_API_DEFINE(vkDestroyFence);
  VK_API_DEFINE(vkWaitForFences);
  VK_API_DEFINE(vkCreateFramebuffer);
  VK_API_DEFINE(vkDestroyFramebuffer);
  VK_API_DEFINE(vkCreateRenderPass);
  VK_API_DEFINE(vkDestroyRenderPass);
  VK_API_DEFINE(vkCmdBeginRenderPass);
  VK_API_DEFINE(vkCmdEndRenderPass);
  VK_API_DEFINE(vkCmdNextSubpass);
  VK_API_DEFINE(vkCmdSetViewport);
  VK_API_DEFINE(vkCmdSetScissor);
  VK_API_DEFINE(vkCmdSetBlendConstants);
  VK_API_DEFINE(vkCmdClearAttachments);
  VK_API_DEFINE(vkBindImageMemory);
  VK_API_DEFINE(vkCmdCopyBuffer);
  VK_API_DEFINE(vkCmdCopyImage);
  VK_API_DEFINE(vkCmdCopyBufferToImage);
  VK_API_DEFINE(vkCmdCopyImageToBuffer);
  VK_API_DEFINE(vkCreateEvent);
  VK_API_DEFINE(vkDestroyEvent);
  VK_API_DEFINE(vkCmdSetEvent);
  VK_API_DEFINE(vkCmdWaitEvents);
  VK_API_DEFINE(vkResetEvent);
  VK_API_DEFINE(vkAllocateDescriptorSets);
  VK_API_DEFINE(vkFreeDescriptorSets);
  VK_API_DEFINE(vkUpdateDescriptorSets);
  VK_API_DEFINE(vkDestroyDescriptorPool);
  VK_API_DEFINE(vkCreateDescriptorPool);
  VK_API_DEFINE(vkCmdUpdateBuffer);
  VK_API_DEFINE(vkCmdPipelineBarrier);

#undef VK_API_DEFINE

  absl::StatusOr<std::vector<VkQueueFamilyProperties>> get_physical_device_queue_family_properties(
      VkPhysicalDevice physical_device) const;

 private:
  VkApi();

  ~VkApi();

  void* shared_library_handle_ = nullptr;
};

std::string VkResult_name(VkResult ret_code);

std::string VkFormat_name(VkFormat f);

std::string VkVertexInputRate_name(VkVertexInputRate rate);

bool is_depth_stencil_supported(VkFormat f);

}  // namespace rendering
}  // namespace lance

template <typename Sink>
inline void AbslStringify(Sink& sink, const VkRect2D& t) {
  absl::Format(&sink, "<VkRect2D offset: (%d,%d), extent: (%d,%d)>", t.offset.x, t.offset.y,
               t.extent.width, t.extent.height);
}

template <typename Sink>
inline void AbslStringify(Sink& sink, const VkVertexInputBindingDescription& v) {
  return absl::StrFormat(&sink, "<binding:%d,stride:%d,input_rate:%s>", v.binding, v.stride,
                         lance::rendering::VkVertexInputRate_name(v.inputRate));
}

#define VK_RETURN_IF_FAILED(EXPR)                                                                \
  do {                                                                                           \
    VkResult ret = (EXPR);                                                                       \
    if (ret != VK_SUCCESS) {                                                                     \
      return absl::UnknownError(                                                                 \
          absl::StrFormat("%s failed, ret: %s", #EXPR, ::lance::rendering::VkResult_name(ret))); \
    }                                                                                            \
  } while (false);
