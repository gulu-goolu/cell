#pragma once

#include "vulkan/vulkan.h"

namespace cell {
namespace rendering {
class VkApi {
 public:
  static const VkApi* get();

#define VK_API_DEFINE(API) decltype(::API)* API = nullptr;

  VK_API_DEFINE(vkCreateInstance);
  VK_API_DEFINE(vkCreateDevice);
  VK_API_DEFINE(vkAllocateMemory);
  VK_API_DEFINE(vkCreateBuffer);
  VK_API_DEFINE(vkCreateBufferView);
  VK_API_DEFINE(vkCreateImage);
  VK_API_DEFINE(vkCreateImageView);

#undef VK_API_DEFINE

 private:
  VkApi();

  ~VkApi();

  void* shared_library_handle_ = nullptr;
};
}  // namespace rendering
}  // namespace cell
