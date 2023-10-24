#include "vk_api.h"

#include "absl/strings/str_format.h"
#include "glog/logging.h"

#if defined(__linux__)
#include <dlfcn.h>
#endif

namespace lance {
namespace rendering {
const VkApi* VkApi::get() {
  static const VkApi api;
  return &api;
}

VkApi::VkApi() {
#if defined(__linux__)
  shared_library_handle_ = dlopen("libvulkan.so.1", RTLD_NOW | RTLD_LOCAL);
  CHECK(shared_library_handle_ != nullptr) << "err_msg: " << dlerror();
#else
#error "unsupported platform"
#endif

  const auto get_symbol_by_name = [this](const char* name) {
    return dlsym(shared_library_handle_, name);
  };

#define VK_API_LOAD(API)                                                \
  do {                                                                  \
    API = reinterpret_cast<decltype(::API)*>(get_symbol_by_name(#API)); \
    CHECK(API != nullptr) << dlerror();                                 \
  } while (false)

  VK_API_LOAD(vkEnumerateInstanceLayerProperties);
  VK_API_LOAD(vkEnumerateInstanceExtensionProperties);

  VK_API_LOAD(vkCreateInstance);
  VK_API_LOAD(vkCreateDevice);
  VK_API_LOAD(vkAllocateMemory);
  VK_API_LOAD(vkCreateBuffer);
  VK_API_LOAD(vkCreateBufferView);
  VK_API_LOAD(vkCreateImage);
  VK_API_LOAD(vkCreateImageView);

#undef VK_API_LOAD
}

VkApi::~VkApi() {
#if defined(__linux__)
  if (shared_library_handle_) {
    dlclose(shared_library_handle_);
  }
#else
#error "unsupported platform"
#endif
}

std::string VkResult_name(VkResult ret_code) {
  switch (ret_code) {
#define CASE_TO_STRING(NAME) \
  case NAME: {               \
    return #NAME;            \
  } break

    CASE_TO_STRING(VK_ERROR_OUT_OF_POOL_MEMORY);

#undef CASE_TO_STRING

    default: {
      return absl::StrFormat("UnknownError, ret_code: %d", static_cast<int32_t>(ret_code));
    }
  }
}
}  // namespace rendering
}  // namespace lance
