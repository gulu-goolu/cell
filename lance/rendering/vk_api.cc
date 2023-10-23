#include "vk_api.h"

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
  shared_library_handle_ = dlopen("libvulkan-1.so", RTLD_NOW | RTLD_LOCAL);
#else
#error "unsupported platform"
#endif
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
}  // namespace rendering
}  // namespace lance
