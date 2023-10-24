#include "device.h"

#include "absl/strings/str_format.h"
#include "lance/core/util.h"

namespace lance {
namespace rendering {
absl::StatusOr<core::RefCountPtr<Instance>> Instance::create(absl::Span<const char *> layers,
                                                             absl::Span<const char *> extensions) {
  VkInstanceCreateInfo instance_create_info = {};
  instance_create_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  instance_create_info.enabledLayerCount = layers.size();
  instance_create_info.ppEnabledLayerNames = layers.data();
  instance_create_info.enabledExtensionCount = extensions.size();
  instance_create_info.ppEnabledExtensionNames = extensions.data();

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
  const char *enabled_extensions[] = {"VK_KHR_swapchain"};
  return create({}, enabled_extensions);
}

}  // namespace rendering
}  // namespace lance
