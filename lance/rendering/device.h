#pragma once

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "lance/core/object.h"
#include "lance/core/util.h"
#include "vulkan/vulkan_core.h"

namespace lance {
namespace rendering {
class Device;

class Instance : public core::Inherit<Instance, core::Object> {
 public:
  // create instance
  static absl::StatusOr<core::RefCountPtr<Instance>> create(absl::Span<const char*> layers,
                                                            absl::Span<const char*> extensions);

  // create a instance for 3d rendering
  static absl::StatusOr<core::RefCountPtr<Instance>> create_for_3d();

  VkInstance vk_instance() const { return vk_instance_; }

  absl::StatusOr<std::vector<VkPhysicalDevice>> enumerate_physical_devices() const;

  absl::StatusOr<core::RefCountPtr<Device>> create_device(absl::Span<const char*> extensions);

  absl::StatusOr<core::RefCountPtr<Device>> create_for_graphics();

 private:
  VkInstance vk_instance_{VK_NULL_HANDLE};
};

class Device : public core::Inherit<Device, core::Object> {
 private:
  explicit Device(VkDevice vk_device, core::RefCountPtr<Instance> instance);

  VkDevice vk_device() const { return vk_device_; }

 public:
  core::RefCountPtr<Instance> instance_;

  VkDevice vk_device_{VK_NULL_HANDLE};
};
}  // namespace rendering
}  // namespace lance
