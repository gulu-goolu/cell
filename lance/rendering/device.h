#pragma once

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "lance/core/object.h"
#include "lance/core/util.h"
#include "lance/rendering/vk_api.h"

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

 private:
  VkInstance vk_instance_{VK_NULL_HANDLE};
};

class Device : public core::Inherit<Device, core::Object> {
 private:
  VkDevice vk_device() const { return vk_device_; }

 public:
  VkDevice vk_device_{VK_NULL_HANDLE};
};
}  // namespace rendering
}  // namespace lance
