#pragma once

#include "lance/core/object.h"
#include "lance/rendering/vk_api.h"

namespace lance {
namespace rendering {
class Device;

class Instance : public core::Inherit<Instance, core::Object> {
 public:
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
