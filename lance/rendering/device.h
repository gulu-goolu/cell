#pragma once

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "lance/core/object.h"
#include "lance/core/util.h"
#include "vulkan/vulkan_core.h"

namespace lance {
namespace rendering {
class Device;
class ShaderModule;

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

  absl::StatusOr<core::RefCountPtr<Device>> create_device_for_graphics();

 private:
  VkInstance vk_instance_{VK_NULL_HANDLE};
};

class Device : public core::Inherit<Device, core::Object> {
 public:
  explicit Device(core::RefCountPtr<Instance> instance, VkDevice vk_device);

  ~Device();

  VkDevice vk_device() const { return vk_device_; }

  absl::StatusOr<core::RefCountPtr<ShaderModule>> create_shader_module(const core::Blob* blob);

 private:
  core::RefCountPtr<Instance> instance_;
  VkDevice vk_device_{VK_NULL_HANDLE};
};

class Image : public core::Inherit<Image, core::Object> {
 public:
  Image(core::RefCountPtr<Device> device, VkImage vk_image)
      : device_(device), vk_image_(vk_image) {}

  ~Image();

  VkImage vk_image() const { return vk_image_; }

 private:
  core::RefCountPtr<Device> device_;
  VkImage vk_image_{VK_NULL_HANDLE};
};

class ImageView : public core::Inherit<ImageView, core::Object> {
 public:
  ImageView(core::RefCountPtr<Device> device, VkImageView image_view)
      : device_(device), vk_image_view_(image_view) {}

  ~ImageView();

  VkImageView vk_image_view() const { return vk_image_view_; }

 private:
  core::RefCountPtr<Device> device_;
  VkImageView vk_image_view_{VK_NULL_HANDLE};
};

class ShaderModule : public core::Inherit<ShaderModule, core::Object> {
 public:
  ShaderModule(core::RefCountPtr<Device> device, VkShaderModule vk_shader_module)
      : device_(device), vk_shader_module_(vk_shader_module) {}

  ~ShaderModule();

  VkShaderModule vk_shader_module() const { return vk_shader_module_; }

 private:
  core::RefCountPtr<Device> device_;
  VkShaderModule vk_shader_module_{VK_NULL_HANDLE};
};

class Pipeline : public core::Inherit<Pipeline, core::Object> {};

}  // namespace rendering
}  // namespace lance
