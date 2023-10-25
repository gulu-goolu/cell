#include "device.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "vk_api.h"

namespace lance {
namespace rendering {
TEST(instance, 3d) {
  auto instance = Instance::create_for_3d().value();

  ASSERT_TRUE(instance != nullptr);

  auto physical_devices = instance->enumerate_physical_devices().value();
  LOG(INFO) << "num_physical_devices: " << physical_devices.size();

  for (const auto physical_device : physical_devices) {
    VkPhysicalDeviceProperties props;
    VkApi::get()->vkGetPhysicalDeviceProperties(physical_device, &props);

    LOG(INFO) << "device_name: " << props.deviceName;
  }
}
}  // namespace rendering
}  // namespace lance
