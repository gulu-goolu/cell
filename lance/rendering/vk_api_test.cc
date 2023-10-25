#include "vk_api.h"

#include "device.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace lance {
namespace rendering {
TEST(vk_api_test, create_instance) {
  auto api = VkApi::get();
  ASSERT_TRUE(api != nullptr);

  auto instance = Instance::create({}, {}).value();
  CHECK(instance != nullptr);
}
}  // namespace rendering
}  // namespace lance
