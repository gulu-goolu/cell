#include "vk_api.h"

#include "gtest/gtest.h"

namespace lance {
namespace rendering {
TEST(vk_api_test, create_instance) {
  auto api = VkApi::get();
  ASSERT_TRUE(api != nullptr);
}
}  // namespace rendering
}  // namespace lance
