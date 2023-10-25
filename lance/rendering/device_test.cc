#include "device.h"

#include "gtest/gtest.h"

namespace lance {
namespace rendering {
TEST(instance, 3d) {
  auto instance = Instance::create_for_3d().value();

  ASSERT_TRUE(instance != nullptr);
}
}  // namespace rendering
}  // namespace lance
