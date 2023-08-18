#pragma once

#include "vulkan/vulkan.h"

namespace cell {
namespace rendering {
class VkApi {
public:
  static const VkApi* get();
};
}
}
