#include "vk_api.h"

namespace cell {
namespace rendering {
const VkApi* VkApi::get() {
  static const VkApi api;
  return &api;
}
}
}
