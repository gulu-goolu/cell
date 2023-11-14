#pragma once

#include "absl/status/status.h"
#include "vulkan/vulkan_core.h"

namespace lance {
namespace platform {
struct AppStartupOptions {
  // -1 means default width
  int32_t width = 640;

  // -1 means default height
  int32_t height = 480;

  const char* title = "";
};

struct StartupContext {
  virtual ~StartupContext() = default;

  virtual VkSurfaceKHR vk_surface() const = 0;
};

class IApp {
 public:
  // set application startup options
  virtual absl::Status fill_startup_options(AppStartupOptions* options) { return absl::OkStatus(); }

  virtual absl::Status startup(StartupContext* ctx) { return absl::OkStatus(); }

  virtual absl::Status update(float elapsed_ms) = 0;
};
}  // namespace platform
}  // namespace lance

extern ::lance::platform::IApp* lance_get_app();

#define LANCE_APP(APP)                       \
  ::lance::platform::IApp* lance_get_app() { \
    static APP app;                          \
    return &app;                             \
  }
