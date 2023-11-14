#pragma once

#include "absl/status/status.h"

namespace lance {
namespace platform {
struct AppStartupOptions {
  // -1 means default width
  int32_t width = -1;

  // -1 means default height
  int32_t height = -1;
};

class IApp {
 public:
  virtual absl::Status fill_startup_options(AppStartupOptions* options) { return absl::OkStatus(); }

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
