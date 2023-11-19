#include "app.h"

class TestApp : public lance::platform::IApp {
 public:
  absl::Status startup(lance::platform::StartupContext* ctx) override {
    ctx->instance();

    return absl::OkStatus();
  }

  absl::Status update(float elapsed_ms) override {
    return absl::OkStatus();  //
  }
};

LANCE_APP(TestApp)
