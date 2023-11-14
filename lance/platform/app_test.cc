#include "app.h"

class TestApp : public lance::platform::IApp {
 public:
  absl::Status update(float elapsed_ms) override {
    return absl::OkStatus();  //
  }
};

LANCE_APP(TestApp)
