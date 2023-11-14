#include "app.h"

#include "glfw/glfw3.h"
#include "lance/core/util.h"

int main(int argc, char* argv[]) {
  if (glfwInit() != GLFW_TRUE) {
    return -1;
  }

  auto app = lance_get_app();

  ::lance::platform::AppStartupOptions startup_options;
  LANCE_THROW_IF_FAILED(app->fill_startup_options(&startup_options));

  return 0;
}
