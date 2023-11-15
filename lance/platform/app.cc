#include "app.h"

#include "glfw/glfw3.h"
#include "glog/logging.h"
#include "lance/core/util.h"
#include "lance/rendering/vk_api.h"

namespace {
class StartupContextImpl : public ::lance::platform::StartupContext {
 public:
  lance::rendering::Instance* instance() const override { return instance_.get(); }

  lance::rendering::Surface* surface() const override { return surface_.get(); }

  lance::core::RefCountPtr<lance::rendering::Instance> instance_;
  lance::core::RefCountPtr<lance::rendering::Surface> surface_;
};

lance::core::RefCountPtr<lance::rendering::Instance> create_instance_or_die(
    const ::lance::platform::AppStartupOptions* options) {
  uint32_t extension_count = 0;
  const char* const* extensions = glfwGetRequiredInstanceExtensions(&extension_count);

  VkInstance vk_instance = VK_NULL_HANDLE;
  VkInstanceCreateInfo instance_create_info = {};
  instance_create_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  instance_create_info.enabledExtensionCount = extension_count;
  instance_create_info.ppEnabledExtensionNames = extensions;
  CHECK(::lance::rendering::VkApi::get()->vkCreateInstance(&instance_create_info, nullptr,
                                                           &vk_instance) == VK_SUCCESS);

  return lance::core::make_refcounted<lance::rendering::Instance>(vk_instance);
}

}  // namespace

int main(int argc, char* argv[]) {
  if (glfwInit() != GLFW_TRUE) {
    return -1;
  }

  auto app = lance_get_app();

  ::lance::platform::AppStartupOptions startup_options;
  LANCE_THROW_IF_FAILED(app->fill_startup_options(&startup_options));

  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  auto window = glfwCreateWindow(startup_options.width, startup_options.height,
                                 startup_options.title, nullptr, nullptr);
  CHECK(window != nullptr);

  StartupContextImpl startup_context;

  // create instance
  startup_context.instance_ = create_instance_or_die(&startup_options);

  // create surface
  VkSurfaceKHR vk_surface;
  CHECK(glfwCreateWindowSurface(startup_context.instance_->vk_instance(), window, nullptr,
                                &vk_surface) == VK_SUCCESS);
  startup_context.surface_ = lance::core::make_refcounted<lance::rendering::Surface>(
      startup_context.instance_, vk_surface);

  LANCE_THROW_IF_FAILED(app->startup(&startup_context));

  // main loop
  auto tp0 = std::chrono::steady_clock::now();
  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    auto tp = std::chrono::steady_clock::now();
    const float elapsed_ms = (tp - tp0).count() / 1.0e6;

    LANCE_THROW_IF_FAILED(app->update(elapsed_ms));

    tp0 = tp;
  }

  return 0;
}
