#include "util.h"

#include "glog/logging.h"
#include "renderdoc_app/renderdoc_app.h"

#if defined(_WIN64)
#include <Windows.h>
#endif

namespace lance {
namespace rendering {

struct RenderDocApp {
  static RenderDocApp* get() {
    static RenderDocApp app;
    return &app;
  }

  RenderDocApp() {
#ifdef _WIN64
    shared_library_ = GetModuleHandleA("renderdoc.dll");

    if (shared_library_) {
      pRENDERDOC_GetAPI get_api =
          reinterpret_cast<pRENDERDOC_GetAPI>(GetProcAddress(shared_library_, "RENDERDOC_GetAPI"));

      int ret = get_api(eRENDERDOC_API_Version_1_6_0, reinterpret_cast<void**>(&rdoc_api));
      CHECK(ret == 1);
    }
#endif
  }

  ~RenderDocApp() {
#ifdef _WIN64
    if (shared_library_) {
      CloseHandle(shared_library_);
    }
#endif
  }

#ifdef _WIN64
  HMODULE shared_library_ = nullptr;
#endif

  RENDERDOC_API_1_6_0* rdoc_api = nullptr;
};

void render_doc_begin_capture() {
  if (RenderDocApp::get()->rdoc_api) {
    RenderDocApp::get()->rdoc_api->StartFrameCapture(nullptr, nullptr);
  }
}

void render_doc_end_capture() {
  if (RenderDocApp::get()->rdoc_api) {
    RenderDocApp::get()->rdoc_api->EndFrameCapture(nullptr, nullptr);
  }
}
}  // namespace rendering
}  // namespace lance
