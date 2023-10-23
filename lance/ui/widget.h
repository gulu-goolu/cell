#pragma once

#include "lance/core/object.h"

namespace lance {
namespace ui {
enum class EventStatus {
  handled,
};

enum class UiEventType {
  clicked,
  mouse_move,
};

struct UiEvent {
  UiEventType type;
};

class Widget : public core::Inherit<Widget, core::Object> {
 public:
  virtual EventStatus handle_event(const UiEvent* e) = 0;
  virtual void render() = 0;
};
}  // namespace ui
}  // namespace lance