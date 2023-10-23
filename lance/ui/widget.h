#pragma once

#include "lance/core/object.h"

namespace lance {
namespace ui {
enum class EventStatus {
    handled,
};
class Widget: public Inherit<Widget, Object> {
public:
    virtual EventStatus handle_event() = 0;
    virtual void render() = 0;
};
}
}
