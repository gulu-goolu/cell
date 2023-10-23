#pragma once

#include "absl/status/status.h"

namespace cell {
namespace core {
class RefCounted {
 public:
  virtual ~RefCounted() = default;
};
}  // namespace core
}  // namespace cell

// return if expr failed
#define CELL_RETURN_IF_FAILED(EXPR) \
  do {                              \
    auto st = (EXPR);               \
    if (!st.ok()) {                 \
      return st;                    \
    }                               \
  } while (false)
