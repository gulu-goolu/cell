#pragma once

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "lance/core/object.h"

namespace lance {
namespace rendering {
class Resource : public core::Inherit<Resource, core::Object> {};

class RenderGraph {
 public:
  absl::Status compile();

  absl::Status execute(absl::Span<const core::RefCountPtr<Resource>> resources);
};
}  // namespace rendering
}  // namespace lance
