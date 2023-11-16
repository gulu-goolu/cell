#pragma once

#include "absl/status/statusor.h"
#include "lance/core/file_system.h"
#include "lance/scene/scene.h"

namespace lance {
namespace scene {
absl::StatusOr<core::RefCountPtr<Scene>> load_gltf(rendering::Device* device,
                                                   const core::InputStream* input_stream);
}
}  // namespace lance
