#pragma once

#include <string_view>

#include "absl/status/statusor.h"
#include "object.h"

namespace lance {
namespace core {
class InputStream : public core::Inherit<InputStream, core::Object> {
 public:
  virtual absl::Status read(size_t offset, size_t length, void* out) = 0;
};

class OutputStream : public core::Inherit<OutputStream, core::Object> {
 public:
  virtual absl::Status write(size_t offset, size_t length, const void* data) = 0;
};

class FileSystem : public core::Inherit<FileSystem, core::Object> {
 public:
  virtual absl::StatusOr<core::RefCountPtr<InputStream>> create_input_stream(
      std::string_view uri) = 0;

  virtual absl::StatusOr<core::RefCountPtr<OutputStream>> create_output_stream(
      std::string_view uri) = 0;
};
}  // namespace core
}  // namespace lance
