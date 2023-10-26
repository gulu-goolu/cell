#pragma once

#include "lance/core/util.h"

namespace lance {
namespace core {
struct TypeMetadata {};

class Object : public RefCounted {
 public:
  virtual ~Object() = default;
};

template <typename T, typename Base>
class Inherit : public Base {};

class Blob : public Inherit<Blob, Object> {
 public:
  static RefCountPtr<Blob> create(const void* data, size_t size);

  virtual const void* data() const = 0;
  virtual size_t size() const = 0;
};
}  // namespace core
}  // namespace lance
