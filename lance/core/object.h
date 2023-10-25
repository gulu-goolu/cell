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
}  // namespace core
}  // namespace lance
