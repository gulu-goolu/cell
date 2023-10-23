#pragma once

namespace lance {
namespace core {
class Object {
 public:
  virtual ~Object() = default;
};

template <typename T, typename Base>
class Inherit : public Base {};
}  // namespace core
}  // namespace lance
