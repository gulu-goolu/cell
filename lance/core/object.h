#pragma once

#include <typeindex>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "lance/core/util.h"

namespace lance {
namespace core {

struct TypeMetadata {
  const TypeMetadata* base = nullptr;

  std::type_index id{typeid(nullptr)};

  bool is_type(std::type_index type) const {
    const TypeMetadata* ptr = this;
    while (ptr) {
      if (ptr->id == type) {
        return true;
      }

      ptr = ptr->base;
    }

    return false;
  }

  template <typename T>
  bool is_type_of() const {
    return is_type(typeid(T));
  }
};

class Object : public RefCounted {
 public:
  virtual ~Object() = default;

  virtual const TypeMetadata* type_metadata() const = 0;

  virtual std::string_view type_name() const;

  // for simplify
  template <typename T>
  bool is_type_of() const {
    return type_metadata()->is_type_of<T>();
  }

  template <typename T>
  absl::StatusOr<const T*> cast_to() const {
    if (is_type_of<T>()) {
      return reinterpret_cast<const T*>(this);
    }

    return absl::InvalidArgumentError(absl::StrFormat("unexcepted type, expect: %s, got: %s",
                                                      typeid(T).name(), this->type_name()));
  }

  template <typename T>
  absl::StatusOr<T*> cast_to() {
    if (is_type_of<T>()) {
      return reinterpret_cast<T*>(this);
    }

    return absl::InvalidArgumentError(absl::StrFormat("unexcepted type, got: %s, expect: %s",
                                                      this->type_name(), typeid(T).name()));
  }
};

template <typename T, typename Base>
class Inherit : public Base {
 public:
  using Base::Base;

  using BaseType = Base;

  const TypeMetadata* type_metadata() const override { return static_type_medadata(); }

  static const TypeMetadata* static_type_medadata();
};

namespace detail {
template <typename T>
constexpr size_t type_chain_depth() {
  if constexpr (std::is_same_v<T, Object>) {
    return 1;
  } else {
    return type_chain_depth<typename T::BaseType>() + 1;
  }
}

template <typename T>
constexpr void type_metadata_set_id(TypeMetadata* v) {
  v->id = typeid(T);

  if constexpr (!std::is_same_v<T, Object>) {
    type_metadata_set_id<typename T::BaseType>(v + 1);
  }
}

}  // namespace detail

template <typename T, typename Base>
inline const TypeMetadata* Inherit<T, Base>::static_type_medadata() {
  static const TypeMetadata* chain = []() -> const TypeMetadata* {
    constexpr size_t num_depth = detail::type_chain_depth<T>();

    static TypeMetadata list[num_depth];

    detail::type_metadata_set_id<T>(list);

    // initialize type chain list
    list[0].base = nullptr;
    for (size_t i = 0; i < num_depth - 1; ++i) {
      list[i].base = &list[i + 1];
    }

    return list;
  }();

  return chain;
}

class Blob : public Inherit<Blob, Object> {
 public:
  static RefCountPtr<Blob> create(const void* data, size_t size);

  virtual const void* data() const = 0;
  virtual size_t size() const = 0;
};
}  // namespace core
}  // namespace lance
