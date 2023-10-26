#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>

#include "absl/status/status.h"

namespace lance {
namespace core {
class RefCounted {
 public:
  virtual ~RefCounted() = default;

  virtual void add_ref() const = 0;
  virtual void release() const = 0;
  virtual void delete_this() const = 0;
  virtual uint64_t reference_count() const = 0;
};

template <typename T>
class RefCountPtr {
 public:
  static_assert(std::is_base_of_v<RefCounted, T>);

  RefCountPtr() = default;

  RefCountPtr(std::nullptr_t) {}

  RefCountPtr(T* ptr) { reset(ptr); }

  RefCountPtr(const RefCountPtr<T>& other) { reset(other.get()); }
  RefCountPtr(RefCountPtr<T>&& other) : ptr_(other.release()) {}

  template <typename U, typename = std::enable_if_t<std::is_base_of_v<T, U>, void>>
  RefCountPtr(const RefCountPtr<U>& other) {
    reset(other.get());
  }

  template <typename U, typename = std::enable_if_t<std::is_base_of_v<T, U>, void>>
  RefCountPtr(RefCountPtr<U>&& other) : ptr_(other.release()) {}

  ~RefCountPtr() {
    if (ptr_) {
      ptr_->release();
      ptr_ = nullptr;
    }
  }

  void reset(T* p) {
    if (p) {
      p->add_ref();
    }

    if (ptr_) {
      ptr_->release();
    }

    ptr_ = p;
  }

  T* get() const { return ptr_; }

  T* release() {
    T* p = ptr_;
    ptr_ = nullptr;
    return p;
  }

  constexpr T* operator->() const noexcept { return ptr_; }

  constexpr bool operator!=(std::nullptr_t) const noexcept { return ptr_ != nullptr; }

 private:
  T* ptr_ = nullptr;
};

template <typename T, typename... Args>
inline RefCountPtr<T> make_refcounted(Args&&... args) {
  static_assert(std::is_base_of_v<RefCounted, T>);

  class Impl : public T {
   public:
    using T::T;

    void add_ref() const final { count_.fetch_add(1, std::memory_order_relaxed); }
    void release() const final {
      if (count_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
        delete_this();
      }
    }
    void delete_this() const final { delete this; }
    uint64_t reference_count() const final { return count_.load(); }

   private:
    mutable std::atomic_uint_fast64_t count_{0};
  };

  return RefCountPtr<Impl>(new Impl(std::forward<Args>(args)...));
}

}  // namespace core
}  // namespace lance

// return if expr failed
#define LANCE_RETURN_IF_FAILED(EXPR) \
  do {                               \
    auto st = (EXPR);                \
    if (!st.ok()) {                  \
      return st;                     \
    }                                \
  } while (false)

#define LANCE_CONCAT_V2(STR1, STR2) STR1##STR2

#define LANCE_CONCAT(STR1, STR2) LANCE_CONCAT_V2(STR1, STR2)

#define LANCE_ASSIGN_OR_RETURN(VAR, EXPR)        \
  auto LANCE_CONCAT(VAR, __LINE__) = (EXPR);     \
  if (!LANCE_CONCAT(VAR, __LINE__).ok()) {       \
    return LANCE_CONCAT(VAR, __LINE__).status(); \
  }                                              \
  auto VAR = std::move(LANCE_CONCAT(VAR, __LINE__).value())
