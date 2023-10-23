#pragma once

#include <atomic>
#include <cstdint>
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

  RefCountPtr(T* ptr) { reset(ptr); }

  template <typename U>
  RefCountPtr(const RefCountPtr<U>& other) {
    reset(other.get());
  }

  template <typename U>
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

  T* operator->() const { return ptr_; }

 private:
  T* ptr_ = nullptr;
};

template <typename T, typename... Args>
inline RefCountPtr<T> make_refcounted(Args&&... args) {
  static_assert(std::is_base_of_v<RefCounted, T>);

  class Impl : public T {
   public:
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

  return RefCountPtr<Impl>(new Impl);
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
