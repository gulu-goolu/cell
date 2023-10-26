#include "object.h"

#include <cstdlib>

namespace lance {
namespace core {
RefCountPtr<Blob> Blob::create(const void* data, size_t size) {
  class BlobFromData : public Inherit<BlobFromData, Blob> {
   public:
    BlobFromData(void* data, size_t size) : data_(data), size_(size) {}

    ~BlobFromData() { ::free(data_); }

    const void* data() const override { return data_; }
    size_t size() const override { return size_; }

   private:
    void* data_ = nullptr;
    size_t size_ = 0;
  };

  void* addr = std::malloc(size);
  memcpy(addr, data, size);

  return make_refcounted<BlobFromData>(addr, size);
}
}  // namespace core
}  // namespace lance
