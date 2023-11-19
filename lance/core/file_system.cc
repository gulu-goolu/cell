#include "file_system.h"

namespace lance {
namespace core {

namespace {
class LocalInputStream : public Inherit<LocalInputStream, InputStream> {
 public:
};

class LocalFileSystem : public Inherit<LocalFileSystem, FileSystem> {};
}  // namespace

absl::StatusOr<core::RefCountPtr<FileSystem>> create_local_file_system() { return nullptr; }
}  // namespace core
}  // namespace lance