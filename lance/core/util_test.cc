#include "util.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace lance {
namespace core {
class UtilRefCountedTest : public RefCounted {
 public:
  ~UtilRefCountedTest() override { LOG(INFO) << "~UtilRefCountedTest"; }
};

TEST(util, ref_counted) {
  auto t = make_refcounted<UtilRefCountedTest>();

  ASSERT_EQ(1, t->reference_count());
}
}  // namespace core
}  // namespace lance
