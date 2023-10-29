#pragma once

#include "absl/strings/str_format.h"

namespace lance {
namespace core {
struct float3 {
  float x, y, z;
};

struct float4 {
  union {
    struct {
      float x, y, z, w;
    };
    float v[4];
  };

  float4() = default;

  explicit float4(float3 t, float _w) : x(t.x), y(t.y), z(t.z), w(_w) {}

  float operator[](size_t i) const { return v[i]; }

  float& operator[](size_t i) { return v[i]; }
};

struct matrix4x4 {
  float m[4][4];

  float4 operator*(const float4& t) const {
    float4 result;
    for (size_t i = 0; i < 4; ++i) {
      result.v[i] = 0;
      for (size_t j = 0; j < 4; ++j) {
        result.v[i] += m[i][j] * t.v[j];
      }
    }
    return result;
  }
};

template <typename Sink>
inline void AbslStringify(Sink& sink, const float3& v) {
  absl::Format(&sink, "(%f, %f, %f)", v.x, v.y, v.z);
}

template <typename Sink>
inline void AbslStringify(Sink& sink, const float4& v) {
  absl::Format(&sink, "(%f, %f, %f, %f)", v.x, v.y, v.z, v.w);
}
}  // namespace core
}  // namespace lance
