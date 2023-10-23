#pragma once

#include <cstdint>

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

  explicit float4(float3 t, float _w) : x(t.x), y(t.y), z(t.z), w(_w) {}

  float operator[](size_t i) const { return v[i]; }

  float& operator[](size_t i) { return &v[i]; }
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
}  // namespace core
}  // namespace lance
