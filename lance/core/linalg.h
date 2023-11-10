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

  explicit float4(float3 v, float _w) : x(v.x), y(v.y), z(v.z), w(_w) {}

  float operator[](size_t i) const { return v[i]; }

  float& operator[](size_t i) { return v[i]; }
};

// row major matrix
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

  matrix4x4 operator*(const matrix4x4& l) const {
    matrix4x4 result;
    for (size_t i = 0; i < 4; ++i) {
      for (size_t j = 0; j < 4; ++j) {
        result.m[i][j] = 0;

        for (size_t k = 0; k < 4; ++k) {
          result.m[i][j] += m[i][k] * l.m[k][j];
        }
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

template <typename Sink>
inline void AbslStringify(Sink& sink, const matrix4x4& value) {
  absl::Format(&sink, "[[%f,%f,%f,%f],[%f,%f,%f,%f],[%f,%f,%f,%f],[%f,%f,%f,%f]]",  //
               value.m[0][0], value.m[0][1], value.m[0][2], value.m[0][3],          //
               value.m[1][0], value.m[1][1], value.m[1][2], value.m[1][3],          //
               value.m[2][0], value.m[2][1], value.m[2][2], value.m[2][3],          //
               value.m[3][0], value.m[3][1], value.m[3][2], value.m[3][3]);
}
}  // namespace core
}  // namespace lance
