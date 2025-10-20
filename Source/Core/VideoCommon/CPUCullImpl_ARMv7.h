#pragma once

#ifdef _M_ARM_32

#include <arm_neon.h>

// ARM64 -> ARMv7 intrinsic compatibility layer

// Unzip operations - add vuzp1q_f32 for ARMv7
inline float32x4_t vuzp1q_f32_compat(float32x4_t a, float32x4_t b) {
  float32x4x2_t result = vuzpq_f32(a, b);
  return result.val[0];
}
#define vuzp1q_f32 vuzp1q_f32_compat

// vmulq_laneq_f32: lane must be immediate -> dispatch per lane
inline float32x4_t vmulq_laneq_f32_compat(float32x4_t a, float32x4_t v, int lane) {
  switch (lane) {
    case 0: return vmulq_n_f32(a, vgetq_lane_f32(v, 0));
    case 1: return vmulq_n_f32(a, vgetq_lane_f32(v, 1));
    case 2: return vmulq_n_f32(a, vgetq_lane_f32(v, 2));
    case 3: return vmulq_n_f32(a, vgetq_lane_f32(v, 3));
    default: return a; // or assert
  }
}
#define vmulq_laneq_f32 vmulq_laneq_f32_compat

// vfmaq_laneq_f32: ARMv7 uses vmla; lane must be immediate -> dispatch per lane
inline float32x4_t vfmaq_laneq_f32_compat(float32x4_t a, float32x4_t b, float32x4_t v, int lane) {
  switch (lane) {
    case 0: return vmlaq_n_f32(a, b, vgetq_lane_f32(v, 0));
    case 1: return vmlaq_n_f32(a, b, vgetq_lane_f32(v, 1));
    case 2: return vmlaq_n_f32(a, b, vgetq_lane_f32(v, 2));
    case 3: return vmlaq_n_f32(a, b, vgetq_lane_f32(v, 3));
    default: return a; // or assert
  }
}
#define vfmaq_laneq_f32 vfmaq_laneq_f32_compat

// vdupq_laneq_f32: lane must be immediate -> dispatch per lane
inline float32x4_t vdupq_laneq_f32_compat(float32x4_t v, int lane) {
  switch (lane) {
    case 0: return vdupq_n_f32(vgetq_lane_f32(v, 0));
    case 1: return vdupq_n_f32(vgetq_lane_f32(v, 1));
    case 2: return vdupq_n_f32(vgetq_lane_f32(v, 2));
    case 3: return vdupq_n_f32(vgetq_lane_f32(v, 3));
    default: return vdupq_n_f32(0.0f); // or assert
  }
}
#define vdupq_laneq_f32 vdupq_laneq_f32_compat

// Zip operations - ARM64 has zip1/zip2, ARMv7 has vzipq returning pair
inline float32x4_t vzip1q_f32_compat(float32x4_t a, float32x4_t b) {
  float32x4x2_t r = vzipq_f32(a, b);
  return r.val[0];
}
#define vzip1q_f32 vzip1q_f32_compat

inline float32x4_t vzip2q_f32_compat(float32x4_t a, float32x4_t b) {
  float32x4x2_t r = vzipq_f32(a, b);
  return r.val[1];
}
#define vzip2q_f32 vzip2q_f32_compat

// Unzip operations
inline float32x4_t vuzp2q_f32_compat(float32x4_t a, float32x4_t b) {
  float32x4x2_t r = vuzpq_f32(a, b);
  return r.val[1];
}
#define vuzp2q_f32 vuzp2q_f32_compat

inline uint16x8_t vuzp1q_u16_compat(uint16x8_t a, uint16x8_t b) {
  uint16x8x2_t r = vuzpq_u16(a, b);
  return r.val[0];
}
#define vuzp1q_u16 vuzp1q_u16_compat

inline uint8x16_t vuzp1q_u8_compat(uint8x16_t a, uint8x16_t b) {
  uint8x16x2_t r = vuzpq_u8(a, b);
  return r.val[0];
}
#define vuzp1q_u8 vuzp1q_u8_compat

// Horizontal add - sum all lanes
inline float vaddvq_f32_compat(float32x4_t v) {
  float32x2_t s = vadd_f32(vget_low_f32(v), vget_high_f32(v)); // [v0+v2, v1+v3]
  s = vpadd_f32(s, s);                                         // [v0+v1+v2+v3, ...]
  return vget_lane_f32(s, 0);
}
#define vaddvq_f32 vaddvq_f32_compat

// Horizontal max
inline uint32_t vmaxvq_u32_compat(uint32x4_t v) {
  uint32x2_t m = vmax_u32(vget_low_u32(v), vget_high_u32(v)); // [max(v0,v2), max(v1,v3)]
  m = vpmax_u32(m, m);                                        // [max(all), ...]
  return vget_lane_u32(m, 0);
}
#define vmaxvq_u32 vmaxvq_u32_compat

// f64 note: ARMv7 NEON lacks IEEE f64 vector ops.
// Use uint64x2_t as a transport when you only need lane shuffle,
// and keep math in scalar if needed.
#define float64x2_t uint64x2_t
#define vreinterpretq_f64_f32 vreinterpretq_u64_f32
#define vreinterpretq_f32_f64 vreinterpretq_f32_u64

inline uint64x2_t vzip1q_f64_compat(uint64x2_t a, uint64x2_t b) {
  return vcombine_u64(vget_low_u64(a), vget_low_u64(b));
}
#define vzip1q_f64 vzip1q_f64_compat

inline uint64x2_t vzip2q_f64_compat(uint64x2_t a, uint64x2_t b) {
  return vcombine_u64(vget_high_u64(a), vget_high_u64(b));
}
#define vzip2q_f64 vzip2q_f64_compat

#endif
