#pragma once

#include <array>
#include "Common/CommonTypes.h"

namespace HeadTracking
{
struct Eye
{
  // x' = kx*x + ax*w + bx ; y' = ky*y + ay*w   (clip space, geometry shader)
  float kx = 1.0f, ax = 0.0f, bx = 0.0f;
  float ky = 1.0f, ay = 0.0f;
};

struct State
{
  bool active = false;

  // Row-major, column-vector convention (same layout as VertexShaderManager::m_projection_matrix).
  std::array<float, 16> view{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

  // Mono (averaged) frustum, tangents of half-angles.
  bool has_fov = false;
  float tan_left = 1, tan_right = 1, tan_up = 1, tan_down = 1;

  // Per-eye correction relative to the mono frustum.
  bool has_eyes = false;
  std::array<Eye, 2> eyes{};

  u64 generation = 0;
};

void Set(State state);
void Clear();
u64 Generation();     // bumps every Set/Clear (pose changes every frame)
u64 EyeGeneration();  // bumps only when the per-eye terms actually change
State Get();
}  // namespace HeadTracking
