// Copyright 2014 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "VideoCommon/GeometryShaderManager.h"

#include "Common/ChunkFile.h"
#include "Common/CommonTypes.h"
#include "VideoCommon/BPMemory.h"
#include "VideoCommon/RenderState.h"
#include "VideoCommon/VideoConfig.h"
#include "VideoCommon/XFMemory.h"

#ifdef __LIBRETRO__
#include "DolphinLibretro/VideoCommon/HeadTracking.h"
#endif

static constexpr int LINE_PT_TEX_OFFSETS[8] = {0, 16, 8, 4, 2, 1, 1, 1};

#ifdef __LIBRETRO__
static u64 s_head_eye_generation = 0;
#endif

void GeometryShaderManager::Init()
{
  constants = {};

  // Init any initial constants which aren't zero when bpmem is zero.
  SetViewportChanged();
  SetProjectionChanged();

  dirty = true;
}

void GeometryShaderManager::Dirty()
{
  // This function is called after a savestate is loaded.
  // Any constants that can changed based on settings should be re-calculated
  m_projection_changed = true;

  // Uses EFB scale config
  SetLinePtWidthChanged();

  dirty = true;
}

void GeometryShaderManager::SetVSExpand(VSExpand expand)
{
  if (constants.vs_expand != expand)
  {
    constants.vs_expand = expand;
    dirty = true;
  }
}

void GeometryShaderManager::SetConstants(PrimitiveType prim)
{
#ifdef __LIBRETRO__
  const u64 eye_generation = HeadTracking::EyeGeneration();
  if (eye_generation != s_head_eye_generation)
  {
    s_head_eye_generation = eye_generation;
    m_projection_changed = true;
  }

  if (m_projection_changed && g_ActiveConfig.stereo_mode != StereoMode::Off)
  {
    m_projection_changed = false;

    if (xfmem.projection.type == ProjectionType::Perspective)
    {
      const float offset = g_ActiveConfig.stereo_depth;
      constants.stereoparams[0] = g_ActiveConfig.bStereoSwapEyes ? offset : -offset;
      constants.stereoparams[1] = g_ActiveConfig.bStereoSwapEyes ? -offset : offset;
    }
    else
    {
      constants.stereoparams[0] = constants.stereoparams[1] = 0;
    }

    constants.stereoparams[2] = g_ActiveConfig.stereo_convergence;

    // NEW: per-eye affine terms (head-tracked or legacy)
    const bool persp = xfmem.projection.type == ProjectionType::Perspective;
    const HeadTracking::State head = HeadTracking::Get();
    const bool swap = g_ActiveConfig.bStereoSwapEyes;

    for (int eye = 0; eye < 2; eye++)
    {
      auto& ex = constants.stereo_eye[eye * 2];
      auto& ey = constants.stereo_eye[eye * 2 + 1];
      if (persp && head.active && head.has_eyes)
      {
        const auto& e = head.eyes[swap ? 1 - eye : eye];
        ex = {e.kx, e.ax, e.bx, 0.0f};
        ey = {e.ky, e.ay, 0.0f, 0.0f};
      }
      else
      {
        // Legacy mapping: x += h * (w - convergence). Orthographic gives h = 0.
        const float h = constants.stereoparams[eye];
        ex = {1.0f, h, -h * constants.stereoparams[2], 0.0f};
        ey = {1.0f, 0.0f, 0.0f, 0.0f};
      }
    }

    dirty = true;
  }
#else
  if (m_projection_changed && g_ActiveConfig.stereo_mode != StereoMode::Off)
  {
    m_projection_changed = false;

    if (xfmem.projection.type == ProjectionType::Perspective)
    {
      const float offset = g_ActiveConfig.stereo_depth;
      constants.stereoparams[0] = g_ActiveConfig.bStereoSwapEyes ? offset : -offset;
      constants.stereoparams[1] = g_ActiveConfig.bStereoSwapEyes ? -offset : offset;
    }
    else
    {
      constants.stereoparams[0] = constants.stereoparams[1] = 0;
    }

    constants.stereoparams[2] = g_ActiveConfig.stereo_convergence;

    dirty = true;
  }
#endif

  if (g_ActiveConfig.UseVSForLinePointExpand())
  {
    if (prim == PrimitiveType::Points)
      SetVSExpand(VSExpand::Point);
    else if (prim == PrimitiveType::Lines)
      SetVSExpand(VSExpand::Line);
    else
      SetVSExpand(VSExpand::None);
  }

  if (m_viewport_changed)
  {
    m_viewport_changed = false;

    constants.lineptparams[0] = 2.0f * xfmem.viewport.wd;
    constants.lineptparams[1] = -2.0f * xfmem.viewport.ht;

    dirty = true;
  }
}

void GeometryShaderManager::SetViewportChanged()
{
  m_viewport_changed = true;
}

void GeometryShaderManager::SetProjectionChanged()
{
  m_projection_changed = true;
}

void GeometryShaderManager::SetLinePtWidthChanged()
{
  constants.lineptparams[2] = bpmem.lineptwidth.linesize / 6.f;
  constants.lineptparams[3] = bpmem.lineptwidth.pointsize / 6.f;
  constants.texoffset[2] = LINE_PT_TEX_OFFSETS[bpmem.lineptwidth.lineoff];
  constants.texoffset[3] = LINE_PT_TEX_OFFSETS[bpmem.lineptwidth.pointoff];
  dirty = true;
}

void GeometryShaderManager::SetTexCoordChanged(u8 texmapid)
{
  TCoordInfo& tc = bpmem.texcoords[texmapid];
  int bitmask = 1 << texmapid;
  constants.texoffset[0] &= ~bitmask;
  constants.texoffset[0] |= tc.s.line_offset << texmapid;
  constants.texoffset[1] &= ~bitmask;
  constants.texoffset[1] |= tc.s.point_offset << texmapid;
  dirty = true;
}

void GeometryShaderManager::DoState(PointerWrap& p)
{
  p.Do(m_projection_changed);
  p.Do(m_viewport_changed);

  p.Do(constants);

  if (p.IsReadMode())
  {
    // Fixup the current state from global GPU state
    // NOTE: This requires that all GPU memory has been loaded already.
    Dirty();
  }
}
