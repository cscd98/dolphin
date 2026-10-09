// Copyright 2014 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include "Common/CommonTypes.h"
#include "VideoCommon/ConstantManager.h"

#ifdef __LIBRETRO__
#include "DolphinLibretro/VideoCommon/HeadTracking.h"
#endif

class PointerWrap;
enum class PrimitiveType : u32;

enum class APIType;

// The non-API dependent parts.
class GeometryShaderManager
{
public:
  void Init();
  void Dirty();
  void DoState(PointerWrap& p);

  void SetConstants(PrimitiveType prim);
  void SetViewportChanged();
  void SetProjectionChanged();
  void SetLinePtWidthChanged();
  void SetTexCoordChanged(u8 texmapid);

  GeometryShaderConstants constants{};
  bool dirty = false;

private:
  void SetVSExpand(VSExpand expand);

  bool m_projection_changed = false;
  bool m_viewport_changed = false;
};
