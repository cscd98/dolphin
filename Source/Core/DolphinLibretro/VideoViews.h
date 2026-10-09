#pragma once

#include <libretro.h>

namespace Libretro::Video::Views
{
void Reset();
void Update(float display_aspect);
unsigned GetFrameWidthMultiplier();
unsigned GetReservedWidthMultiplier();
bool IsStereoPacked();
bool HasFrameState();
const retro_vr_frame_state& GetFrameState();
}  // namespace Libretro::Video::Views
