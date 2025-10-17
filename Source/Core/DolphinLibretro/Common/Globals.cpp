#include "Globals.h"

namespace Libretro
{
retro_environment_t environ_cb;
bool g_emuthread_launched = false;

namespace Video
{
retro_video_refresh_t video_cb = nullptr;
retro_hw_render_callback hw_render;
}  // namespace Video
}  // namespace Libretro
