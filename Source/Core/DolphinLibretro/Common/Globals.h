#include <libretro.h>

namespace Libretro
{
extern retro_environment_t environ_cb;
extern bool g_emuthread_launched;

namespace Video
{
extern retro_video_refresh_t video_cb;
extern struct retro_hw_render_callback hw_render;
}  // namespace Video
}  // namespace Libretro
