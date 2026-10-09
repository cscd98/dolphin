#include "GLContextLR.h"
#include "Common/Logging/Log.h"
#include "Core/Config/MainSettings.h"
#include "VideoCommon/VideoCommon.h"
#include "VideoCommon/Present.h"
#include "DolphinLibretro/Common/Options.h"
#include "DolphinLibretro/VideoContexts/ContextStatus.h"
#include "DolphinLibretro/Video.h"
#include "DolphinLibretro/VideoViews.h"

#include <vector>
#include "Common/GL/GLExtensions/GLExtensions.h"
#include "Common/Logging/Log.h"

GLContextLR::GLContextLR()
{
}

GLContextLR::~GLContextLR() = default;

void* GLContextLR::GetFuncAddress(const std::string& name)
{
  return (void*)Libretro::Video::hw_render.get_proc_address(name.c_str());
}

bool GLContextLR::Initialize(const WindowSystemInfo& wsi, bool stereo, bool core)
{
  UpdateBackbufferSize();

  switch (Libretro::Video::hw_render.context_type)
  {
    case RETRO_HW_CONTEXT_OPENGLES_VERSION:
    case RETRO_HW_CONTEXT_OPENGLES3:
      m_opengl_mode = Mode::OpenGLES;
      break;
    case RETRO_HW_CONTEXT_OPENGL_CORE:
    case RETRO_HW_CONTEXT_OPENGL:
    default:
      m_opengl_mode = Mode::OpenGL;
      break;
  }

  if (!g_context_status.IsReady())
  {
    m_initialized = false;
    return false;
  }

  m_initialized = true;

  return true;
}

void GLContextLR::UpdateBackbufferSize()
{
  const int efbScale =
      Libretro::Options::GetCached<int>(Libretro::Options::gfx_settings::EFB_SCALE, 1);
  m_backbuffer_width =
      EFB_WIDTH * efbScale * Libretro::Video::Views::GetFrameWidthMultiplier();
  m_backbuffer_height = Libretro::Video::GetAdjustedBaseHeight() * efbScale;
}

void GLContextLR::Update()
{
  UpdateBackbufferSize();
}

void GLContextLR::Swap()
{
  static int s_probe = 0;
  static int s_last_l = -1, s_last_r = -1;
  if ((s_probe++ % 30) == 0 && m_backbuffer_width >= 4)
  {
    const int w = static_cast<int>(m_backbuffer_width);
    const int half = w / 2;
    std::vector<u32> row(w);
    glReadPixels(0, static_cast<int>(m_backbuffer_height) / 2, w, 1, GL_RGBA, GL_UNSIGNED_BYTE,
                 row.data());
    int lit_l = 0, lit_r = 0, same = 0;
    for (int i = 0; i < half; i++)
    {
      lit_l += (row[i] & 0x00FFFFFF) != 0;
      lit_r += (row[half + i] & 0x00FFFFFF) != 0;
      same += row[i] == row[half + i];
    }
    if (lit_l != s_last_l || lit_r != s_last_r)
    {
      s_last_l = lit_l;
      s_last_r = lit_r;
      INFO_LOG_FMT(VIDEO, "Views: probe w={} half={} lit_left={} lit_right={} identical={}", w, half,
                   lit_l, lit_r, same);
    }
  }

  Libretro::Video::video_cb(VideoCommon::g_is_duplicate_frame ? nullptr : RETRO_HW_FRAME_BUFFER_VALID,
        m_backbuffer_width, m_backbuffer_height, 0);
}

void GLContextLR::Shutdown()
{
  m_initialized = false;
}
