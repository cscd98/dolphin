#include "GLContextLR.h"
#include "Common/Logging/Log.h"
//#include "Core/Config/MainSettings.h"
#include "VideoCommon/VideoCommon.h"
//#include "VideoCommon/Present.h"
#include "DolphinLibretro/Common/Options.h"
#include "DolphinLibretro/VideoContexts/ContextStatus.h"
#include "DolphinLibretro/Video.h"

GLContextLR::GLContextLR()
{
}

GLContextLR::~GLContextLR() = default;

void* GLContextLR::GetFuncAddress(const std::string& name)
{
  return (void*)Libretro::Video::hw_render.get_proc_address(name.c_str());
}

/*bool GLContextLR::Initialize(const WindowSystemInfo& wsi, bool stereo, bool core)
{
  s_backbuffer_width = EFB_WIDTH * Libretro::Options::GetCached<int>(Libretro::Options::gfx_settings::EFB_SCALE, 1);
  s_backbuffer_height = EFB_HEIGHT * Libretro::Options::GetCached<int>(Libretro::Options::gfx_settings::EFB_SCALE, 1);

  switch (Libretro::Video::hw_render.context_type)
  {
    case RETRO_HW_CONTEXT_OPENGLES_VERSION:
    case RETRO_HW_CONTEXT_OPENGLES3:
      m_opengl_mode = GLInterfaceMode::MODE_OPENGLES3;
      break;
    case RETRO_HW_CONTEXT_OPENGL_CORE:
    case RETRO_HW_CONTEXT_OPENGL:
    default:
      m_opengl_mode = GLInterfaceMode::MODE_OPENGL;
      break;
  }

  if (!g_context_status.IsReady())
  {
    m_initialized = false;
    return false;
  }

  m_initialized = true;

  return true;
}*/

void GLContextLR::Swap()
{
  Libretro::Video::video_cb(RETRO_HW_FRAME_BUFFER_VALID,
        s_backbuffer_width, s_backbuffer_height, 0);
}

void GLContextLR::Shutdown()
{
  m_initialized = false;
}
