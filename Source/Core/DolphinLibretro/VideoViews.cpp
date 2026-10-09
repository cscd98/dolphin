#include "DolphinLibretro/VideoViews.h"

#include <atomic>
#include <cmath>

#include "Common/Logging/Log.h"
#include "Core/Config/GraphicsSettings.h"
#include "DolphinLibretro/Common/Globals.h"
#include "DolphinLibretro/Common/Options.h"
#include "DolphinLibretro/VideoCommon/HeadTracking.h"
#include "DolphinLibretro/Video.h"
#include "DolphinLibretro/VideoContexts/ContextStatus.h"
#include "VideoCommon/Present.h"
#include "VideoCommon/VideoCommon.h"
#include "VideoCommon/VideoConfig.h"

namespace Libretro::Video::Views
{
static bool s_presents = false;
static bool s_stereo = false;
static bool s_hmd = false;
static bool s_map_sent = false;
static bool s_sbs_applied = false;
static unsigned s_reserved = 1;
static std::atomic<unsigned> s_mult{1};
static retro_vr_frame_state s_frame_state{};
static bool s_have_frame_state = false;
static unsigned s_last_rec_w = 0, s_last_rec_h = 0;
static bool s_have_ref = false;
static float s_ref_pos[3] = {};
static float s_ref_yaw = 0.0f;

struct Frustum
{
  float S, C, T, Cy;
};

static bool IsGL()
{
  switch (hw_render.context_type)
  {
  case RETRO_HW_CONTEXT_OPENGL:
  case RETRO_HW_CONTEXT_OPENGL_CORE:
  case RETRO_HW_CONTEXT_OPENGLES3:
  case RETRO_HW_CONTEXT_OPENGLES_VERSION:
    return true;
  default:
    return false;
  }
}

static void PollStatus()
{
  // assume all true until API converged
  s_presents = true;
  s_stereo = true;
  s_hmd = true;

  /*unsigned flags = 0;
  if (!environ_cb || !environ_cb(RETRO_ENVIRONMENT_GET_VIDEO_VIEWS_STATUS, &flags))
    flags = 0;
  s_hmd = (flags & RETRO_VIDEO_VIEWS_STATUS_HMD) != 0;

  DEBUG_LOG_FMT(BOOT, "Views: s_presents = {} s_stereo = {} s_hmd = {}", s_presents, s_stereo, s_hmd);*/
}

// The frontend wants stereo and we could in principle supply it.
static bool WantsStereo()
{
  INFO_LOG_FMT(BOOT, "Views: s_stereo = {} IsGL() = {} WantsStereo() = {}", s_stereo, IsGL(),
    Libretro::Options::GetCached<bool>(Libretro::Options::retroarch_core::STEREO_VIEWS, true));
  return s_stereo && IsGL() &&
         Libretro::Options::GetCached<bool>(Libretro::Options::retroarch_core::STEREO_VIEWS, true);
}

// Backend info is only filled once the context exists; be optimistic until then.
static bool GeometryShadersOk()
{
  return !g_context_status.IsInitialized() || g_backend_info.bSupportsGeometryShaders;
}

void Reset()
{
  s_presents = s_stereo = s_hmd = false;
  s_map_sent = false;
  s_sbs_applied = false;
  s_reserved = 1;
  s_mult = 1;
  s_have_frame_state = false;
  s_last_rec_w = s_last_rec_h = 0;
  s_have_ref = false;
  HeadTracking::Clear();
}

unsigned GetFrameWidthMultiplier() { return s_mult.load(std::memory_order_relaxed); }
bool IsStereoPacked() { return GetFrameWidthMultiplier() == 2; }

unsigned GetReservedWidthMultiplier()
{
  PollStatus();
  if (WantsStereo())
    s_reserved = 2;
  return s_reserved;
}

bool HasFrameState() { return s_have_frame_state; }
const retro_vr_frame_state& GetFrameState() { return s_frame_state; }

static void ClearMap()
{
  if (!s_map_sent)
    return;
  retro_video_views map{};
  map.num_views = 0;
  environ_cb(RETRO_ENVIRONMENT_SET_VIDEO_VIEWS, &map);
  s_map_sent = false;
}

static void SendMap(float display_aspect)
{
  if (!s_presents || !IsGL())
  {
    INFO_LOG_FMT(VIDEO, "Views: not sending map because s_presents = {} IsGL() = {}", s_presents, IsGL());
    ClearMap();
    return;
  }

  const int efb_scale =
      Libretro::Options::GetCached<int>(Libretro::Options::gfx_settings::EFB_SCALE, 1);
  const unsigned mult = GetFrameWidthMultiplier();
  const unsigned eye_w = EFB_WIDTH * efb_scale;
  const unsigned h = GetAdjustedBaseHeight() * efb_scale;

  retro_video_view views[2] = {};
  retro_video_views map{};
  map.views = views;
  map.flags = Libretro::Options::GetCached<bool>(
                  Libretro::Options::retroarch_core::REQUEST_FLAT, false) ?
                  RETRO_VIDEO_VIEWS_FLAG_REQUEST_FLAT :
                  0;
  map.ipd_hint_m = 0.0f;  // let the frontend/runtime decide
  // map.reference_space: left zero-initialised, see notes.

  views[0] = {0, 0, eye_w, h, 0,
            static_cast<unsigned int>(
                mult == 2 ? RETRO_VIDEO_VIEW_EYE_LEFT
                          : RETRO_VIDEO_VIEW_EYE_NONE),
            display_aspect};
  map.num_views = 1;
  if (mult == 2)
  {
    views[1] = {eye_w, 0, eye_w, h, 0, RETRO_VIDEO_VIEW_EYE_RIGHT, display_aspect};
    map.num_views = 2;
  }

  s_map_sent = environ_cb(RETRO_ENVIRONMENT_SET_VIDEO_VIEWS, &map);

  if (s_map_sent && s_hmd &&
      (map.recommended_view_width != s_last_rec_w || map.recommended_view_height != s_last_rec_h))
  {
    s_last_rec_w = map.recommended_view_width;
    s_last_rec_h = map.recommended_view_height;
    INFO_LOG_FMT(VIDEO, "Views: frontend recommends {}x{} per eye", s_last_rec_w, s_last_rec_h);
  }

  INFO_LOG_FMT(VIDEO, "Views: mult = {}", mult);
  INFO_LOG_FMT(VIDEO, "Views: map.flags = {}", map.flags);
  INFO_LOG_FMT(VIDEO, "Views: map.num_views = {}", map.num_views);
  for (unsigned i = 0; i < map.num_views; ++i)
  {
    const retro_video_view& v = map.views[i];
    INFO_LOG_FMT(VIDEO, "Views: view[{}] = {{x={}, y={}, w={}, h={}}}", i, v.x, v.y, v.width, v.height);
  }
}

// x_ndc = (S*x + C*z)/-z, y_ndc = (T*y + Cy*z)/-z, from tangents {left, right, up, down}.
static Frustum MakeFrustum(const float t[4])
{
  const float lr = t[0] + t[1];
  const float ud = t[2] + t[3];
  return {2.0f / lr, (t[1] - t[0]) / lr, 2.0f / ud, (t[2] - t[3]) / ud};
}

static bool ValidTangents(const float t[4])
{
  return t[0] > 0.0f && t[1] > 0.0f && t[2] > 0.0f && t[3] > 0.0f;
}

static void UpdateHeadTracking()
{
  using namespace Libretro::Options;

  if (!s_hmd || !IsGL() || !s_have_frame_state ||
      !GetCached<bool>(retroarch_core::HEAD_TRACKING, true))
  {
    HeadTracking::Clear();
    s_have_ref = false;
    return;
  }

  retro_vr_head_pose hp{};
  if (!environ_cb(RETRO_ENVIRONMENT_GET_VR_HEAD_POSE, &hp) ||
      !(hp.flags & RETRO_VR_HEAD_POSE_POSITION_VALID) ||
      !(hp.flags & RETRO_VR_HEAD_POSE_ORIENTATION_VALID))
  {
    HeadTracking::Clear();
    s_have_ref = false;
    return;
  }

  const retro_vr_eye_state& e0 = s_frame_state.eyes[0];
  const retro_vr_eye_state& e1 = s_frame_state.eyes[1];
  if (!ValidTangents(e0.fov_tan) || !ValidTangents(e1.fov_tan))
  {
    HeadTracking::Clear();
    return;
  }

  // Head-to-tracking rotation from the (x, y, z, w) quaternion.
  const float x = hp.orientation[0], y = hp.orientation[1], z = hp.orientation[2],
              w = hp.orientation[3];
  const float R[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
                         {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
                         {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}};
  const float yaw = std::atan2(R[0][2], R[2][2]);

  // Camera origin = midpoint between the eyes.
  const float mid[3] = {0.5f * (e0.position[0] + e1.position[0]),
                        0.5f * (e0.position[1] + e1.position[1]),
                        0.5f * (e0.position[2] + e1.position[2])};

  if (!s_have_ref || (s_frame_state.flags & RETRO_VR_FRAME_RECENTERED))
  {
    std::copy_n(mid, 3, s_ref_pos);
    s_ref_yaw = yaw;
    s_have_ref = true;
  }

  // Remove the reference yaw about +Y: Ry(-ref).
  const float c = std::cos(s_ref_yaw), s = std::sin(s_ref_yaw);
  const float Ry[3][3] = {{c, 0, -s}, {0, 1, 0}, {s, 0, c}};

  const float scale = static_cast<float>(GetCached<int>(retroarch_core::VR_WORLD_SCALE, 1));
  const float d[3] = {mid[0] - s_ref_pos[0], mid[1] - s_ref_pos[1], mid[2] - s_ref_pos[2]};

  float p[3];
  float Rr[3][3];
  for (int i = 0; i < 3; i++)
  {
    p[i] = scale * (Ry[i][0] * d[0] + Ry[i][1] * d[1] + Ry[i][2] * d[2]);
    for (int j = 0; j < 3; j++)
      Rr[i][j] = Ry[i][0] * R[0][j] + Ry[i][1] * R[1][j] + Ry[i][2] * R[2][j];
  }

  HeadTracking::State st;
  st.active = true;

  // View = inverse of the relative head transform: [Rr^T | -Rr^T p].
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
      st.view[i * 4 + j] = Rr[j][i];
    st.view[i * 4 + 3] = -(Rr[0][i] * p[0] + Rr[1][i] * p[1] + Rr[2][i] * p[2]);
  }

  // Mono frustum = average of the two eyes' tangents.
  float avg[4];
  for (int i = 0; i < 4; i++)
    avg[i] = 0.5f * (e0.fov_tan[i] + e1.fov_tan[i]);
  st.tan_left = avg[0];
  st.tan_right = avg[1];
  st.tan_up = avg[2];
  st.tan_down = avg[3];
  st.has_fov = true;

  // Per-eye correction relative to the mono frustum (applied in the geometry shader).
  const Frustum mono = MakeFrustum(avg);
  const retro_vr_eye_state* eyes_in[2] = {&e0, &e1};
  for (int i = 0; i < 2; i++)
  {
    const Frustum f = MakeFrustum(eyes_in[i]->fov_tan);
    const float de[3] = {eyes_in[i]->position[0] - mid[0], eyes_in[i]->position[1] - mid[1],
                         eyes_in[i]->position[2] - mid[2]};
    // Offset along the head's right axis (first column of R), in game units.
    const float e = scale * (R[0][0] * de[0] + R[1][0] * de[1] + R[2][0] * de[2]);

    HeadTracking::Eye& o = st.eyes[i];
    o.kx = f.S / mono.S;
    o.ax = f.S * mono.C / mono.S - f.C;
    o.bx = -f.S * e;
    o.ky = f.T / mono.T;
    o.ay = f.T * mono.Cy / mono.T - f.Cy;
  }
  st.has_eyes = true;

  HeadTracking::Set(st);
}

void Update(float display_aspect)
{
  PollStatus();

  const bool want_sbs = WantsStereo() && GeometryShadersOk();

  INFO_LOG_FMT(BOOT, "Views: want_sbs = {} WantsStereo() = {} GeGeometryShadersOk() = {}", want_sbs, WantsStereo(), GeometryShadersOk());

  const unsigned want_mult = want_sbs ? 2 : 1;

  if (want_sbs != s_sbs_applied)
  {
    INFO_LOG_FMT(BOOT, "Views: setting StereoMode to {}", want_sbs ? "SideBySide" : "Off");

    Config::SetCurrent(Config::GFX_STEREO_MODE, want_sbs ? StereoMode::SideBySide : StereoMode::Off);
    s_sbs_applied = want_sbs;
  }

  if (want_mult != GetFrameWidthMultiplier())
  {
    s_mult = want_mult;
    const bool grew = want_mult > s_reserved;
    if (grew)
      s_reserved = want_mult;

    // The GL context re-derives its backbuffer size in GLContext::Update().
    if (g_presenter)
      g_presenter->ResizeSurface();

    retro_system_av_info info;
    retro_get_system_av_info(&info);
    if (grew)  // beyond the reserved max_width: needs a full reinit
      environ_cb(RETRO_ENVIRONMENT_SET_SYSTEM_AV_INFO, &info);
    environ_cb(RETRO_ENVIRONMENT_SET_GEOMETRY, &info);
  }

  if (s_hmd)
  {
    retro_vr_frame_state fs{};
    s_have_frame_state = environ_cb(RETRO_ENVIRONMENT_GET_VR_FRAME_STATE, &fs);
    if (s_have_frame_state)
    {
      s_frame_state = fs;
      if (fs.flags & RETRO_VR_FRAME_RECENTERED)
        DEBUG_LOG_FMT(VIDEO, "Views: user recentered");  // TODO phase 2: reset camera reference
      // RETRO_VR_FRAME_TARGET_RESIZED: map is resent below every frame anyway.
    }
  }
  else
  {
    s_have_frame_state = false;
  }

  UpdateHeadTracking();

  SendMap(display_aspect);
}
}  // namespace Libretro::Video::Views
