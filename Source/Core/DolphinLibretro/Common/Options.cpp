
#include <libretro.h>

#include "DolphinLibretro/Common/Options.h"

namespace Libretro
{
namespace Options
{
static std::vector<retro_variable> optionsList;
static std::vector<bool*> dirtyPtrList;

template <typename T>
void Option<T>::Register()
{
  if (!m_options.empty())
    return;

  m_options = m_name;
  m_options.push_back(';');
  for (auto& option : m_list)
  {
    if (option.first == m_list.begin()->first)
      m_options += std::string(" ") + option.first;
    else
      m_options += std::string("|") + option.first;
  }
  optionsList.push_back({m_id, m_options.c_str()});
  dirtyPtrList.push_back(&m_dirty);
  Updated();
  m_dirty = true;
}

template <typename T>
Option<T>::Option(const char* id, const char* name,
                  const std::vector<std::pair<std::string, T>>& list)
  : m_id(id), m_name(name), m_list(list)
{
  Register();
}

template <>
void Option<PowerPC::CPUCore>::FilterForJitCapability()
{
  bool can_jit = false;
  if (Libretro::environ_cb &&
      Libretro::environ_cb(RETRO_ENVIRONMENT_GET_JIT_CAPABLE, &can_jit) &&
      !can_jit)
  {
    m_list.erase(
      std::remove_if(m_list.begin(), m_list.end(),
                     [](const auto& p) {
#if defined(_M_X86_64)
                       return p.second == PowerPC::CPUCore::JIT64;
#elif defined(_M_ARM_64)
                       return p.second == PowerPC::CPUCore::JITARM64;
#else
                       return false;
#endif
                     }),
      m_list.end());

    Libretro::Options::cpu_core.Rebuild();
    Libretro::Options::SetVariables();
  }
}

static std::vector<std::pair<std::string, PowerPC::CPUCore>> BuildCPUCoreList()
{
  std::vector<std::pair<std::string, PowerPC::CPUCore>> list;

#if defined(_M_X86_64)
  list.emplace_back("JIT64", PowerPC::CPUCore::JIT64);
#endif
#if defined(_M_ARM_64)
  list.emplace_back("JITARM64", PowerPC::CPUCore::JITARM64);
#endif

  list.emplace_back("Interpreter", PowerPC::CPUCore::Interpreter);
  list.emplace_back("Cached Interpreter", PowerPC::CPUCore::CachedInterpreter);

  return list;
}

void SetVariables()
{
  if (optionsList.empty())
    return;
  if (optionsList.back().key)
    optionsList.push_back({});
  ::Libretro::environ_cb(RETRO_ENVIRONMENT_SET_VARIABLES, (void*)optionsList.data());
}

void CheckVariables()
{
  bool updated = false;
  if (::Libretro::environ_cb(RETRO_ENVIRONMENT_GET_VARIABLE_UPDATE, &updated) && !updated)
    return;

  for (bool* ptr : dirtyPtrList)
    *ptr = true;
}

template <typename T>
Option<T>::Option(const char* id, const char* name,
                  std::initializer_list<std::pair<const char*, T>> list)
    : m_id(id), m_name(name), m_list(list.begin(), list.end())
{
  Register();
}

template <typename T>
Option<T>::Option(const char* id, const char* name, std::initializer_list<const char*> list)
    : m_id(id), m_name(name)
{
  for (auto option : list)
    m_list.push_back({option, (T)m_list.size()});
  Register();
}
template <>
Option<std::string>::Option(const char* id, const char* name,
                            std::initializer_list<const char*> list)
    : m_id(id), m_name(name)
{
  for (auto option : list)
    m_list.push_back({option, option});
  Register();
}
template <>
Option<const char*>::Option(const char* id, const char* name,
                            std::initializer_list<const char*> list)
    : m_id(id), m_name(name)
{
  for (auto option : list)
    m_list.push_back({option, option});
  Register();
}

template <typename T>
Option<T>::Option(const char* id, const char* name, T first,
                  std::initializer_list<const char*> list)
    : m_id(id), m_name(name)
{
  for (auto option : list)
    m_list.push_back({option, first + (int)m_list.size()});
  Register();
}

template <typename T>
Option<T>::Option(const char* id, const char* name, T first, int count, int step)
    : m_id(id), m_name(name)
{
  for (T i = first; i < first + count; i += step)
    m_list.push_back({std::to_string(i), i});
  Register();
}

template <>
Option<bool>::Option(const char* id, const char* name, bool initial) : m_id(id), m_name(name)
{
  m_list.push_back({initial ? "enabled" : "disabled", initial});
  m_list.push_back({!initial ? "enabled" : "disabled", !initial});
  Register();
}

template <>
Option<int>::Option(const char* id, const char* name, int initial)
    : m_id(id), m_name(name)
{
    m_list.push_back({std::to_string(initial), initial});
    Register();
}

Option<std::string> renderer("dolphin_renderer", "Renderer", {"Hardware"
#if defined(_DEBUG) || defined(DEBUGFAST)
    , "Software", "Null"
#endif
});

// Main.Core
Option<PowerPC::CPUCore> cpu_core("dolphin_cpu_core", "CPU Core", BuildCPUCoreList());
Option<float> cpuClockRate("dolphin_cpu_clock_rate", "CPU Clock Rate",
                           {{"100%", 1.0f}, {"150%", 1.5f}, {"200%", 2.0f}, {"250%", 2.5f}, {"300%", 3.0f},
                            {"5%", 0.05f}, {"10%", 0.1f}, {"20%", 0.2f}, {"30%", 0.3f}, {"40%", 0.4f},
                            {"50%", 0.5f}, {"60%", 0.6f}, {"70%", 0.7f}, {"80%", 0.8f}, {"90%", 0.9f}});
Option<float> EmulationSpeed("dolphin_emulation_speed", "Emulation Speed",
                             {{"unlimited", 0.0f}, {"100%", 1.0f}});
Option<bool> fastmem("dolphin_fastmem", "Fastmem", true);
Option<bool> DSPHLE("dolphin_dsp_hle", "DSP HLE", true);
Option<bool> cheatsEnabled("dolphin_cheats_enabled", "Internal Cheats Enabled", false);
Option<DiscIO::Language> Language("dolphin_language", "Language",
                                  {{"English", DiscIO::Language::English},
                                   {"Japanese", DiscIO::Language::Japanese},
                                   {"German", DiscIO::Language::German},
                                   {"French", DiscIO::Language::French},
                                   {"Spanish", DiscIO::Language::Spanish},
                                   {"Italian", DiscIO::Language::Italian},
                                   {"Dutch", DiscIO::Language::Dutch},
                                   {"Simplified Chinese", DiscIO::Language::SimplifiedChinese},
                                   {"Traditional Chinese", DiscIO::Language::TraditionalChinese},
                                   {"Korean", DiscIO::Language::Korean}});
Option<bool> fastDiscSpeed("dolphin_fast_disc_speed", "Speed Up Disc Transfer Rate", false);
Option<bool> WiimoteContinuousScanning("dolphin_wiimote_continuous_scanning", "Wiimote Continuous Scanning", false);
Option<bool> altGCPorts("dolphin_alt_gc_ports_on_wii", "Use ports 5-8 for GameCube controllers in Wii mode", false);
Option<unsigned int> audioMixerRate("dolphin_mixer_rate", "Audio Mixer Rate",
                                    {{"32000", 32000u}, {"48000", 48000u}});

// Main.Interface
Option<bool> osdEnabled("dolphin_osd_enabled", "OSD Enabled", true);

// Main.General
Option<bool> mainTimeTracking("dolphin_time_tracking", "Time Tracking", false);

// Main.DSP
Option<bool> DSPEnableJIT("dolphin_dsp_jit", "DSP Enable JIT", true);

// SYSCONF.IPL
Option<bool> Widescreen("dolphin_widescreen", "Widescreen (Wii)", true);
Option<bool> progressiveScan("dolphin_progressive_scan", "Progressive Scan", true);
Option<bool> pal60("dolphin_pal60", "PAL60", true);

// SYSCONF.BT
Option<u32> sensorBarPosition("dolphin_sensor_bar_position", "Sensor Bar Position", {"Bottom", "Top"});
Option<bool> enableRumble("dolphin_enable_rumble", "Rumble", true);

// Graphics.Settings
Option<bool> WidescreenHack("dolphin_widescreen_hack", "WideScreen Hack", false);
Option<int> efbScale("dolphin_efb_scale", "Internal Resolution", 1,
                     {"x1 (640 x 528)", "x2 (1280 x 1056)", "x3 (1920 x 1584)",
                      "x4 (2560 x 2112)", "x5 (3200 x 2640)", "x6 (3840 x 3168)"});
Option<ShaderCompilationMode> shaderCompilationMode(
    "dolphin_shader_compilation_mode", "Shader Compilation Mode",
    {{"sync", ShaderCompilationMode::Synchronous},
     {"a-sync Skip Rendering", ShaderCompilationMode::AsynchronousSkipRendering},
     {"sync UberShaders", ShaderCompilationMode::SynchronousUberShaders},
     {"a-sync UberShaders", ShaderCompilationMode::AsynchronousUberShaders}});
Option<bool> waitForShaders("dolphin_wait_for_shaders", "Wait for Shaders before Starting", false);
Option<int> antiAliasing("dolphin_anti_aliasing", "Anti-Aliasing",
                         {"None", "2x MSAA", "4x MSAA", "8x MSAA", "2x SSAA", "4x SSAA", "8x SSAA"});
Option<int> textureCacheAccuracy("dolphin_texture_cache_accuracy", "Texture Cache Accuracy",
                                 {{"Fast", 128}, {"Middle", 512}, {"Safe", 0}});
Option<bool> loadCustomTextures("dolphin_load_custom_textures", "Load Custom Textures", false);
Option<bool> cacheCustomTextures("dolphin_cache_custom_textures", "Prefetch Custom Textures", false);
Option<bool> gpuTextureDecoding("dolphin_gpu_texture_decoding", "GPU Texture Decoding", false);
Option<bool> fastDepthCalc("dolphin_fast_depth_calculation", "Fast Depth Calculation", true);

// Graphics.Enhancements
Option<TextureFilteringMode> forceTextureFilteringMode("dolphin_force_texture_filtering_mode", "Force Texture Filtering Mode",
                                                       {{"Disabled", TextureFilteringMode::Default},
                                                        {"Nearest", TextureFilteringMode::Nearest},
                                                        {"Linear", TextureFilteringMode::Linear}});
Option<AnisotropicFilteringMode> maxAnisotropy("dolphin_max_anisotropy", "Max Anisotropy",
                                               {{"1x", AnisotropicFilteringMode::Force1x},
                                                {"2x", AnisotropicFilteringMode::Force2x},
                                                {"4x", AnisotropicFilteringMode::Force4x},
                                                {"8x", AnisotropicFilteringMode::Force8x},
                                                {"16x", AnisotropicFilteringMode::Force16x}});

// Graphics.Hacks
Option<bool> efbAccessEnable("dolphin_efb_access_enable", "EFB Access Enable", false);
Option<bool> efbAccessDeferInvalidation("dolphin_efb_access_defer_invalidation", "EFB Access Defer Invalidation", false);
Option<int> efbAccessTileSize("dolphin_efb_access_tile_size", "EFB Access Tile Size", 64);
Option<bool> bboxEnabled("dolphin_bbox_enabled", "Bounding Box Emulation", false);
Option<bool> forceProgressive("dolphin_force_progressive", "Force Progressive", true);
Option<bool> efbToTexture("dolphin_efb_to_texture", "Skip EFB Copy to RAM", true);
Option<bool> xfbToTextureEnable("dolphin_xfb_to_texture_enable", "Skip XFB Copy to RAM", true);
Option<bool> efbToVram("dolphin_efb_to_vram", "Disable EFB to VRAM", false);
Option<bool> deferEfbCopies("dolphin_defer_efb_copies", "Defer EFB Copies to RAM", true);
Option<bool> immediatexfb("dolphin_immediate_xfb", "Immediate XFB", false);
Option<bool> skipDupeFrames("dolphin_skip_dupe_frames", "Skip Presenting Duplicate Frames", true);
Option<bool> earlyXFBOutput("dolphin_early_xfb_output", "Early XFB Output", true);
Option<bool> efbScaledCopy("dolphin_efb_scaled_copy", "Scaled EFB Copy", true);
Option<bool> efbEmulateFormatChanges("dolphin_efb_emulate_format_changes", "EFB Emulate Format Changes", false);
Option<bool> vertexRounding("dolphin_vertex_rounding", "Vertex Rounding", false);
Option<bool> viSkip("dolphin_vi_skip", "VI Skip", false);
//Option<u32> missingColorValue("dolphin_missing_color_value", "Missing Color Value", static_cast<u32>(0xFFFFFFFFu));
Option<bool> fastTextureSampling("dolphin_fast_texture_sampling", "Fast Texture Sampling", true);
#ifdef __APPLE__
Option<bool> noMipmapping("dolphin_no_mipmapping", "Disable Mipmapping", false);
#endif

// Wiimote IR
Option<int> irMode("dolphin_ir_mode", "Wiimote IR Mode", 1,
                   {"Right Stick controls pointer (relative)", "Right Stick controls pointer (absolute)", "Mouse controls pointer"});
Option<int> irCenter("dolphin_ir_offset", "Wiimote IR Vertical Offset",
    {{"10", 10}, {"11", 11}, {"12", 12}, {"13", 13}, {"14", 14}, {"15", 15}, {"16", 16}, {"17", 17}, {"18", 18}, {"19", 19},
     {"20", 20}, {"21", 21}, {"22", 22}, {"23", 23}, {"24", 24}, {"25", 25}, {"26", 26}, {"27", 27}, {"28", 28}, {"29", 29},
     {"30", 30}, {"31", 31}, {"32", 32}, {"33", 33}, {"34", 34}, {"35", 35}, {"36", 36}, {"37", 37}, {"38", 38}, {"39", 39},
     {"40", 40}, {"41", 41}, {"42", 42}, {"43", 43}, {"44", 44}, {"45", 45}, {"46", 46}, {"47", 47}, {"48", 48}, {"49", 49},
     {"50", 50}, {"-50", -50}, {"-49", -49}, {"-48", -48}, {"-47", -47}, {"-46", -46}, {"-45", -45}, {"-44", -44}, {"-43", -43},
     {"-42", -42}, {"-41", -41}, {"-40", -40}, {"-39", -39}, {"-38", -38}, {"-37", -37}, {"-36", -36}, {"-35", -35}, {"-34", -34},
     {"-33", -33}, {"-32", -32}, {"-31", -31}, {"-30", -30}, {"-29", -29}, {"-28", -28}, {"-27", -27}, {"-26", -26}, {"-25", -25},
     {"-24", -24}, {"-23", -23}, {"-22", -22}, {"-21", -21}, {"-20", -20}, {"-19", -19}, {"-18", -18}, {"-17", -17}, {"-16", -16},
     {"-15", -15}, {"-14", -14}, {"-13", -13}, {"-12", -12}, {"-11", -11}, {"-10", -10}, {"-9", -9}, {"-8", -8}, {"-7", -7},
     {"-6", -6}, {"-5", -5}, {"-4", -4}, {"-3", -3}, {"-2", -2}, {"-1", -1}, {"0", 0}, {"1", 1}, {"2", 2}, {"3", 3}, {"4", 4},
     {"5", 5}, {"6", 6}, {"7", 7}, {"8", 8}, {"9", 9}});
Option<int> irWidth("dolphin_ir_yaw", "Wiimote IR Total Yaw",
    {{"15", 15}, {"16", 16}, {"17", 17}, {"18", 18}, {"19", 19}, {"20", 20}, {"21", 21}, {"22", 22}, {"23", 23}, {"24", 24},
     {"25", 25}, {"26", 26}, {"27", 27}, {"28", 28}, {"29", 29}, {"30", 30}, {"31", 31}, {"32", 32}, {"33", 33}, {"34", 34},
     {"35", 35}, {"36", 36}, {"37", 37}, {"38", 38}, {"39", 39}, {"40", 40}, {"41", 41}, {"42", 42}, {"43", 43}, {"44", 44},
     {"45", 45}, {"46", 46}, {"47", 47}, {"48", 48}, {"49", 49}, {"50", 50}, {"51", 51}, {"52", 52}, {"53", 53}, {"54", 54},
     {"55", 55}, {"56", 56}, {"57", 57}, {"58", 58}, {"59", 59}, {"60", 60}, {"61", 61}, {"62", 62}, {"63", 63}, {"64", 64},
     {"65", 65}, {"66", 66}, {"67", 67}, {"68", 68}, {"69", 69}, {"70", 70}, {"71", 71}, {"72", 72}, {"73", 73}, {"74", 74},
     {"75", 75}, {"76", 76}, {"77", 77}, {"78", 78}, {"79", 79}, {"80", 80}, {"81", 81}, {"82", 82}, {"83", 83}, {"84", 84},
     {"85", 85}, {"86", 86}, {"87", 87}, {"88", 88}, {"89", 89}, {"90", 90}, {"91", 91}, {"92", 92}, {"93", 93}, {"94", 94},
     {"95", 95}, {"96", 96}, {"97", 97}, {"98", 98}, {"99", 99}, {"100", 100}, {"0", 0}, {"1", 1}, {"2", 2}, {"3", 3},
     {"4", 4}, {"5", 5}, {"6", 6}, {"7", 7}, {"8", 8}, {"9", 9}, {"10", 10}, {"11", 11}, {"12", 12}, {"13", 13}, {"14", 14}});
Option<int> irHeight("dolphin_ir_pitch", "Wiimote IR Total Pitch",
    {{"15", 15}, {"16", 16}, {"17", 17}, {"18", 18}, {"19", 19}, {"20", 20}, {"21", 21}, {"22", 22}, {"23", 23}, {"24", 24},
     {"25", 25}, {"26", 26}, {"27", 27}, {"28", 28}, {"29", 29}, {"30", 30}, {"31", 31}, {"32", 32}, {"33", 33}, {"34", 34},
     {"35", 35}, {"36", 36}, {"37", 37}, {"38", 38}, {"39", 39}, {"40", 40}, {"41", 41}, {"42", 42}, {"43", 43}, {"44", 44},
     {"45", 45}, {"46", 46}, {"47", 47}, {"48", 48}, {"49", 49}, {"50", 50}, {"51", 51}, {"52", 52}, {"53", 53}, {"54", 54},
     {"55", 55}, {"56", 56}, {"57", 57}, {"58", 58}, {"59", 59}, {"60", 60}, {"61", 61}, {"62", 62}, {"63", 63}, {"64", 64},
     {"65", 65}, {"66", 66}, {"67", 67}, {"68", 68}, {"69", 69}, {"70", 70}, {"71", 71}, {"72", 72}, {"73", 73}, {"74", 74},
     {"75", 75}, {"76", 76}, {"77", 77}, {"78", 78}, {"79", 79}, {"80", 80}, {"81", 81}, {"82", 82}, {"83", 83}, {"84", 84},
     {"85", 85}, {"86", 86}, {"87", 87}, {"88", 88}, {"89", 89}, {"90", 90}, {"91", 91}, {"92", 92}, {"93", 93}, {"94", 94},
     {"95", 95}, {"96", 96}, {"97", 97}, {"98", 98}, {"99", 99}, {"100", 100}, {"0", 0}, {"1", 1}, {"2", 2}, {"3", 3},
     {"4", 4}, {"5", 5}, {"6", 6}, {"7", 7}, {"8", 8}, {"9", 9}, {"10", 10}, {"11", 11}, {"12", 12}, {"13", 13}, {"14", 14}});

// Other
Option<Common::Log::LogLevel> logLevel("dolphin_log_level", "Log Level", {
                                      {"Info", Common::Log::LogLevel::LINFO},
#if defined(_DEBUG) || defined(DEBUGFAST)
                                      {"Debug", Common::Log::LogLevel::LDEBUG},
#endif
                                      {"Notice", Common::Log::LogLevel::LNOTICE},
                                      {"Error", Common::Log::LogLevel::LERROR},
                                      {"Warning", Common::Log::LogLevel::LWARNING}});
Option<bool> callBackAudio("dolphin_call_back_audio", "Use async audio", false);
}  // namespace Options
}  // namespace Libretro
