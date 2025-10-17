#pragma once

#include <cassert>
#include <libretro.h>
#include <string>
#include <vector>

#include <libretro.h>
#include "Common/Logging/Log.h"
#include "Core/PowerPC/PowerPC.h"
#include "DiscIO/Enums.h"
#include "VideoCommon/VideoConfig.h"

namespace Libretro
{
extern retro_environment_t environ_cb;

namespace Options
{
void SetVariables();
void CheckVariables();

template <typename T>
class Option
{
public:
  Option(const char* id, const char* name, std::initializer_list<std::pair<const char*, T>> list);
  Option(const char* id, const char* name, std::initializer_list<const char*> list);
  Option(const char* id, const char* name, T first, std::initializer_list<const char*> list);
  Option(const char* id, const char* name, T first, int count, int step = 1);
  Option(const char* id, const char* name, bool initial);
  Option(const char* id, const char* name,
    const std::vector<std::pair<std::string, T>>& list);
  Option(const char* id, const char* name, int initial);

  bool Updated()
  {
    if (m_dirty)
    {
      m_dirty = false;

      retro_variable var{m_id};
      T value = m_list.front().second;

      if (::Libretro::environ_cb && ::Libretro::environ_cb(RETRO_ENVIRONMENT_GET_VARIABLE, &var) && var.value)
      {
        for (auto option : m_list)
        {
          if (option.first == var.value)
          {
            value = option.second;
            break;
          }
        }
      }

      if (m_value != value)
      {
        m_value = value;
        return true;
      }
    }
    return false;
  }

  operator T() const
  {
    const_cast<Option*>(this)->Updated();
    return m_value;
  }

  template <typename S>
  bool operator==(S value)
  {
    return (T)(*this) == value;
  }

  template <typename S>
  bool operator!=(S value)
  {
    return (T)(*this) != value;
  }

  void Rebuild()
  {
    m_options = m_name;
    m_options.push_back(';');

    bool first = true;
    for (auto& option : m_list)
    {
      if (first) {
        m_options += " " + option.first;
        first = false;
      } else {
        m_options += "|" + option.first;
      }
    }
  }

  T Get() const;
  void FilterForJitCapability();
private:
  void Register();

  const char* m_id;
  const char* m_name;
  T m_value;
  bool m_dirty = true;
  std::string m_options;
  std::vector<std::pair<std::string, T>> m_list;
};

// Core
extern Option<PowerPC::CPUCore> cpu_core;
extern Option<float> cpuClockRate;
extern Option<float> EmulationSpeed;
extern Option<bool> fastmem;
extern Option<bool> cheatsEnabled;
extern Option<DiscIO::Language> Language;
extern Option<bool> fastDiscSpeed;

// DSP / Audio
extern Option<bool> DSPHLE;
extern Option<bool> DSPEnableJIT;
extern Option<unsigned int> audioMixerRate;

// Interface / General
extern Option<bool> osdEnabled;
extern Option<Common::Log::LogLevel> logLevel;

// SYSCONF
extern Option<bool> Widescreen;
extern Option<bool> progressiveScan;
extern Option<bool> pal60;
extern Option<u32> sensorBarPosition;
extern Option<bool> enableRumble;
extern Option<bool> WiimoteContinuousScanning;
extern Option<bool> altGCPorts;

// Graphics.Settings
extern Option<bool> WidescreenHack;
extern Option<std::string> renderer;
extern Option<int> efbScale;
extern Option<ShaderCompilationMode> shaderCompilationMode;
extern Option<bool> waitForShaders;
extern Option<int> antiAliasing;
extern Option<AnisotropicFilteringMode> maxAnisotropy;
extern Option<TextureFilteringMode> forceTextureFilteringMode;
extern Option<int> textureCacheAccuracy;
extern Option<bool> gpuTextureDecoding;
extern Option<bool> fastDepthCalc;

// Graphics.Enhancements
extern Option<bool> loadCustomTextures;
extern Option<bool> cacheCustomTextures;

// Graphics.Hacks
extern Option<bool> efbAccessEnable;
extern Option<bool> efbAccessDeferInvalidation;
extern Option<int> efbAccessTileSize;
extern Option<bool> bboxEnabled;
extern Option<bool> forceProgressive;
extern Option<bool> efbToTexture;
extern Option<bool> xfbToTextureEnable;
extern Option<bool> efbToVram;
extern Option<bool> deferEfbCopies;
extern Option<bool> immediatexfb;
extern Option<bool> skipDupeFrames;
extern Option<bool> earlyXFBOutput;
extern Option<bool> efbScaledCopy;
extern Option<bool> efbEmulateFormatChanges;
extern Option<bool> vertexRounding;
extern Option<bool> viSkip;
extern Option<u32> missingColorValue;
extern Option<bool> fastTextureSampling;
extern Option<bool> noMipmapping;

// Wiimote IR
extern Option<int> irMode;
extern Option<int> irCenter;
extern Option<int> irWidth;
extern Option<int> irHeight;

// Other
extern Option<bool> callBackAudio;

template <typename T>
T Libretro::Options::Option<T>::Get() const
{
  struct retro_variable var = {m_id, nullptr};
  if (environ_cb && environ_cb(RETRO_ENVIRONMENT_GET_VARIABLE, &var) && var.value)
  {
    std::string selected = var.value;
    for (const auto& pair : m_list)
    {
      if (pair.first == selected)
        return pair.second;
    }
  }
  return m_list.empty() ? T{} : m_list.front().second;
}

}  // namespace Options
}  // namespace Libretro
