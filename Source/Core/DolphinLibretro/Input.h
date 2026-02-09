#pragma once

#include <libretro.h>
#include "Common/WindowSystemInfo.h"

namespace Libretro
{
namespace Input
{
static retro_sensor_interface sensor_interface = {};
void Init(const WindowSystemInfo& wsi);
void InitStage2();
void Update();
void Shutdown();
void ResetControllers();
void BluetoothPassthroughBind();

class AccelInput : public ciface::Core::Device::Input
{
public:
  AccelInput(unsigned port, int axis, const char* name)
      : m_port(port), m_axis(axis), m_name(name)
  {
  }

  std::string GetName() const override { return m_name; }

  ControlState GetState() const override
  {
    if (!sensor_interface.get_sensor_input)
      return 0.0;

    // RETRO_SENSOR_ACCELEROMETER_X = 0, Y = 1, Z = 2
    float value = sensor_interface.get_sensor_input(m_port, RETRO_SENSOR_ACCELEROMETER_X + m_axis);

    if(value != 0.00)
    {
      fprintf(stderr, "Accel raw port=%u axis=%d value=%f\n", m_port, m_axis, value);
      fflush(stdout);
    }

    float x = sensor_interface.get_sensor_input(m_port, RETRO_SENSOR_ACCELEROMETER_X);
    float y = sensor_interface.get_sensor_input(m_port, RETRO_SENSOR_ACCELEROMETER_Y);
    float z = sensor_interface.get_sensor_input(m_port, RETRO_SENSOR_ACCELEROMETER_Z);

    if(x != 0.00 || y != 0.00 || z != 0.00)
    {
      fprintf(stderr, "Accel raw x=%f y=%f z=%f\n", x, y, z);
      fflush(stdout);
    }

    // normalize to ±1 g
    return value / 9.8f;
  }

private:
  const unsigned m_port;
  const int m_axis; // 0=X, 1=Y, 2=Z
  const char* m_name;
};

class GyroInput : public ciface::Core::Device::Input
{
public:
  GyroInput(unsigned port, int axis, const char* name)
      : m_port(port), m_axis(axis), m_name(name)
  {
  }

  std::string GetName() const override { return m_name; }

  ControlState GetState() const override
  {
    if (!sensor_interface.get_sensor_input)
      return 0.0;

    // RETRO_SENSOR_GYROSCOPE_X = 3, Y = 4, Z = 5
    float value = sensor_interface.get_sensor_input(m_port, RETRO_SENSOR_GYROSCOPE_X + m_axis);

    // Gyro returns rad/s, might want to normalize
    return value;
  }

private:
  const unsigned m_port;
  const int m_axis; // 0=X, 1=Y, 2=Z
  const char* m_name;
};

} // namespace Input
} // namespace Libretro
