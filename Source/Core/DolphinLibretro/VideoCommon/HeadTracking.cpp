#include <atomic>
#include <cmath>
#include <mutex>

#include "DolphinLibretro/VideoCommon/HeadTracking.h"

namespace HeadTracking
{
static std::mutex s_lock;
static State s_state;
static std::atomic<u64> s_generation{1};
static std::atomic<u64> s_eye_generation{1};

static bool EyesDiffer(const State& a, const State& b)
{
  if (a.has_eyes != b.has_eyes)
    return true;
  if (!a.has_eyes)
    return false;
  const auto d = [](float x, float y) { return std::fabs(x - y) > 1e-5f; };
  for (int i = 0; i < 2; i++)
  {
    const Eye& p = a.eyes[i];
    const Eye& q = b.eyes[i];
    if (d(p.kx, q.kx) || d(p.ax, q.ax) || d(p.bx, q.bx) || d(p.ky, q.ky) || d(p.ay, q.ay))
      return true;
  }
  return false;
}

void Set(State state)
{
  std::lock_guard guard{s_lock};
  state.generation = s_generation.fetch_add(1, std::memory_order_relaxed) + 1;
  if (EyesDiffer(state, s_state))
    s_eye_generation.fetch_add(1, std::memory_order_relaxed);
  s_state = state;
}

void Clear()
{
  std::lock_guard guard{s_lock};
  if (!s_state.active)
    return;
  if (s_state.has_eyes)
    s_eye_generation.fetch_add(1, std::memory_order_relaxed);
  s_state = State{};
  s_state.generation = s_generation.fetch_add(1, std::memory_order_relaxed) + 1;
}

u64 Generation()
{
  return s_generation.load(std::memory_order_relaxed);
}

u64 EyeGeneration()
{
  return s_eye_generation.load(std::memory_order_relaxed);
}

State Get()
{
  std::lock_guard guard{s_lock};
  return s_state;
}
}  // namespace HeadTracking
