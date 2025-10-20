// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/PowerPC/JitArm32/JitFPRCache.h"

#include <algorithm>
#include <cstddef>
#include <vector>

#include "Common/Assert.h"
#include "Common/BitSet.h"
#include "Common/CommonTypes.h"
#include "Common/Logging/Log.h"

using namespace ArmGen;

ArmFPRCache::ArmFPRCache()
  : m_guest_registers_ps0(GUEST_FPR_COUNT),
    m_guest_registers_ps1(GUEST_FPR_COUNT)
{
}

void ArmFPRCache::Init(ArmGen::ARMXEmitter* emitter)
{
  m_emit = emitter;
  GetAllocationOrder();
}

void ArmFPRCache::DiscardRegisters(BitSet32 regs)
{
  for (int i : regs)
  {
    DiscardRegister(i, false);
    DiscardRegister(i, true);
  }
}

void ArmFPRCache::ResetRegisters(BitSet32 regs)
{
  for (int i : regs)
  {
    FPROpArg& reg_ps0 = m_guest_registers_ps0[i];
    FPROpArg& reg_ps1 = m_guest_registers_ps1[i];

    ARMReg host_reg_ps0 = reg_ps0.GetReg();
    ARMReg host_reg_ps1 = reg_ps1.GetReg();

    ASSERT_MSG(DYNA_REC, host_reg_ps0 == ARMReg::INVALID_REG &&
                         host_reg_ps1 == ARMReg::INVALID_REG,
               "Attempted to reset a loaded register (did you mean to flush it?)");

    reg_ps0.Flush();
    reg_ps1.Flush();
  }
}

ARMReg ArmFPRCache::GetReg()
{
  // If we have no registers left, dump the most stale register first
  if (GetUnlockedRegisterCount() == 0)
    FlushMostStaleRegister();

  for (auto& it : m_host_registers)
  {
    if (!it.IsLocked())
    {
      it.Lock();
      return it.GetReg();
    }
  }

  ASSERT_MSG(DYNA_REC, 0, "All available FPR registers are locked!");
  return ARMReg::INVALID_REG;
}

void ArmFPRCache::UpdateLastUsed(BitSet32 regs_used)
{
  for (size_t i = 0; i < GUEST_FPR_COUNT; ++i)
  {
    FPROpArg& reg_ps0 = m_guest_registers_ps0[i];
    FPROpArg& reg_ps1 = m_guest_registers_ps1[i];

    if (i < 32 && regs_used[i])
    {
      reg_ps0.ResetLastUsed();
      reg_ps1.ResetLastUsed();
    }
    else
    {
      reg_ps0.IncrementLastUsed();
      reg_ps1.IncrementLastUsed();
    }
  }
}

u32 ArmFPRCache::GetUnlockedRegisterCount() const
{
  u32 unlocked_registers = 0;
  for (const auto& it : m_host_registers)
  {
    if (!it.IsLocked())
      ++unlocked_registers;
  }
  return unlocked_registers;
}

void ArmFPRCache::LockRegister(ARMReg host_reg)
{
  auto reg = std::find_if(m_host_registers.begin(), m_host_registers.end(),
                          [host_reg](const HostReg& r) { return r.GetReg() == host_reg; });
  ASSERT_MSG(DYNA_REC, reg != m_host_registers.end(),
             "Don't try locking a register that isn't in the cache. Reg {}",
             static_cast<int>(host_reg));
  reg->Lock();
}

void ArmFPRCache::UnlockRegister(ARMReg host_reg)
{
  auto reg = std::find_if(m_host_registers.begin(), m_host_registers.end(),
                          [host_reg](const HostReg& r) { return r.GetReg() == host_reg; });
  ASSERT_MSG(DYNA_REC, reg != m_host_registers.end(),
             "Don't try unlocking a register that isn't in the cache. Reg {}",
             static_cast<int>(host_reg));
  reg->Unlock();
}

void ArmFPRCache::FlushMostStaleRegister()
{
  size_t most_stale_preg = 0;
  bool most_stale_is_ps1 = false;
  u32 most_stale_amount = 0;

  // Check PS0 registers
  for (size_t i = 0; i < GUEST_FPR_COUNT; ++i)
  {
    const auto& reg = m_guest_registers_ps0[i];
    const u32 last_used = reg.GetLastUsed();

    if (last_used > most_stale_amount && reg.IsInHostRegister())
    {
      most_stale_preg = i;
      most_stale_is_ps1 = false;
      most_stale_amount = last_used;
    }
  }

  // Check PS1 registers
  for (size_t i = 0; i < GUEST_FPR_COUNT; ++i)
  {
    const auto& reg = m_guest_registers_ps1[i];
    const u32 last_used = reg.GetLastUsed();

    if (last_used > most_stale_amount && reg.IsInHostRegister())
    {
      most_stale_preg = i;
      most_stale_is_ps1 = true;
      most_stale_amount = last_used;
    }
  }

  FlushRegister(most_stale_preg, most_stale_is_ps1, FlushMode::All);
}

void ArmFPRCache::DiscardRegister(size_t preg, bool ps1)
{
  FPROpArg& reg = ps1 ? m_guest_registers_ps1[preg] : m_guest_registers_ps0[preg];
  ARMReg host_reg = reg.GetReg();

  reg.Discard();
  if (host_reg != ARMReg::INVALID_REG)
    UnlockRegister(host_reg);
}

void ArmFPRCache::Flush(FlushMode mode, ARMReg tmp_reg, IgnoreDiscardedRegisters ignore_discarded_registers)
{
  for (size_t i = 0; i < GUEST_FPR_COUNT; ++i)
  {
    if (m_guest_registers_ps0[i].IsInHostRegister())
    {
      FlushRegister(i, false, mode);
    }
    else
    {
      ASSERT_MSG(DYNA_REC,
                 ignore_discarded_registers != IgnoreDiscardedRegisters::No ||
                     m_guest_registers_ps0[i].IsInPPCState(),
                 "Attempted to flush discarded PS0 register");
    }

    if (m_guest_registers_ps1[i].IsInHostRegister())
    {
      FlushRegister(i, true, mode);
    }
    else
    {
      ASSERT_MSG(DYNA_REC,
                 ignore_discarded_registers != IgnoreDiscardedRegisters::No ||
                     m_guest_registers_ps1[i].IsInPPCState(),
                 "Attempted to flush discarded PS1 register");
    }
  }
}

ARMReg ArmFPRCache::GetPPCReg(size_t preg, bool ps1, FPRRegType format,
                               bool for_write, bool set_dirty)
{
  FPROpArg& reg = ps1 ? m_guest_registers_ps1[preg] : m_guest_registers_ps0[preg];

  IncrementAllUsed();
  reg.ResetLastUsed();

  if (!reg.IsInHostRegister())
  {
    ASSERT_MSG(DYNA_REC, !for_write || reg.IsInPPCState(),
               "Attempted to read discarded register");

    ARMReg host_reg = GetReg();
    reg.Load(host_reg, format);
    reg.SetDirty(for_write && set_dirty);

    if (!for_write)
    {
      s16 offset = ps1 ? PPCSTATE_OFF_PS1(preg) : PPCSTATE_OFF_PS0(preg);
      m_emit->VLDR(host_reg, PPC_REG, offset);
    }

    return host_reg;
  }

  ARMReg host_reg = reg.GetReg();

  // Handle type conversions if needed
  // For simplicity, we'll just use the register as-is for ARM32
  // ARM64 version has complex single/double conversions which ARM32 VFP handles differently

  if (for_write && set_dirty)
  {
    reg.Load(host_reg, format);
    reg.SetDirty(true);
  }

  return host_reg;
}

ARMReg ArmFPRCache::R0(size_t preg, FPRRegType format)
{
  return GetPPCReg(preg, false, format, false, false);
}

ARMReg ArmFPRCache::R1(size_t preg, FPRRegType format)
{
  return GetPPCReg(preg, true, format, false, false);
}

ARMReg ArmFPRCache::RW0(size_t preg, FPRRegType format, bool set_dirty)
{
  return GetPPCReg(preg, false, format, true, set_dirty);
}

ARMReg ArmFPRCache::RW1(size_t preg, FPRRegType format, bool set_dirty)
{
  return GetPPCReg(preg, true, format, true, set_dirty);
}

void ArmFPRCache::GetAllocationOrder()
{
  // ARM VFP allocation order
  // Callee-saved: D8-D15 (must be preserved across function calls)
  // Caller-saved: D0-D7, D16-D31 (can be clobbered by function calls)

  static constexpr auto allocation_order = {
      // Callee saved - prefer these for register cache
      D8, D9, D10, D11, D12, D13, D14, D15,

      // Caller saved - use for temporaries
      D16, D17, D18, D19, D20, D21, D22, D23,
      D24, D25, D26, D27, D28, D29, D30, D31,
      D4, D5, D6, D7,
      D0, D1, D2, D3,
  };

  for (ARMReg reg : allocation_order)
    m_host_registers.push_back(HostReg(reg));
}

void ArmFPRCache::FlushByHost(ARMReg host_reg, ARMReg tmp_reg)
{
  // Check PS0 registers
  for (size_t i = 0; i < GUEST_FPR_COUNT; ++i)
  {
    const FPROpArg& reg = m_guest_registers_ps0[i];
    if (reg.IsInHostRegister() && reg.GetReg() == host_reg)
    {
      FlushRegister(i, false, FlushMode::All);
      return;
    }
  }

  // Check PS1 registers
  for (size_t i = 0; i < GUEST_FPR_COUNT; ++i)
  {
    const FPROpArg& reg = m_guest_registers_ps1[i];
    if (reg.IsInHostRegister() && reg.GetReg() == host_reg)
    {
      FlushRegister(i, true, FlushMode::All);
      return;
    }
  }
}

bool ArmFPRCache::IsCallerSaved(ARMReg reg) const
{
  // D0-D7 and D16-D31 are caller-saved in ARM VFP AAPCS
  int reg_num = static_cast<int>(reg);

  // D0-D7 are caller-saved
  if (reg >= D0 && reg <= D7)
    return true;

  // D16-D31 are caller-saved (if available on the CPU)
  if (reg >= D16 && reg <= D31)
    return true;

  // D8-D15 are callee-saved
  return false;
}

void ArmFPRCache::FlushRegister(size_t preg, bool ps1, FlushMode mode)
{
  FPROpArg& reg = ps1 ? m_guest_registers_ps1[preg] : m_guest_registers_ps0[preg];
  const ARMReg host_reg = reg.GetReg();
  const bool dirty = !reg.IsInPPCState();
  FPRRegType type = reg.GetFPRType();

  if (dirty)
  {
    s16 offset = ps1 ? PPCSTATE_OFF_PS1(preg) : PPCSTATE_OFF_PS0(preg);
    m_emit->VSTR(host_reg, PPC_REG, offset);
  }

  if (mode == FlushMode::All)
  {
    UnlockRegister(host_reg);
    reg.Flush();
  }
}

void ArmFPRCache::FlushRegisters(BitSet32 regs, FlushMode mode,
                                 ArmGen::ARMReg tmp_reg,
                                 IgnoreDiscardedRegisters ignore_discarded_registers)
{
  for (int j : regs)
  {
    FlushRegister(j, false, mode);
    FlushRegister(j, true, mode);
  }
}

BitSet32 ArmFPRCache::GetCallerSavedUsed() const
{
  BitSet32 registers(0);
  for (const auto& it : m_host_registers)
  {
    if (it.IsLocked() && IsCallerSaved(it.GetReg()))
    {
      ARMReg reg = it.GetReg();
      int reg_num = static_cast<int>(reg);
      if (reg_num >= 0 && reg_num < 32)
        registers[reg_num] = true;
    }
  }
  return registers;
}

bool ArmFPRCache::IsSingle(size_t preg, bool lower_only) const
{
  const FPRRegType type_ps0 = m_guest_registers_ps0[preg].GetFPRType();
  const FPRRegType type_ps1 = m_guest_registers_ps1[preg].GetFPRType();

  if (lower_only)
  {
    return type_ps0 == FPRRegType::Single ||
           type_ps0 == FPRRegType::DuplicatedSingle ||
           type_ps0 == FPRRegType::LowerPairSingle;
  }

  return (type_ps0 == FPRRegType::Single || type_ps0 == FPRRegType::DuplicatedSingle) &&
         (type_ps1 == FPRRegType::Single || type_ps1 == FPRRegType::DuplicatedSingle);
}
