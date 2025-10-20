// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/PowerPC/JitArm32/JitRegCache.h"

#include <algorithm>
#include <cstddef>
#include <vector>

#include "Common/Assert.h"
#include "Common/BitSet.h"
#include "Common/CommonTypes.h"
#include "Common/Logging/Log.h"

using namespace ArmGen;

// Base ArmRegCache implementation

ArmRegCache::ArmRegCache(size_t guest_reg_count) : m_guest_registers(guest_reg_count)
{
}

void ArmRegCache::Init(ArmGen::ARMXEmitter* emitter)
{
  m_emit = emitter;
  GetAllocationOrder();
}

void ArmRegCache::DiscardRegisters(BitSet32 regs)
{
  for (int i : regs)
    DiscardRegister(i);
}

void ArmRegCache::ResetRegisters(BitSet32 regs)
{
  for (int i : regs)
  {
    OpArg& reg = m_guest_registers[i];
    ARMReg host_reg = reg.GetReg();

    ASSERT_MSG(DYNA_REC, host_reg == ARMReg::INVALID_REG,
               "Attempted to reset a loaded register (did you mean to flush it?)");
    reg.Flush();
  }
}

// TODO: parameter non existant on new version
ARMReg ArmRegCache::GetReg(bool AutoLock)
{
  // If we have no registers left, dump the most stale register first
  if (GetUnlockedRegisterCount() == 0)
    FlushMostStaleRegister();

  for (auto& it : m_host_registers)
  {
    if (!it.IsLocked())
    {
      if (AutoLock)
        it.Lock();
      return it.GetReg();
    }
  }
  // Holy cow, how did you run out of registers?
  // We can't return anything reasonable in this case. Return INVALID_REG and watch the failure
  // happen
  ASSERT_MSG(DYNA_REC, 0, "All available registers are locked!");
  return ARMReg::INVALID_REG;
}

void ArmRegCache::UpdateLastUsed(BitSet32 regs_used)
{
  for (size_t i = 0; i < m_guest_registers.size(); ++i)
  {
    OpArg& reg = m_guest_registers[i];
    if (i < 32 && regs_used[i])
      reg.ResetLastUsed();
    else
      reg.IncrementLastUsed();
  }
}

u32 ArmRegCache::GetUnlockedRegisterCount() const
{
  u32 unlocked_registers = 0;
  for (const auto& it : m_host_registers)
  {
    if (!it.IsLocked())
      ++unlocked_registers;
  }
  return unlocked_registers;
}

void ArmRegCache::LockRegister(ARMReg host_reg)
{
  auto reg = std::find_if(m_host_registers.begin(), m_host_registers.end(),
                          [host_reg](const HostReg& r) { return r.GetReg() == host_reg; });
  ASSERT_MSG(DYNA_REC, reg != m_host_registers.end(),
             "Don't try locking a register that isn't in the cache. Reg {}",
             static_cast<int>(host_reg));
  reg->Lock();
}

void ArmRegCache::UnlockRegister(ARMReg host_reg)
{
  auto reg = std::find_if(m_host_registers.begin(), m_host_registers.end(),
                          [host_reg](const HostReg& r) { return r.GetReg() == host_reg; });
  ASSERT_MSG(DYNA_REC, reg != m_host_registers.end(),
             "Don't try unlocking a register that isn't in the cache. Reg {}",
             static_cast<int>(host_reg));
  reg->Unlock();
}

void ArmRegCache::FlushMostStaleRegister()
{
  size_t most_stale_preg = 0;
  u32 most_stale_amount = 0;

  for (size_t i = 0; i < m_guest_registers.size(); ++i)
  {
    const auto& reg = m_guest_registers[i];
    const u32 last_used = reg.GetLastUsed();

    if (last_used > most_stale_amount && reg.IsInHostRegister())
    {
      most_stale_preg = i;
      most_stale_amount = last_used;
    }
  }

  FlushRegister(most_stale_preg, FlushMode::All, ARMReg::INVALID_REG);
}

void ArmRegCache::DiscardRegister(size_t preg)
{
  OpArg& reg = m_guest_registers[preg];
  ARMReg host_reg = reg.GetReg();

  reg.Discard();
  if (host_reg != ARMReg::INVALID_REG)
    UnlockRegister(host_reg);
}

// ArmGPRCache implementation

ArmGPRCache::ArmGPRCache() : ArmRegCache(GUEST_GPR_COUNT + GUEST_CR_COUNT)
{
}

void ArmGPRCache::Start(PPCAnalyst::BlockRegStats& stats)
{
}

// Returns if a register is set as an immediate. Only valid for guest GPRs.
bool ArmGPRCache::IsImm(size_t preg) const
{
  return m_guest_registers[GUEST_GPR_OFFSET + preg].GetType() == RegType::Immediate;
}

// Gets the immediate that a register is set to. Only valid for guest GPRs.
u32 ArmGPRCache::GetImm(size_t preg) const
{
  ASSERT(m_guest_registers[GUEST_GPR_OFFSET + preg].GetType() == RegType::Immediate);
  return m_guest_registers[GUEST_GPR_OFFSET + preg].GetImm();
}

bool ArmGPRCache::IsCallerSaved(ARMReg reg) const
{
  switch (reg)
  {
  case R0:
  case R1:
  case R2:
  case R3:
  case R12:
    return true;
  default:
    return false;
  }
}

const OpArg& ArmGPRCache::GetGuestGPROpArg(size_t preg) const
{
  ASSERT(preg < GUEST_GPR_COUNT);
  return m_guest_registers[preg];
}

ArmGPRCache::GuestRegInfo ArmGPRCache::GetGuestGPR(size_t preg)
{
  ASSERT(preg < GUEST_GPR_COUNT);
  return {32, PPCSTATE_OFF_GPR(preg), m_guest_registers[GUEST_GPR_OFFSET + preg]};
}

ArmGPRCache::GuestRegInfo ArmGPRCache::GetGuestCR(size_t preg)
{
  ASSERT(preg < GUEST_CR_COUNT);
  return {32, PPCSTATE_OFF_CR(preg), m_guest_registers[GUEST_CR_OFFSET + preg]};
}

ArmGPRCache::GuestRegInfo ArmGPRCache::GetGuestByIndex(size_t index)
{
  static_assert(GUEST_GPR_OFFSET == 0);
  if (index < GUEST_GPR_OFFSET + GUEST_GPR_COUNT)
    return GetGuestGPR(index - GUEST_GPR_OFFSET);
  if (index >= GUEST_CR_OFFSET && index < GUEST_CR_OFFSET + GUEST_CR_COUNT)
    return GetGuestCR(index - GUEST_CR_OFFSET);
  ASSERT_MSG(DYNA_REC, false, "Invalid index for guest register");
  return GetGuestGPR(0);
}

void ArmGPRCache::FlushRegister(size_t index, FlushMode mode, ARMReg tmp_reg)
{
  //JIT_LOG("FlushRegister: index=%zu tmp_reg invalid=%d", index, tmp_reg == ARMReg::INVALID_REG ? 1 : 0);

  GuestRegInfo guest_reg = GetGuestByIndex(index);
  OpArg& reg = guest_reg.reg;
  size_t bitsize = guest_reg.bitsize;
  const bool is_gpr = index >= GUEST_GPR_OFFSET && index < GUEST_GPR_OFFSET + GUEST_GPR_COUNT;

  if (reg.IsInHostRegister())
  {
    ARMReg host_reg = reg.GetReg();
    if (!reg.IsInPPCState())
      m_emit->STR(host_reg, PPC_REG, u32(guest_reg.ppc_offset));

    if (mode == FlushMode::All)
    {
      UnlockRegister(host_reg);
      reg.Flush();
    }
  }
  else if (is_gpr && IsImm(index - GUEST_GPR_OFFSET))
  {
    if (!reg.IsInPPCState())
    {
      const u32 imm = GetImm(index - GUEST_GPR_OFFSET);
      bool allocated_tmp_reg = false;

      //JIT_LOG("FlushRegister: index=%zu, ppc_offset=%d (0x%x), imm=%u",
      //  index, guest_reg.ppc_offset, guest_reg.ppc_offset, imm);

      //ARMReg tmp_reg2 = GetReg();
      //allocated_tmp_reg = true;

      if (tmp_reg == ARMReg::INVALID_REG)
      {
        //JIT_LOG("FlushRegister: tmp_reg == ARMReg::INVALID_REG");

        tmp_reg = GetReg();
        allocated_tmp_reg = true;
      }

      if (imm == 0)
      {
        m_emit->MOV(tmp_reg, 0);
        m_emit->STR(tmp_reg, PPC_REG, u32(guest_reg.ppc_offset));
      }
      else
      {
        m_emit->MOVI2R(tmp_reg, imm);
        m_emit->STR(tmp_reg, PPC_REG, u32(guest_reg.ppc_offset));
      }

      if (allocated_tmp_reg)
        UnlockRegister(tmp_reg);
    }

    if (mode == FlushMode::All)
      reg.Flush();
  }
}

void ArmGPRCache::FlushRegisters(BitSet32 regs, FlushMode mode,
                                 ArmGen::ARMReg tmp_reg,
                                 IgnoreDiscardedRegisters ignore_discarded_registers)
{
  for (int i : regs)
  {
    OpArg& reg = m_guest_registers[GUEST_GPR_OFFSET + i];
    ASSERT_MSG(DYNA_REC,
               ignore_discarded_registers != IgnoreDiscardedRegisters::No || reg.IsInPPCState() ||
                   reg.IsInHostRegister() || IsImm(i),
               "Attempted to flush discarded register");

    FlushRegister(GUEST_GPR_OFFSET + i, mode, tmp_reg);
  }
}

void ArmGPRCache::FlushCRRegisters(BitSet8 regs, FlushMode mode,
                                   ArmGen::ARMReg tmp_reg,
                                   IgnoreDiscardedRegisters ignore_discarded_registers)
{
  for (int i : regs)
  {
    OpArg& reg = m_guest_registers[GUEST_CR_OFFSET + i];
    ASSERT_MSG(DYNA_REC,
               ignore_discarded_registers != IgnoreDiscardedRegisters::No || reg.IsInPPCState() ||
                   reg.IsInHostRegister(),
               "Attempted to flush discarded register");

    FlushRegister(GUEST_CR_OFFSET + i, mode, tmp_reg);
  }
}

void ArmGPRCache::DiscardCRRegisters(BitSet8 regs)
{
  for (int i : regs)
    DiscardRegister(GUEST_CR_OFFSET + i);
}

void ArmGPRCache::ResetCRRegisters(BitSet8 regs)
{
  for (int i : regs)
  {
    OpArg& reg = m_guest_registers[GUEST_CR_OFFSET + i];
    ARMReg host_reg = reg.GetReg();

    ASSERT_MSG(DYNA_REC, host_reg == ARMReg::INVALID_REG,
               "Attempted to reset a loaded register (did you mean to flush it?)");
    reg.Flush();
  }
}

void ArmGPRCache::Flush(FlushMode mode, ARMReg tmp_reg, IgnoreDiscardedRegisters ignore_discarded_registers)
{
  FlushRegisters(BitSet32(0xFFFFFFFF), mode, tmp_reg, ignore_discarded_registers);
  FlushCRRegisters(BitSet8(0xFF), mode, tmp_reg, ignore_discarded_registers);
}

ARMReg ArmGPRCache::BindForRead(size_t index)
{
  GuestRegInfo guest_reg = GetGuestByIndex(index);
  OpArg& reg = guest_reg.reg;
  size_t bitsize = guest_reg.bitsize;
  const bool is_gpr = index >= GUEST_GPR_OFFSET && index < GUEST_GPR_OFFSET + GUEST_GPR_COUNT;

  IncrementAllUsed();
  reg.ResetLastUsed();

  if (reg.IsInHostRegister())
  {
    return reg.GetReg();
  }
  else if (is_gpr && IsImm(index - GUEST_GPR_OFFSET))
  {
    ARMReg host_reg = GetReg();
    m_emit->MOVI2R(host_reg, GetImm(index - GUEST_GPR_OFFSET));
    reg.Load(host_reg);
    return host_reg;
  }
  else  // Register isn't loaded at all
  {
    ASSERT_MSG(DYNA_REC, reg.IsInPPCState(), "Attempted to read discarded register");
    ARMReg host_reg = GetReg();
    reg.Load(host_reg);
    reg.SetDirty(false);
    m_emit->LDR(host_reg, PPC_REG, u32(guest_reg.ppc_offset));
    return host_reg;
  }
}

void ArmGPRCache::SetImmediateInternal(size_t index, u32 imm, bool dirty)
{
  OpArg& reg = m_guest_registers[index];

  if (reg.IsInHostRegister())
    UnlockRegister(reg.GetReg());

  reg.Discard();
  reg.LoadToImm(imm);
  reg.SetDirty(dirty);
}

void ArmGPRCache::BindForWrite(size_t index, bool will_read, bool will_write)
{
  GuestRegInfo guest_reg = GetGuestByIndex(index);
  OpArg& reg = guest_reg.reg;
  const size_t bitsize = guest_reg.bitsize;
  const bool is_gpr = index >= GUEST_GPR_OFFSET && index < GUEST_GPR_OFFSET + GUEST_GPR_COUNT;

  reg.ResetLastUsed();

  if (!reg.IsInHostRegister())
  {
    if (is_gpr && IsImm(index - GUEST_GPR_OFFSET))
    {
      ARMReg host_reg = GetReg();
      if (will_read || !will_write)
      {
        m_emit->MOVI2R(host_reg, GetImm(index - GUEST_GPR_OFFSET));
      }
      reg.Load(host_reg);
    }
    else
    {
      ASSERT_MSG(DYNA_REC, !will_read || reg.IsInPPCState(), "Attempted to load a discarded value");
      ARMReg host_reg = GetReg();
      reg.Load(host_reg);
      reg.SetDirty(will_write);
      if (will_read)
        m_emit->LDR(host_reg, PPC_REG, u32(guest_reg.ppc_offset));
      return;
    }
  }

  if (will_write)
    reg.SetDirty(true);
}

void ArmGPRCache::GetAllocationOrder()
{
  // Callee saved registers first in hopes that we will keep everything stored there first
  // Use callee-saved registers that survive function calls per AAPCS
  static constexpr auto allocation_order = {
      // Callee saved (non-volatile) - survive function calls
      R4, R5, R6,
      // R7 is DISPATCHER_PC - reserved
      // R8 is MEM_REG - reserved
      // R9 is PPC_REG - reserved
      R10,
#ifdef FOMIT_FRAME_POINTER
      R11, // R11 could be used as a frame pointer
#endif
      // Caller saved (volatile) - for short-term temporaries only
      R0, R1, R2, R3, R12,
      // R14 is LR - don't use for allocation
  };

  for (ARMReg reg : allocation_order)
    m_host_registers.push_back(HostReg(reg));
}

BitSet32 ArmGPRCache::GetCallerSavedUsed() const
{
  BitSet32 registers(0);
  for (const auto& it : m_host_registers)
  {
    if (it.IsLocked() && IsCallerSaved(it.GetReg()))
    {
      ARMReg reg = it.GetReg();
      // ARM registers are R0-R15, use the register number directly
      int reg_num = static_cast<int>(reg);
      if (reg_num >= 0 && reg_num < 32)
        registers[reg_num] = true;
    }
  }
  return registers;
}

BitSet32 ArmGPRCache::GetDirtyGPRs() const
{
  BitSet32 registers(0);
  for (size_t i = 0; i < GUEST_GPR_COUNT; ++i)
  {
    const OpArg& arg = m_guest_registers[GUEST_GPR_OFFSET + i];
    registers[i] = !arg.IsInPPCState();
  }
  return registers;
}

void ArmGPRCache::FlushByHost(ARMReg host_reg, ARMReg tmp_reg)
{
  for (size_t i = 0; i < m_guest_registers.size(); ++i)
  {
    const OpArg& reg = m_guest_registers[i];
    if (reg.IsInHostRegister() && reg.GetReg() == host_reg)
    {
      FlushRegister(i, FlushMode::All, tmp_reg);
      return;
    }
  }
}
