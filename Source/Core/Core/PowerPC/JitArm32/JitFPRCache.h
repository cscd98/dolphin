// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <cstddef>
#include <vector>

#include "Common/ArmEmitter.h"
#include "Common/CommonTypes.h"
#include "Core/PowerPC/Gekko.h"
#include "Core/PowerPC/PPCAnalyst.h"
#include "Core/PowerPC/JitArm32/JitRegCache.h"

// ARM VFP has 32 double-precision registers (D0-D31)
constexpr size_t GUEST_FPR_COUNT = 32;

enum class FPRRegType
{
  Register,          // Full paired-single (PS0 and PS1)
  LowerPair,         // PS0 only
  Duplicated,        // PS0 == PS1, only one copy stored
  Single,            // Single-precision (32-bit)
  LowerPairSingle,   // PS0 only, single-precision
  DuplicatedSingle,  // PS0 == PS1, single-precision
};

class FPROpArg
{
public:
  FPROpArg() = default;

  FPRRegType GetFPRType() const { return m_fpr_type; }
  ArmGen::ARMReg GetReg() const { return m_reg; }

  void Load(ArmGen::ARMReg reg, FPRRegType format = FPRRegType::Register)
  {
    m_reg = reg;
    m_fpr_type = format;
    m_in_host_register = true;
  }

  void Discard()
  {
    m_reg = ArmGen::ARMReg::INVALID_REG;
    m_fpr_type = FPRRegType::Register;
    m_in_ppc_state = false;
    m_in_host_register = false;
    m_last_used = 0xFFFF;
  }

  void Flush()
  {
    m_reg = ArmGen::ARMReg::INVALID_REG;
    m_fpr_type = FPRRegType::Register;
    m_in_ppc_state = true;
    m_in_host_register = false;
    m_last_used = 0xFFFF;
  }

  u32 GetLastUsed() const { return m_last_used; }
  void ResetLastUsed() { m_last_used = 0; }
  void IncrementLastUsed() { ++m_last_used; }
  void SetDirty(bool dirty) { m_in_ppc_state = !dirty; }
  bool IsInPPCState() const { return m_in_ppc_state; }
  bool IsInHostRegister() const { return m_in_host_register; }

private:
  ArmGen::ARMReg m_reg = ArmGen::ARMReg::INVALID_REG;
  FPRRegType m_fpr_type = FPRRegType::Register;
  u32 m_last_used = 0;
  bool m_in_ppc_state = true;
  bool m_in_host_register = false;
};

class ArmFPRCache
{
public:
  ArmFPRCache();
  ~ArmFPRCache() = default;

  void Init(ArmGen::ARMXEmitter* emitter);

  virtual void Start(PPCAnalyst::BlockRegStats& stats) {}

  // Flushes the register cache in different ways depending on the mode.
  void Flush(FlushMode mode = FlushMode::All, // previous version default was FLUSH_ALL
             ArmGen::ARMReg tmp_reg = ArmGen::INVALID_REG,
             IgnoreDiscardedRegisters ignore_discarded_registers =
             IgnoreDiscardedRegisters::No);

  // Returns a guest register inside of a host register
  // PS0 (lower half of paired single)
  ArmGen::ARMReg R0(size_t preg, FPRRegType format);

  // PS1 (upper half of paired single)
  ArmGen::ARMReg R1(size_t preg, FPRRegType format);

  // Read-write version
  ArmGen::ARMReg RW0(size_t preg, FPRRegType format, bool set_dirty = true);
  ArmGen::ARMReg RW1(size_t preg, FPRRegType format, bool set_dirty = true);

  BitSet32 GetCallerSavedUsed() const;

  bool IsSingle(size_t preg, bool lower_only = false) const;

  void StoreRegisters(BitSet32 regs, ArmGen::ARMReg tmp_reg = ArmGen::ARMReg::INVALID_REG,
                      FlushMode flush_mode = FlushMode::All)
  {
    FlushRegisters(regs, flush_mode, tmp_reg, IgnoreDiscardedRegisters::No);
  }

  void DiscardRegisters(BitSet32 regs);
  void ResetRegisters(BitSet32 regs);

  // Returns a temporary register for use
  // Requires unlocking after done
  ArmGen::ARMReg GetReg();

  class ScopedARMReg
  {
  public:
    inline ScopedARMReg() = default;
    ScopedARMReg(const ScopedARMReg&) = delete;
    explicit inline ScopedARMReg(ArmFPRCache& cache) : m_reg(cache.GetReg()), m_fpr(&cache) {}
    inline ScopedARMReg(ArmGen::ARMReg reg) : m_reg(reg) {}
    inline ScopedARMReg(ScopedARMReg&& scoped_reg) { *this = std::move(scoped_reg); }
    inline ~ScopedARMReg() { Unlock(); }

    inline ScopedARMReg& operator=(const ScopedARMReg&) = delete;
    inline ScopedARMReg& operator=(ArmGen::ARMReg reg)
    {
      Unlock();
      m_reg = reg;
      return *this;
    }
    inline ScopedARMReg& operator=(ScopedARMReg&& scoped_reg)
    {
      m_reg = scoped_reg.m_reg;
      m_fpr = scoped_reg.m_fpr;
      scoped_reg.Invalidate();
      return *this;
    }

    inline ArmGen::ARMReg GetReg() const { return m_reg; }
    inline operator ArmGen::ARMReg() const { return GetReg(); }
    inline void Unlock()
    {
      if (m_fpr != nullptr)
      {
        m_fpr->Unlock(m_reg);
      }
      Invalidate();
    }

  private:
    inline void Invalidate()
    {
      m_reg = ArmGen::ARMReg::INVALID_REG;
      m_fpr = nullptr;
    }

    ArmGen::ARMReg m_reg = ArmGen::ARMReg::INVALID_REG;
    ArmFPRCache* m_fpr = nullptr;
  };

  inline ScopedARMReg GetScopedReg() { return ScopedARMReg(*this); }

  void UpdateLastUsed(BitSet32 regs_used);

  u32 GetUnlockedRegisterCount() const;

  template <typename T = ArmGen::ARMReg, typename... Args>
  void Lock(Args... args)
  {
    for (T reg : {args...})
    {
      FlushByHost(reg);
      LockRegister(reg);
    }
  }

  template <typename T = ArmGen::ARMReg, typename... Args>
  void Unlock(Args... args)
  {
    for (T reg : {args...})
    {
      FlushByHost(reg);
      UnlockRegister(reg);
    }
  }

protected:
  void GetAllocationOrder();

  void FlushMostStaleRegister();

  void LockRegister(ArmGen::ARMReg host_reg);
  void UnlockRegister(ArmGen::ARMReg host_reg);

  void FlushByHost(ArmGen::ARMReg host_reg, ArmGen::ARMReg tmp_reg = ArmGen::INVALID_REG);

  void DiscardRegister(size_t preg, bool ps1);
  void FlushRegister(size_t preg, bool ps1, FlushMode mode);

  void IncrementAllUsed()
  {
    for (auto& reg : m_guest_registers_ps0)
      reg.IncrementLastUsed();
    for (auto& reg : m_guest_registers_ps1)
      reg.IncrementLastUsed();
  }

private:
  bool IsCallerSaved(ArmGen::ARMReg reg) const;

  ArmGen::ARMReg GetPPCReg(size_t preg, bool ps1, FPRRegType format, bool for_write, bool set_dirty);

  void FlushRegisters(BitSet32 regs, FlushMode mode,
                      ArmGen::ARMReg tmp_reg,
                      IgnoreDiscardedRegisters ignore_discarded_registers);

  // Code emitter
  ArmGen::ARMXEmitter* m_emit = nullptr;

  // Host side registers that hold the host registers in order of use
  std::vector<HostReg> m_host_registers;

  // Guest FPRs - split into PS0 and PS1
  // ARM VFP double registers can hold one half of a PowerPC paired single
  std::vector<FPROpArg> m_guest_registers_ps0;
  std::vector<FPROpArg> m_guest_registers_ps1;

  // Register stats for the current block
  PPCAnalyst::BlockRegStats* m_reg_stats = nullptr;
};
