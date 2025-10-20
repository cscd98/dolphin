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
#include "Core/PowerPC/PowerPC.h"

// Dedicated host registers

// memory base register
constexpr ArmGen::ARMReg MEM_REG = ArmGen::ARMReg::R8;
// ppcState pointer
constexpr ArmGen::ARMReg PPC_REG = ArmGen::ARMReg::R9;
// PC register when calling the dispatcher
constexpr ArmGen::ARMReg DISPATCHER_PC = ArmGen::ARMReg::R7;

// -----------------------------------------------------------------------------
// ARM32 PPCState offset macros
// -----------------------------------------------------------------------------

// Struct member offset
#define PPCSTATE_OFF(elem)                                                      \
  ([]() consteval {                                                             \
    _Pragma("GCC diagnostic push")                                              \
    _Pragma("GCC diagnostic ignored \"-Winvalid-offsetof\"")                    \
    return static_cast<s32>(offsetof(PowerPC::PowerPCState, elem));             \
    _Pragma("GCC diagnostic pop")                                               \
  }())

// Array base and element size
#define PPCSTATE_ARRAY_BASE(elem)                                               \
  ([]() consteval {                                                             \
    _Pragma("GCC diagnostic push")                                              \
    _Pragma("GCC diagnostic ignored \"-Winvalid-offsetof\"")                    \
    return static_cast<s32>(offsetof(PowerPC::PowerPCState, elem[0]));          \
    _Pragma("GCC diagnostic pop")                                               \
  }())

#define PPCSTATE_ARRAY_ELEM_SIZE(elem)                                          \
  ([]() consteval {                                                             \
    return static_cast<s32>(sizeof(PowerPC::PowerPCState::elem[0]));            \
  }())

// Runtime-safe array offset (accepts dynamic indices)
#define PPCSTATE_OFF_ARRAY(elem, i)                                             \
  (PPCSTATE_ARRAY_BASE(elem) +                                                  \
   PPCSTATE_ARRAY_ELEM_SIZE(elem) * static_cast<s32>(i))

// Convenience wrappers
#define PPCSTATE_OFF_GPR(i) PPCSTATE_OFF_ARRAY(gpr, i)
#define PPCSTATE_OFF_CR(i)  PPCSTATE_OFF_ARRAY(cr.fields, i)
#define PPCSTATE_OFF_SR(i)                                                      \
  (PPCSTATE_OFF(sr) +                                                           \
   PPCSTATE_ARRAY_ELEM_SIZE(sr) * static_cast<s32>(i))
#define PPCSTATE_OFF_SPR(i) PPCSTATE_OFF_ARRAY(spr, i)
#define PPCSTATE_OFF_PS(i)  PPCSTATE_OFF_ARRAY(ps, i)

// PairedSingle lanes
#define PPCSTATE_OFF_PS0(i)                                                     \
  (PPCSTATE_OFF_PS(i) +                                                         \
   ([]() consteval {                                                            \
     _Pragma("GCC diagnostic push")                                             \
     _Pragma("GCC diagnostic ignored \"-Winvalid-offsetof\"")                   \
     return static_cast<s32>(offsetof(PowerPC::PairedSingle, ps0));             \
     _Pragma("GCC diagnostic pop")                                              \
   }()))
#define PPCSTATE_OFF_PS1(i)                                                     \
  (PPCSTATE_OFF_PS(i) +                                                         \
   ([]() consteval {                                                            \
     _Pragma("GCC diagnostic push")                                             \
     _Pragma("GCC diagnostic ignored \"-Winvalid-offsetof\"")                   \
     return static_cast<s32>(offsetof(PowerPC::PairedSingle, ps1));             \
     _Pragma("GCC diagnostic pop")                                              \
   }()))

// SPR sub-base and imm12-safe displacement
#define PPCSTATE_SPR_BASE PPCSTATE_OFF_SPR(0)
#define SPR_OFFSET(i)     (PPCSTATE_OFF_SPR(i) - PPCSTATE_SPR_BASE)

//#define PPCSTATE_OFF_DEBUG_SCRATCH(i) \
//  (static_cast<s32>(offsetof(PowerPC::PowerPCState, jit_debug_scratch)) + sizeof(u32) * (i))

// Range/alignment checks (constexpr-safe now)
static_assert(SPR_OFFSET(1023) < 4096, "LDR/STR can't reach the last SPR with imm12");
static_assert(PPCSTATE_OFF(xer_ca)    < 4096, "STRB can't store xer_ca!");
static_assert(PPCSTATE_OFF(xer_so_ov) < 4096, "STRB can't store xer_so_ov!");
static_assert((PPCSTATE_OFF_PS0(0) % 8) == 0, "VLDR/VSTR (64-bit) require 8-byte alignment");
static_assert((PPCSTATE_OFF_PS1(0) % 8) == 0, "VLDR/VSTR (64-bit) require 8-byte alignment");

enum class RegType
{
  NotLoaded = 0,
  Register,  // Loaded in host register
  Immediate, // Immediate value
};

enum class FlushMode : bool
{
  // Flushes all registers, no exceptions
  All,
  // Flushes registers in a conditional branch
  // Doesn't wipe the state of the registers from the cache
  MaintainState,
};

enum class IgnoreDiscardedRegisters
{
  No,
  Yes,
};

class OpArg
{
public:
  OpArg() = default;

  RegType GetType() const { return m_type; }
  ArmGen::ARMReg GetReg() const { return m_reg; }
  u32 GetImm() const { return m_value; }

  void Load(ArmGen::ARMReg reg)
  {
    m_type = RegType::Register;
    m_reg = reg;
    m_in_host_register = true;
  }

  void LoadToImm(u32 imm)
  {
    m_type = RegType::Immediate;
    m_value = imm;
    m_in_host_register = false;
  }

  void Discard()
  {
    // Invalidate any previous information
    m_reg = ArmGen::ARMReg::INVALID_REG;
    m_type = RegType::NotLoaded;
    m_in_ppc_state = false;
    m_in_host_register = false;

    // Arbitrarily large value that won't roll over on a lot of increments
    m_last_used = 0xFFFF;
  }

  void Flush()
  {
    // Invalidate any previous information
    m_reg = ArmGen::ARMReg::INVALID_REG;
    m_type = RegType::NotLoaded;
    m_in_ppc_state = true;
    m_in_host_register = false;

    // Arbitrarily large value that won't roll over on a lot of increments
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
  RegType m_type = RegType::NotLoaded;
  u32 m_value = 0;
  u32 m_last_used = 0;
  bool m_in_ppc_state = true;
  bool m_in_host_register = false;
};

class HostReg
{
public:
  HostReg() = default;
  HostReg(ArmGen::ARMReg reg) : m_reg(reg) {}

  bool IsLocked() const { return m_locked; }
  void Lock() { m_locked = true; }
  void Unlock() { m_locked = false; }
  ArmGen::ARMReg GetReg() const { return m_reg; }

private:
  ArmGen::ARMReg m_reg = ArmGen::ARMReg::INVALID_REG;
  bool m_locked = false;
};

class ArmRegCache
{
public:
  explicit ArmRegCache(size_t guest_reg_count);
  virtual ~ArmRegCache() = default;

  void Init(ArmGen::ARMXEmitter* emitter);

  virtual void Start(PPCAnalyst::BlockRegStats& stats) {}

  void DiscardRegisters(BitSet32 regs);
  void ResetRegisters(BitSet32 regs);

  // Flushes the register cache in different ways depending on the mode.
  virtual void Flush(FlushMode mode,
                     ArmGen::ARMReg tmp_reg = ArmGen::INVALID_REG,
                     IgnoreDiscardedRegisters ignore_discarded_registers =
                     IgnoreDiscardedRegisters::No) = 0;

  virtual BitSet32 GetCallerSavedUsed() const = 0;

  // Returns a temporary register for use
  // Requires unlocking after done
  ArmGen::ARMReg GetReg(bool AutoLock = true); // TODO: previous AutoLock = true, none existant on new version

  class ScopedARMReg
  {
  public:
    inline ScopedARMReg() = default;
    ScopedARMReg(const ScopedARMReg&) = delete;
    explicit inline ScopedARMReg(ArmRegCache& cache) : m_reg(cache.GetReg()), m_gpr(&cache) {}
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
      m_gpr = scoped_reg.m_gpr;
      scoped_reg.Invalidate();
      return *this;
    }

    inline ArmGen::ARMReg GetReg() const { return m_reg; }
    inline operator ArmGen::ARMReg() const { return GetReg(); }
    inline void Unlock()
    {
      if (m_gpr != nullptr)
      {
        m_gpr->Unlock(m_reg);
      }
      Invalidate();
    }

  private:
    inline void Invalidate()
    {
      m_reg = ArmGen::ARMReg::INVALID_REG;
      m_gpr = nullptr;
    }

    ArmGen::ARMReg m_reg = ArmGen::ARMReg::INVALID_REG;
    ArmRegCache* m_gpr = nullptr;
  };

  // Returns a temporary register
  // Unlocking is implicitly handled through RAII
  inline ScopedARMReg GetScopedReg() { return ScopedARMReg(*this); }

  void UpdateLastUsed(BitSet32 regs_used);

  // Get available host registers
  u32 GetUnlockedRegisterCount() const;

  // Locks a register so a cache cannot use it
  // Useful for function calls
  template <typename T = ArmGen::ARMReg, typename... Args>
  void Lock(Args... args)
  {
    for (T reg : {args...})
    {
      FlushByHost(reg);
      LockRegister(reg);
    }
  }

  // Unlocks a locked register
  // Unlocks registers locked with both GetReg and LockRegister
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
  // Get the order of the host registers
  virtual void GetAllocationOrder() = 0;

  // Flushes the most stale register
  void FlushMostStaleRegister();

  // Lock a register
  void LockRegister(ArmGen::ARMReg host_reg);

  // Unlock a register
  void UnlockRegister(ArmGen::ARMReg host_reg);

  // Flushes a guest register by host provided
  virtual void FlushByHost(ArmGen::ARMReg host_reg, ArmGen::ARMReg tmp_reg = ArmGen::INVALID_REG) = 0;

  void DiscardRegister(size_t preg);
  virtual void FlushRegister(size_t preg, FlushMode mode, ArmGen::ARMReg tmp_reg) = 0;

  void IncrementAllUsed()
  {
    for (auto& reg : m_guest_registers)
      reg.IncrementLastUsed();
  }

  // Code emitter
  ArmGen::ARMXEmitter* m_emit = nullptr;

  // Host side registers that hold the host registers in order of use
  std::vector<HostReg> m_host_registers;

  // Our guest registers (GPRs and CRs)
  std::vector<OpArg> m_guest_registers;

  // Register stats for the current block
  PPCAnalyst::BlockRegStats* m_reg_stats = nullptr;
};

class ArmGPRCache : public ArmRegCache
{
public:
  ArmGPRCache();

  void Start(PPCAnalyst::BlockRegStats& stats) override;

  void Flush(FlushMode mode = FlushMode::All, // previous version default was FLUSH_ALL
             ArmGen::ARMReg tmp_reg = ArmGen::INVALID_REG,
             IgnoreDiscardedRegisters ignore_discarded_registers =
             IgnoreDiscardedRegisters::No) override;

  // Returns a guest GPR inside of a host register.
  // Will dump an immediate to the host register as well.
  ArmGen::ARMReg R(size_t preg) { return BindForRead(GUEST_GPR_OFFSET + preg); }

  // Returns a guest CR inside of a host register.
  ArmGen::ARMReg CR(size_t preg) { return BindForRead(GUEST_CR_OFFSET + preg); }

  // Set a register to an immediate. Only valid for guest GPRs.
  void SetImmediate(size_t preg, u32 imm, bool dirty = true)
  {
    SetImmediateInternal(GUEST_GPR_OFFSET + preg, imm, dirty);
  }

  // Returns whether a register is set as an immediate. Only valid for guest GPRs.
  bool IsImm(size_t preg) const;

  // Gets the immediate that a register is set to. Only valid for guest GPRs.
  u32 GetImm(size_t preg) const;

  bool IsImm(size_t preg, u32 imm) const { return IsImm(preg) && GetImm(preg) == imm; }

  // Binds a guest GPR to a host register, optionally loading its value.
  void BindToRegister(size_t preg, bool will_read, bool will_write = true)
  {
    BindForWrite(GUEST_GPR_OFFSET + preg, will_read, will_write);
  }

  // Binds a guest CR to a host register, optionally loading its value.
  void BindCRToRegister(size_t preg, bool will_read, bool will_write = true)
  {
    BindForWrite(GUEST_CR_OFFSET + preg, will_read, will_write);
  }

  BitSet32 GetCallerSavedUsed() const override;

  BitSet32 GetDirtyGPRs() const;

  void StoreRegisters(BitSet32 regs, ArmGen::ARMReg tmp_reg = ArmGen::ARMReg::INVALID_REG,
                     FlushMode flush_mode = FlushMode::All)
  {
    FlushRegisters(regs, flush_mode, tmp_reg, IgnoreDiscardedRegisters::No);
  }

  void StoreCRRegisters(BitSet8 regs, ArmGen::ARMReg tmp_reg = ArmGen::ARMReg::INVALID_REG,
                        FlushMode flush_mode = FlushMode::All)
  {
    FlushCRRegisters(regs, flush_mode, tmp_reg, IgnoreDiscardedRegisters::No);
  }

  void DiscardCRRegisters(BitSet8 regs);
  void ResetCRRegisters(BitSet8 regs);

protected:
  // Get the order of the host registers
  void GetAllocationOrder() override;

  // Flushes a guest register by host provided
  void FlushByHost(ArmGen::ARMReg host_reg, ArmGen::ARMReg tmp_reg = ArmGen::INVALID_REG) override;

  void FlushRegister(size_t index, FlushMode mode, ArmGen::ARMReg tmp_reg) override;

private:
  bool IsCallerSaved(ArmGen::ARMReg reg) const;

  struct GuestRegInfo
  {
    size_t bitsize;
    size_t ppc_offset;
    OpArg& reg;
  };

  const OpArg& GetGuestGPROpArg(size_t preg) const;
  GuestRegInfo GetGuestGPR(size_t preg);
  GuestRegInfo GetGuestCR(size_t preg);
  GuestRegInfo GetGuestByIndex(size_t index);

  ArmGen::ARMReg BindForRead(size_t index);
  void BindForWrite(size_t index, bool will_read, bool will_write = true);

  void FlushRegisters(BitSet32 regs, FlushMode mode, ArmGen::ARMReg tmp_reg,
                      IgnoreDiscardedRegisters ignore_discarded_registers);
  void FlushCRRegisters(BitSet8 regs, FlushMode mode, ArmGen::ARMReg tmp_reg,
                        IgnoreDiscardedRegisters ignore_discarded_registers);

  void SetImmediateInternal(size_t index, u32 imm, bool dirty);

  static constexpr size_t GUEST_GPR_COUNT = 32;
  static constexpr size_t GUEST_CR_COUNT = 8;
  static constexpr size_t GUEST_GPR_OFFSET = 0;
  static constexpr size_t GUEST_CR_OFFSET = GUEST_GPR_COUNT;
};
