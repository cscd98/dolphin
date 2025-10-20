// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include "Common/ArmEmitter.h"
#include "Core/PowerPC/Gekko.h"
#include "Core/PowerPC/PPCAnalyst.h"

// This ARM Register cache actually pre loads the most used registers before
// the block to increase speed since every memory load requires two
// instructions to load it. We are going to use R0-RMAX as registers for the
// use of PPC Registers.
// Allocation order as follows
#define ARMREGS 16
// Allocate R0 to R9 for PPC first.
// For General registers on the host side, start with R14 and go down as we go
// R13 is reserved for our stack pointer, don't ever use that. Unless you save
// it
// So we have R14, R12, R11, R10 to work with instructions

// Dedicated host registers

// memory base register
constexpr ArmGen::ARMReg MEM_REG = ArmGen::ARMReg::R8;
// ppcState pointer
constexpr ArmGen::ARMReg PPC_REG = ArmGen::ARMReg::R9;
// PC register when calling the dispatcher
constexpr ArmGen::ARMReg DISPATCHER_PC = ArmGen::ARMReg::R7;

enum RegType
{
	REG_NOTLOADED = 0,
	REG_REG, // Reg type is register
	REG_IMM, // Reg is really a IMM
	REG_AWAY, // Bound to a register, but not preloaded
};

enum FlushMode
{
	FLUSH_ALL = 0,
	FLUSH_MAINTAIN_STATE,
};

class OpArg
{
	private:
	RegType m_type; // store type
	u8 m_reg; // index to register
	u32 m_value; // IMM value
	bool m_dirty = false;

	public:
	OpArg()
	{
		m_type = REG_NOTLOADED;
		m_reg = 33;
		m_value = 0;
	}

	RegType GetType()
	{
		return m_type;
	}

	u8 GetRegIndex()
	{
		return m_reg;
	}
	u32 GetImm()
	{
		return m_value;
	}
	void LoadToAway(u8 reg)
	{
		m_type = REG_AWAY;
		m_reg = reg;
	}
	void LoadToReg(u8 reg)
	{
		m_type = REG_REG;
		m_reg = reg;
	}
	void LoadToImm(u32 imm)
	{
		m_type = REG_IMM;
		m_value = imm;
	}
	void Flush()
	{
		m_type = REG_NOTLOADED;
	}
	void SetDirty(bool dirty) { m_dirty = dirty; }
  bool IsDirty() const { return m_dirty; }
};

struct JRCPPC
{
	u32 PPCReg; // Tied to which PPC Register
	bool PS1;
	ArmGen::ARMReg Reg; // Tied to which ARM Register
	u32 LastLoad;
};
struct JRCReg
{
	ArmGen::ARMReg Reg; // Which reg this is.
	bool free;
};
class ArmRegCache
{
private:
	// WEBOS TODO: replace with std::vector<OpArg> m_guest_registers;
	OpArg regs[32];
	JRCPPC ArmCRegs[ARMREGS];
	JRCReg ArmRegs[ARMREGS]; // Four registers remaining

	int NUMPPCREG;
	int NUMARMREG;

	ArmGen::ARMReg *GetAllocationOrder(int &count);
	ArmGen::ARMReg *GetPPCAllocationOrder(int &count);

	u32 GetLeastUsedRegister(bool increment);
	bool FindFreeRegister(u32 &regindex);

	// Private function can kill immediates
	ArmGen::ARMReg BindToRegister(u32 preg, bool doLoad, bool kill_imm);

protected:
	ArmGen::ARMXEmitter *emit;

public:
	ArmRegCache();
	~ArmRegCache() {}

	void Init(ArmGen::ARMXEmitter *emitter);
	virtual void Start(PPCAnalyst::BlockRegStats& stats) {}
	void ResetRegisters(BitSet32 regs_to_reset);
	void ResetCRRegisters(BitSet8 regs);

	ArmGen::ARMReg GetReg(bool AutoLock = true); // Return a ARM register we can use.

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
      // Taking ownership of an existing scoped register, no need to release.
      m_reg = scoped_reg.m_reg;
      m_gpr = scoped_reg.m_gpr;
      scoped_reg.Invalidate();
      return *this;
    }

    inline ArmGen::ARMReg GetReg() const { return m_reg; }
    inline operator ArmGen::ARMReg() const { return GetReg(); }
    inline void Unlock()
    {
      // Only unlock the register if GPR is set.
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

	void Unlock(ArmGen::ARMReg R0, ArmGen::ARMReg R1 = ArmGen::INVALID_REG, ArmGen::ARMReg R2 = ArmGen::INVALID_REG, ArmGen::ARMReg R3 = ArmGen::INVALID_REG);
	void Flush(FlushMode mode = FLUSH_ALL);
	ArmGen::ARMReg R(u32 preg); // Returns a cached register
	bool IsImm(u32 preg) { return regs[preg].GetType() == REG_IMM; }
	u32 GetImm(u32 preg) { return regs[preg].GetImm(); }
	void SetImmediate(u32 preg, u32 imm, bool dirty = true);

	// Public function doesn't kill immediates
	// In reality when you call R(u32) it'll bind an immediate there
	void BindToRegister(u32 preg, bool doLoad = true);

	void StoreFromRegister(u32 preg);

	void Lock(ArmGen::ARMReg R0,
          ArmGen::ARMReg R1 = ArmGen::ARMReg::INVALID_REG,
          ArmGen::ARMReg R2 = ArmGen::ARMReg::INVALID_REG,
          ArmGen::ARMReg R3 = ArmGen::ARMReg::INVALID_REG);
};
