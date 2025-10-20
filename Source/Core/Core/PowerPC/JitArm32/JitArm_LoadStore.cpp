// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Common/ArmEmitter.h"
#include "Common/CommonTypes.h"

#include "Core/ConfigManager.h"
#include "Core/Core.h"
#include "Core/CoreTiming.h"
#include "Core/System.h"
#include "Core/HW/Memmap.h"
#include "Core/PowerPC/PowerPC.h"
#include "Core/PowerPC/PPCTables.h"
#include "Core/PowerPC/MMU.h"

#include "Core/PowerPC/JitArm32/Jit.h"
#include "Core/PowerPC/JitArm32/JitRegCache.h"

using namespace ArmGen;

void JitArm::SafeStoreFromReg(s32 dest, u32 value, s32 regOffset, int accessSize, s32 offset)
{
  // Reserve scratch: R12 as address, R10/R11 as temporaries for gather-pipe.
  // Ensure your reg cache never hands out R12/R10/R11 concurrently in this emission path.
  ARMReg addr_reg = R12;

  // Value register (use zero optimization).
  const bool value_is_zero = gpr.IsImm(value) && (gpr.GetImm(value) == 0);
  ARMReg RS = value_is_zero ? INVALID_REG : gpr.R(value);

  // Compute effective address into addr_reg or imm_addr.
  u32 imm_addr = 0;
  bool is_immediate = false;

  if (regOffset == -1)
  {
    if (dest != -1)
    {
      if (gpr.IsImm(dest))
      {
        is_immediate = true;
        imm_addr = gpr.GetImm(dest) + offset;
      }
      else
      {
        Operand2 off;
        if (TryMakeOperand2(offset, off))
          ADD(addr_reg, gpr.R(dest), off);
        else
        {
          MOVI2R(addr_reg, offset);
          ADD(addr_reg, addr_reg, gpr.R(dest));
        }
      }
    }
    else
    {
      is_immediate = true;
      imm_addr = offset;
    }
  }
  else
  {
    if (dest != -1)
    {
      if (gpr.IsImm(dest) && gpr.IsImm(regOffset))
      {
        is_immediate = true;
        imm_addr = gpr.GetImm(dest) + gpr.GetImm(regOffset);
      }
      else if (gpr.IsImm(dest) && !gpr.IsImm(regOffset))
      {
        Operand2 off;
        const u32 imm = gpr.GetImm(dest);
        if (TryMakeOperand2(imm, off))
          ADD(addr_reg, gpr.R(regOffset), off);
        else
        {
          MOVI2R(addr_reg, imm);
          ADD(addr_reg, addr_reg, gpr.R(regOffset));
        }
      }
      else if (!gpr.IsImm(dest) && gpr.IsImm(regOffset))
      {
        Operand2 off;
        const u32 imm = gpr.GetImm(regOffset);
        if (TryMakeOperand2(imm, off))
          ADD(addr_reg, gpr.R(dest), off);
        else
        {
          MOVI2R(addr_reg, imm);
          ADD(addr_reg, addr_reg, gpr.R(dest));
        }
      }
      else
      {
        ADD(addr_reg, gpr.R(dest), gpr.R(regOffset));
      }
    }
    else
    {
      if (gpr.IsImm(regOffset))
      {
        is_immediate = true;
        imm_addr = gpr.GetImm(regOffset);
      }
      else
      {
        MOV(addr_reg, gpr.R(regOffset));
      }
    }
  }

  // Flags
  u32 flags = BackPatchInfo::FLAG_STORE;
  if (accessSize == 32)      flags |= BackPatchInfo::FLAG_SIZE_32;
  else if (accessSize == 16) flags |= BackPatchInfo::FLAG_SIZE_16;
  else                       flags |= BackPatchInfo::FLAG_SIZE_8;

  const u32 size_bytes = BackPatchInfo::GetFlagSize(flags);

  // Optional early write-back update (ARM64 does this when memcheck is off and dest != value).
  const bool update = false; // If your caller needs RA update semantics, pass it in and use it.
  const bool early_update = !jo.memcheck && (value != static_cast<u32>(dest));
  if (update && early_update && dest != -1)
  {
    // RA := EA (write-back)
    if (is_immediate)
      MOVI2R(addr_reg, imm_addr);
    STR(addr_reg, PPC_REG, PPCSTATE_OFF_GPR(dest)); // store-back to RA
  }

  // MMIO detection for immediate addresses.
  u32 mmio_address = 0;
  if (is_immediate)
    mmio_address = m_mmu.IsOptimizableMMIOAccess(imm_addr, size_bytes);

  // Alignment guard for fastmem stores.
  const bool aligned = (size_bytes == 1) || ((imm_addr & (size_bytes - 1)) == 0);

  // Gather pipe fast path (immediate-only).
  if (is_immediate && jo.optimizeGatherPipe && m_mmu.IsOptimizableGatherPipeWrite(imm_addr))
  {
    MOVI2R(R12, (u32)&m_ppc_state.gather_pipe_ptr);
    MOVI2R(R10, (u32)m_ppc_state.gather_pipe_base_ptr);
    LDR(R11, R12);

    if (accessSize == 32)
    {
      if (!value_is_zero)
      {
        REV(RS, RS);
        STR(RS, R10, R11);
        REV(RS, RS);
      }
      else
      {
        auto tmp = gpr.GetScopedReg();
        MOVI2R(tmp, 0);
        STR(tmp, R10, R11);
      }
    }
    else if (accessSize == 16)
    {
      if (!value_is_zero)
      {
        REV16(RS, RS);
        STRH(RS, R10, R11);
        REV16(RS, RS);
      }
      else
      {
        auto tmp = gpr.GetScopedReg();
        MOVI2R(tmp, 0);
        STRH(tmp, R10, R11);
      }
    }
    else
    {
      if (!value_is_zero)
        STRB(RS, R10, R11);
      else
      {
        auto tmp = gpr.GetScopedReg();
        MOVI2R(tmp, 0);
        STRB(tmp, R10, R11);
      }
    }

    ADD(R11, R11, size_bytes);
    STR(R11, R12);
    js.fifoBytesSinceCheck += size_bytes;
  }
  // Fast RAM path (immediate, aligned, non-MMIO)
  else if (is_immediate && aligned && m_mmu.IsOptimizableRAMAddress(imm_addr, size_bytes) && !mmio_address)
  {
    MOVI2R(addr_reg, imm_addr);
    EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysFastAccess, true,
                         value_is_zero ? INVALID_REG : RS);
  }
  // MMIO path (route to helper; do not crash host)
  else if (mmio_address)
  {
    // Route via MMIO helper; avoid host fault.
    // Ensure RS is materialized (or pass zero).
    MMIOWriteRegToAddr(m_system,
			m_system.GetMemory().GetMMIOMapping(),
			this,
			value_is_zero ? ArmGen::INVALID_REG : RS,
			mmio_address,
			flags);
  }
  // Generic path (auto backpatch: DSI if illegal, slow fallback)
  else
  {
    // If address was computed, addr_reg already holds it; otherwise materialize imm.
    if (is_immediate)
      MOVI2R(addr_reg, imm_addr);
    EmitBackpatchRoutine(this, flags, MemAccessMode::Auto, true,
                         value_is_zero ? INVALID_REG : RS);
  }

  // Late write-back update if not done early.
  if (update && !early_update && dest != -1)
  {
    if (is_immediate)
      MOVI2R(addr_reg, imm_addr);
    STR(addr_reg, PPC_REG, PPCSTATE_OFF(gpr) + dest * 4);
  }
}

void JitArm::stX(UGeckoInstruction inst)
{
  printf("stx!\n");
  fflush(stdout);

  INSTRUCTION_START
  JITDISABLE(bJITLoadStoreOff);

  u32 a = inst.RA, b = inst.RB, s = inst.RS;
  s32 offset = inst.SIMM_16;
  u32 accessSize = 0;
  s32 regOffset = -1;
  bool update = false;

  switch (inst.OPCD)
  {
  case 45: // sthu
    update = true;
    [[fallthrough]];
  case 44: // sth
    accessSize = 16;
    break;
  case 31:
    switch (inst.SUBOP10)
    {
    case 183: // stwux
      update = true;
      [[fallthrough]];
    case 151: // stwx
      accessSize = 32;
      regOffset = b;
      break;
    case 247: // stbux
      update = true;
      [[fallthrough]];
    case 215: // stbx
      accessSize = 8;
      regOffset = b;
      break;
    case 439: // sthux
      update = true;
      [[fallthrough]];
    case 407: // sthx
      accessSize = 16;
      regOffset = b;
      break;
    }
    break;
  case 37: // stwu
    update = true;
    [[fallthrough]];
  case 36: // stw
    accessSize = 32;
    break;
  case 39: // stbu
    update = true;
    [[fallthrough]];
  case 38: // stb
    accessSize = 8;
    break;
  }

  int effectiveRA = (update || a != 0) ? a : -1;
  SafeStoreFromReg(effectiveRA, s, regOffset, accessSize, offset);

  if (update)
  {
    ARMReg rA = gpr.GetReg();
    ARMReg RA = gpr.R(a);
    ARMReg RB = INVALID_REG;
    if (regOffset != -1)
      RB = gpr.R(regOffset);

    // Check for DSI exception prior to writing back address
    LDR(rA, PPC_REG, PPCSTATE_OFF(Exceptions));
    TST(rA, EXCEPTION_DSI);
    FixupBranch has_exception = B_CC(CC_NEQ);

    if (regOffset == -1)
    {
      MOVI2R(rA, offset);
      ADD(RA, RA, rA);
    }
    else
    {
      ADD(RA, RA, RB);
    }

    SetJumpTarget(has_exception);
    gpr.Unlock(rA);
  }
}

void JitArm::SafeLoadToReg(ARMReg dest, s32 addr, s32 offsetReg, int accessSize, s32 offset, bool signExtend, bool reverse, bool update)
{
	// We want to make sure to not get LR as a temp register
	ARMReg rA = R12;

	u32 imm_addr = 0;
	bool is_immediate = false;

	if (offsetReg == -1)
	{
		if (addr != -1)
		{
			if (gpr.IsImm(addr))
			{
				is_immediate = true;
				imm_addr = gpr.GetImm(addr) + offset;
			}
			else
			{
				Operand2 off;
				if (TryMakeOperand2(offset, off))
				{
					ADD(rA, gpr.R(addr), off);
				}
				else
				{
					MOVI2R(rA, offset);
					ADD(rA, rA, gpr.R(addr));
				}
			}
		}
		else
		{
			is_immediate = true;
			imm_addr = offset;
		}
	}
	else
	{
		if (addr != -1)
		{
			if (gpr.IsImm(addr) && gpr.IsImm(offsetReg))
			{
				is_immediate = true;
				imm_addr = gpr.GetImm(addr) + gpr.GetImm(offsetReg);
			}
			else if (gpr.IsImm(addr) && !gpr.IsImm(offsetReg))
			{
				Operand2 off;
				if (TryMakeOperand2(gpr.GetImm(addr), off))
				{
					ADD(rA, gpr.R(offsetReg), off);
				}
				else
				{
					MOVI2R(rA, gpr.GetImm(addr));
					ADD(rA, rA, gpr.R(offsetReg));
				}
			}
			else if (!gpr.IsImm(addr) && gpr.IsImm(offsetReg))
			{
				Operand2 off;
				if (TryMakeOperand2(gpr.GetImm(offsetReg), off))
				{
					ADD(rA, gpr.R(addr), off);
				}
				else
				{
					MOVI2R(rA, gpr.GetImm(offsetReg));
					ADD(rA, rA, gpr.R(addr));
				}
			}
			else
			{
				ADD(rA, gpr.R(addr), gpr.R(offsetReg));
			}
		}
		else
		{
			if (gpr.IsImm(offsetReg))
			{
				is_immediate = true;
				imm_addr = gpr.GetImm(offsetReg);
			}
			else
			{
				MOV(rA, gpr.R(offsetReg));
			}
		}
	}

	if (is_immediate)
		MOVI2R(rA, imm_addr);

	u32 flags = BackPatchInfo::FLAG_LOAD;
	if (accessSize == 32)
		flags |= BackPatchInfo::FLAG_SIZE_32;
	else if (accessSize == 16)
		flags |= BackPatchInfo::FLAG_SIZE_16;
	else
		flags |= BackPatchInfo::FLAG_SIZE_8;

	if (reverse)
		flags |= BackPatchInfo::FLAG_REVERSE;

	if (signExtend)
		flags |= BackPatchInfo::FLAG_EXTEND;

  // Use fastmem with backpatching for non-immediate addresses
  // For immediate addresses, check if they're in optimizable RAM
  bool use_fastmem = jo.fastmem;

  if (is_immediate)
  {
    // For known addresses, check if they're safe for fastmem
    use_fastmem = use_fastmem &&
                  m_mmu.IsOptimizableRAMAddress(imm_addr, BackPatchInfo::GetFlagSize(flags));
  }

  if (use_fastmem)
  {
    // Emit fastmem code - will be backpatched to slowmem if it faults
    EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysFastAccess, true, dest);
  }
  else
  {
    // Direct slowmem access
    EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysSlowAccess, false, dest);
  }

	if (update)
		MOV(gpr.R(addr), rA);
}

void JitArm::lXX(UGeckoInstruction inst)
{
  INSTRUCTION_START
  JITDISABLE(bJITLoadStoreOff);

  const u32 a = inst.RA;
  const u32 b = inst.RB;
  const u32 d = inst.RD;
  const s32 offset = inst.SIMM_16;

  s32 offsetReg = -1;
  u32 flags = BackPatchInfo::FLAG_LOAD;
  bool update = false;

  switch (inst.OPCD)
  {
  case 31:
    offsetReg = static_cast<s32>(b);
    switch (inst.SUBOP10)
    {
    case 55:  // lwzux
      update = true;
      [[fallthrough]];
    case 23:  // lwzx
      flags |= BackPatchInfo::FLAG_SIZE_32;
      break;

    case 119:  // lbzux
      update = true;
      [[fallthrough]];
    case 87:   // lbzx
      flags |= BackPatchInfo::FLAG_SIZE_8;
      break;

    case 311:  // lhzux
      update = true;
      [[fallthrough]];
    case 279:  // lhzx
      flags |= BackPatchInfo::FLAG_SIZE_16;
      break;

    case 375:  // lhaux
      update = true;
      [[fallthrough]];
    case 343:  // lhax
      flags |= BackPatchInfo::FLAG_EXTEND | BackPatchInfo::FLAG_SIZE_16;
      break;

    case 534:  // lwbrx
      flags |= BackPatchInfo::FLAG_REVERSE | BackPatchInfo::FLAG_SIZE_32;
      break;

    case 790:  // lhbrx
      flags |= BackPatchInfo::FLAG_REVERSE | BackPatchInfo::FLAG_SIZE_16;
      break;
    }
    break;

  case 33:  // lwzu
    update = true;
    [[fallthrough]];
  case 32:  // lwz
    flags |= BackPatchInfo::FLAG_SIZE_32;
    break;

  case 35:  // lbzu
    update = true;
    [[fallthrough]];
  case 34:  // lbz
    flags |= BackPatchInfo::FLAG_SIZE_8;
    break;

  case 41:  // lhzu
    update = true;
    [[fallthrough]];
  case 40:  // lhz
    flags |= BackPatchInfo::FLAG_SIZE_16;
    break;

  case 43:  // lhau
    update = true;
    [[fallthrough]];
  case 42:  // lha
    flags |= BackPatchInfo::FLAG_EXTEND | BackPatchInfo::FLAG_SIZE_16;
    break;
  }

  // Translate flags to Arm32 SafeLoadToReg parameters.
  u32 accessSize = 0;
  const bool signExtend = (flags & BackPatchInfo::FLAG_EXTEND) != 0;
  const bool reverse    = (flags & BackPatchInfo::FLAG_REVERSE) != 0;

  if (flags & BackPatchInfo::FLAG_SIZE_8)
    accessSize = 8;
  else if (flags & BackPatchInfo::FLAG_SIZE_16)
    accessSize = 16;
  else if (flags & BackPatchInfo::FLAG_SIZE_32)
    accessSize = 32;

  // Exception check before loading.
  ARMReg rA = gpr.GetReg(false);
  ARMReg RD = gpr.R(d);

  LDR(rA, PPC_REG, PPCSTATE_OFF(Exceptions));
  TST(rA, EXCEPTION_DSI);
  FixupBranch DoNotLoad = B_CC(CC_NEQ);

  SafeLoadToReg(RD, update ? static_cast<s32>(a) : (a ? static_cast<s32>(a) : -1),
                offsetReg, accessSize, offset, signExtend, reverse, update);

  SetJumpTarget(DoNotLoad);

  // We branched here because Exceptions had EXCEPTION_DSI set.
  // Log the fact that we skipped the SafeLoadToReg.
  ARMReg tmp = gpr.GetReg(false);
  LDR(tmp, PPC_REG, PPCSTATE_OFF(Exceptions));
  LogNumFromJIT("Skipped SafeLoadToReg due to exception, Exceptions=", tmp);


  Core::CPUThreadGuard guard(m_system);
}

// Some games use this heavily in video codecs
// We make the assumption that this pulls from main RAM at /all/ times
void JitArm::lmw(UGeckoInstruction inst)
{
	INSTRUCTION_START
	JITDISABLE(bJITLoadStoreOff);
	FALLBACK_IF(!jo.fastmem);

	u32 a = inst.RA;
	ARMReg rA = gpr.GetReg();
	MOVI2R(rA, inst.SIMM_16);
	if (a)
		ADD(rA, rA, gpr.R(a));
	Operand2 mask(2, 1); // ~(Memory::MEMVIEW32_MASK)
	BIC(rA, rA, mask);
	ADD(rA, rA, R8);

	for (int i = inst.RD; i < 32; i++)
	{
		ARMReg RX = gpr.R(i);
		LDR(RX, rA, (i - inst.RD) * 4);
		REV(RX, RX);
	}
	gpr.Unlock(rA);
}

void JitArm::stmw(UGeckoInstruction inst)
{
	INSTRUCTION_START
	JITDISABLE(bJITLoadStoreOff);
	FALLBACK_IF(!jo.fastmem);

	u32 a = inst.RA;
	ARMReg rA = gpr.GetReg();
	ARMReg rB = gpr.GetReg();
	MOVI2R(rA, inst.SIMM_16);
	if (a)
		ADD(rA, rA, gpr.R(a));
	Operand2 mask(2, 1); // ~(Memory::MEMVIEW32_MASK)
	BIC(rA, rA, mask);
	ADD(rA, rA, R8);

	for (int i = inst.RD; i < 32; i++)
	{
		ARMReg RX = gpr.R(i);
		REV(rB, RX);
		STR(rB, rA, (i - inst.RD) * 4);
	}
	gpr.Unlock(rA, rB);
}

void JitArm::dcbst(UGeckoInstruction inst)
{
	INSTRUCTION_START
	JITDISABLE(bJITLoadStoreOff);

	// If the dcbst instruction is preceded by dcbt, it is flushing a prefetched
	// memory location.  Do not invalidate the JIT cache in this case as the memory
	// will be the same.
	// dcbt = 0x7c00022c
	Core::CPUThreadGuard guard(m_system);

	FALLBACK_IF((PowerPC::MMU::HostRead<u32>(guard, js.compilerPC - 4) & 0x7c00022c) != 0x7c00022c);
}

void JitArm::icbi(UGeckoInstruction inst)
{
	FallBackToInterpreter(inst);
	WriteExit(js.compilerPC + 4);
}
