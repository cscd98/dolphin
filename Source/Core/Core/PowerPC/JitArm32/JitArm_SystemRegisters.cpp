// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Common/ArmEmitter.h"
#include "Common/CommonTypes.h"

#include "Core/Core.h"
#include "Core/CoreTiming.h"
#include "Core/PowerPC/PowerPC.h"
#include "Core/PowerPC/PPCTables.h"
#include "Core/PowerPC/ConditionRegister.h"

#include "Core/PowerPC/JitArm32/Jit.h"
#include "Core/PowerPC/JitArm32/JitRegCache.h"

using namespace ArmGen;

FixupBranch JitArm::JumpIfCRFieldBit(int field, int bit, bool jump_if_set)
{
  auto RA = gpr.GetScopedReg();

  FixupBranch branch;
  switch (bit)
  {
  case PowerPC::CR_SO_BIT:  // check summary overflow bit
    // SO is stored in the high word, so load +4
    LDR(RA, PPC_REG, PPCSTATE_OFF_CR(field) + 4);
    TST(RA, IMM(1u << (PowerPC::CR_EMU_SO_BIT - 32)));
    branch = B_CC(jump_if_set ? CC_NEQ : CC_EQ);
    break;

  case PowerPC::CR_EQ_BIT:  // check if field == 0
    LDR(RA, PPC_REG, PPCSTATE_OFF_CR(field));
    CMP(RA, 0);
    branch = B_CC(jump_if_set ? CC_EQ : CC_NEQ);
    break;

  case PowerPC::CR_GT_BIT:  // check val > 0
    LDR(RA, PPC_REG, PPCSTATE_OFF_CR(field));
    CMP(RA, 0);
    branch = B_CC(jump_if_set ? CC_GT : CC_LE);
    break;

  case PowerPC::CR_LT_BIT:  // check less-than bit
    // LT is stored in the high word, so load +4
    LDR(RA, PPC_REG, PPCSTATE_OFF_CR(field) + 4);
    TST(RA, IMM(1u << (PowerPC::CR_EMU_LT_BIT - 32)));
    branch = B_CC(jump_if_set ? CC_NEQ : CC_EQ);
    break;

  default:
    ASSERT_MSG(DYNA_REC, false, "Invalid CR bit");
    break;
  }

  return branch;
}

// In JitArm32/Jit_SystemRegisters.cpp, replace the mtspr function:

// In JitArm32/Jit_SystemRegisters.cpp, replace the mtspr function:

void JitArm::mtspr(UGeckoInstruction inst)
{
  INSTRUCTION_START
  JITDISABLE(bJITSystemRegistersOff);

  const u32 iIndex = (inst.SPRU << 5) | (inst.SPRL & 0x1F);

  // Resolve source GPR once
  ARMReg rd = gpr.R(inst.RD);

  // Snapshot the source into a temp
  auto src = gpr.GetScopedReg();
  MOV(src, rd);

  switch (iIndex)
  {
    case SPR_LR:
    case SPR_CTR:
      // Branch registers are stored directly - alignment is handled by bclrx/bcctrlx
      LogRegFromJIT("mtspr -> storing to LR/CTR value", src);
      STR(src, PPC_REG, PPCSTATE_OFF_SPR(iIndex));
      break;

    case SPR_XER:
    {
      auto tmp  = gpr.GetScopedReg();
      auto mask = gpr.GetScopedReg();

      MOVI2R(mask, 0xFF7F);
      AND(tmp, src, R(mask));
      STRH(tmp, PPC_REG, PPCSTATE_OFF(xer_stringctrl));

      LSR(tmp, src, XER_CA_SHIFT);
      AND(tmp, tmp, IMM(1));
      STRB(tmp, PPC_REG, PPCSTATE_OFF(xer_ca));

      LSR(tmp, src, XER_OV_SHIFT);
      STRB(tmp, PPC_REG, PPCSTATE_OFF(xer_so_ov));
      break;
    }

    // GQRs are contiguous in spr[]
    case SPR_GQR0 + 0:
    case SPR_GQR0 + 1:
    case SPR_GQR0 + 2:
    case SPR_GQR0 + 3:
    case SPR_GQR0 + 4:
    case SPR_GQR0 + 5:
    case SPR_GQR0 + 6:
    case SPR_GQR0 + 7:
      STR(src, PPC_REG, PPCSTATE_OFF_SPR(iIndex));
      break;

    default:
      // All other SPRs (SRR0, SRR1, HID*, WPAR, DMAU, DMAL, etc.)
      STR(src, PPC_REG, PPCSTATE_OFF_SPR(iIndex));
      break;
  }
}

void JitArm::mftb(UGeckoInstruction inst)
{
	INSTRUCTION_START
	JITDISABLE(bJITSystemRegistersOff);
	mfspr(inst);
}

void JitArm::mfspr(UGeckoInstruction inst)
{
  INSTRUCTION_START
  JITDISABLE(bJITSystemRegistersOff);

  const u32 iIndex = (inst.SPRU << 5) | (inst.SPRL & 0x1F);

  switch (iIndex)
  {
    case SPR_XER:
    {
      gpr.BindToRegister(inst.RD, false);
      ARMReg RD = gpr.R(inst.RD);
      auto tmp  = gpr.GetScopedReg();

      LDRH(RD, PPC_REG, PPCSTATE_OFF(xer_stringctrl));
      LDRB(tmp, PPC_REG, PPCSTATE_OFF(xer_ca));
      LSL(tmp, tmp, XER_CA_SHIFT);
      ORR(RD, RD, static_cast<ARMReg>(tmp));

      LDRB(tmp, PPC_REG, PPCSTATE_OFF(xer_so_ov));
      LSL(tmp, tmp, XER_OV_SHIFT);
      ORR(RD, RD, static_cast<ARMReg>(tmp));
    }
    break;

    case SPR_WPAR:
    case SPR_DEC:
    case SPR_TL:
    case SPR_TU:
      FALLBACK_IF(true);
      break;

    default:
      gpr.BindToRegister(inst.RD, false);
      {
        ARMReg RD = gpr.R(inst.RD);
        LDR(RD, PPC_REG, PPCSTATE_OFF_SPR(iIndex));
      }
      break;
  }

  if (iIndex == SPR_LR)
  {
    auto verify = gpr.GetScopedReg();
    LDR(verify, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
    LogRegFromJIT("mfspr -> loaded LR value", verify);
  }
}

void JitArm::mtsr(UGeckoInstruction inst)
{
	INSTRUCTION_START
	JITDISABLE(bJITSystemRegistersOff);

	STR(gpr.R(inst.RS), PPC_REG, PPCSTATE_OFF_SR(inst.SR));
}

void JitArm::mfsr(UGeckoInstruction inst)
{
	INSTRUCTION_START
	JITDISABLE(bJITSystemRegistersOff);

	gpr.BindToRegister(inst.RD, false);
	LDR(gpr.R(inst.RD), PPC_REG, PPCSTATE_OFF_SR(inst.SR));
}

void JitArm::mtmsr(UGeckoInstruction inst)
{
	INSTRUCTION_START
	// Don't interpret this, if we do we get thrown out
	//JITDISABLE(bJITSystemRegistersOff);

	STR(gpr.R(inst.RS), PPC_REG, PPCSTATE_OFF(msr));

	gpr.Flush();
	fpr.Flush();

	WriteExit(js.compilerPC + 4);
}

void JitArm::mfmsr(UGeckoInstruction inst)
{
	INSTRUCTION_START
	JITDISABLE(bJITSystemRegistersOff);

	gpr.BindToRegister(inst.RD, false);
	LDR(gpr.R(inst.RD), PPC_REG, PPCSTATE_OFF(msr));
}

void JitArm::mcrf(UGeckoInstruction inst)
{
	INSTRUCTION_START
	JITDISABLE(bJITSystemRegistersOff);

	auto rA = gpr.GetScopedReg();

	if (inst.CRFS != inst.CRFD)
	{
		// load from CR[inst.CRFS]
		LDR(rA, PPC_REG, PPCSTATE_OFF_CR(inst.CRFS));
		// store into CR[inst.CRFD]
		STR(rA, PPC_REG, PPCSTATE_OFF_CR(inst.CRFD));
		// load from CR[inst.CRFS] + 1 word
		LDR(rA, PPC_REG, PPCSTATE_OFF_CR(inst.CRFS) + static_cast<s32>(sizeof(u32)));
		// store into CR[inst.CRFD] + 1 word
		STR(rA, PPC_REG, PPCSTATE_OFF_CR(inst.CRFD) + static_cast<s32>(sizeof(u32)));
	}
}

