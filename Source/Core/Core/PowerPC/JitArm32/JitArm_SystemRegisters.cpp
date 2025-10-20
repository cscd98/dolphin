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
  ARMReg cr_reg = gpr.CR(field);  // Load the CR value

  JIT_LOG("[JumpIfCRFieldBit] field=%d bit=%d cr_reg=R%d", field, bit, cr_reg);

  FixupBranch branch;
  switch (bit)
  {
  case PowerPC::CR_EQ_BIT:  // check if value == 0
    CMP(cr_reg, 0);
    branch = B_CC(jump_if_set ? CC_EQ : CC_NEQ);
    break;

  case PowerPC::CR_GT_BIT:  // check if value > 0
    CMP(cr_reg, 0);
    branch = B_CC(jump_if_set ? CC_GT : CC_LE);
    break;

  case PowerPC::CR_LT_BIT:  // check if value < 0 (sign bit)
    CMP(cr_reg, 0);
    branch = B_CC(jump_if_set ? CC_LT : CC_GE);
    break;

  case PowerPC::CR_SO_BIT:  // check SO bit
    // SO needs to be stored separately - for now stub
    TST(cr_reg, PowerPC::CR_SO);
    branch = B_CC(jump_if_set ? CC_NEQ : CC_EQ);
    break;

  default:
    ASSERT_MSG(DYNA_REC, false, "Invalid CR bit");
    break;
  }

  return branch;
}

void JitArm::mtspr(UGeckoInstruction inst)
{
  INSTRUCTION_START
  JITDISABLE(bJITSystemRegistersOff);

  const u32 iIndex = (inst.SPRU << 5) | (inst.SPRL & 0x1F);

  // Get source GPR (will be a callee-saved register from the cache)
  ARMReg rd = gpr.R(inst.RD);

  switch (iIndex)
  {
    case SPR_LR:
    case SPR_CTR:
      // Branch registers are stored directly
      // No need to worry about function calls - rd is already callee-saved
      JIT_LOG("[mtspr CTR] Setting CTR from r%d, value before store: (R%d) iIndex: %d",
             inst.RD, rd, iIndex);

      JIT_LOG_REG("mtspr -> storing to LR/CTR value", rd);
      STR(rd, PPC_REG, PPCSTATE_OFF_SPR(iIndex));
      break;

    case SPR_XER:
    {
      auto tmp  = gpr.GetScopedReg();
      auto mask = gpr.GetScopedReg();

      MOVI2R(mask, 0xFF7F);
      AND(tmp, rd, R(mask));
      STRH(tmp, PPC_REG, PPCSTATE_OFF(xer_stringctrl));

      LSR(tmp, rd, XER_CA_SHIFT);
      AND(tmp, tmp, 1);
      STRB(tmp, PPC_REG, PPCSTATE_OFF(xer_ca));

      LSR(tmp, rd, XER_OV_SHIFT);
      STRB(tmp, PPC_REG, PPCSTATE_OFF(xer_so_ov));
      break;
    }

    case SPR_GQR0 + 0:
    case SPR_GQR0 + 1:
    case SPR_GQR0 + 2:
    case SPR_GQR0 + 3:
    case SPR_GQR0 + 4:
    case SPR_GQR0 + 5:
    case SPR_GQR0 + 6:
    case SPR_GQR0 + 7:
      STR(rd, PPC_REG, PPCSTATE_OFF_SPR(iIndex));
      break;

    default:
      STR(rd, PPC_REG, PPCSTATE_OFF_SPR(iIndex));
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
      ORR(RD, RD, R(tmp));

      LDRB(tmp, PPC_REG, PPCSTATE_OFF(xer_so_ov));
      LSL(tmp, tmp, XER_OV_SHIFT);
      ORR(RD, RD, R(tmp));
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
    JIT_LOG_REG("mfspr -> loaded LR value", verify);
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

void JitArm::mtfsfx(UGeckoInstruction inst)
{
  INSTRUCTION_START
  JITDISABLE(bJITSystemRegistersOff);
  FALLBACK_IF(inst.Rc);
  FALLBACK_IF(jo.fp_exceptions);

  const u32 fm = inst.FM;
  const u32 fb = inst.FB;

  // Build mask
  u32 mask = 0;
  for (int i = 0; i < 8; i++)
  {
    if (fm & (1 << i))
      mask |= 0xFU << (4 * i);
  }

  ARMReg fpscrReg = gpr.GetReg();
  LDR(fpscrReg, PPC_REG, PPCSTATE_OFF(fpscr));

  // Load FB (single) into a temp FP reg
  ARMReg VB = fpr.R0(fb, FPRRegType::LowerPair);
  ARMReg WA = gpr.GetReg();
  ARMReg WB = gpr.GetReg();

  // Convert FB (float) → WA (u32)
  // VMOV cannot move FP→GPR directly on ARM32, so:
  // 1. Move FP→FPSCR flags (VMRS)
  // 2. Use VCVT to convert to integer
  // 3. VMOV from FP reg to GPR
  {
    ARMReg VT = fpr.GetReg();
    VCVT(VT, VB, TO_INT | IS_SIGNED);
    VMOV(WA, VT);
    fpr.Unlock(VT);
  }

  if (mask == 0xFFFFFFFF)
  {
    // Full overwrite
    MOV(fpscrReg, WA);
    UpdateFPExceptionSummary(fpscrReg);
    STR(fpscrReg, PPC_REG, PPCSTATE_OFF(fpscr));
    gpr.Unlock(WA, WB, fpscrReg);
    return;
  }

  if (mask != 0)
  {
    // Partial update
    // fpscr = (fpscr & ~mask) | (WA & mask)

    // WB = mask
    MOVI2R(WB, mask);

    // Clear masked bits in fpscr
    BIC(fpscrReg, fpscrReg, WB);

    // Keep only masked bits from WA
    AND(WA, WA, WB);

    // Merge
    ORR(fpscrReg, fpscrReg, WA);

    // Update exception summary if needed
    if (mask & (FPSCR_FEX | FPSCR_VX | FPSCR_ANY_X | FPSCR_ANY_E))
      UpdateFPExceptionSummary(fpscrReg);

    STR(fpscrReg, PPC_REG, PPCSTATE_OFF(fpscr));
  }

  if (fm & 1)
    UpdateRoundingMode();

  gpr.Unlock(WA, WB, fpscrReg);
}

void JitArm::UpdateFPExceptionSummary(ARMReg fpscr)
{
  ARMReg WA = gpr.GetReg();
  ARMReg WB = gpr.GetReg();

  // --------------------------------------------------
  // 1. fpscr.VX = (fpscr & FPSCR_VX_ANY) != 0
  // --------------------------------------------------

  MOVI2R(WA, FPSCR_VX_ANY);
  TST(fpscr, WA);

  // WA = (fpscr & FPSCR_VX_ANY) != 0 ? 1 : 0
  FixupBranch is_zero = B_CC(CC_EQ);
  MOVI2R(WA, 1);
  FixupBranch done1 = B();
  SetJumpTarget(is_zero);
  MOVI2R(WA, 0);
  SetJumpTarget(done1);

  {
    const int bit = MathUtil::IntLog2(FPSCR_VX);

    // Clear bit
    MOVI2R(WB, 1 << bit);
    BIC(fpscr, fpscr, WB);

    // Shift WA into position and ORR
    LSL(WA, WA, Operand2(bit));
    ORR(fpscr, fpscr, WA);
  }

  // --------------------------------------------------
  // 2. fpscr.FEX = ((fpscr >> 22) & (fpscr & FPSCR_ANY_E)) != 0
  // --------------------------------------------------

  // WA = fpscr & FPSCR_ANY_E
  MOVI2R(WA, FPSCR_ANY_E);
  AND(WA, fpscr, WA);

  // WB = fpscr >> 22
  LSR(WB, fpscr, Operand2(22));

  // Test (WA & WB) != 0
  TST(WA, WB);

  // WA = ((fpscr >> 22) & (fpscr & FPSCR_ANY_E)) != 0 ? 1 : 0
  FixupBranch is_zero2 = B_CC(CC_EQ);
  MOVI2R(WA, 1);
  FixupBranch done2 = B();
  SetJumpTarget(is_zero2);
  MOVI2R(WA, 0);
  SetJumpTarget(done2);

  {
    const int bit = MathUtil::IntLog2(FPSCR_FEX);

    MOVI2R(WB, 1 << bit);
    BIC(fpscr, fpscr, WB);

    LSL(WA, WA, Operand2(bit));
    ORR(fpscr, fpscr, WA);
  }
}

void JitArm::UpdateRoundingMode()
{
  const BitSet32 gprs_to_save = gpr.GetCallerSavedUsed();
  const BitSet32 fprs_to_save = fpr.GetCallerSavedUsed();

  // Save caller-saved GPRs
  ABI_PushRegisters(gprs_to_save);

  // TODO: not saving FPRs...

  // Call void RoundingModeUpdated(PowerPC::PowerPCState&)
  // QuickCallFunction takes a scratch reg and a raw function pointer.
  QuickCallFunction(R0, reinterpret_cast<void*>(&PowerPC::RoundingModeUpdated));

  // Restore GPRs
  ABI_PopRegisters(gprs_to_save);
}
