// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Common/ArmEmitter.h"
#include "Common/CommonTypes.h"

#include "Core/Core.h"
#include "Core/CoreTiming.h"
#include "Core/PowerPC/PowerPC.h"
#include "Core/PowerPC/PPCTables.h"
#include "Core/PowerPC/JitArm32/Jit.h"
#include "Core/PowerPC/JitArm32/JitRegCache.h"

// The branches are known good, or at least reasonably good.
// No need for a disable-mechanism.

using namespace ArmGen;

void JitArm::sc(UGeckoInstruction inst)
{
  INSTRUCTION_START
  JITDISABLE(bJITBranchOff);

  gpr.Flush();
  fpr.Flush();

  {
    auto rA = gpr.GetScopedReg();
    MOVI2R(rA, js.compilerPC + 4);
    STR(rA, PPC_REG, PPCSTATE_OFF(pc));
    LDR(rA, PPC_REG, PPCSTATE_OFF(Exceptions));
    ORR(rA, rA, EXCEPTION_SYSCALL);
    STR(rA, PPC_REG, PPCSTATE_OFF(Exceptions));
  }

  WriteExceptionExit(js.compilerPC + 4, false, true);
}

template <bool condition>
void JitArm::WriteBranchWatch(u32 origin, u32 destination, UGeckoInstruction inst,
                              ArmGen::ARMReg reg_a, ArmGen::ARMReg reg_b,
                              BitSet32 /*gpr_caller_save*/, BitSet32 /*fpr_caller_save*/)
{
  // reg_a: scratch used to hold &m_branch_watch
  // reg_b: scratch used to hold recordingActive byte
  // Note: GPR/FPR caches should already be flushed by caller when needed.

  // 1) Load BranchWatch* and check recordingActive
  MOVI2R(reg_a, reinterpret_cast<u32>(&m_branch_watch));
  constexpr u32 kRecordingActiveOff = Core::BranchWatch::GetOffsetOfRecordingActive();
  LDRB(reg_b, reg_a, kRecordingActiveOff);

  // If not active, skip the rest
  CMP(reg_b, 0);
  FixupBranch over = B_CC(CC_EQ);

  // 2) Choose function pointer based on IR and template<bool>
  const void* fn =
      m_ppc_state.msr.IR
          ? (condition ? reinterpret_cast<const void*>(&Core::BranchWatch::HitVirtualTrue_fk)
                       : reinterpret_cast<const void*>(&Core::BranchWatch::HitVirtualFalse_fk))
          : (condition ? reinterpret_cast<const void*>(&Core::BranchWatch::HitPhysicalTrue_fk)
                       : reinterpret_cast<const void*>(&Core::BranchWatch::HitPhysicalFalse_fk));

  // 3) Prepare ABI arguments
  //   R0 = BranchWatch*
  //   R1 = origin
  //   R2 = destination
  //   R3 = inst.hex
  MOVI2R(R0, reinterpret_cast<u32>(&m_branch_watch));
  MOVI2R(R1, origin);
  MOVI2R(R2, destination);
  MOVI2R(R3, inst.hex);

  // 4) Call function pointer (preserve LR) and skip if inactive
  MOVI2R(R12, reinterpret_cast<u32>(fn));
  PUSH(_LR);
  BLX(R12);
  POP(_LR);

  SetJumpTarget(over);
}

template void JitArm::WriteBranchWatch<true>(u32, u32, UGeckoInstruction,
  ArmGen::ARMReg, ArmGen::ARMReg,
  BitSet32, BitSet32);
template void JitArm::WriteBranchWatch<false>(u32, u32, UGeckoInstruction,
   ArmGen::ARMReg, ArmGen::ARMReg,
   BitSet32, BitSet32);

void JitArm::rfi(UGeckoInstruction inst)
{
  LogNumFromJIT("rfi instruction!");

  INSTRUCTION_START
  JITDISABLE(bJITBranchOff);

  gpr.Flush();
  fpr.Flush();

  const u32 mask = 0x87C0FFFF;
  const u32 clearMSR13 = 0xFFFBFFFF;

  auto WA = gpr.GetScopedReg();
  {
    auto WB = gpr.GetScopedReg();
    auto WC = gpr.GetScopedReg();

    // Load current MSR
    LDR(WC, PPC_REG, PPCSTATE_OFF(msr));

    // WC &= (~mask) & clearMSR13
    MOVI2R(WA, (~mask) & clearMSR13);
    AND(WC, WC, Operand2(static_cast<ARMReg>(WA)));

    // Load SRR1
    LDR(WA, PPC_REG, PPCSTATE_OFF_SPR(SPR_SRR1));

    // WA &= mask & clearMSR13
    MOVI2R(WB, mask & clearMSR13);
    AND(WA, WA, Operand2(static_cast<ARMReg>(WB)));

    // WA |= WC
    ORR(WA, WA, Operand2(static_cast<ARMReg>(WC)));

    // Store back to MSR
    STR(WA, PPC_REG, PPCSTATE_OFF(msr));
  }

  // Update derived state (membase, feature_flags)
  MSRUpdated(static_cast<ARMReg>(WA));

  // NPC = SRR0 (holds the return address after the one that caused the exception)
  LDR(WA, PPC_REG, PPCSTATE_OFF_SPR(SPR_SRR0));
  //WriteExceptionExit(static_cast<ARMReg>(WA));
  WriteRfiExitDestInR(static_cast<ARMReg>(WA));
}


void JitArm::bx(UGeckoInstruction inst)
{
  INSTRUCTION_START;
  JITDISABLE(bJITBranchOff);

  ArmRegCache::ScopedARMReg WA = ArmGen::INVALID_REG;

  if (inst.LK)
  {
    LogNumFromJIT("(bx): inst.LK", -1);
    WA = gpr.GetScopedReg();
    MOVI2R(WA, js.compilerPC + 4);
    STR(WA, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
  }

  // Mid-block: no forced dispatch. Let the block continue.
  if (!js.isLastInstruction)
  {
    LogNumFromJIT("(bx): not last instruction", -1);
    if (IsDebuggingEnabled())
    {
      auto wb = gpr.GetScopedReg();
      auto wc = gpr.GetScopedReg();
      WriteBranchWatch<true>(js.compilerPC, js.op->branchTo, inst, wb, wc, {}, {});
    }

    if (inst.LK && !js.op->skipLRStack)
    {
      // We have to fake the stack as the RET instruction was not
      // found in the same block. This is a big overhead, but still
      // better than calling the dispatcher.
      LogRegFromJIT("(bx): calling FakeLKExit: WA", WA);
      LogNumFromJIT("(bx): calling FakeLKExit: js.compilerPC + 4", js.compilerPC + 4);
      FakeLKExit(js.compilerPC + 4, WA);

      // FakeLKExit will NOT jump if m_enable_blr_optimization is false
      if (!m_enable_blr_optimization)
      {
        // Fallback: bail to dispatcher instead of optimizing
        WriteExit(js.op->branchTo, true, js.compilerPC + 4);
        return;
      }
    }
    return;
  }

  // Block boundary: single authoritative exit
  gpr.Flush();
  fpr.Flush();

  const u32 target = js.op->branchTo;

  if (js.op->branchIsIdleLoop)
  {
    LogNumFromJIT("(bx): branchIsIdleLoop");

    ABI_PushCalleeGPRsAndAdjustStack(true);

    auto tmp = gpr.GetScopedReg();
    MOVI2R(tmp, reinterpret_cast<u32>(&CoreTiming::GlobalIdle));
    BLX(tmp);

    ABI_PopCalleeGPRsAndAdjustStack(true);

    WriteExceptionExit(target);

    return;
  }

  if (IsDebuggingEnabled())
  {
    LogNumFromJIT("(bx): IsDebuggingEnabled true", -1);
    auto wb = gpr.GetScopedReg();
    auto wc = gpr.GetScopedReg();
    WriteBranchWatch<true>(js.compilerPC, target, inst, wb, wc, {}, {});
  }

  LogNumFromJIT("(bx): calling write exit with target: ", target);
  LogNumFromJIT("(bx): calling write exit compiler PC: ", js.compilerPC + 4);

  // Single exit: npc, optional LR, downcount, dispatch
  WriteExit(target, inst.LK, js.compilerPC + 4);
}


void JitArm::bcx(UGeckoInstruction inst)
{
  INSTRUCTION_START
  JITDISABLE(bJITBranchOff);
  // USES_CR

  auto rA = gpr.GetScopedReg();
  auto rB = gpr.GetScopedReg();
  FixupBranch pCTRDontBranch;
  if ((inst.BO & BO_DONT_DECREMENT_FLAG) == 0)  // Decrement and test CTR
  {
    LDR(rB, PPC_REG, PPCSTATE_OFF_SPR(SPR_CTR));
    SUBS(rB, rB, 1);
    STR(rB, PPC_REG, PPCSTATE_OFF_SPR(SPR_CTR));

    //SUB(32, M(&CTR), Imm8(1));
    if (inst.BO & BO_BRANCH_IF_CTR_0)
      pCTRDontBranch = B_CC(CC_NEQ);
    else
      pCTRDontBranch = B_CC(CC_EQ);
  }

  FixupBranch pConditionDontBranch;
  if ((inst.BO & BO_DONT_CHECK_CONDITION) == 0)  // Test a CR bit
  {
    pConditionDontBranch = JumpIfCRFieldBit(inst.BI >> 2, 3 - (inst.BI & 3),
                                            !(inst.BO_2 & BO_BRANCH_IF_TRUE));
  }

  if (inst.LK)
  {
    u32 Jumpto = js.compilerPC + 4;
    MOVI2R(rB, Jumpto);
    STR(rB, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
    //ARMABI_MOVI2M((u32)&LR, js.compilerPC + 4); // Careful, destroys R14, R12
  }

  u32 destination;
  if (inst.AA)
    destination = SignExt16(inst.BD << 2);
  else
    destination = js.compilerPC + SignExt16(inst.BD << 2);

  gpr.Flush(FLUSH_MAINTAIN_STATE);
  fpr.Flush(FLUSH_MAINTAIN_STATE);
  WriteExit(destination);

  if ((inst.BO & BO_DONT_CHECK_CONDITION) == 0)
    SetJumpTarget( pConditionDontBranch );
  if ((inst.BO & BO_DONT_DECREMENT_FLAG) == 0)
    SetJumpTarget( pCTRDontBranch );

  if (!analyzer.HasOption(PPCAnalyst::PPCAnalyzer::OPTION_CONDITIONAL_CONTINUE))
  {
    gpr.Flush();
    fpr.Flush();
    WriteExit(js.compilerPC + 4);
  }
}

void JitArm::bcctrx(UGeckoInstruction inst)
{
  INSTRUCTION_START
  JITDISABLE(bJITBranchOff);

  // bcctrx does not decrement/test CTR
  DEBUG_ASSERT_MSG(POWERPC, inst.BO_2 & BO_DONT_DECREMENT_FLAG,
                   "bcctrx with decrement and test CTR option is invalid!");

  if (inst.BO_2 & BO_DONT_CHECK_CONDITION)
  {
    // Unconditional branch to CTR
    gpr.Flush();
    fpr.Flush();

    ARMReg rTarget = gpr.GetScopedReg();      // exit target (raw; WriteExitDestInR will release)
    auto rTmp = gpr.GetScopedReg();     // temp for CTR/LR materialization

    // LK: LR := PC+4 (use temp, not the target)
    if (inst.LK_3)
    {
      const u32 retaddr = js.compilerPC + 4;
      MOVI2R(rTmp, retaddr);
      STR(rTmp, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
    }

    // rTmp := CTR, mask low 2 bits
    LDR(rTmp, PPC_REG, PPCSTATE_OFF_SPR(SPR_CTR));
    BIC(rTmp, rTmp, Operand2(0x3));
    // rTarget := rTmp (reg-to-reg copy)
    MOV(rTarget, static_cast<ARMReg>(rTmp));

    // Stabilize caches before emitting the exit
    gpr.Flush(FLUSH_MAINTAIN_STATE);
    fpr.Flush(FLUSH_MAINTAIN_STATE);

    WriteExitDestInR(rTarget);
  }
  else
  {
    // Conditional branch on CR bit
    auto rTmp = gpr.GetScopedReg();     // temp for CTR/LR
    ARMReg rTarget = gpr.GetScopedReg();      // exit target (raw)
    FixupBranch b = JumpIfCRFieldBit(inst.BI >> 2, 3 - (inst.BI & 3),
                                     !(inst.BO_2 & BO_BRANCH_IF_TRUE));

    // rTmp := CTR, mask low 2 bits
    LDR(rTmp, PPC_REG, PPCSTATE_OFF_SPR(SPR_CTR));
    BIC(rTmp, rTmp, Operand2(0x3));
    MOV(rTarget, static_cast<ARMReg>(rTmp));

    // LK: LR := PC+4 (use temp, not the target)
    if (inst.LK_3)
    {
      const u32 retaddr = js.compilerPC + 4;
      MOVI2R(rTmp, retaddr);
      STR(rTmp, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
    }

    gpr.Flush(FLUSH_MAINTAIN_STATE);
    fpr.Flush(FLUSH_MAINTAIN_STATE);
    WriteExitDestInR(rTarget);

    SetJumpTarget(b);

    if (!analyzer.HasOption(PPCAnalyst::PPCAnalyzer::OPTION_CONDITIONAL_CONTINUE))
    {
      gpr.Flush();
      fpr.Flush();
      WriteExit(js.compilerPC + 4);
    }
  }
}

void JitArm::bclrx(UGeckoInstruction inst)
{
  LogNumFromJIT("bclrx");

  INSTRUCTION_START;
  JITDISABLE(bJITBranchOff);

  const bool conditional =
      ((inst.BO & BO_DONT_DECREMENT_FLAG) == 0) ||
      ((inst.BO & BO_DONT_CHECK_CONDITION) == 0);

  auto WA = gpr.GetScopedReg();
  auto WB = ArmGen::INVALID_REG;
  if (conditional || inst.LK || IsDebuggingEnabled())
    WB = gpr.GetScopedReg();

  {
    LogNumFromJIT("bclrx: conditional || inst.LK || IsDebuggingEnabled()");

    FixupBranch pCTRDontBranch;
    if ((inst.BO & BO_DONT_DECREMENT_FLAG) == 0)  // Decrement and test CTR
    {
      LogNumFromJIT("bclrx: ctest CTR");

      LDR(WA, PPC_REG, PPCSTATE_OFF_SPR(SPR_CTR));
      SUBS(WA, WA, 1);
      STR(WA, PPC_REG, PPCSTATE_OFF_SPR(SPR_CTR));

      LogRegFromJIT("bclrx: updated CTR", WA);
      pCTRDontBranch = B_CC((inst.BO & BO_BRANCH_IF_CTR_0) ? CC_NEQ : CC_EQ);
    }

    FixupBranch pConditionDontBranch;
    if ((inst.BO & BO_DONT_CHECK_CONDITION) == 0)  // Test a CR bit
    {
      LogNumFromJIT("bclrx: testing CR bit", inst.BI);
      pConditionDontBranch =
          JumpIfCRFieldBit(inst.BI >> 2, 3 - (inst.BI & 3),
                           !(inst.BO_2 & BO_BRANCH_IF_TRUE));
    }

    LogNumFromJIT("bclrx: Load LR and clear low 2 bits");

    {
      auto tmp = gpr.GetScopedReg();
      MOV(tmp, PPC_REG);
      LogRegFromJIT("bclrx: r9 (PPCSTATE base)", tmp);
    }

    // Load LR and clear low 2 bits
    LDR(WA, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
    BIC(WA, WA, 0x3);
    LogRegFromJIT("bclrx: LR masked", WA);

    if (inst.LK)
    {
      LogNumFromJIT("bclrx: Load PC into WB");

      MOVI2R(WB, js.compilerPC + 4);
      STR(WB, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
      LogNumFromJIT("bclrx: wrote LR (link)", js.compilerPC + 4);
    }

    // Flush for exit emission
    gpr.Flush(conditional ? FLUSH_MAINTAIN_STATE : FLUSH_ALL);
    fpr.Flush(conditional ? FLUSH_MAINTAIN_STATE : FLUSH_ALL);

    if (js.op->branchIsIdleLoop)
    {
      ABI_PushCalleeGPRsAndAdjustStack(true);

      auto X = gpr.GetScopedReg();
      MOVI2R(X, reinterpret_cast<u32>(&CoreTiming::GlobalIdle));
      BLX(X);

      ABI_PopCalleeGPRsAndAdjustStack(true);

      LogNumFromJIT("bclrx: WriteExceptionExit", js.op->branchTo);
      WriteExceptionExit(js.op->branchTo);
    }
    else
    {
      LogRegFromJIT("bclrx: exiting via WA", WA);
      WriteBLRExit(WA);  // ensure this exits via WA (BX WA), not via LR
    }

    if ((inst.BO & BO_DONT_CHECK_CONDITION) == 0)
      SetJumpTarget(pConditionDontBranch);
    if ((inst.BO & BO_DONT_DECREMENT_FLAG) == 0)
      SetJumpTarget(pCTRDontBranch);
  }

  if (!analyzer.HasOption(PPCAnalyst::PPCAnalyzer::OPTION_CONDITIONAL_CONTINUE))
  {
    gpr.Flush(FLUSH_ALL);
    fpr.Flush(FLUSH_ALL);
    if (IsDebuggingEnabled())
    {
      WriteBranchWatch<false>(js.compilerPC, js.compilerPC + 4, inst, WA, WB, {}, {});
    }
    LogNumFromJIT("bclrx: !OPTION_CONDITIONAL_CONTINUE", js.compilerPC + 4);
    WriteExit(js.compilerPC + 4);
  }
  else if (IsDebuggingEnabled())
  {
    WriteBranchWatch<false>(js.compilerPC, js.compilerPC + 4, inst, WA, WB, {}, {});
  }
}

// Thread-local buffer removed: we allocate immutable messages per call.
static constexpr u32 LOG_SENTINEL = static_cast<u32>(-1);

// Allocate a unique immutable message for each emit site.
static const char* MakeLogMsg(const char* base, ArmGen::ARMReg reg)
{
  if (reg != ArmGen::INVALID_REG)
  {
    char tmp[128];
    const int reg_num = static_cast<int>(reg);
    int n = snprintf(tmp, sizeof(tmp), "%s (R%d)", base, reg_num);
    if (n < 0) n = 0;
    char* s = static_cast<char*>(malloc(static_cast<size_t>(n + 1)));
    if (!s) return base;
    memcpy(s, tmp, static_cast<size_t>(n + 1));
    return s;
  }
  size_t len = strlen(base);
  char* s = static_cast<char*>(malloc(len + 1));
  if (!s) return base;
  memcpy(s, base, len + 1);
  return s;
}

void JitArm::LogRegFromJIT(const char* msg, ArmGen::ARMReg reg)
{
  // Save caller state
  PUSH(14, ArmGen::R0, ArmGen::R1, ArmGen::R2, ArmGen::R3,
           ArmGen::R4, ArmGen::R5, ArmGen::R6, ArmGen::R7,
           ArmGen::R8, PPC_REG, ArmGen::R10, ArmGen::R11,
           ArmGen::R12, ArmGen::_LR);

  const char* m = MakeLogMsg(msg, reg);
  MOVI2R(ArmGen::R0, static_cast<u32>(reinterpret_cast<uintptr_t>(m)));

  // Ensure 8-byte stack alignment for the BL
  SUB(ArmGen::_SP, ArmGen::_SP, 4);

  if (reg == ArmGen::INVALID_REG)
  {
    MOVI2R(ArmGen::R1, LOG_SENTINEL);
    QuickCallFunction(ArmGen::R12, reinterpret_cast<void*>(&LogRegHelper));
  }
  else if (reg == ArmGen::R12)
  {
    // Spill R12 while using it as call-scratch
    SUB(ArmGen::_SP, ArmGen::_SP, 4);
    STR(ArmGen::R12, ArmGen::_SP, 0);

    MOV(ArmGen::R1, ArmGen::R12);
    QuickCallFunction(ArmGen::R12, reinterpret_cast<void*>(&LogRegHelper));

    LDR(ArmGen::R12, ArmGen::_SP, 0);
    ADD(ArmGen::_SP, ArmGen::_SP, 4);
  }
  else
  {
    MOV(ArmGen::R1, reg);
    QuickCallFunction(ArmGen::R12, reinterpret_cast<void*>(&LogRegHelper));
  }

  // Undo the alignment pad
  ADD(ArmGen::_SP, ArmGen::_SP, 4);

  POP(14, ArmGen::R0, ArmGen::R1, ArmGen::R2, ArmGen::R3,
          ArmGen::R4, ArmGen::R5, ArmGen::R6, ArmGen::R7,
          ArmGen::R8, PPC_REG, ArmGen::R10, ArmGen::R11,
          ArmGen::R12, ArmGen::_LR);
}

void JitArm::LogNumFromJIT(const char* msg, u32 value)
{
  PUSH(14, ArmGen::R0, ArmGen::R1, ArmGen::R2, ArmGen::R3,
           ArmGen::R4, ArmGen::R5, ArmGen::R6, ArmGen::R7,
           ArmGen::R8, PPC_REG, ArmGen::R10, ArmGen::R11,
           ArmGen::R12, ArmGen::_LR);

  // Pre-bake the message on the host (emission-time), pass pointer to helper
  size_t len = std::strlen(msg);
  char* m = static_cast<char*>(std::malloc(len + 1));
  if (m)
    std::memcpy(m, msg, len + 1);
  else
    m = const_cast<char*>(msg);

  MOVI2R(ArmGen::R0, static_cast<u32>(reinterpret_cast<uintptr_t>(m)));
  MOVI2R(ArmGen::R1, value);

  // Align SP for the BL
  SUB(ArmGen::_SP, ArmGen::_SP, 4);
  QuickCallFunction(ArmGen::R12, reinterpret_cast<void*>(&LogRegHelper));
  ADD(ArmGen::_SP, ArmGen::_SP, 4);

  POP(14, ArmGen::R0, ArmGen::R1, ArmGen::R2, ArmGen::R3,
          ArmGen::R4, ArmGen::R5, ArmGen::R6, ArmGen::R7,
          ArmGen::R8, PPC_REG, ArmGen::R10, ArmGen::R11,
          ArmGen::R12, ArmGen::_LR);
}
