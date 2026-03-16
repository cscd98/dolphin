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
  const u32 kRecordingActiveOff = Core::FakeBranchWatchCollectionKey{origin, destination};
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
  JIT_LOG_MSG("rfi instruction!");

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
    AND(WC, WC, R(WA));

    // Load SRR1
    LDR(WA, PPC_REG, PPCSTATE_OFF_SPR(SPR_SRR1));

    // WA &= mask & clearMSR13
    MOVI2R(WB, mask & clearMSR13);
    AND(WA, WA, R(WB));

    // WA |= WC
    ORR(WA, WA, R(WC));

    // Store back to MSR
    STR(WA, PPC_REG, PPCSTATE_OFF(msr));
  }

  // Update derived state (membase, feature_flags)
  MSRUpdated(WA);

  // NPC = SRR0 (holds the return address after the one that caused the exception)
  LDR(WA, PPC_REG, PPCSTATE_OFF_SPR(SPR_SRR0));
  //WriteExceptionExit(WA);
  WriteRfiExitDestInR(WA);
}

void JitArm::bx(UGeckoInstruction inst)
{
  JIT_LOG("bx(): PC=%08x LI=%08x LK=%d AA=%d target=%08x",
         js.compilerPC, inst.LI, inst.LK, inst.AA, js.op->branchTo);
  INSTRUCTION_START;
  JITDISABLE(bJITBranchOff);

  JIT_LOG("bx(): After JITDISABLE");

  ArmRegCache::ScopedARMReg WA = ArmGen::INVALID_REG;

  if (inst.LK)
  {
    JIT_LOG("bx(): inst.LK is set, getting scoped reg");

    WA = gpr.GetScopedReg();

    JIT_LOG("bx(): Got scoped reg, emitting MOVI2R");

    MOVI2R(WA, js.compilerPC + 4);

    JIT_LOG("bx(): Emitting STR to LR");

    STR(WA, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
    JIT_LOG_REG("(bx): inst.LK is set (WA holds js.compilerPC + 4): ", WA);

        JIT_LOG("bx(): Stored LR");
  }

  // Mid-block: no forced dispatch. Let the block continue.
  if (!js.isLastInstruction)
  {
    JIT_LOG("bx(): Not last instruction, handling mid-block");

    JIT_LOG_MSG("(bx): not last instruction");
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
      JIT_LOG_REG("(bx): calling FakeLKExit: WA", WA);
      JIT_LOG_NUM("(bx): calling FakeLKExit: js.compilerPC + 4", js.compilerPC + 4);
      FakeLKExit(js.compilerPC + 4, WA);

      // FakeLKExit will NOT jump if m_enable_blr_optimization is false
      if (!m_enable_blr_optimization)
      {
        JIT_LOG_NUM("(bx): blr optimization disabled so jumping to js.compilerPC + 4", js.compilerPC + 4);
        // Fallback: bail to dispatcher instead of optimizing
        WriteExit(js.op->branchTo, true, js.compilerPC + 4);
        return;
      }
    }
    return;
  }

  JIT_LOG("bx(): Last instruction, flushing registers");
  // Block boundary: single authoritative exit
  gpr.Flush();
  fpr.Flush();

  const u32 target = js.op->branchTo;

  if (js.op->branchIsIdleLoop)
  {
    JIT_LOG_MSG("(bx): branchIsIdleLoop");

    //ABI_PushCalleeGPRsAndAdjustStack(true);

    auto tmp = gpr.GetScopedReg();
    MOVI2R(tmp, reinterpret_cast<u32>(&CoreTiming::GlobalIdle));
    BLX(tmp);

    //ABI_PopCalleeGPRsAndAdjustStack(true);

    WriteExceptionExit(target);

    return;
  }

  if (IsDebuggingEnabled())
  {
    JIT_LOG_MSG("(bx): IsDebuggingEnabled true");
    auto wb = gpr.GetScopedReg();
    auto wc = gpr.GetScopedReg();
    WriteBranchWatch<true>(js.compilerPC, target, inst, wb, wc, {}, {});
  }

  JIT_LOG_NUM("(bx): calling write exit with target: ", target);
  JIT_LOG_NUM("(bx): calling write exit compiler PC: ", js.compilerPC + 4);

  // Single exit: npc, optional LR, downcount, dispatch
  //WriteExit(target, inst.LK, js.compilerPC + 4);

  JIT_LOG("bx(): Calling WriteExit with target=%08x", js.op->branchTo);

  WriteExit(js.op->branchTo, inst.LK, js.compilerPC + 4, WA);
  //WriteExit(js.op->branchTo, inst.LK, inst.LK ? js.compilerPC + 4 : 0, inst.LK ? WA : INVALID_REG);
}

void JitArm::bcx(UGeckoInstruction inst)
{
  INSTRUCTION_START
  JITDISABLE(bJITBranchOff);

  JIT_LOG("[bcx] ---- ENTER ---- PC=0x%08x OPCD=%u BO=%02x BI=%02x BD=%04x AA=%u LK=%u",
         js.compilerPC, inst.OPCD, inst.BO, inst.BI, inst.BD, inst.AA, inst.LK);

  auto WA = gpr.GetScopedReg();
  auto WB = (inst.LK || IsDebuggingEnabled()) ? gpr.GetScopedReg() : ArmRegCache::ScopedARMReg(WA.GetReg());

  // CTR handling
  FixupBranch pCTRDontBranch;
  if ((inst.BO & BO_DONT_DECREMENT_FLAG) == 0)
  {
    JIT_LOG("[bcx] CTR decrement required (BO=%02x)", inst.BO);

    LDR(WA, PPC_REG, PPCSTATE_OFF_SPR(SPR_CTR));
    JIT_LOG("[bcx]   CTR before decrement: (runtime)");

    SUBS(WA, WA, 1);
    STR(WA, PPC_REG, PPCSTATE_OFF_SPR(SPR_CTR));

    // Branch condition based on CTR==0
    bool branch_if_ctr_zero = (inst.BO & BO_BRANCH_IF_CTR_0);
    JIT_LOG("[bcx]   Branch-if-CTR-zero? %s", branch_if_ctr_zero ? "YES" : "NO");
    pCTRDontBranch = branch_if_ctr_zero ? B_CC(CC_NEQ) : B_CC(CC_EQ);
  }
  else
  {
    JIT_LOG("[bcx] CTR NOT decremented (BO=%02x)", inst.BO);
  }

  // CR bit test
  FixupBranch pConditionDontBranch;
  if ((inst.BO & BO_DONT_CHECK_CONDITION) == 0)
  {
    JIT_LOG("[bcx] CR test required: BI=%u (field=%u bit=%u)",
           inst.BI, inst.BI >> 2, 3 - (inst.BI & 3));

    bool branch_if_true = inst.BO_2 & BO_BRANCH_IF_TRUE;
    JIT_LOG("[bcx]   Branch-if-CR-true? %s", branch_if_true ? "YES" : "NO");
    pConditionDontBranch =
        JumpIfCRFieldBit(inst.BI >> 2, 3 - (inst.BI & 3), !branch_if_true);
  }
  else
  {
    JIT_LOG("[bcx] CR NOT tested (BO=%02x)", inst.BO);
  }

  // LR update
  if (inst.LK)
  {
    JIT_LOG("[bcx] LK=1, writing LR = 0x%08x", js.compilerPC + 4);
    MOVI2R(WB, js.compilerPC + 4);
    STR(WB, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
  }

  // Destination calculation
  u32 destination = inst.AA ? SignExt16(inst.BD << 2)
                            : js.compilerPC + SignExt16(inst.BD << 2);

  JIT_LOG("[bcx] Branch destination = 0x%08x (AA=%u)", destination, inst.AA);

  gpr.Flush(FlushMode::MaintainState);
  fpr.Flush(FlushMode::MaintainState);

  if (js.op->branchIsIdleLoop)
  {
    JIT_LOG("[bcx] Idle loop detected — calling GlobalIdle");

    MOVI2R(WA, (u32)&CoreTiming::GlobalIdle);
    BL((const void*)&CoreTiming::GlobalIdle);
    WriteExceptionExit(js.op->branchTo);
  }
  else
  {
    JIT_LOG("[bcx] Writing exit to destination 0x%08x", destination);
    WriteExit(destination);
  }

  // Patch fall-throughs
  if (!(inst.BO & BO_DONT_CHECK_CONDITION))
  {
    JIT_LOG("[bcx] Setting CR fall-through target");
    SetJumpTarget(pConditionDontBranch);
  }

  if (!(inst.BO & BO_DONT_DECREMENT_FLAG))
  {
    JIT_LOG("[bcx] Setting CTR fall-through target");
    SetJumpTarget(pCTRDontBranch);
  }

  if (!analyzer.HasOption(PPCAnalyst::PPCAnalyzer::OPTION_CONDITIONAL_CONTINUE))
  {
    JIT_LOG("[bcx] Conditional continue disabled — forcing exit to next PC");
    gpr.Flush(FlushMode::All);
    fpr.Flush(FlushMode::All);
    WriteExit(js.compilerPC + 4);
  }

  JIT_LOG("[bcx] ---- EXIT ---- code_ptr=%p", GetCodePtr());
}

/*void JitArm::bcx(UGeckoInstruction inst)
{
  INSTRUCTION_START
  JITDISABLE(bJITBranchOff);
  // USES_CR

  auto rA = gpr.GetScopedReg();
  auto rB = gpr.GetScopedReg();

  FixupBranch pCTRDontBranch;
  if ((inst.BO & BO_DONT_DECREMENT_FLAG) == 0)
  {
    LDR(rB, PPC_REG, PPCSTATE_OFF_SPR(SPR_CTR));
    SUBS(rB, rB, 1);
    STR(rB, PPC_REG, PPCSTATE_OFF_SPR(SPR_CTR));

    if (inst.BO & BO_BRANCH_IF_CTR_0)
      pCTRDontBranch = B_CC(CC_NEQ);  // Don't branch if CTR != 0
    else
      pCTRDontBranch = B_CC(CC_EQ);   // Don't branch if CTR == 0
  }

  FixupBranch pConditionDontBranch;
  if ((inst.BO & BO_DONT_CHECK_CONDITION) == 0)
  {
    pConditionDontBranch = JumpIfCRFieldBit(inst.BI >> 2, 3 - (inst.BI & 3),
                                            !(inst.BO_2 & BO_BRANCH_IF_TRUE));
  }

  // BRANCH TAKEN PATH:
  if (inst.LK)
  {
    u32 Jumpto = js.compilerPC + 4;
    MOVI2R(rB, Jumpto);
    STR(rB, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
  }

  u32 destination;
  if (inst.AA)
    destination = SignExt16(inst.BD << 2);
  else
    destination = js.compilerPC + SignExt16(inst.BD << 2);

  gpr.Flush(FlushMode::MaintainState);
  fpr.Flush(FlushMode::MaintainState);
  WriteExit(destination);  // Branch taken - goes to 0x800043d4

  // FALL-THROUGH PATH (branch NOT taken):
  if ((inst.BO & BO_DONT_CHECK_CONDITION) == 0)
    SetJumpTarget(pConditionDontBranch);  // ← Lands HERE
  if ((inst.BO & BO_DONT_DECREMENT_FLAG) == 0)
    SetJumpTarget(pCTRDontBranch);        // ← Lands HERE

  if (!analyzer.HasOption(PPCAnalyst::PPCAnalyzer::OPTION_CONDITIONAL_CONTINUE))
  {
    gpr.Flush();
    fpr.Flush();
    WriteExit(js.compilerPC + 4);  // Fall through to next instruction
  }

  JIT_LOG("[bcx] After WriteExit(destination=0x%08x), code_ptr=%p\n", destination, GetCodePtr());
  if ((inst.BO & BO_DONT_CHECK_CONDITION) == 0)
  {
    JIT_LOG("[bcx] Setting condition jump target to %p\n", GetCodePtr());
    SetJumpTarget(pConditionDontBranch);
  }
  if ((inst.BO & BO_DONT_DECREMENT_FLAG) == 0)
  {
    JIT_LOG("[bcx] Setting CTR jump target to %p\n", GetCodePtr());
    SetJumpTarget(pCTRDontBranch);
  }
}*/

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
    MOV(rTarget, rTmp);

    // Stabilize caches before emitting the exit
    gpr.Flush(FlushMode::MaintainState);
    fpr.Flush(FlushMode::MaintainState);

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
    MOV(rTarget, rTmp);

    // LK: LR := PC+4 (use temp, not the target)
    if (inst.LK_3)
    {
      const u32 retaddr = js.compilerPC + 4;
      MOVI2R(rTmp, retaddr);
      STR(rTmp, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
    }

    gpr.Flush(FlushMode::MaintainState);
    fpr.Flush(FlushMode::MaintainState);
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
  JIT_LOG(">>> bclrx reached: inst.hex=%08x compilerPC=%08x", inst.hex, js.compilerPC);

  JIT_LOG_NUM("bclrx: current inst.hex", inst.hex);
  JIT_LOG_NUM("bclrx: current compiler.PC", js.compilerPC);

  /*{
    auto tmp = gpr.GetScopedReg();
    LDR(tmp, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
    JIT_LOG_REG("bclrx: LR at start is:", tmp);
  }*/

  INSTRUCTION_START;
  JITDISABLE(bJITBranchOff);

  const bool conditional =
      ((inst.BO & BO_DONT_DECREMENT_FLAG) == 0) ||
      ((inst.BO & BO_DONT_CHECK_CONDITION) == 0);

  auto WA = gpr.GetScopedReg();
  auto WB = ArmGen::INVALID_REG;
  if (conditional || inst.LK)
    WB = gpr.GetScopedReg();

  {
    JIT_LOG_MSG("bclrx: matches either conditional || inst.LK");

    auto LR_safe = gpr.GetScopedReg();
    JIT_LOG_REG("bclrx: LR_safe BEFORE loading LR:", LR_safe);

    // Load LR into the callee-saved register
    LDR(LR_safe, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
    JIT_LOG_REG("bclrx: LR_safe AFTER loading LR:", LR_safe);

    BIC(LR_safe, LR_safe, 0x3);  // Clear low 2 bits
    JIT_LOG_REG("bclrx: LR_safe AFTER masking:", LR_safe);

    FixupBranch pCTRDontBranch;
    if ((inst.BO & BO_DONT_DECREMENT_FLAG) == 0)  // Decrement and test CTR
    {
      LDR(WA, PPC_REG, PPCSTATE_OFF_SPR(SPR_CTR));
      SUBS(WA, WA, 1);
      STR(WA, PPC_REG, PPCSTATE_OFF_SPR(SPR_CTR));

      JIT_LOG_REG("bclrx: updated CTR - WA is now", WA);
      pCTRDontBranch = B_CC((inst.BO & BO_BRANCH_IF_CTR_0) ? CC_NEQ : CC_EQ);
    }

    FixupBranch pConditionDontBranch;
    if ((inst.BO & BO_DONT_CHECK_CONDITION) == 0)  // Test a CR bit
    {
      JIT_LOG_NUM("bclrx: testing CR bit", inst.BI);
      pConditionDontBranch =
          JumpIfCRFieldBit(inst.BI >> 2, 3 - (inst.BI & 3),
                           !(inst.BO_2 & BO_BRANCH_IF_TRUE));
    }

    if (inst.LK)
    {
      JIT_LOG_MSG("bclrx: Load PC into WB");

      MOVI2R(WB, js.compilerPC + 4);
      STR(WB, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
      JIT_LOG_NUM("bclrx: wrote LR (link)", js.compilerPC + 4);
    }

    JIT_LOG_REG("bclrx: LR_safe before flush: ", LR_safe);

    // Flush for exit emission
    // Use IgnoreDiscardedRegisters::No to ensure all registers are properly handled
    gpr.Flush(conditional ? FlushMode::MaintainState : FlushMode::All,
              ArmGen::INVALID_REG, // TODO
              IgnoreDiscardedRegisters::No);
    fpr.Flush(conditional ? FlushMode::MaintainState : FlushMode::All,
              ArmGen::INVALID_REG, // TODO
              IgnoreDiscardedRegisters::No);

    JIT_LOG_REG("bclrx: LR_safe after flush: ", LR_safe);

    if (js.op->branchIsIdleLoop)
    {
      JIT_LOG_REG("bclrx: LR_safe before ABI_PushCalleeGPRsAndAdjustStack: ", LR_safe);

      //ABI_PushCalleeGPRsAndAdjustStack(true);

      {
        auto X = gpr.GetScopedReg();
        MOVI2R(X, reinterpret_cast<u32>(&CoreTiming::GlobalIdle));
        BLX(X);
      }

      //ABI_PopCalleeGPRsAndAdjustStack(true);

      JIT_LOG_NUM("bclrx: WriteExceptionExit", js.op->branchTo);
      WriteExceptionExit(js.op->branchTo);
    }
    else
    {
      {
        auto tmp = gpr.GetScopedReg();
        LDR(tmp, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
        JIT_LOG_REG("bclrx: LR now is:", tmp);
      }

      JIT_LOG_REG("bclrx: exiting via LR_safe: ", LR_safe);
      WriteBLRExit(LR_safe);  // Use LR_safe (callee-saved) instead of WA
    }

    if ((inst.BO & BO_DONT_CHECK_CONDITION) == 0)
      SetJumpTarget(pConditionDontBranch);
    if ((inst.BO & BO_DONT_DECREMENT_FLAG) == 0)
      SetJumpTarget(pCTRDontBranch);
  }

  if (conditional && !analyzer.HasOption(PPCAnalyst::PPCAnalyzer::OPTION_CONDITIONAL_CONTINUE))
  {
    gpr.Flush(FlushMode::All, ArmGen::INVALID_REG, IgnoreDiscardedRegisters::No);
    fpr.Flush(FlushMode::All, ArmGen::INVALID_REG, IgnoreDiscardedRegisters::No);
    if (IsDebuggingEnabled())
    {
      WriteBranchWatch<false>(js.compilerPC, js.compilerPC + 4, inst, WA, WB, {}, {});
    }
    JIT_LOG_NUM("bclrx: !OPTION_CONDITIONAL_CONTINUE", js.compilerPC + 4);
    WriteExit(js.compilerPC + 4);
  }
  else if (IsDebuggingEnabled())
  {
    WriteBranchWatch<false>(js.compilerPC, js.compilerPC + 4, inst, WA, WB, {}, {});
  }
}

// Thread-local buffer removed: we allocate immutable messages per call.
/*static constexpr u32 LOG_SENTINEL = static_cast<u32>(-1);

static char g_log_buffers[32][512];
static int g_log_buffer_index = 0;

static const char* MakeLogMsg(const char* base, ArmGen::ARMReg reg)
{
  int idx = g_log_buffer_index;
  g_log_buffer_index = (g_log_buffer_index + 1) % 32;

  char* buffer = g_log_buffers[idx];

  if (reg != ArmGen::INVALID_REG)
  {
    const int reg_num = static_cast<int>(reg);
    snprintf(buffer, 512, "%s (R%d)", base, reg_num);
  }
  else
  {
    snprintf(buffer, 512, "%s", base);
  }

  return buffer;
}*/

// FIXED logging functions that don't corrupt the stack
/*void JitArm::LogRegFromJIT(const char* msg, ArmGen::ARMReg reg)
{
//#ifdef JITARM_DEBUG
  // Save all caller-saved registers + LR
  // ARM calling convention: R0-R3, R12, LR are caller-saved
  // We also need to save R4-R11 if we're using them (callee-saved)
  // PPC_REG (R9) must be preserved!

  // Push registers in pairs for proper alignment (ARM requires 8-byte stack alignment)
  // Total: 14 registers = 56 bytes
  PUSH(6, R0, R1, R2, R3, R12, _LR); // caller-saved regs + LR
  //PUSH(14, ArmGen::R0, ArmGen::R1, ArmGen::R2, ArmGen::R3,
  //         ArmGen::R4, ArmGen::R5, ArmGen::R6, ArmGen::R7,
  //         ArmGen::R8, PPC_REG, ArmGen::R10, ArmGen::R11,
  //         ArmGen::R12, ArmGen::_LR);

  // Stack is now automatically 8-byte aligned after PUSH (56 bytes)
  // NO need to adjust SP further!

  const char* m = MakeLogMsg(msg, reg);
  MOVI2R(ArmGen::R0, static_cast<u32>(reinterpret_cast<uintptr_t>(m)));

  if (reg == ArmGen::INVALID_REG)
  {
    MOVI2R(ArmGen::R1, LOG_SENTINEL);
  }
  else if (reg == ArmGen::R12)
  {
    // R12 is already saved on stack, but we need its value
    // It's at offset: 13 registers down from top = 52 bytes
    LDR(ArmGen::R1, ArmGen::_SP, 52);  // Load saved R12
  }
  else
  {
    MOV(ArmGen::R1, reg);
  }

  QuickCallFunction(ArmGen::R12, reinterpret_cast<void*>(&LogRegHelper));

  POP(6, R0, R1, R2, R3, R12, _LR); // caller-saved regs + LR

  // Restore all registers
  //POP(14, ArmGen::R0, ArmGen::R1, ArmGen::R2, ArmGen::R3,
  //        ArmGen::R4, ArmGen::R5, ArmGen::R6, ArmGen::R7,
  //        ArmGen::R8, PPC_REG, ArmGen::R10, ArmGen::R11,
  //        ArmGen::R12, ArmGen::_LR);
//#endif
}

void JitArm::JIT_LOG_NUM(const char* msg, u32 value)
{
//#ifdef JITARM_DEBUG
  // Same register save pattern
  PUSH(6, R0, R1, R2, R3, R12, _LR); // caller-saved regs + LR
  //PUSH(14, ArmGen::R0, ArmGen::R1, ArmGen::R2, ArmGen::R3,
  //         ArmGen::R4, ArmGen::R5, ArmGen::R6, ArmGen::R7,
  //         ArmGen::R8, PPC_REG, ArmGen::R10, ArmGen::R11,
  //         ArmGen::R12, ArmGen::_LR);

  // Stack is 8-byte aligned after pushing 14 registers (56 bytes)

  size_t len = std::strlen(msg);
  char* m = static_cast<char*>(std::malloc(len + 1));
  if (m)
    std::memcpy(m, msg, len + 1);
  else
    m = const_cast<char*>(msg);

  MOVI2R(ArmGen::R0, static_cast<u32>(reinterpret_cast<uintptr_t>(m)));
  MOVI2R(ArmGen::R1, value);

  QuickCallFunction(ArmGen::R12, reinterpret_cast<void*>(&LogRegHelper));

  POP(6, R0, R1, R2, R3, R12, _LR); // caller-saved regs + LR
  //POP(14, ArmGen::R0, ArmGen::R1, ArmGen::R2, ArmGen::R3,
  //        ArmGen::R4, ArmGen::R5, ArmGen::R6, ArmGen::R7,
  //        ArmGen::R8, PPC_REG, ArmGen::R10, ArmGen::R11,
  //        ArmGen::R12, ArmGen::_LR);
//#endif
}*/

/*void JitArm::LogRegFromJIT(const char* msg, ArmGen::ARMReg reg)
{
  auto scratch = R12;   // caller-saved, safe
  auto tmp     = R11;   // caller-saved, safe

  // Load head index
  LDR(scratch, PPC_REG, PPCSTATE_OFF(jit_log_head));

  // idx = head & (CAP - 1)
  AND(scratch, scratch, JIT_LOG_CAP - 1);

  // entry = base + idx * sizeof(JitLogEntry)
  // sizeof(JitLogEntry) = 8
  MOVI2R(tmp, PPCSTATE_OFF(jit_log_buf));
  ADD(tmp, tmp, Operand2(scratch, ST_LSL, 3));  // idx * 8

  // Store msg pointer
  MOVI2R(scratch, reinterpret_cast<u32>(msg));
  STR(scratch, PPC_REG, tmp);

  // Store value
  if (reg != INVALID_REG)
    STR(reg, PPC_REG, tmp + 4);
  else {
    MOVI2R(scratch, 0);
    STR(scratch, PPC_REG, tmp + 4);
  }

  // head++
  LDR(scratch, PPC_REG, PPCSTATE_OFF(jit_log_head));
  ADD(scratch, scratch, 1);
  STR(scratch, PPC_REG, PPCSTATE_OFF(jit_log_head));
}

void JitArm::LogNumFromJIT(const char* msg, u32 value)
{
  auto scratch = R12;
  auto tmp     = R11;   // caller-saved, safe

  // load head
  MOVI2R(scratch, PPCSTATE_OFF(jit_log_head));
  LDR(scratch, PPC_REG, PPCSTATE_OFF(jit_log_head));

  // idx = head & (CAP-1)
  AND(scratch, scratch, JIT_LOG_CAP - 1);

// entry = base + idx * sizeof(JitLogEntry)
// sizeof(JitLogEntry) = 8
  MOVI2R(tmp, PPCSTATE_OFF(jit_log_buf));
  ADD(tmp, tmp, Operand2(scratch, ST_LSL, 3));  // idx * 8

  // store msg
  MOVI2R(R10, (u32)msg);
  STR(R10, PPC_REG, R11);

  // store value
  MOVI2R(R10, value);
  STR(R10, PPC_REG, R11 + 4);

  // head++
  ADD(scratch, scratch, 1);
  STR(scratch, PPC_REG, PPCSTATE_OFF(jit_log_head));
}*/

// In Jit.h or Jit.cpp
void JitArm::LogRegFromJIT(const char* msg, ArmGen::ARMReg reg)
{
  printf("[JIT_REG] %s reg=%d\n", msg, static_cast<int>(reg));
  fflush(stdout);

  // Get buffer address
  /*MOVI2R(R11, reinterpret_cast<u32>(&g_jit_log_write_idx));

  // Load current write index
  LDR(R12, R11, 0);

  // Calculate buffer entry address: &g_jit_log_buffer[index]
  // Each entry is 8 bytes (msg ptr + value)
  auto entry_addr = R10;
  LSL(entry_addr, R12, 3);  // index * 8
  MOVI2R(R11, reinterpret_cast<u32>(g_jit_log_buffer));
  ADD(entry_addr, entry_addr, R11);

  // Store message pointer
  MOVI2R(R11, reinterpret_cast<u32>(msg));
  STR(R11, entry_addr, 0);

  // Store value
  if (reg != INVALID_REG)
  {
    STR(reg, entry_addr, 4);
  }
  else
  {
    MOVI2R(R11, 0);
    STR(R11, entry_addr, 4);
  }

  // Increment write index (mod 256)
  MOVI2R(R11, reinterpret_cast<u32>(&g_jit_log_write_idx));
  LDR(R12, R11, 0);
  ADD(R12, R12, 1);
  AND(R12, R12, 0xFF);
  STR(R12, R11, 0);*/
}

/*void JitArm::LogRegFromJIT(const char* msg, ArmGen::ARMReg reg)
{
  // Save all registers except PPC_REG (R9)
  for (int i = 0; i <= 8; ++i) {
    STR(static_cast<ArmGen::ARMReg>(i), PPC_REG, PPCSTATE_OFF_DEBUG_SCRATCH(i));
  }
  for (int i = 10; i <= 12; ++i) {
    STR(static_cast<ArmGen::ARMReg>(i), PPC_REG, PPCSTATE_OFF_DEBUG_SCRATCH(i));
  }
  STR(_LR, PPC_REG, PPCSTATE_OFF_DEBUG_SCRATCH(13));

  // Save VFP registers (caller-saved: D0-D7 / S0-S15)
  // Store at PPCSTATE_OFF_DEBUG_SCRATCH(14) onwards
  //for (int i = 0; i < 8; ++i) {
  //  VSTR(static_cast<ArmGen::ARMReg>(ArmGen::D0 + i), PPC_REG,
  //       PPCSTATE_OFF_DEBUG_SCRATCH(14 + i*2));
 // }

  const char* m = MakeLogMsg(msg, reg);
  MOVI2R(R0, static_cast<u32>(reinterpret_cast<uintptr_t>(m)));
  if (reg == ArmGen::INVALID_REG) {
    MOVI2R(R1, LOG_SENTINEL);
  } else {
    MOV(R1, reg);
  }

  QuickCallFunction(R12, reinterpret_cast<void*>(&LogRegHelper));

  // Restore VFP registers
  for (int i = 0; i < 8; ++i) {
    VLDR(static_cast<ArmGen::ARMReg>(ArmGen::D0 + i), PPC_REG,
         PPCSTATE_OFF_DEBUG_SCRATCH(14 + i*2));
  }

  // Restore GPRs
  for (int i = 0; i <= 8; ++i) {
    LDR(static_cast<ArmGen::ARMReg>(i), PPC_REG, PPCSTATE_OFF_DEBUG_SCRATCH(i));
  }
  for (int i = 10; i <= 12; ++i) {
    LDR(static_cast<ArmGen::ARMReg>(i), PPC_REG, PPCSTATE_OFF_DEBUG_SCRATCH(i));
  }
  LDR(_LR, PPC_REG, PPCSTATE_OFF_DEBUG_SCRATCH(13));
}*/

/*void JitArm::LogNumFromJIT(const char* msg, unsigned int value)
{
  for (int i = 0; i <= 8; ++i) {
    STR(static_cast<ArmGen::ARMReg>(i), PPC_REG, PPCSTATE_OFF_DEBUG_SCRATCH(i));
  }
  for (int i = 10; i <= 12; ++i) {
    STR(static_cast<ArmGen::ARMReg>(i), PPC_REG, PPCSTATE_OFF_DEBUG_SCRATCH(i));
  }
  STR(_LR, PPC_REG, PPCSTATE_OFF_DEBUG_SCRATCH(13));

  // Save VFP registers (caller-saved: D0-D7 / S0-S15)
  // Store at PPCSTATE_OFF_DEBUG_SCRATCH(14) onwards
  for (int i = 0; i < 8; ++i) {
    VSTR(static_cast<ArmGen::ARMReg>(ArmGen::D0 + i), PPC_REG,
         PPCSTATE_OFF_DEBUG_SCRATCH(14 + i*2));
  }

  size_t len = std::strlen(msg);
  char* m = static_cast<char*>(std::malloc(len + 1));
  if (m) {
    std::memcpy(m, msg, len + 1);
  } else {
    m = const_cast<char*>(msg);
  }

  MOVI2R(R0, static_cast<u32>(reinterpret_cast<uintptr_t>(m)));
  MOVI2R(R1, value);

  QuickCallFunction(R12, reinterpret_cast<void*>(&LogRegHelper));

  // Restore VFP registers
  for (int i = 0; i < 8; ++i) {
    VLDR(static_cast<ArmGen::ARMReg>(ArmGen::D0 + i), PPC_REG,
         PPCSTATE_OFF_DEBUG_SCRATCH(14 + i*2));
  }

  for (int i = 0; i <= 8; ++i) {
    LDR(static_cast<ArmGen::ARMReg>(i), PPC_REG, PPCSTATE_OFF_DEBUG_SCRATCH(i));
  }
  for (int i = 10; i <= 12; ++i) {
    LDR(static_cast<ArmGen::ARMReg>(i), PPC_REG, PPCSTATE_OFF_DEBUG_SCRATCH(i));
  }
  LDR(_LR, PPC_REG, PPCSTATE_OFF_DEBUG_SCRATCH(13));
}*/

void JitArm::LogNumFromJIT(const char* msg, u32 value)
{
  printf("[JIT_NUM] %s: 0x%08x (%u)\n", msg, value, value);
  fflush(stdout);

  /*// Get buffer address
  MOVI2R(R11, reinterpret_cast<u32>(&g_jit_log_write_idx));

  // Load current write index
  LDR(R12, R11, 0);

  // Calculate buffer entry address: &g_jit_log_buffer[index]
  // Each entry is 8 bytes (msg ptr + value)
  auto entry_addr = R10;
  LSL(entry_addr, R12, 3);  // index * 8
  MOVI2R(R11, reinterpret_cast<u32>(g_jit_log_buffer));
  ADD(entry_addr, entry_addr, R11);

  // Store message pointer
  MOVI2R(R11, reinterpret_cast<u32>(msg));
  STR(R11, entry_addr, 0);

  // Store the immediate value
  MOVI2R(R11, value);
  STR(R11, entry_addr, 4);

  // Increment write index (mod 256)
  MOVI2R(R11, reinterpret_cast<u32>(&g_jit_log_write_idx));
  LDR(R12, R11, 0);
  ADD(R12, R12, 1);
  AND(R12, R12, 0xFF);
  STR(R12, R11, 0);*/
}

//alignas(16) static u32 g_jit_debug_scratch[64];

/*void JitArm::LogRegFromJIT(const char* msg, ArmGen::ARMReg reg)
{
  // Load base pointer to debug scratch
  MOVI2R(R10, (u32)g_jit_debug_scratch);

  // Save GPRs r0–r8
  for (int i = 0; i <= 8; ++i)
    STR(static_cast<ArmGen::ARMReg>(i), R10, i * 4);

  // Save r10–r12
  for (int i = 10; i <= 12; ++i)
    STR(static_cast<ArmGen::ARMReg>(i), R10, i * 4);

  // Save LR
  STR(_LR, R10, 13 * 4);

  // Save VFP D0–D7 (caller-saved)
  for (int i = 0; i < 8; ++i)
    VSTR(static_cast<ArmGen::ARMReg>(ArmGen::D0 + i), R10, (14 + i*2) * 4);

  // Prepare args
  const char* m = MakeLogMsg(msg, reg);
  MOVI2R(R0, (u32)m);
  if (reg == ArmGen::INVALID_REG)
    MOVI2R(R1, LOG_SENTINEL);
  else
    MOV(R1, reg);

  // Call helper
  QuickCallFunction(R12, (void*)&LogRegHelper);

  // Restore VFP
  for (int i = 0; i < 8; ++i)
    VLDR(static_cast<ArmGen::ARMReg>(ArmGen::D0 + i), R10, (14 + i*2) * 4);

  // Restore GPRs
  for (int i = 0; i <= 8; ++i)
    LDR(static_cast<ArmGen::ARMReg>(i), R10, i * 4);

  for (int i = 10; i <= 12; ++i)
    LDR(static_cast<ArmGen::ARMReg>(i), R10, i * 4);

  LDR(_LR, R10, 13 * 4);
}

void JitArm::LogNumFromJIT(const char* msg, unsigned int value)
{
  MOVI2R(R10, (u32)g_jit_debug_scratch);

  for (int i = 0; i <= 8; ++i)
    STR(static_cast<ArmGen::ARMReg>(i), R10, i * 4);

  for (int i = 10; i <= 12; ++i)
    STR(static_cast<ArmGen::ARMReg>(i), R10, i * 4);

  STR(_LR, R10, 13 * 4);

  for (int i = 0; i < 8; ++i)
    VSTR(static_cast<ArmGen::ARMReg>(ArmGen::D0 + i), R10, (14 + i*2) * 4);

  const char* m = MakeLogMsg(msg, ArmGen::INVALID_REG);

  MOVI2R(R0, (u32)m);
  MOVI2R(R1, value);

  QuickCallFunction(R12, (void*)&LogRegHelper);

  for (int i = 0; i < 8; ++i)
    VLDR(static_cast<ArmGen::ARMReg>(ArmGen::D0 + i), R10, (14 + i*2) * 4);

  for (int i = 0; i <= 8; ++i)
    LDR(static_cast<ArmGen::ARMReg>(i), R10, i * 4);

  for (int i = 10; i <= 12; ++i)
    LDR(static_cast<ArmGen::ARMReg>(i), R10, i * 4);

  LDR(_LR, R10, 13 * 4);
}*/

/*void JitArm::LogNumFromJIT(const char* msg, unsigned int value)
{
  //assert(PPC_REG == reinterpret_cast<u32>(&m_ppc_state));

  // Save LR only
  STR(_LR, PPC_REG, PPCSTATE_OFF_DEBUG_SCRATCH(13));

  // Prepare message
  size_t len = std::strlen(msg);
  char* m = static_cast<char*>(std::malloc(len + 1));
  if (m) {
    std::memcpy(m, msg, len + 1);
  } else {
    m = const_cast<char*>(msg);
  }

  // Pass args to helper
  MOVI2R(R0, static_cast<u32>(reinterpret_cast<uintptr_t>(m)));
  MOVI2R(R1, value);

  // Call logging helper
  QuickCallFunction(R12, reinterpret_cast<void*>(&LogRegHelper));

  // Restore LR only
  LDR(_LR, PPC_REG, PPCSTATE_OFF_DEBUG_SCRATCH(13));
  //assert(PPC_REG == reinterpret_cast<u32>(&m_ppc_state));
}*/

/*void JitArm::LogRegFromJIT(const char* msg, ArmGen::ARMReg reg)
{
  //assert(PPC_REG == reinterpret_cast<u32>(&m_ppc_state));
  // Save LR only
  STR(_LR, PPC_REG, PPCSTATE_OFF_DEBUG_SCRATCH(13));

  // Prepare message
  const char* m = MakeLogMsg(msg, reg);

  // Pass args to helper
  MOVI2R(R0, static_cast<u32>(reinterpret_cast<uintptr_t>(m)));
  if (reg == ArmGen::INVALID_REG) {
    MOVI2R(R1, LOG_SENTINEL);
  } else {
    MOV(R1, reg);
  }

  // Call logging helper
  QuickCallFunction(R12, reinterpret_cast<void*>(&LogRegHelper));

  // Restore LR only
  LDR(_LR, PPC_REG, PPCSTATE_OFF_DEBUG_SCRATCH(13));
  //assert(PPC_REG == reinterpret_cast<u32>(&m_ppc_state));
}
*/

