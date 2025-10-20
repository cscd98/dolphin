// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <map>

#include "Common/ArmEmitter.h"
#include "Common/CommonTypes.h"

#include "Core/ConfigManager.h"
#include "Core/Core.h"
#include "Core/CoreTiming.h"
#include "Core/PatchEngine.h"
#include "Core/System.h"
#include "Core/Config/MainSettings.h"
#include "Core/Host.h"
#include "Core/HW/CPU.h"
#include "Core/HLE/HLE.h"
#include "Core/HW/GPFifo.h"
#include "Core/HW/Memmap.h"
#include "Core/PowerPC/JitInterface.h"
#include "Core/PowerPC/PowerPC.h"
#include "Core/PowerPC/PPCAnalyst.h"
#include "Core/PowerPC/PPCTables.h"
#include "Core/PowerPC/JitArm32/Jit.h"
#include "Core/PowerPC/JitArm32/JitArm_Tables.h"
#include "Core/PowerPC/Interpreter/Interpreter.h"
#include "Common/HostDisassembler.h"
#include "Common/IOFile.h"
#include "Common/Profiler.h"

constexpr size_t CODE_SIZE      = 1024 * 1024 * 32;   // 32 MB near
constexpr size_t FARCODE_SIZE   = 1024 * 1024 * 64;   // 64 MB far
constexpr size_t FARCODE_SIZE_MMU = 1024 * 1024 * 128; // 128 MB if MMU enabled

// C++ side helper that actually prints, called from JIT code!
#include <cstdio>
#include <cstdint>

JitLogEntry g_jit_log_buffer[256];
std::atomic<u32> g_jit_log_write_idx{0};
std::atomic<u32> g_jit_log_read_idx{0};

/*extern "C" void LogRegHelper(const char* msg, uint32_t value)
{
  if (!msg)
  {
    fputs("LogRegHelper: null msg\n", stderr);
    return;
  }

  constexpr uint32_t INVALID_NUM = 0xFFFFFFFF;

  // Print to stdout
  if (value != INVALID_NUM)
    printf("%s 0x%08x\n", msg, value);
  else
    printf("%s\n", msg);
  fflush(stdout);
  //free((void*)msg);
}*/

using namespace ArmGen;

JitArm::JitArm(Core::System& system)
    : JitBase(system),
      m_disassembler(HostDisassembler::Factory(HostDisassembler::Platform::arm))
{
  JIT_LOG("JIT: Creating JIT ARM32");
  JIT_LOG("JitArm this=%p", (void*)this);
}

JitArm::~JitArm() = default;

void JitArm::Init()
{
  JIT_LOG("Initializing JIT ARM32...");

#ifdef FOMIT_FRAME_POINTER
  JIT_LOG("FOMIT_FRAME_POINTER");
#else
  JIT_LOG("NO FOMIT_FRAME_POINTER");
#endif

  if(Config::Get(Config::MAIN_FASTMEM_ARENA))
    JIT_LOG("FASTMEM_ARENA config option is enabled");

  //printf("PPCSTATE_OFF(jit_return_address) = %d\n", PPCSTATE_OFF(jit_return_address));
  printf("PPCSTATE_OFF(gpr[0]) = %d\n", PPCSTATE_OFF_GPR(0));
  //printf("PPCSTATE_OFF(jit_log_msg)) = %d\n", PPCSTATE_OFF(jit_log_msg));
  //printf("PPCSTATE_OFF(jit_log_value)) = %d\n", PPCSTATE_OFF(jit_log_value));
  //printf("PPCSTATE_OFF(jit_log_ready)) = %d\n", PPCSTATE_OFF(jit_log_ready));
  fflush(stdout);

  InitFastmemArena();

  RefreshConfig();

  const size_t routines_size = CODE_SIZE;
  const size_t farcode_size = jo.memcheck ? FARCODE_SIZE_MMU : FARCODE_SIZE;
  AllocCodeSpace(CODE_SIZE + routines_size + farcode_size);
  AddChildCodeSpace(&m_far_code, farcode_size);

  jo.optimizeGatherPipe = true;
  SetBlockLinkingEnabled(true);
  SetOptimizationEnabled(false); // TEMP DISABLE OPTIMIZATIONS
  gpr.Init(this);
  fpr.Init(this);
  blocks.Init();

  if (IsProfilingEnabled())
    JIT_LOG("PROFILING IS ENABLED");

  if (IsDebuggingEnabled())
    JIT_LOG("DEBUGGING IS ENABLED");

  JIT_LOG("m_enable_blr_optimization = %d", m_enable_blr_optimization);

  JIT_LOG("JIT ARM32: Entry points = %p",
    (void*)blocks.GetEntryPoints());
  JIT_LOG("JIT ARM32: Fast block map fallback = %p",
    (void*)blocks.GetFastBlockMapFallback());

  code_block.m_stats = &js.st;
  code_block.m_gpa = &js.gpa;
  code_block.m_fpa = &js.fpa;

  //InitBLROptimization();
  m_enable_blr_optimization = false;

  GenerateAsmAndResetFreeMemoryRanges();

  JIT_LOG("JIT ARM32 initialized successfully.");

  // Disable all loadstores
  // Ever since the MMU has been optimized for x86, loadstores on ARMv7 have been knackered
  // XXX: Investigate exactly why these are broken
  //Config::SetCurrent(Config::MAIN_DEBUG_JIT_LOAD_STORE_OFF, true);
  //Config::SetCurrent(Config::MAIN_DEBUG_JIT_LOAD_STORE_FLOATING_OFF, true);
  //Config::SetCurrent(Config::MAIN_DEBUG_JIT_LOAD_STORE_PAIRED_OFF, true);

  //Config::SetCurrent(Config::MAIN_DEBUG_JIT_LOAD_STORE_LXZ_OFF, true);
  //Config::SetCurrent(Config::MAIN_DEBUG_JIT_LOAD_STORE_LWZ_OFF, true);
  //Config::SetCurrent(Config::MAIN_DEBUG_JIT_LOAD_STORE_LBZX_OFF, true);

  g_jit_log_buffer[0].msg = "TEST FROM C++";
  g_jit_log_buffer[0].value = 0x12345678;
  g_jit_log_write_idx = 1;

  printf("JIT Init: write_idx=%u\n", g_jit_log_write_idx.load());

  LogNumFromJIT("HELLO FROM JIT INIT", 0x12345678);
}

void JitArm::SetBlockLinkingEnabled(bool enabled)
{
  jo.enableBlocklink = enabled && !SConfig::GetInstance().bJITNoBlockLinking;
}

void JitArm::SetOptimizationEnabled(bool enabled)
{
  if (enabled)
  {
    analyzer.SetOption(PPCAnalyst::PPCAnalyzer::OPTION_CONDITIONAL_CONTINUE);
    analyzer.SetOption(PPCAnalyst::PPCAnalyzer::OPTION_CARRY_MERGE);
    analyzer.SetOption(PPCAnalyst::PPCAnalyzer::OPTION_BRANCH_FOLLOW);
  }
  else
  {
    analyzer.ClearOption(PPCAnalyst::PPCAnalyzer::OPTION_CONDITIONAL_CONTINUE);
    analyzer.ClearOption(PPCAnalyst::PPCAnalyzer::OPTION_CARRY_MERGE);
    analyzer.ClearOption(PPCAnalyst::PPCAnalyzer::OPTION_BRANCH_FOLLOW);
  }
}

void JitArm::ClearCache()
{
  JIT_LOG("Clearing JIT ARM32 cache...");

  m_fault_to_handler.clear();
  blocks.Clear();
  blocks.ClearRangesToFree();
  const Common::ScopedJITPageWriteAndNoExecute enable_jit_page_writes(GetRegionPtr());
  m_far_code.ClearCodeSpace();
  ClearCodeSpace();
  RefreshConfig();

  GenerateAsmAndResetFreeMemoryRanges();
}

void JitArm::GenerateAsmAndResetFreeMemoryRanges()
{
  JIT_LOG("Freeing JIT ARM32 code ranges...");

  // Reclaim previously freed slices
  for (const auto& [from, to] : blocks.GetRangesToFreeNear())
    m_free_ranges_near.insert(from, to);
  for (const auto& [from, to] : blocks.GetRangesToFreeFar())
    m_free_ranges_far.insert(from, to);

  // Reset free lists to entire regions
  m_free_ranges_near.clear();
  m_free_ranges_near.insert(region, region + region_size);

  m_free_ranges_far.clear();
  m_free_ranges_far.insert(m_far_code.GetRegionPtr(), m_far_code.GetWritableCodeEnd());

  // Bind emitter to a free near/far slice
  if (!SetEmitterStateToFreeCodeRegion())
  {
    JIT_LOG("JIT ARM32: no free code region available for dispatcher.");
    PanicAlertFmtT("JIT ARM32: no free code region available for dispatcher.");
    return;
  }

  // Record slice start
  u8* dispatcher_begin = GetWritableCodePtr();

  // Emit dispatcher + common routines; this sets enter_code
  GenerateAsm();

  // Generates the slow/fastmem stubs
  InitBackpatch();

  // Record slice end and mark it used
  u8* dispatcher_end = GetWritableCodePtr();
  if (dispatcher_begin && dispatcher_end && dispatcher_begin < dispatcher_end)
    m_free_ranges_near.erase(dispatcher_begin, dispatcher_end);

  Host_JitCacheInvalidation();
}

void JitArm::FreeRanges()
{
  blocks.ClearRangesToFree();
}

void JitArm::Shutdown()
{
  FreeCodeSpace();
  blocks.Shutdown();
}

// Execute one instruction via the interpreter.
// Do NOT write directly to pc; commit only npc when ending the block.
void JitArm::WriteCallInterpreter(UGeckoInstruction inst)
{
  gpr.Flush();
  fpr.Flush();

  Interpreter::Instruction instr = Interpreter::GetInterpreterOp(inst);

  // If this instruction can end the block, precommit resume address to npc.
  if (js.op->canEndBlock)
  {
    JIT_ERR("js.op->canEndBlock is true");
    auto tmp = gpr.GetScopedReg();

    // Resume at next word (PPC fallthrough)
    MOVI2R(tmp, js.compilerPC);
    STR(tmp, PPC_REG, PPCSTATE_OFF(pc));
    ADD(tmp, tmp, 4);
    STR(tmp, PPC_REG, PPCSTATE_OFF(npc));
  }

  //ABI_PushCalleeGPRsAndAdjustStack(true);

  MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetInterpreter()));
  MOVI2R(R1, inst.hex);
  MOVI2R(R2, 0);  // Clear R2
  MOVI2R(R3, 0);  // Clear R3

  QuickCallFunction(R12, reinterpret_cast<void*>(instr));

  //ABI_PopCalleeGPRsAndAdjustStack(true);

  // TODO
  JIT_LOG_REG("After ABI_PopCalleeGPRsAndAdjustStack: PPC_REG - before reload", PPC_REG);
  MOVI2R(PPC_REG, static_cast<u32>(reinterpret_cast<uintptr_t>(&m_ppc_state)));
  JIT_LOG_REG("After ABI_PopCalleeGPRsAndAdjustStack: PPC_REG - after reload", PPC_REG);
  // Invalidate register cache - don't trust any cached values!
  gpr.Flush();
  fpr.Flush();
}

// Interpreter fallback wrapper with coherent exit semantics.
// No direct STR to pc; use WriteExceptionExit(resume) to advance.
void JitArm::FallBackToInterpreter(UGeckoInstruction inst)
{
  // Host-side logging, executes immediately
  JIT_ERR("[FallBackToInterpreter] PC=0x%08x opname=%s",
          js.compilerPC,
          js.op->opinfo ? js.op->opinfo->opname : "<null>");

  //JIT_LOG_NUM("JIT ARM32: Falling back to interpreter for instruction at PC", js.compilerPC);

  JIT_ERR("[FallBackToInterpreter] About to call WriteCallInterpreter");
  WriteCallInterpreter(inst);

  JIT_LOG("[FallBackToInterpreter] returned from WriteCallInterpreter: regsOut=0x%08x GetFregsOut()=0x%08x",
    js.op->regsOut.m_val, js.op->GetFregsOut().m_val);

  // Reset registers the interpreter may have written
  gpr.ResetRegisters(js.op->regsOut);

  JIT_ERR("[FallBackToInterpreter] reset fpr registers");

  fpr.ResetRegisters(js.op->GetFregsOut());

  JIT_ERR("[FallBackToInterpreter] reset gpr registers");
  gpr.ResetCRRegisters(js.op->crOut);

  if (js.op->canEndBlock)
  {
    JIT_ERR("[FallBackToInterpreter] canEndBlock");
    const u32 resume = js.compilerPC + 4;

    // If this was the last instruction in the block, exit with explicit resume.
    if (js.isLastInstruction)
    {
      JIT_ERR("[FallBackToInterpreter] Writing exception exit (last instruction) resume=0x%08x", resume);

      WriteExceptionExit(resume);
      return;
    }

    // Mid-block: only exit if npc differs from expected resume.
    auto WA = gpr.GetScopedReg();
    LDR(WA, PPC_REG, PPCSTATE_OFF(npc));

    auto WB = gpr.GetScopedReg();
    MOVI2R(WB, resume);
    CMP(WA, R(WB));

    FixupBranch skip_exit = B_CC(CC_EQ);

    JIT_ERR("[FallBackToInterpreter] Writing exception exit (not last instruction) resume=0x%08x", resume);
    WriteExceptionExit(resume);
    SetJumpTarget(skip_exit);
  }
  else if (ShouldHandleFPExceptionForInstruction(js.op))
  {
    JIT_ERR("ShouldHandleFPExceptionForInstruction is true");
    WriteConditionalExceptionExit(EXCEPTION_PROGRAM);
  }

  if (jo.memcheck && (js.op->opinfo->flags & FL_LOADSTORE))
  {
    JIT_ERR("FL_LOADSTORE is true");
    WriteConditionalExceptionExit(EXCEPTION_DSI);
  }

  JIT_ERR("[FallBackToInterpreter] Finished");
}

void JitArm::HLEFunction(UGeckoInstruction _inst)
{
  JIT_LOG_MSG("HLE Function");

  gpr.Flush();
  fpr.Flush();
  MOVI2R(R0, js.compilerPC);
  MOVI2R(R1, _inst.hex);
  MOVI2R(R2, reinterpret_cast<u32>(&m_system));
  QuickCallFunction(R12, (void*)&HLE::ExecuteFromJIT);
  ARMReg rA = gpr.GetScopedReg(); // unlocked in WriteExitDestInR
  LDR(rA, PPC_REG, PPCSTATE_OFF(npc));
  WriteExitDestInR(rA);
}

void JitArm::DoNothing(UGeckoInstruction _inst)
{
  // Yup, just don't do anything.
}

static const bool ImHereDebug = false;
static const bool ImHereLog = false;
static std::map<u32, int> been_here;

static void ImHere(JitArm& jit)
{
  JIT_LOG("ImHere called");
  auto& ppc_state = jit.m_ppc_state;
  static File::IOFile f;
  if (ImHereLog)
  {
    if (!f)
    {
      f.Open("log32.txt", "w");
    }
    fprintf(f.GetHandle(), "%08x\n", ppc_state.pc);
  }

  if (been_here.find(ppc_state.pc) != been_here.end())
  {
    been_here.find(ppc_state.pc)->second++;
    if ((been_here.find(ppc_state.pc)->second) & 1023)
      return;
  }

  DEBUG_LOG_FMT(DYNA_REC, "I'm here - PC = {:08x} , LR = {:08x}", ppc_state.pc, LR(ppc_state));
  been_here[ppc_state.pc] = 1;
}

void JitArm::Cleanup()
{
  JIT_LOG_NUM("JIT ARM32: Cleaning up before exit at PC", js.compilerPC);

  if (jo.optimizeGatherPipe && js.fifoBytesSinceCheck > 0)
  {
    JIT_LOG_MSG("JIT ARM32: Cleaning up gather pipe - jo.optimizeGatherPipe && js.fifoBytesSinceCheck > 0");
    PUSH(4, R0, R1, R2, R3);
    MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetGPFifo()));
    QuickCallFunction(R12, (void*)&GPFifo::FastCheckGatherPipe);
    POP(4, R0, R1, R2, R3);
  }
}

void JitArm::DoDownCount()
{
  JIT_LOG_NUM("JIT ARM32: [downcount] update at PC", js.compilerPC);

  ARMReg rA = gpr.GetScopedReg();

  LDR(rA, PPC_REG, PPCSTATE_OFF(downcount));
  JIT_LOG_REG("JIT ARM32: [downcount] current downcount value", rA);

  if (js.downcountAmount < 255)
  {
    SUBS(rA, rA, js.downcountAmount);
  }
  else
  {
    auto rB_scoped = gpr.GetScopedReg();
    ARMReg rB = rB_scoped;

    JIT_LOG_REG("JIT ARM32: [downcount] (js.downcountAmount > 255)", rB);

    MOVI2R(rB, js.downcountAmount);
    SUBS(rA, rA, rB);
  }

  JIT_LOG_REG("JIT ARM32: [downcount] new value", rA);
  STR(rA, PPC_REG, PPCSTATE_OFF(downcount));
}


void JitArm::ResetStack()
{
  JIT_LOG_NUM("ResetStack() - m_enable_blr_optimization", m_enable_blr_optimization);

  if (!m_enable_blr_optimization)
  {
    JIT_LOG_MSG("ResetStack() - m_enable_blr_optimization - doing nothing");
    return;
  }

  auto tmp = gpr.GetScopedReg();
  // Load baseline SP
  LDR(tmp, PPC_REG, PPCSTATE_OFF(stored_stack_pointer));

  // Runtime guards - don't restore if 0
  CMP(tmp, 0);
  FixupBranch skip = B_CC(CC_EQ);
  TST(tmp, 0x7); // 8-byte alignment expected in this function
  FixupBranch skip2 = B_CC(CC_NEQ);

  // Restore SP
  MOV(_SP, R(tmp));

  SetJumpTarget(skip);
  JIT_LOG_NUM("ResetStack() - guard failed: tmp==0", 0);

  SetJumpTarget(skip2);
  JIT_LOG_NUM("ResetStack() - guard failed: misaligned", 0);
}

void JitArm::IntializeSpeculativeConstants()
{
  //JIT_LOG_NUM("JIT ARM32: Initializing speculative constants at PC", js.compilerPC);
  JIT_LOG("JIT ARM32: Initializing speculative constants at PC %08x", js.blockStart);  // Compile-time only!

  const u8* fail = nullptr;

  for (auto i : code_block.m_gpr_inputs)
  {
    //JIT_LOG_NUM("JIT ARM32: Initializing speculative constant for GPR", i);
    const u32 compile_time_value = m_ppc_state.gpr[i];

    JIT_LOG("JIT ARM32: Initializing speculative constant for GPR %d, PPCState value = 0x%08x", i, compile_time_value);

    if (m_mmu.IsOptimizableGatherPipeWrite(compile_time_value) ||
        m_mmu.IsOptimizableGatherPipeWrite(compile_time_value - 0x8000) ||
        compile_time_value == 0xCC000000)
    {
      if (!fail)
      {
        //JIT_LOG_NUM("JIT ARM32: Setting up speculative constant fail path");
        JIT_LOG("JIT ARM32: Setting up speculative constant fail path");

        // Far failure code
        SwitchToFarCode();
        fail = GetCodePtr();

        MOVI2R(DISPATCHER_PC, js.blockStart);
        STR(DISPATCHER_PC, PPC_REG, PPCSTATE_OFF(pc));

        //PUSH(6, R0, R1, R2, R3, R12, _LR);

        MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetJitInterface()));
        MOVI2R(R1, static_cast<u32>(JitInterface::ExceptionType::SpeculativeConstants));
        auto cal = gpr.GetScopedReg();
        MOVI2R(cal, reinterpret_cast<u32>(&JitInterface::CompileExceptionCheckFromJIT));
        BLX(cal);

        //POP(6, R0, R1, R2, R3, R12, _LR);

        B(dispatcher_no_check);
        SwitchToNearCode();
      }

      // Near runtime check
      {
        auto tmp = gpr.GetScopedReg();
        auto val = gpr.R(i);
        MOVI2R(tmp, compile_time_value);
        CMP(val, R(tmp));

        JIT_LOG("JIT ARM32: Speculative constant check - GPR %d", i);
        //JIT_LOG_NUM("JIT ARM32: Speculative constant check - GPR", i);
      }

      FixupBranch no_fail = B_CC(CC_EQ);
      B(fail);
      SetJumpTarget(no_fail);

      JIT_LOG("JIT ARM32: Speculative constant failed - GPR %d", i);
      //JIT_LOG_NUM("JIT ARM32: Speculative constant failed - GPR", i);
      gpr.SetImmediate(i, compile_time_value, true);
    }
  }

  JIT_LOG("JIT ARM32: Finished initializing speculative constants at PC %08x", js.compilerPC);
  //JIT_LOG_NUM("JIT ARM32: Finished initializing speculative constants at PC", js.compilerPC);
}

void JitArm::MSRUpdated(ARMReg msr)
{
  auto WA = gpr.GetScopedReg();
  auto WB = gpr.GetScopedReg();

  auto& memory = m_system.GetMemory();

  // Load logical base into MEM_REG
  MOVI2R(MEM_REG,
         static_cast<u32>(reinterpret_cast<uintptr_t>(
           jo.fastmem ? memory.GetLogicalBase()
                      : memory.GetLogicalPageMappingsBase())));

  // Load physical base into WA
  MOVI2R(WA,
         static_cast<u32>(reinterpret_cast<uintptr_t>(
           jo.fastmem ? memory.GetPhysicalBase()
                      : memory.GetPhysicalPageMappingsBase())));

  // Test MSR.DR (bit 27)
  TST(msr, Operand2(1, 27));
  FixupBranch dr_clear = B_CC(CC_EQ);  // DR == 0 → keep logical base

  // DR != 0 → use physical base
  MOV(MEM_REG, WA);
  B_CC(CC_AL);

  SetJumpTarget(dr_clear);
  // DR == 0 → MEM_REG already holds logical base

  STR(MEM_REG, PPC_REG, PPCSTATE_OFF(mem_ptr));

  // Update feature_flags
  static_assert(UReg_MSR{}.DR.StartBit() == 4);
  static_assert(UReg_MSR{}.IR.StartBit() == 5);
  static_assert(FEATURE_FLAG_MSR_DR == 1 << 0);
  static_assert(FEATURE_FLAG_MSR_IR == 1 << 1);

  const u32 other_feature_flags = m_ppc_state.feature_flags & ~0x3;

  UBFX(WA, msr, 4, 2);
  if (other_feature_flags != 0)
  {
    MOVI2R(WB, other_feature_flags);
    ORR(WA, WA, R(WB));
  }

  STR(WA, PPC_REG, PPCSTATE_OFF(feature_flags));
}

void JitArm::SafeB(const void* fnptr, bool optimize)
{
  if (IsBranchInRange(fnptr, GetCodePtr()))
  {
    B(fnptr);
  }
  else
  {
    auto reg = gpr.GetScopedReg();
    MOVI2R(reg, reinterpret_cast<u32>(fnptr), optimize);
    BX(reg);
  }
}

void JitArm::SafeSetJumpTarget(const ArmGen::FixupBranch& branch)
{
  const void* target = GetCodePtr();

  if (IsBranchInRange(target, branch.ptr))
  {
    SetJumpTarget(branch);
  }
  else
  {
    auto reg = gpr.GetScopedReg();
    MOVI2R(reg, static_cast<u32>(reinterpret_cast<uintptr_t>(target)));

    switch (branch.type)
    {
      case ArmGen::FixupBranch::Type::BL:
        BLX(reg);
        break;
      default:
        BX(reg);
        break;
    }
  }
}

extern "C" {
  // Stable addresses in .rodata
  static const char kMsgNpcAfterStore[] = "npc after store:";
  static const char kMsgPcAfterStore[]  = "pc after store:";
}

// Writes destination from Reg into DISPATCHER_PC, validates, and dispatches exactly once.
// No direct STR to pc or npc. BKPT paths do NOT fall through.
void JitArm::WriteExitDestInR(ArmGen::ARMReg Reg)
{
  JIT_LOG("JIT ARM32: WriteExitDestInR: register %d, PC: %08x", Reg, js.compilerPC);

  // Validate Reg != 0
  CMP(Reg, 0);
  FixupBranch nonzero = B_CC(CC_NEQ);
  // Zero path: trap and bail to dispatcher immediately
  BKPT(0x4321);
  {
    auto t = gpr.GetScopedReg();
    MOVI2R(t, reinterpret_cast<u32>(dispatcher));
    B(t);
  }
  SetJumpTarget(nonzero);

  // Optional range check: top nibble must be 0x8 (tune if needed)
  auto tmp = gpr.GetScopedReg();
  AND(tmp, Reg, Operand2(0xF0000000));
  MOVI2R(R12, 0x80000000);
  CMP(tmp, R12);
  FixupBranch in_range = B_CC(CC_EQ);
  // Out-of-range path: trap and bail
  BKPT(0x4322);
  {
    auto t = gpr.GetScopedReg();
    MOVI2R(t, reinterpret_cast<u32>(dispatcher));
    B(t);
  }
  SetJumpTarget(in_range);

  // Arm64 parity: do NOT touch npc/pc here. Seed dispatcher with the destination.
  MOV(DISPATCHER_PC, Reg);
  JIT_LOG_REG("WriteExitDestInR: Exit sets DISPATCHER_PC:", DISPATCHER_PC);

  // Diagnostics (optional)
  JIT_LOG_REG("Exit target (DISPATCHER_PC):", DISPATCHER_PC);

  // Single cleanup + downcount + dispatch
  Cleanup();
  DoDownCount();
  if (IsProfilingEnabled())
    EndTimeProfile(js.curBlock);

  MOVI2R(tmp, reinterpret_cast<u32>(dispatcher));
  B(tmp);
}

void JitArm::WriteRfiExitDestInR(ARMReg Reg)
{
  JIT_LOG_REG("WriteRfiExitDestInR: Reg is", Reg);

  // Commit SRR0 into pc/npc and set DISPATCHER_PC
  STR(Reg, PPC_REG, PPCSTATE_OFF(pc));
  STR(Reg, PPC_REG, PPCSTATE_OFF(npc));
  LDR(DISPATCHER_PC, PPC_REG, PPCSTATE_OFF(npc));

  Cleanup();

  if (IsProfilingEnabled())
    EndTimeProfile(js.curBlock);

  // Call exceptions routine
  MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetPowerPC()));
  QuickCallFunction(R12, (void*)&PowerPC::CheckExceptionsFromJIT);

  // Update membase after exception handling
  EmitUpdateMembase();

  gpr.Unlock(Reg);

  // Log final pc/npc
  {
    auto W = gpr.GetScopedReg();
    LDR(W, PPC_REG, PPCSTATE_OFF(pc));
    JIT_LOG_REG("WriteRfiExitDestInR: Updated PPCSTATE_OFF(pc)", W);
    LDR(W, PPC_REG, PPCSTATE_OFF(npc));
    JIT_LOG_REG("WriteRfiExitDestInR: Updated PPCSTATE_OFF(npc)", W);
  }

  DoDownCount();

  // Branch back to dispatcher
  auto disp = gpr.GetScopedReg();
  MOVI2R(disp, reinterpret_cast<u32>(dispatcher));
  B(disp);
}


void JitArm::WriteConditionalExceptionExit(int exception, u32 increment_sp_on_exit)
{
  auto tmp = gpr.GetScopedReg();
  WriteConditionalExceptionExit(exception, tmp, ArmGen::INVALID_REG, increment_sp_on_exit);
}

void JitArm::WriteConditionalExceptionExit(int exception,
                                           ARMReg temp_gpr,
                                           ARMReg /* temp_fpr */,
                                           u32 /* increment_sp_on_exit */)
{
  if (temp_gpr == ArmGen::INVALID_REG)
    temp_gpr = gpr.GetScopedReg();

  // Load Exceptions word and AND with mask
  LDR(temp_gpr, PPC_REG, PPCSTATE_OFF(Exceptions));

  auto tmp_mask = gpr.GetScopedReg();
  MOVI2R(tmp_mask, static_cast<u32>(exception));
  AND(temp_gpr, temp_gpr, Operand2(tmp_mask));
  FixupBranch no_exception = B_CC(CC_EQ);  // skip if zero

  // Branch to far code for slow path
  FixupBranch handle_exception = B();
  SwitchToFarCode();
  SetJumpTarget(handle_exception);

  // Flush registers but maintain state
  gpr.Flush(FlushMode::MaintainState);
  fpr.Flush(FlushMode::MaintainState);

  // Inline exception exit
  WriteExceptionExit(js.compilerPC, false, true);

  // Return to near path
  SwitchToNearCode();

  SetJumpTarget(no_exception);
}

void JitArm::FakeLKExit(u32 exit_address_after_return, ArmGen::ARMReg exit_address_after_return_reg)
{
  JIT_LOG_NUM("FakeLKExit: calling exit_address_after_return", exit_address_after_return);
  JIT_LOG_NUM("WriteExit: exit_address_after_return_reg",
    static_cast<u32>(exit_address_after_return_reg));

  if (!m_enable_blr_optimization)
  {
    JIT_LOG_NUM("FakeLKExit: m_enable_blr_optimization", m_enable_blr_optimization);
    return;
  }

  auto lr_tmp = gpr.GetReg();
  {
    if (exit_address_after_return_reg == ArmGen::INVALID_REG)
    {
      MOVI2R(lr_tmp, exit_address_after_return);
    }
    else
    {
      MOV(lr_tmp, exit_address_after_return_reg);
    }
    STR(lr_tmp, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
  }

  // Build the packed key: (feature_flags << 30) | (exit_address_after_return >> 2)
  const u32 feature_flags = m_ppc_state.feature_flags;

  // Use raw regs for stack ops (avoid copying ScopedARMReg).
  ARMReg retaddr = gpr.GetReg();
  ARMReg key     = gpr.GetReg();

  gpr.Unlock(lr_tmp);

  // Compute host address after return: current code ptr + (link stub + 2 words)
  // ARM64 used: BLOCK_LINK_SIZE + sizeof(u32)*2; keep the same layout for parity.
  const u32 adr_offset = static_cast<u32>(JitArmBlockCache::BLOCK_LINK_SIZE + sizeof(u32) * 2);
  const u8* host_address_after_return = GetCodePtr() + adr_offset;

  // retaddr := host_address_after_return
  MOVI2R(retaddr, reinterpret_cast<u32>(host_address_after_return));

  // key := packed key
  if (exit_address_after_return_reg == ArmGen::INVALID_REG)
  {
    MOVI2R(key, (feature_flags << 30) | (exit_address_after_return >> 2));
  }
  else if (feature_flags == 0)
  {
    MOV(key, exit_address_after_return_reg);
    LSR(key, key, 2);
  }
  else
  {
    auto tmp = gpr.GetScopedReg();
    LSR(tmp, exit_address_after_return_reg, 2);
    ORR(key, tmp, Operand2(feature_flags << 30));
  }

  // FIX: Push {retaddr, key} onto the stack (pair store). Order matches ARM64 STP.
  // The issue was that we were pushing these values but the emulated code was expecting
  // the PPC return address to be in LR, not on the host stack.
  // Instead, we should write the PPC return address to the PPC LR register.

  // Store exit_address_after_return to PPC LR
  /*auto lr_tmp = gpr.GetScopedReg();
  if (exit_address_after_return_reg == ArmGen::INVALID_REG)
  {
    MOVI2R(lr_tmp, exit_address_after_return);
  }
  else
  {
    MOV(lr_tmp, exit_address_after_return_reg);
  }
  STR(lr_tmp, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));*/

  // Now push host return info for BLR optimization
  PUSH(2, retaddr, key);

  // Call dispatcher (materialize address, use BLX)
  {
    auto call = gpr.GetScopedReg();
    MOVI2R(call, reinterpret_cast<u32>(dispatcher));
    BLX(call);
  }

  // Write the regular exit node after the return.
  JitBlock* b = js.curBlock;
  JitBlock::LinkData linkData{};
  linkData.exitAddress = exit_address_after_return;
  linkData.exitPtrs = GetWritableCodePtr();
  linkData.linkStatus = false;
  linkData.call = false;

  const bool switch_to_far_code = !IsInFarCode();
  if (switch_to_far_code)
  {
    SwitchToFarCode();
    linkData.exitFarcode = GetCodePtr();
    SwitchToNearCode();
  }
  else
  {
    linkData.exitFarcode = GetCodePtr() + JitArmBlockCache::BLOCK_LINK_SIZE;
  }
  b->linkData.push_back(linkData);

  blocks.WriteLinkBlock(*this, linkData);

  if (switch_to_far_code)
    SwitchToFarCode();

  // Seed dispatcher and go to timing path.
  MOVI2R(DISPATCHER_PC, exit_address_after_return);
  auto t = gpr.GetScopedReg();
  MOVI2R(t, reinterpret_cast<u32>(GetAsmRoutines()->do_timing));
  B(t);

  if (switch_to_far_code)
    SwitchToNearCode();

  // Release the raw regs we reserved for PUSH
  gpr.Unlock(retaddr);
  gpr.Unlock(key);
}

void JitArm::WriteExceptionExit(u32 destination /*=0*/,
                                bool only_external /*=false*/,
                                bool always_exception /*=false*/)
{
  MOVI2R(DISPATCHER_PC, destination);
  WriteExceptionExit(DISPATCHER_PC, only_external, always_exception);
}

void JitArm::WriteExceptionExit(ARMReg dest, bool only_external, bool always_exception)
{
  JIT_LOG_REG("WriteExceptionExit(reg): dest", dest);
  JIT_LOG_NUM("WriteExceptionExit(reg): only_external", only_external ? 1u : 0u);
  JIT_LOG_NUM("WriteExceptionExit(reg): always_exception", always_exception ? 1u : 0u);

  JIT_LOG_REG("WriteExceptionExit (reg): DISPATCHER_PC before MOV", DISPATCHER_PC);
  if (dest != DISPATCHER_PC)
  {
    JIT_LOG_MSG("WriteExceptionExit (reg): Updating dispatcher PC as does not match");
    MOV(DISPATCHER_PC, dest);
  }
  JIT_LOG_REG("WriteExceptionExit (reg): DISPATCHER_PC after MOV", DISPATCHER_PC);

  Cleanup();

  FixupBranch no_exceptions;
  if (!always_exception)
  {
    auto ex = gpr.GetScopedReg();
    LDR(ex, PPC_REG, PPCSTATE_OFF(Exceptions));
    CMP(ex, 0);
    JIT_LOG_REG("WriteExceptionExit: Exceptions before helper", ex);
    no_exceptions = B_CC(CC_EQ);
  }

  static_assert(PPCSTATE_OFF(pc) <= 252);
  static_assert(PPCSTATE_OFF(pc) + 4 == PPCSTATE_OFF(npc));

  STR(DISPATCHER_PC, PPC_REG, PPCSTATE_OFF(pc));
  STR(DISPATCHER_PC, PPC_REG, PPCSTATE_OFF(npc));
  JIT_LOG_MSG("WriteExceptionExit(reg): stored DISPATCHER_PC into ppcstate");

  const auto f = only_external
      ? &PowerPC::CheckExternalExceptionsFromJIT
      : &PowerPC::CheckExceptionsFromJIT;

  JIT_LOG_MSG("WriteExceptionExit(reg): calling exception function");
  MOVI2R(R0, static_cast<u32>(reinterpret_cast<uintptr_t>(&m_system.GetPowerPC())));
  MOVI2R(R12, static_cast<u32>(reinterpret_cast<uintptr_t>(f)));
  BLX(R12);

  EmitUpdateMembase();

  // reload dispatcher pc from ppc_state.npc
  LDR(DISPATCHER_PC, PPC_REG, PPCSTATE_OFF(npc));
  JIT_LOG_REG("WriteExceptionExit: Reloaded DISPATCHER_PC:", DISPATCHER_PC);

  if (!always_exception)
    SetJumpTarget(no_exceptions);

  JIT_LOG_MSG("WriteExceptionExit(reg): took no_exceptions skip");

  if (IsProfilingEnabled())
  {
    MOVI2R(R0, static_cast<u32>(reinterpret_cast<uintptr_t>(js.curBlock->profile_data.get())));
    MOVI2R(R1, static_cast<u32>(js.downcountAmount));
    MOVI2R(R12, static_cast<u32>(reinterpret_cast<uintptr_t>(&JitBlock::ProfileData::EndProfiling)));
    BLX(R12);
  }

  DoDownCount();
  SafeB(dispatcher);
}

void JitArm::WriteExit(u32 destination, bool LK, u32 exit_address_after_return,
                       ArmGen::ARMReg exit_address_after_return_reg)
{
  JIT_LOG("[WriteExit] dest=0x%08x LK=%u after_return=0x%08x exit_address_after_return=%d js.curBlock=%p js.curBlock->effectiveAddress=0x%08x",
          destination, LK ? 1 : 0, exit_address_after_return, exit_address_after_return_reg,
          js.curBlock, js.curBlock->effectiveAddress);

  JIT_LOG_NUM("WriteExit: destination", static_cast<u32>(destination));
  JIT_LOG_NUM("WriteExit: LK", LK ? 1u : 0u);
  JIT_LOG_NUM("WriteExit: exit_address_after_return", static_cast<u32>(exit_address_after_return));
  JIT_LOG_REG("WriteExit: exit_address_after_return_reg",
                exit_address_after_return_reg);
  {
    auto tmp = gpr.GetScopedReg();
    LDR(tmp, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
    JIT_LOG_REG("WriteExit: LR:", tmp);
  }

  // fix LR if m_enable_blr_optimization is false
  if (LK && !m_enable_blr_optimization)
  {
    if (exit_address_after_return_reg != ArmGen::INVALID_REG)
    {
      // LR value is in a register
      STR(exit_address_after_return_reg, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
    }
    else if(exit_address_after_return != 0)
    {
      auto tmp = gpr.GetScopedReg();
      // LR value is immediate
      MOVI2R(tmp, exit_address_after_return);
      STR(tmp, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
    }
  }

  // Common cleanup + optional profiling + downcount
  Cleanup();
  if (IsProfilingEnabled())
  {
    JIT_LOG("[WriteExit] Profiling enabled, downcount=%u", js.downcountAmount);

    auto cal = gpr.GetScopedReg();
    MOVI2R(R0, reinterpret_cast<u32>(js.curBlock->profile_data.get()));
    MOVI2R(R1, js.downcountAmount);
    MOVI2R(cal, reinterpret_cast<u32>(&JitBlock::ProfileData::EndProfiling));
    BLX(cal);
  }
  DoDownCount();

  LK &= m_enable_blr_optimization;
  JIT_LOG("[WriteExit] LK after optimization=%u", LK);

  const u8* host_address_after_return = nullptr;

  if (LK)
  {
    JIT_LOG("[WriteExit] Handling LK branch, exit_address_after_return=0x%08x",
           exit_address_after_return);

    auto lr_tmp = gpr.GetReg();
    {
      if (exit_address_after_return_reg == ArmGen::INVALID_REG)
      {
        MOVI2R(lr_tmp, exit_address_after_return);
      }
      else
      {
        MOV(lr_tmp, exit_address_after_return_reg);
      }
      STR(lr_tmp, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
    }

    const u32 feature_flags = m_ppc_state.feature_flags;
    const u32 adr_offset = static_cast<u32>(JitArmBlockCache::BLOCK_LINK_SIZE + sizeof(u32) * 2);
    host_address_after_return = GetCodePtr() + adr_offset;

    JIT_LOG("[WriteExit] feature_flags=0x%08x host_after_return=%p",
           feature_flags, host_address_after_return);

    ARMReg retaddr = gpr.GetReg();
    ARMReg key     = gpr.GetReg();

    gpr.Unlock(lr_tmp);

    MOVI2R(retaddr, reinterpret_cast<u32>(host_address_after_return));

    if (exit_address_after_return_reg == ArmGen::INVALID_REG)
    {
      MOVI2R(key, (feature_flags << 30) | (exit_address_after_return >> 2));
    }
    else if (feature_flags == 0)
    {
      MOV(key, exit_address_after_return_reg);
      LSR(key, key, 2);
    }
    else
    {
      auto tmp = gpr.GetScopedReg();
      LSR(tmp, exit_address_after_return_reg, 2);
      ORR(key, tmp, Operand2(feature_flags << 30));
    }

    PUSH(2, retaddr, key);

    auto call = gpr.GetScopedReg();
    MOVI2R(call, reinterpret_cast<u32>(dispatcher));
    BLX(call);

    gpr.Unlock(retaddr);
    gpr.Unlock(key);
  }

  // Reserve farcode addresses for timing stubs
  constexpr size_t primary_farcode_size = 3 * sizeof(u32);
  const bool switch_to_far_code = !IsInFarCode();
  const u8* primary_farcode_addr;
  if (switch_to_far_code)
  {
    SwitchToFarCode();
    primary_farcode_addr = GetCodePtr();
    SwitchToNearCode();
    JIT_LOG("[WriteExit] Switched to far code, primary_farcode_addr=%p", primary_farcode_addr);
  }
  else
  {
    primary_farcode_addr = GetCodePtr() + JitArmBlockCache::BLOCK_LINK_SIZE +
                           (LK ? JitArmBlockCache::BLOCK_LINK_SIZE : 0);
    JIT_LOG("[WriteExit] Using near code, primary_farcode_addr=%p", primary_farcode_addr);
  }
  const u8* return_farcode_addr = primary_farcode_addr + primary_farcode_size;

  JIT_LOG_MSG("WriteExit - writing link block");
  JIT_LOG("[WriteExit] Writing primary link block, dest=0x%08x", destination);

  if (!js.curBlock) {
    JIT_LOG("[WriteExit] ERROR: js.curBlock is null!");
  }

  // Log values before adding to linkData
JIT_LOG("[WriteExit] Preparing linkData: destination=0x%08x", destination);
JIT_LOG("[WriteExit] Preparing linkData: exitPtrs=%p", GetWritableCodePtr());
JIT_LOG("[WriteExit] Preparing linkData: linkStatus=%d", false);
JIT_LOG("[WriteExit] Preparing linkData: call=%u", LK);
JIT_LOG("[WriteExit] Preparing linkData: exitFarcode=%p", primary_farcode_addr);

  JitBlock* b = js.curBlock;
  JitBlock::LinkData linkData{};
  linkData.exitAddress = destination;

  JIT_LOG("[WriteExit] About to call GetWritableCodePtr()");
u8* writablePtr = GetWritableCodePtr();

JIT_LOG("[WriteExit] GetWritableCodePtr returned %p", writablePtr);

linkData.exitPtrs = writablePtr;

  linkData.linkStatus = false;
  linkData.call = LK;
  linkData.exitFarcode = primary_farcode_addr;

  JIT_LOG("[WriteExit] Calling push_back");

  b->linkData.push_back(linkData);

  JIT_LOG("[WriteExit] Calling WriteLinkBlock");
  blocks.WriteLinkBlock(*this, linkData);

  if (LK)
  {
    JIT_LOG_MSG("WriteExit - LK - writing link block");
    JIT_LOG("[WriteExit] Writing secondary link block, after_return=0x%08x",
           exit_address_after_return);
    DEBUG_ASSERT(GetCodePtr() == host_address_after_return || HasWriteFailed());

    linkData.exitAddress = exit_address_after_return;
    linkData.exitPtrs = GetWritableCodePtr();
    linkData.linkStatus = false;
    linkData.call = false;
    linkData.exitFarcode = return_farcode_addr;
    b->linkData.push_back(linkData);
    blocks.WriteLinkBlock(*this, linkData);
  }

  if (switch_to_far_code)
    SwitchToFarCode();

  JIT_LOG("[WriteExit] Emitting timing stubs, dest=0x%08x", destination);
  MOVI2R(DISPATCHER_PC, destination);
  if (LK)
    BL(GetAsmRoutines()->do_timing);
  else
    SafeB(GetAsmRoutines()->do_timing);

  if (LK)
  {
    MOVI2R(DISPATCHER_PC, exit_address_after_return);
    B(GetAsmRoutines()->do_timing);
    JIT_LOG("[WriteExit] LK timing stub for after_return=0x%08x", exit_address_after_return);
  }

  if (switch_to_far_code)
    SwitchToNearCode();
}

void JitArm::WriteExit(ArmGen::ARMReg dest, bool LK,
                       u32 exit_address_after_return,
                       ArmGen::ARMReg exit_address_after_return_reg)
{
  JIT_LOG("[WriteExit] (reg version) LK=%u after_return=0x%08x exit_address_after_return_reg=%d js.curBlock=%p js.curBlock->effectiveAddress=0x%08x",
         LK ? 1 : 0, exit_address_after_return, exit_address_after_return_reg,
         js.curBlock, js.curBlock->effectiveAddress);

  JIT_LOG_REG("WriteExit (reg): dest", dest);
  JIT_LOG_NUM("WriteExit (reg): LK", LK ? 1 : 0);
  JIT_LOG_NUM("WriteExit (reg): exit_address_after_return", exit_address_after_return);
  if (exit_address_after_return_reg != ArmGen::INVALID_REG)
    JIT_LOG_REG("WriteExit (reg): exit_address_after_return_reg", exit_address_after_return_reg);
  else
    JIT_LOG_MSG("WriteExit (reg): exit_address_after_return_reg invalid");

  {
    auto tmp = gpr.GetScopedReg();
    LDR(tmp, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
    JIT_LOG_REG("WriteExit: LR:", tmp);
  }

  if (dest != DISPATCHER_PC)
  {
    JIT_LOG_REG("WriteExit (reg): Updating dispatcher PC as does not match: dest", dest);
    MOV(DISPATCHER_PC, dest);
  }

  // Save LR BEFORE any optimization checks if m_enable_blr_optimization is false!
  if (LK && !m_enable_blr_optimization)
  {
    if (exit_address_after_return_reg != ArmGen::INVALID_REG)
    {
      // LR value is in a register
      STR(exit_address_after_return_reg, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
    }
    else if(exit_address_after_return != 0)
    {
      auto tmp = gpr.GetScopedReg();
      // LR value is immediate
      MOVI2R(tmp, exit_address_after_return);
      STR(tmp, PPC_REG, PPCSTATE_OFF_SPR(SPR_LR));
    }
  }

  Cleanup();
  if (IsProfilingEnabled())
  {
    //ABI_PushCalleeGPRsAndAdjustStack(true);

    auto cal = gpr.GetScopedReg();
    MOVI2R(R0, reinterpret_cast<u32>(js.curBlock->profile_data.get()));
    MOVI2R(R1, js.downcountAmount);
    MOVI2R(cal, reinterpret_cast<u32>(&JitBlock::ProfileData::EndProfiling));
    BLX(cal);

    //ABI_PopCalleeGPRsAndAdjustStack(true);
  }
  DoDownCount();

  LK &= m_enable_blr_optimization;

  if (!LK)
  {
    JIT_LOG_REG("WriteExit (reg): No LK so branching to dispatcher: DISPATCHER_PC", DISPATCHER_PC);
    B(reinterpret_cast<const void*>(dispatcher));
    return;
  }

  JIT_LOG_MSG("WriteExit (reg): build packed key");

  // Build packed key = (feature_flags << 30) | (exit_address_after_return >> 2)
  const u32 feature_flags = m_ppc_state.feature_flags;
  ARMReg key = gpr.GetReg();
  if (exit_address_after_return_reg == ArmGen::INVALID_REG)
  {
    MOVI2R(key, (feature_flags << 30) | (exit_address_after_return >> 2));
  }
  else if (feature_flags == 0)
  {
    MOV(key, exit_address_after_return_reg);
    LSR(key, key, 2);
  }
  else
  {
    auto tmp = gpr.GetScopedReg();
    LSR(tmp, exit_address_after_return_reg, 2);
    ORR(key, tmp, Operand2(feature_flags << 30));
  }

  JIT_LOG_MSG("WriteExit (reg): Push key");
  // Push only the packed key (avoid ADR/BX on ARM32)
  PUSH(1, key);
  gpr.Unlock(key);

  // Call dispatcher via BLX
  auto call = gpr.GetScopedReg();
  MOVI2R(call, reinterpret_cast<u32>(dispatcher));
  BLX(call);

  // Write the regular exit node after the return.
  JitBlock* b = js.curBlock;
  JitBlock::LinkData linkData;
  linkData.exitAddress = exit_address_after_return;
  linkData.exitPtrs = GetWritableCodePtr();
  linkData.linkStatus = false;
  linkData.call = false;
  const bool switch_to_far_code = !IsInFarCode();
  if (switch_to_far_code)
  {
    SwitchToFarCode();
    linkData.exitFarcode = GetCodePtr();
    SwitchToNearCode();
  }
  else
  {
    linkData.exitFarcode = GetCodePtr() + JitArmBlockCache::BLOCK_LINK_SIZE;
  }
  b->linkData.push_back(linkData);

  blocks.WriteLinkBlock(*this, linkData);

  if (switch_to_far_code)
    SwitchToFarCode();
  MOVI2R(DISPATCHER_PC, exit_address_after_return);

  auto t = gpr.GetScopedReg();
  MOVI2R(t, reinterpret_cast<u32>(GetAsmRoutines()->do_timing));
  B(t);

  if (switch_to_far_code)
    SwitchToNearCode();
}

void JitArm::WriteBLRExit(ArmGen::ARMReg dest)
{
  if (js.curBlock)
    JIT_LOG("[WriteBLRExit] js.curBlock->effectiveAddress=0x%08x",
      js.curBlock->effectiveAddress);

  JIT_LOG_REG("WriteBLRExit (reg): dest", dest);

  if (!m_enable_blr_optimization)
  {
    // Fallback: just exit through the generic path
    JIT_LOG_REG("WriteBLRExit - m_enable_blr_optimization is false (using dest instead)", dest);
    WriteExit(dest);
    return;
  }

  // Copy the destination into the reserved dispatcher PC register if needed
  if (dest != DISPATCHER_PC)
    MOV(DISPATCHER_PC, dest);

  // Do any housekeeping before leaving this block
  Cleanup();
  if (IsProfilingEnabled())
  {
    EndTimeProfile(js.curBlock);
  }

  // ARM32 doesn’t have the fast RET‑to‑host‑PC path that ARM64 uses,
  // so we just fall back to the dispatcher.
  DoDownCount();

  // Load the dispatcher entry point into a scratch reg and branch
  auto scratch = gpr.GetScopedReg();
  MOVI2R(scratch, reinterpret_cast<u32>(dispatcher));
  B(scratch);
  BKPT(0xFF);
}

void JitArm::Run()
{
  // emulated PowerPC program counter
  JIT_LOG("JIT ARM32: Running JITed code at PC %08x", m_ppc_state.pc);

  ProtectStack();
  m_system.GetJitInterface().UpdateMembase();

  // host address in ARM32 code cache
  CompiledCode pExecAddr = (CompiledCode)enter_code;

  JIT_LOG("JIT ARM32: Entering JITed code at %p with pc: %08x", pExecAddr, m_ppc_state.pc);

  pExecAddr();

  JIT_LOG("JIT ARM32: Returned from JITed code at PC %08x", m_ppc_state.pc);

  UnprotectStack();
}

void JitArm::SingleStep()
{
  JIT_LOG("JIT ARM32: Single stepping JITed code at PC %08x", m_ppc_state.pc);

  ProtectStack();
  m_system.GetJitInterface().UpdateMembase();

  CompiledCode pExecAddr = (CompiledCode)enter_code;
  pExecAddr();

  UnprotectStack();
}

void JitArm::Trace()
{
  JIT_LOG("JIT ARM32: Tracing JITed code at PC %08x", m_ppc_state.pc);

  std::string regs;
  std::string fregs;

#ifdef JIT_LOG_GPR
  for (int i = 0; i < 32; i++)
  {
    regs += StringFromFormat("r%02d: %08x ", i, PowerPC::ppcState.gpr[i]);
  }
#endif

#ifdef JIT_LOG_FPR
  for (int i = 0; i < 32; i++)
  {
    fregs += StringFromFormat("f%02d: %016x ", i, riPS0(i));
  }
#endif

  DEBUG_LOG_FMT(DYNA_REC, "JIT64 PC: {:08x} SRR0: {:08x} SRR1: {:08x} FPSCR: {:08x} MSR: {:08x} LR: {:08x} {} {}",
    m_ppc_state.pc, SRR0(m_ppc_state), SRR1(m_ppc_state), m_ppc_state.fpscr.Hex,
    m_ppc_state.msr.Hex, m_ppc_state.spr[8], regs, fregs);
}

void JitArm::Jit(u32 em_address)
{
  Jit(em_address, true);
}

void JitArm::Jit(u32 em_address, bool clear_cache_and_retry_on_failure)
{
  static std::unordered_map<u32, int> pc_counts;
  static int total_blocks = 0;

  pc_counts[em_address]++;
  total_blocks++;

  if (total_blocks % 10000 == 0)  // Every 10k blocks
  {
    JIT_LOG("=== Top 10 most compiled blocks ===");
    std::vector<std::pair<u32, int>> sorted(pc_counts.begin(), pc_counts.end());
    std::sort(sorted.begin(), sorted.end(),
              [](auto& a, auto& b) { return a.second > b.second; });

    for (int i = 0; i < 10 && i < sorted.size(); i++)
    {
      JIT_LOG("PC 0x%08x: %d times", sorted[i].first, sorted[i].second);
    }
  }

  JIT_LOG("JIT ARM32: JITing at address %08x, current PC is %08x", em_address, m_ppc_state.pc);

  if (IsAlmostFull() || SConfig::GetInstance().bJITNoBlockCache)
  {
    JIT_LOG("Clearing cache as cache almost full");
    ClearCache();
  }
  FreeRanges();

  const Common::ScopedJITPageWriteAndNoExecute enable_jit_page_writes(GetRegionPtr());

  std::size_t block_size = m_code_buffer.size();

  auto& cpu = m_system.GetCPU();

  if (IsDebuggingEnabled())
  {
    // We can link blocks as long as we are not single stepping
    SetBlockLinkingEnabled(true);
    SetOptimizationEnabled(true);

    if (!IsProfilingEnabled())
    {
      if (cpu.IsStepping())
      {
        block_size = 1;

        // Do not link this block to other blocks while single stepping
        SetBlockLinkingEnabled(false);
        SetOptimizationEnabled(false);
      }
      Trace();
    }
  }

  // Analyze the block, collect all instructions it is made of (including inlining,
  // if that is enabled), reorder instructions for optimal performance, and join joinable
  // instructions.
  const u32 nextPC = analyzer.Analyze(em_address, &code_block, &m_code_buffer, block_size);

  JIT_LOG("Analyzed block: em_address=0x%08x, code_block.m_num_instructions=%u",
       em_address, code_block.m_num_instructions);
  JIT_LOG("First instruction: PC=0x%08x", m_code_buffer[0].address);
  JIT_LOG("Last instruction: PC=0x%08x", m_code_buffer[code_block.m_num_instructions-1].address);

  if (code_block.m_memory_exception)
  {
    // Address of instruction could not be translated
    m_ppc_state.npc = nextPC;
    m_ppc_state.Exceptions |= EXCEPTION_ISI;
    m_system.GetPowerPC().CheckExceptions();
    m_system.GetJitInterface().UpdateMembase();
    WARN_LOG_FMT(POWERPC, "ISI exception at {:#010x}", nextPC);
    return;
  }

  if (SetEmitterStateToFreeCodeRegion())
  {
    u8* near_start = GetWritableCodePtr();
    u8* far_start = m_far_code.GetWritableCodePtr();

    JitBlock* b = blocks.AllocateBlock(em_address);

    JIT_LOG("JIT ARM32: Allocated block at %p for PC %08x", b, em_address);

    if (DoJit(em_address, b, nextPC))
    {
      if (js.curBlock)
      {
        JIT_LOG("Block start (P): %08x", js.curBlock->effectiveAddress);
        JIT_LOG_NUM("Block start:", js.curBlock->effectiveAddress);
      }

      if (js.curBlock && !js.curBlock->original_buffer.empty())
      {
        auto [addr, instr] = js.curBlock->original_buffer.front();
        JIT_LOG_NUM("First opcode address:", addr);
        JIT_LOG_NUM("First opcode value:", instr.hex);
      }

      u8* near_end = GetWritableCodePtr();

      // Mark used memory region in the local free set
      if (near_start != near_end)
        m_free_ranges_near.erase(near_start, near_end);

      u8* far_end = m_far_code.GetWritableCodePtr();
      if (far_start != far_end)
        m_free_ranges_far.erase(far_start, far_end);

      b->near_begin = const_cast<u8*>(near_start);
      b->near_end = const_cast<u8*>(near_end);
      b->far_begin = const_cast<u8*>(far_start);
      b->far_end = const_cast<u8*>(far_end);

      if (js.curBlock)
      {
        JIT_LOG("Finalize block for: %08x", js.curBlock->effectiveAddress);
      }
      blocks.FinalizeBlock(*b, jo.enableBlocklink, code_block, m_code_buffer);

      if (js.curBlock)
      {
        JIT_LOG("JIT ARM32: Complete for PC %08x", em_address);
      }

      return;
    }
  }

  if (clear_cache_and_retry_on_failure)
  {
    // Code generation failed due to not enough free space in either the near or far code regions.
    // Clear the entire JIT cache and retry.
    JIT_LOG("flushing code caches, please report if this happens a lot");

    WARN_LOG_FMT(DYNA_REC, "flushing code caches, please report if this happens a lot");
    ClearCache();
    Jit(em_address, false);
    return;
  }

  PanicAlertFmtT("JIT failed to find code space after a cache clear. This should never happen. "
                 "Please report this incident on the bug tracker. Dolphin will now exit.");
  exit(-1);
}

bool JitArm::SetEmitterStateToFreeCodeRegion()
{
  // Try to find the largest free memory block in the near code region.
  const auto free_near = m_free_ranges_near.by_size_begin();
  if (free_near != m_free_ranges_near.by_size_end())
  {
    // Point the ARM emitter at that free block.
    m_near_code.SetCodePtr(free_near.from(), free_near.to());
    return true;
  }

  // If no near block is available, try the far region.
  const auto free_far = m_free_ranges_far.by_size_begin();
  if (free_far != m_free_ranges_far.by_size_end())
  {
    // Point the ARM emitter at that free block.
    m_far_code.SetCodePtr(free_far.from(), free_far.to());
    return true;
  }

  // Nothing available in either region.
  JIT_LOG("Failed to find free memory region in near or far code region.");

  WARN_LOG_FMT(DYNA_REC, "Failed to find free memory region in near or far code region.");

  return false;
}

void JitArm::Break(UGeckoInstruction inst)
{
  JIT_LOG("JIT ARM32: Break instruction called at PC %08x", m_ppc_state.pc);

  ERROR_LOG_FMT(DYNA_REC, "{} called a Break instruction!",
    PPCTables::GetInstructionName(inst, m_ppc_state.pc));
  BKPT(0x4444);
}

void JitArm::BeginTimeProfile(JitBlock* b)
{
  /*b->ticCounter = 0;
  b->ticStart = 0;
  b->ticStop = 0;

  // Performance counters are bit finnicky on ARM
  // We must first enable and program the PMU before using it
  // This is a per core operation so with thread scheduling we may jump to a core we haven't enabled PMU yet
  // Work around this by enabling PMU each time at the start of a block
  // Some ARM CPUs are getting absurd core counts(48+!)
  // We have to reset counters at the start of every block anyway, so may as well.
  // One thing to note about performance counters on ARM
  // The kernel can block access to these co-processor registers
  // In the case that this happens, these will generate a SIGILL

  // Refer to the ARM ARM about PMCR for what these do exactly
  enum
  {
    PERF_OPTION_ENABLE = (1 << 0),
    PERF_OPTION_RESET_CR = (1 << 1),
    PERF_OPTION_RESET_CCR = (1 << 2),
    PERF_OPTION_DIVIDER_MODE = (1 << 3),
    PERF_OPTION_EXPORT_ENABLE = (1 << 4),
  };
  const u32 perf_options =
    PERF_OPTION_ENABLE |
    PERF_OPTION_RESET_CR |
    PERF_OPTION_RESET_CCR |
    PERF_OPTION_EXPORT_ENABLE;
  MOVI2R(R0, perf_options);
  // Programs the PMCR
  MCR(15, 0, R0, 9, 12, 0);

  MOVI2R(R0, 0x8000000F);
  // Enables all counters
  MCR(15, 0, R0, 9, 12, 1);
  // Clears all counter overflows
  MCR(15, 0, R0, 9, 12, 3);

  // Gets the cycle counter
  MRC(15, 0, R1, 9, 13, 0);
  MOVI2R(R0, (u32)&b->ticStart);
  STR(R1, R0, 0);*/ // TODO WEBOS
}

void JitArm::EndTimeProfile(JitBlock* b)
{
  // Gets the cycle counter
  /*MRC(15, 0, R1, 9, 13, 0);
  MOVI2R(R0, (u32)&b->ticStop);
  STR(R1, R0, 0);

  MOVI2R(R0, (u32)&b->ticStart);
  MOVI2R(R14, (u32)m_increment_profile_counter);
  BL(R14);*/ //TODO WEBOS
}

bool JitArm::DoJit(u32 em_address, JitBlock* b, u32 nextPC)
{
  // Per-block state
  js.isLastInstruction = false;
  js.firstFPInstructionFound = false;
  js.assumeNoPairedQuantize = false;
  js.blockStart = em_address;
  js.fifoBytesSinceCheck = 0;
  js.mustCheckFifo = false;
  js.downcountAmount = 0;
  js.skipInstructions = 0;
  js.curBlock = b;
  js.carryFlag = CarryFlag::InPPCState;
  js.numLoadStoreInst = 0;
  js.numFloatingPointInst = 0;

  b->normalEntry = GetWritableCodePtr();

  // Optional profiling begin
  if (IsProfilingEnabled() && b->profile_data)
  {
    auto rA = gpr.GetScopedReg();
    auto rB = gpr.GetScopedReg();
    MOVI2R(rA, (u32)&b->profile_data->run_count);
    LDR(rB, rA);
    ADD(rB, rB, 1);
    STR(rB, rA);
    BeginTimeProfile(b);
  }

  gpr.Start(js.gpa);
  fpr.Start(js.fpa);

  // disable as broken
  /*if (!js.noSpeculativeConstantsAddresses.contains(js.blockStart))
  {
    JIT_LOG("Calling IntializeSpeculativeConstant - blockStart = %08x", js.blockStart);
    IntializeSpeculativeConstants();

    JIT_LOG("Finished IntializeSpeculativeConstants\n");
  }*/

  // Translate instructions
  for (u32 i = 0; i < code_block.m_num_instructions; i++)
  {
    PPCAnalyst::CodeOp& op = m_code_buffer[i];

    if (i == 0)
    {
      JIT_LOG("DoJit loop starting: blockStart=%08x, first instruction PC=%08x, num_instructions=%u",
            js.blockStart, op.address, code_block.m_num_instructions);
    }

    JIT_LOG("Translating instruction %d, address = %08x", i, op.address);

    js.compilerPC = op.address;
    js.op = &op;
    js.fpr_is_store_safe = op.fprIsStoreSafeBeforeInst;
    js.instructionsLeft = (code_block.m_num_instructions - 1) - i;
    const GekkoOPInfo* opinfo = op.opinfo;
    js.downcountAmount += opinfo->num_cycles;
    js.isLastInstruction = i == (code_block.m_num_instructions - 1);

    // Skip calling UpdateLastUsed for lmw/stmw - it usually hurts more than it helps
    if (op.inst.OPCD != 46 && op.inst.OPCD != 47)
      gpr.UpdateLastUsed(op.regsIn | op.regsOut);

    BitSet32 fpr_used = op.fregsIn;
    if (op.fregOut >= 0)
      fpr_used[op.fregOut] = true;
    fpr.UpdateLastUsed(fpr_used);

    // === op.skip / else parity ===
    if (op.skip)
    {
      JIT_LOG("Skipping instruction %d", i);

      if (IsBranchWatchEnabled())
      {
        const auto bw_reg_a = gpr.GetScopedReg();
        const auto bw_reg_b = gpr.GetScopedReg();
        WriteBranchWatch<true>(op.address, op.branchTo, op.inst,
                               bw_reg_a, bw_reg_b,
                               BitSet32{}, BitSet32{});
      }
    }
    else
    {
      // Breakpoint parity
      if (IsDebuggingEnabled() && !m_system.GetCPU().IsStepping() &&
          m_system.GetPowerPC().GetBreakPoints().IsAddressBreakPoint(op.address))
      {
        JIT_LOG("BREAKPOINT for %d", i);

        FlushCarry(); // ensure carry is in PPCState
        gpr.Flush();
        fpr.Flush();

        auto rA = gpr.GetScopedReg();
        MOVI2R(rA, op.address);
        STR(rA, PPC_REG, PPCSTATE_OFF(pc));
        STR(rA, PPC_REG, PPCSTATE_OFF(npc));

        MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetPowerPC()));
        auto cal = gpr.GetScopedReg();
        MOVI2R(cal, reinterpret_cast<u32>(&PowerPC::CheckAndHandleBreakPointsFromJIT));
        BLX(cal);

        auto rS = gpr.GetScopedReg();
        MOVI2R(rS, reinterpret_cast<u32>(m_system.GetCPU().GetStatePtr()));
        LDR(R0, rS);
        FixupBranch no_breakpoint = B_CC(CC_EQ);

        Cleanup();
        DoDownCount();
        B(dispatcher_exit);

        SetJumpTarget(no_breakpoint);
      }

      JIT_LOG_NUM("js.firstFPInstructionFound BEFORE check ", js.firstFPInstructionFound);

      JIT_LOG("js.firstFPInstructionFound BEFORE check %d", js.firstFPInstructionFound);

      if ((opinfo->flags & FL_USE_FPU) && !js.firstFPInstructionFound)
      {
        //JIT_LOG("ADDING FPU EXCEPTION CHECK!");

        //JIT_LOG_NUM("JIT: FL_USE_FPU was set so setting EXCEPTION_FPU_UNAVAILABLE if appropiate");
        auto WA = gpr.GetScopedReg();
        auto WT = gpr.GetScopedReg();

        // Load MSR
        LDR(WA, PPC_REG, IMM(PPCSTATE_OFF(msr)));

        // Build mask in a GPR using MOVI2R (handles any 32-bit constant)
        MOVI2R(WT, 1u << 13);

        // Test bit 13
        TST(WA, R(WT));

        // Z==1 (FP disabled) -> jump to far bailout
        FixupBranch to_far = B_CC(CC_EQ);

        SwitchToFarCode();
        SafeSetJumpTarget(to_far);

        gpr.Flush(FlushMode::MaintainState);
        fpr.Flush(FlushMode::MaintainState);

        LDR(WA, PPC_REG, PPCSTATE_OFF(Exceptions));
        ORR(WA, WA, EXCEPTION_FPU_UNAVAILABLE);
        STR(WA, PPC_REG, PPCSTATE_OFF(Exceptions));

        WriteExceptionExit(js.compilerPC, false, true);

        SwitchToNearCode();

        js.firstFPInstructionFound = true;
        //JIT_LOG("js.firstFPInstructionFound AFTER set = %d", js.firstFPInstructionFound);
        //JIT_LOG_NUM("js.firstFPInstructionFound AFTER set = ", js.firstFPInstructionFound);
      }

      if (bJITRegisterCacheOff)
      {
        JIT_LOG("bJITRegisterCacheOff");

        FlushCarry(); // flush before discarding caches
        gpr.Flush();
        fpr.Flush();
      }

      // Compile current instruction
      CompileInstruction(op);

      if (js.op->regsOut[0])    // If this instruction writes to r0
      {
        JIT_LOG("Wrote to r0");

        // Add logging AFTER the instruction handler
        auto check_r0 = gpr.GetScopedReg();
        LDR(check_r0, PPC_REG, PPCSTATE_OFF_GPR(0));

        char msg[128];
        snprintf(msg, sizeof(msg), "PC=%08x wrote to r0, now:", op.address);
        JIT_LOG_REG(msg, check_r0);
      }

      js.fpr_is_store_safe = op.fprIsStoreSafeAfterInst;

      // Carry flush parity: if not merging with next integer op
      if (!CanMergeNextInstructions(1) || js.op[1].opinfo->type != ::OpType::Integer)
        FlushCarry();

      // Store from registers per liveness
      //for (int j : ~op.gprInUse)
      //  gpr.StoreFromRegister(j);
      //for (int j : ~op.fprInUse)
      //  fpr.StoreFromRegister(j);
      JIT_LOG("StoreRegisters");

      // TODO rebase check new implementation
      gpr.StoreRegisters(~op.gprInUse & (op.regsIn | op.regsOut));
      fpr.StoreRegisters(~op.fprInUse & (op.fregsIn | op.GetFregsOut()));
      gpr.StoreCRRegisters(~op.crInUse & (op.crIn | op.crOut));

      if (opinfo->flags & FL_LOADSTORE)
        ++js.numLoadStoreInst;

      if (opinfo->flags & FL_USE_FPU)
        ++js.numFloatingPointInst;
    }

    i += js.skipInstructions;
    js.skipInstructions = 0;
  }

  JIT_LOG("code_block.m_broken=%d nextPC=%08x", code_block.m_broken, nextPC);

  if (code_block.m_broken)
  {
    JIT_LOG("m_broken is TRUE");

    FlushCarry();
    gpr.Flush();
    fpr.Flush();
    WriteExit(nextPC);
  }

  if (HasWriteFailed() || m_far_code.HasWriteFailed())
  {
    JIT_LOG("HasWriteFailed");

    if (HasWriteFailed())
      WARN_LOG_FMT(DYNA_REC, "JIT ran out of space in near code region during code generation.");
    if (m_far_code.HasWriteFailed())
      WARN_LOG_FMT(DYNA_REC, "JIT ran out of space in far code region during code generation.");
    return false;
  }

  FlushIcache();
  m_far_code.FlushIcache();

  JIT_LOG("Finished DoJit");

  return true;
}

void JitArm::EraseSingleBlock(const JitBlock& block)
{
  JIT_LOG( "JIT ARM32: Erasing single block at address %08x", block.effectiveAddress);

  blocks.EraseSingleBlock(block);
  FreeRanges();
}

std::vector<JitBase::MemoryStats> JitArm::GetMemoryStats() const
{
  return {{"near", m_free_ranges_near.get_stats()}};
}

std::size_t JitArm::DisassembleNearCode(const JitBlock& block, std::ostream& stream) const
{
  JIT_LOG( "JIT ARM32: Disassembling near code for block at address %08x", block.effectiveAddress);

  return m_disassembler->Disassemble(block.normalEntry, block.near_end, stream);
}

std::size_t JitArm::DisassembleFarCode(const JitBlock& block, std::ostream& stream) const
{
  JIT_LOG( "JIT ARM32: Disassembling far code for block at address %08x", block.effectiveAddress);

  return m_disassembler->Disassemble(block.far_begin, block.far_end, stream);
}
