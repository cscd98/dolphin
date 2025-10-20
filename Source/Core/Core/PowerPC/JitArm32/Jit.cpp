// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <map>

#include "Common/ArmEmitter.h"
#include "Common/CommonTypes.h"
#include "Common/IOFile.h"

#include "Core/Config/MainSettings.h"
#include "Core/ConfigManager.h"
#include "Core/Core.h"
#include "Core/CoreTiming.h"
#include "Core/PatchEngine.h"
#include "Core/HLE/HLE.h"
#include "Core/HW/GPFifo.h"
#include "Core/HW/Memmap.h"
#include "Core/PowerPC/JitInterface.h"
#include "Core/PowerPC/PowerPC.h"
#include "Core/PowerPC/PPCAnalyst.h"
#include "Core/PowerPC/PPCTables.h"
#include "Core/PowerPC/Interpreter/Interpreter.h"
//#include "Core/PowerPC/Profiler.h"
#include "Core/PowerPC/JitArm32/Jit.h"
#include "Core/Core.h"
#include "Core/System.h"

using namespace ArmGen;

constexpr size_t CODE_SIZE = 1024 * 1024 * 32;
constexpr size_t FARCODE_SIZE = 1024 * 1024 * 16;
constexpr size_t FARCODE_SIZE_MMU = 1024 * 1024 * 48;

void JitArm::Init()
{
	size_t child_code_size = Config::Get(Config::MAIN_MMU) ? FARCODE_SIZE_MMU : FARCODE_SIZE;
  AllocCodeSpace(CODE_SIZE + child_code_size);
  AddChildCodeSpace(&farcode, child_code_size);

	blocks.Init();
	asm_routines.Init();
	gpr.Init(this);
	fpr.Init(this);
	jo.enableBlocklink = true;
	jo.optimizeGatherPipe = true;
	RefreshConfig();

	code_block.m_stats = &js.st;
	code_block.m_gpa = &js.gpa;
	code_block.m_fpa = &js.fpa;
	analyzer.SetOption(PPCAnalyst::PPCAnalyzer::OPTION_CONDITIONAL_CONTINUE);
	InitBackpatch();

	// Disable all loadstores
	// Ever since the MMU has been optimized for x86, loadstores on ARMv7 have been knackered
	// XXX: Investigate exactly why these are broken
	Config::SetCurrent(Config::MAIN_DEBUG_JIT_LOAD_STORE_OFF, true);
	Config::SetCurrent(Config::MAIN_DEBUG_JIT_LOAD_STORE_FLOATING_OFF, true);
	Config::SetCurrent(Config::MAIN_DEBUG_JIT_LOAD_STORE_PAIRED_OFF, true);
}

void JitArm::ClearCache()
{
	ClearCodeSpace();
	blocks.Clear();
	farcode.ClearCodeSpace();
	RefreshConfig();
}

void JitArm::Shutdown()
{
	FreeCodeSpace();
	blocks.Shutdown();
	asm_routines.Shutdown();
}

// This is only called by FallBackToInterpreter() in this file. It will execute an instruction with the interpreter functions.
void JitArm::WriteCallInterpreter(UGeckoInstruction inst)
{
	gpr.Flush();
	fpr.Flush();
	Interpreter::Instruction instr = Interpreter::GetInterpreterOp(inst);
	MOVI2R(R0, inst.hex);
	MOVI2R(R12, (u32)instr);
	BL(R12);

	// TODO: webOS: arm64 jit has a lot more to this
}

void JitArm::FallBackToInterpreter(UGeckoInstruction _inst)
{
	WriteCallInterpreter(_inst.hex);
}

void JitArm::HLEFunction(UGeckoInstruction _inst)
{
	gpr.Flush();
	fpr.Flush();
	MOVI2R(R0, js.compilerPC);
	MOVI2R(R1, _inst.hex);
	QuickCallFunction(R14, (void*)&HLE::Execute);
	ARMReg rA = gpr.GetReg();
	LDR(rA, R9, PPCSTATE_OFF(npc));
	WriteExitDestInR(rA);
}

void JitArm::DoNothing(UGeckoInstruction _inst)
{
	// Yup, just don't do anything.
}

static const bool ImHereDebug = false;
static const bool ImHereLog = false;
static std::map<u32, int> been_here;

void JitArm::ImHere(JitArm& jit)
{
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

	DEBUG_LOG_FMT(DYNA_REC, "I'm here - PC = {:08x}, LR = {:08x}", ppc_state.pc, LR(ppc_state));
	been_here[ppc_state.pc] = 1;
}

void JitArm::Cleanup()
{
	if (jo.optimizeGatherPipe && js.fifoBytesSinceCheck > 0)
	{
		PUSH(4, R0, R1, R2, R3);
		QuickCallFunction(R14, (void*)&GPFifo::FastCheckGatherPipe);
		POP(4, R0, R1, R2, R3);
	}
}
void JitArm::DoDownCount()
{
	ARMReg rA = gpr.GetReg();
	LDR(rA, R9, PPCSTATE_OFF(downcount));
	if (js.downcountAmount < 255) // We can enlarge this if we used rotations
	{
		SUBS(rA, rA, js.downcountAmount);
	}
	else
	{
		ARMReg rB = gpr.GetReg(false);
		MOVI2R(rB, js.downcountAmount);
		SUBS(rA, rA, rB);
	}
	STR(rA, R9, PPCSTATE_OFF(downcount));
	gpr.Unlock(rA);
}
void JitArm::WriteExitDestInR(ARMReg Reg)
{
	STR(Reg, R9, PPCSTATE_OFF(pc));
	Cleanup();
	if (IsProfilingEnabled())
  {
    ABI_CallFunction(&JitBlock::ProfileData::EndProfiling, js.curBlock->profile_data.get(),
                     js.downcountAmount);
  }
	DoDownCount();

	MOVI2R(Reg, (u32)asm_routines.dispatcher);
	B(Reg);
	gpr.Unlock(Reg);
}
void JitArm::WriteRfiExitDestInR(ARMReg Reg)
{
	STR(Reg, R9, PPCSTATE_OFF(pc));
	Cleanup();
	if (IsProfilingEnabled())
  {
    ABI_CallFunction(&JitBlock::ProfileData::EndProfiling, js.curBlock->profile_data.get(),
                     js.downcountAmount);
  }
	DoDownCount();

	ARMReg A = gpr.GetReg(false);

	LDR(A, R9, PPCSTATE_OFF(pc));
	STR(A, R9, PPCSTATE_OFF(npc));
		QuickCallFunction(A, (void*)&PowerPC::CheckExceptionsFromJIT);
	LDR(A, R9, PPCSTATE_OFF(npc));
	STR(A, R9, PPCSTATE_OFF(pc));
	gpr.Unlock(Reg); // This was locked in the instruction beforehand

	MOVI2R(A, (u32)asm_routines.dispatcher);
	B(A);
}
void JitArm::WriteExceptionExit()
{
	Cleanup();
	if (IsProfilingEnabled())
  {
    ABI_CallFunction(&JitBlock::ProfileData::EndProfiling, js.curBlock->profile_data.get(),
                     js.downcountAmount);
  }
	DoDownCount();

	ARMReg A = gpr.GetReg(false);

	LDR(A, R9, PPCSTATE_OFF(pc));
	STR(A, R9, PPCSTATE_OFF(npc));
		QuickCallFunction(A, (void*)&PowerPC::CheckExceptionsFromJIT);
	LDR(A, R9, PPCSTATE_OFF(npc));
	STR(A, R9, PPCSTATE_OFF(pc));

	MOVI2R(A, (u32)asm_routines.dispatcher);
	B(A);
}

void JitArm::WriteExit(u32 destination)
{
  Cleanup();
  if (IsProfilingEnabled())
		ABI_CallFunction(&JitBlock::ProfileData::EndProfiling,
			js.curBlock->profile_data.get(),
			js.downcountAmount);
	DoDownCount();

  JitBlock* b = js.curBlock;

  JitBlock::LinkData linkData;
  linkData.exitAddress = destination;
  linkData.exitPtrs = GetWritableCodePtr();
  linkData.linkStatus = false;
  linkData.call = false;

  b->linkData.push_back(linkData);

  // Delegate linking logic to the block cache
  blocks.WriteLinkBlock(*this, linkData);
}

void JitArm::Run()
{
	CompiledCode pExecAddr = (CompiledCode)enter_code;
  pExecAddr();
}

void JitArm::SingleStep()
{
	CompiledCode pExecAddr = (CompiledCode)enter_code;
	pExecAddr();
}

void JitArm::Trace()
{
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

	DEBUG_LOG_FMT(DYNA_REC,
		"JITARM PC: {:08x} SRR0: {:08x} SRR1: {:08x} FPSCR: {:08x} "
		"MSR: {:08x} LR: {:08x} {} {}",
		m_ppc_state.pc, SRR0(m_ppc_state), SRR1(m_ppc_state), m_ppc_state.fpscr.Hex,
		m_ppc_state.msr.Hex, m_ppc_state.spr[8], regs, fregs);
}

void JitArm::Jit(u32 em_address)
{
	if (IsAlmostFull() || farcode.IsAlmostFull() || SConfig::GetInstance().bJITNoBlockCache)
  {
    ClearCache();
  }

	std::size_t block_size = m_code_buffer.size();

	// Analyze the block, collect all instructions it is made of (including inlining,
  // if that is enabled), reorder instructions for optimal performance, and join joinable
  // instructions.
  const u32 nextPC = analyzer.Analyze(em_address, &code_block, &m_code_buffer, block_size);

  if (code_block.m_memory_exception)
  {
    // Address of instruction could not be translated
    m_ppc_state.npc = nextPC;
		m_ppc_state.Exceptions |= EXCEPTION_ISI;
    m_system.GetPowerPC().CheckExceptions();
		m_system.GetJitInterface().UpdateMembase();
    WARN_LOG_FMT(POWERPC, "ISI exception at 0x{:08x}", nextPC);
    return;
  }

	JitBlock* b = blocks.AllocateBlock(em_address);
	const u8* BlockPtr = DoJit(m_ppc_state.pc, &code_buffer, b);
	blocks.FinalizeBlock(*b, jo.enableBlocklink, code_block, m_code_buffer); // TODO WEBOS??
}

void JitArm::Break(UGeckoInstruction inst)
{
	ERROR_LOG_FMT(DYNA_REC, "{} called a Break instruction!", inst.hex);
	BKPT(0x4444);
}

const u8* JitArm::DoJit(u32 em_address, PPCAnalyst::CodeBuffer *code_buf, JitBlock *b)
{
	int blockSize = code_buf->size();

	if (IsDebuggingEnabled())
	{
		if (!IsProfilingEnabled())
    {
      if (cpu.IsStepping())
			{
				// Comment out the following to disable breakpoints (speed-up)
				blockSize = 1;
			}
			Trace();
		}
	}

	if (em_address == 0)
	{
		Core::SetState(Core::System::GetInstance(), Core::State::Paused);
		PanicAlertFmt("ERROR: Compiling at 0. LR={:08x} CTR={:08x}",
			m_ppc_state.spr[SPR_LR], m_ppc_state.spr[SPR_CTR]);
	}

	js.isLastInstruction = false;
	js.blockStart = em_address;
	js.fifoBytesSinceCheck = 0;
	js.curBlock = b;

	u32 nextPC = em_address;
	// Analyze the block, collect all instructions it is made of (including inlining,
	// if that is enabled), reorder instructions for optimal performance, and join joinable instructions.
	nextPC = analyzer.Analyze(em_address, &code_block, code_buf, blockSize);

	u8* const start = GetWritableCodePtr();
  b->checkedEntry = start;
	b->profile_data.runCount = 0;

	// Downcount flag check, Only valid for linked blocks
	{
		FixupBranch no_downcount = B_CC(CC_PL);
		ARMReg rA = gpr.GetReg(false);
		MOVI2R(rA, js.blockStart);
		STR(rA, R9, PPCSTATE_OFF(pc));
		MOVI2R(rA, (u32)asm_routines.do_timing);
		B(rA);
		SetJumpTarget(no_downcount);
	}

	// Normal entry doesn't need to check for downcount.
  b->normalEntry = GetWritableCodePtr();

	if (ImHereDebug)
		QuickCallFunction(R14, (void *)&ImHere); //Used to get a trace of the last few blocks before a crash, sometimes VERY useful

	if (js.fpa.any)
	{
		// This block uses FPU - needs to add FP exception bailout
		ARMReg A = gpr.GetReg();
		ARMReg C = gpr.GetReg();
		Operand2 Shift(2, 10); // 1 << 13
		MOVI2R(C, js.blockStart); // R3
		LDR(A, R9, PPCSTATE_OFF(msr));
		TST(A, Shift);
		FixupBranch no_fpe = B_CC(CC_NEQ);
		STR(C, R9, PPCSTATE_OFF(pc));

		LDR(A, R9, PPCSTATE_OFF(Exceptions));
		ORR(A, A, EXCEPTION_FPU_UNAVAILABLE);
		STR(A, R9, PPCSTATE_OFF(Exceptions));
			QuickCallFunction(A, (void*)&PowerPC::CheckExceptionsFromJIT);
		LDR(A, R9, PPCSTATE_OFF(npc));
		STR(A, R9, PPCSTATE_OFF(pc));

		MOVI2R(A, (u32)asm_routines.dispatcher);
		B(A);

		SetJumpTarget(no_fpe);
		gpr.Unlock(A, C);
	}

  // Conditionally add profiling code.
  if (IsProfilingEnabled())
    ABI_CallFunction(&JitBlock::ProfileData::BeginProfiling, b->profile_data.get());

	gpr.Start(js.gpa);
	fpr.Start(js.fpa);
	js.downcountAmount = 0;

	if (IsDebuggingEnabled())
		js.downcountAmount += PatchEngine::GetSpeedhackCycles(em_address);

	js.skipInstructions = 0;
	js.compilerPC = nextPC;

	// Translate instructions
	for (u32 i = 0; i < code_block.m_num_instructions; i++)
	{
		PPCAnalyst::CodeOp& op = m_code_buffer[i];

		js.compilerPC = op.address;
		js.op = &op;
		js.instructionNumber = i;
		const GekkoOPInfo *opinfo = op.opinfo;
		js.downcountAmount += opinfo->num_cycles;

		if (i == (code_block.m_num_instructions - 1))
		{
			// WARNING - cmp->branch merging will screw this up.
			js.isLastInstruction = true;
		}

		if (jo.optimizeGatherPipe && js.fifoBytesSinceCheck >= 32)
		{
			js.fifoBytesSinceCheck -= 32;
			PUSH(4, R0, R1, R2, R3);
			QuickCallFunction(R14, (void*)&GPFifo::FastCheckGatherPipe);
			POP(4, R0, R1, R2, R3);
		}

		if (!op.skip)
		{
				if (jo.memcheck && (opinfo->flags & FL_USE_FPU))
				{
					// Don't do this yet
					BKPT(0x7777);
				}
				JitArm::CompileInstruction(op);

				// If we have a register that will never be used again, flush it.
				for (int j : ~op.gprInUse)
					gpr.StoreFromRegister(j);
				for (int j : ~op.fprInUse)
					fpr.StoreFromRegister(j);

				if (jo.memcheck && (opinfo->flags & FL_LOADSTORE))
				{
					// Don't do this yet
					BKPT(0x666);
				}
		}
	}

	if (code_block.m_memory_exception)
		BKPT(0x500);

	if (code_block.m_broken)
	{
		printf("Broken Block going to 0x%08x\n", nextPC);
		WriteExit(nextPC);
	}

	b->codeSize = (u32)(GetCodePtr() - start);
	b->originalSize = code_block.m_num_instructions;
	FlushIcache();
	return start;
}
