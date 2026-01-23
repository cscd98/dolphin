// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Common/ArmEmitter.h"
#include "Common/MemoryUtil.h"

#include <inttypes.h>


#include "Core/Core.h"
#include "Core/CoreTiming.h"
#include "Core/Config/MainSettings.h"
#include "Core/System.h"
#include "Core/HW/CPU.h"
#include "Core/HW/GPFifo.h"
#include "Core/HW/Memmap.h"

#include "Core/PowerPC/PowerPC.h"
#include "Core/PowerPC/JitArm32/Jit.h"
#include "Core/PowerPC/JitCommon/JitCache.h"

#include "Core/PowerPC/JitInterface.h"

using namespace ArmGen;

//TODO - make an option
//#if _DEBUG
//	bool enableDebug = false;
//#else
//	bool enableDebug = false;
//#endif

//JitArmAsmRoutineManager asm_routines;

extern "C" __attribute__((noinline, visibility("default")))
void JitArmTrampoline(JitBase& jit, u32 em_address)
{
	JitBase* ptr = &jit;
	if (!ptr) {
		fprintf(stderr, "JitArmTrampoline: jit pointer is NULL!\n");
		fflush(stdout);
		return;
	}

	printf("JIT ARM32: JitArmTrampoline for %08x\n", em_address);
	fflush(stdout);

	// static state to detect repeated addresses
  static u32 last_address = 0;
  static int repeat_count = 0;

	if (em_address == last_address) {
    repeat_count++;
    if (repeat_count >= 3) {
      fprintf(stderr, "JIT ARM32: em_address %08x repeated %d times, aborting!\n",
              em_address, repeat_count);
      fflush(stderr);
      std::abort();
    }
  } else {
    last_address = em_address;
    repeat_count = 1;
  }

	// sanity check: did we end up with a bogus exit PC?
	if (em_address <= 0x00000800)
	{
		const u32 exc = jit.m_ppc_state.Exceptions;

    // allow if the address matches a defined exception vector
    bool allowed =
      em_address == 0x00000300 || // DSI
      em_address == 0x00000400 || // ISI
      em_address == 0x00000500 || // External interrupt
      em_address == 0x00000600 || // Alignment
      em_address == 0x00000700 || // Program
      em_address == 0x00000800;   // FPU unavailable

    if (!allowed)
    {
      fprintf(stderr, "JIT ARM32: unexpected nextPC <= 0x00000800 at em_address %08x (Exceptions=%08x)\n",
        em_address, exc);
      fflush(stderr);
      std::abort();
    }
	}

	jit.Jit(em_address);
}

static void WriteDual8(PowerPC::MMU& mmu, u32 val1, u32 val2, u32 addr)
{
	mmu.Write<u16>(((u16)(u8)val1 << 8) | (u16)(u8)val2, addr); // TODO: WEBOS: or WriteFromJit?
}

static void WriteDual16(PowerPC::MMU& mmu, u32 val1, u32 val2, u32 addr)
{
	mmu.Write<u32>(((u32)(u16)val1 << 16) | (u32)(u16)val2, addr);
}

static void WriteDual32(PowerPC::MMU& mmu, u32 val1, u32 val2, u32 addr)
{
	mmu.Write<u64>(((u64)val1 << 32) | (u64)val2, addr);
}

void JitArm::GenerateAsm()
{
  const Common::ScopedJITPageWriteAndNoExecute enable_jit_page_writes(GetRegionPtr());

  const bool enable_debugging = Config::IsDebuggingEnabled();

  // ============================================================================
  // Entry Point - Save Callee-Saved Registers
  // ============================================================================
  enter_code = GetCodePtr();

  // Push all callee-saved registers (R4-R11, LR)
  PUSH(9, R4, R5, R6, R7, R8, PPC_REG, R10, R11, _LR);

  // Take care to 8-byte align stack for function calls.
  // We are misaligned here because of an odd number of args for PUSH.
  // It's not like x86 where you need to account for an extra 4 bytes consumed by CALL.
  SUB(_SP, _SP, 4);

  // ============================================================================
  // Initialize Base Registers
  // ============================================================================
  // Register allocation strategy:
  // R9  = PowerPCState base pointer (relative to spr)
  // R8  = Physical memory base (for fastmem - currently unused)
  // R10 = Block cache base pointer (entry points or fast map)
  // R11 = Scratch register for addresses/calculations
  // R12 = Scratch register for calls/dispatch
  // R14 = Branch target (LR) - used for function calls and block jumps

  MOVI2R(PPC_REG, static_cast<u32>(reinterpret_cast<uintptr_t>(&m_ppc_state)));
  MOVI2R(MEM_REG, (u32)m_system.GetMemory().GetPhysicalBase());

  // Store the stack pointer, so we can reset it if the BLR optimization fails.
  {
	auto tmp = gpr.GetScopedReg();
	MOV(tmp, _SP);
	STR(tmp, PPC_REG, PPCSTATE_OFF(stored_stack_pointer));
  }

  // The PC will be loaded into DISPATCHER_PC after the call to CoreTiming::Advance().
  // Advance() does an exception check so we don't know what PC to use until afterwards.
  FixupBranch to_start_of_timing_slice = B();

  // ============================================================================
  // Main Dispatcher Loop (ARM64-style)
  // ============================================================================
  // do
  // {
  //   CoreTiming::Advance();  // <-- Checks for exceptions (changes PC)
  //   DISPATCHER_PC = PC;
  //   do
  //   {
  // dispatcher_no_check:
  //     ExecuteBlock(JitBase::Dispatch());
  // dispatcher:
  //   } while (downcount > 0);
  // do_timing:
  //   NPC = PC = DISPATCHER_PC;
  // } while (CPU::GetState() == CPU::State::Running);

  AlignCodePage();
  dispatcher = GetCodePtr();

  // ----------------------------------------------------------------------------
  // Downcount Check - Branch to timing if negative (ARM64 uses LE; ARM32 uses MI for negative)
  // ----------------------------------------------------------------------------
  FixupBranch bail = B_CC(CC_MI);

  // ----------------------------------------------------------------------------
  // Debug Stepping Check
  // ----------------------------------------------------------------------------
  FixupBranch dbg_exit;
  if (enable_debugging)
  {
    // Check if CPU is in stepping mode
    MOVI2R(R11, (u32)m_system.GetCPU().GetStatePtr());
    LDR(R11, R11);
    TST(R11, static_cast<u32>(CPU::State::Stepping));
    FixupBranch not_stepping = B_CC(CC_EQ);
    // XXX: Could check for breakpoints here
    dbg_exit = B();
    SetJumpTarget(not_stepping);
  }

  // ----------------------------------------------------------------------------
  // Dispatcher No Check - Skip downcount/debug checks
  // ----------------------------------------------------------------------------
  dispatcher_no_check = GetCodePtr();

  LogRegFromJIT("GenerateAsm(): dispatcher_no_check - DISPATCHER_PC is", DISPATCHER_PC);

  {
	auto tmpStartUpPC = gpr.GetScopedReg();
	LDR(tmpStartUpPC, PPC_REG, PPCSTATE_OFF(pc));
	LogRegFromJIT("GenerateAsm(): dispatcher_no_check - PPCSTATE_OFF(pc)", tmpStartUpPC);
  }

  {
	auto tmpStartUpPC = gpr.GetScopedReg();
	LDR(tmpStartUpPC, PPC_REG, PPCSTATE_OFF(npc));
	LogRegFromJIT("GenerateAsm(): dispatcher_no_check - PPCSTATE_OFF(npc)", tmpStartUpPC);
 }

  // ============================================================================
  // Assembly Fast Path - Check block cache (uses DISPATCHER_PC, which is set after Advance)
  // ============================================================================
  const bool assembly_dispatcher = true;
  if (assembly_dispatcher)
  {
    // Get pointer to entry points array or fast block map fallback
    u8** entry_points = GetBlockCache()->GetEntryPoints();
    JitBlock** fast_map_fallback = GetBlockCache()->GetFastBlockMapFallback();

    if (entry_points)
    {
		LogNumFromJIT("GenerateAsm(): entry_points",
			static_cast<u32>(reinterpret_cast<uintptr_t>(entry_points)));

      // ====================================================================
      // FAST PATH: Entry Points Array (Direct PC -> Code Lookup)
      // ====================================================================
      // Entry points is indexed by: ((feature_flags << 30) | (pc >> 2))
      // Contains direct pointers to compiled code (u8*)

      // Register allocation:
      // R10 = entry_points base
      // R11 = feature_flags
      // DISPATCHER_PC = PC (set after CoreTiming::Advance)
      // R14 = final code pointer

      // Load feature flags
      LDR(R11, PPC_REG, PPCSTATE_OFF(feature_flags));

      // Compute index: ((feature_flags << 30) | (pc >> 2))
      LSL(R11, R11, 30);         // feature_flags << 30
      LSR(R6, DISPATCHER_PC, 2); // pc >> 2 (scratch in R6)
      ORR(R11, R11, R6);         // combined index

      // Load entry points base
      MOVI2R(R10, (u32)entry_points);

      // Load code pointer: entry_points[index]
      // Each pointer is 4 bytes on ARM32
      LSL(R11, R11, 2);     // index * sizeof(u8*)
      LDR(R14, R10, R11);   // R14 = code pointer

      // Guard against NULL
      CMP(R14, 0);
      FixupBranch not_found = B_CC(CC_EQ);

      // Jump to compiled code
      B(R14);

      SetJumpTarget(not_found);
    }
    else if (fast_map_fallback)
    {
      // ====================================================================
      // FALLBACK PATH: Fast Block Map (PC -> JitBlock* -> Code Lookup)
      // ====================================================================
      // Fast block map is indexed by: ((pc >> 2) & FAST_BLOCK_MAP_FALLBACK_MASK)
      // Contains pointers to JitBlock structures, not direct code pointers
      // We must verify PC and feature_flags match, then extract normalEntry

	  LogNumFromJIT("GenerateAsm(): fast_map_fallback",
	  static_cast<u32>(reinterpret_cast<uintptr_t>(fast_map_fallback)));

      // Register allocation:
      // R10 = fast_map_fallback base
      // R11 = JitBlock* (loaded from map)
      // R6  = scratch for comparisons
      // R14 = final code pointer

      // Compute index: (PC >> 2) & FAST_BLOCK_MAP_FALLBACK_MASK
      MOV(R6, DISPATCHER_PC);          // copy PC to scratch (R6)
      LSR(R6, R6, 2);                  // PC >> 2

      // Mask to get array index
      MOVI2R(R11, JitBaseBlockCache::FAST_BLOCK_MAP_FALLBACK_MASK);
      AND(R6, R6, R11);                // index = (PC >> 2) & mask

      // Load fast_map_fallback base
      MOVI2R(R10, (u32)fast_map_fallback);

      // Load JitBlock* from array: block = fast_map_fallback[index]
      LSL(R6, R6, 2);                  // index * sizeof(JitBlock*)
      LDR(R11, R10, R6);               // R11 = JitBlock*

      // Check if block is NULL
      CMP(R11, 0);
      FixupBranch not_found = B_CC(CC_EQ);

      // Block exists - verify it matches our PC and feature flags
      static_assert(offsetof(JitBlockData, feature_flags) + 4 ==
                    offsetof(JitBlockData, effectiveAddress),
                    "JitBlock layout assumption for efficient access");

      // Load block->feature_flags and compare
      LDR(R6, R11, offsetof(JitBlockData, feature_flags));
      LDR(R12, PPC_REG, PPCSTATE_OFF(feature_flags));
      CMP(R6, R12);
      FixupBranch feature_flags_mismatch = B_CC(CC_NEQ);

      // Compare block->effectiveAddress to DISPATCHER_PC
      LDR(R6, R11, offsetof(JitBlockData, effectiveAddress));
      CMP(R6, DISPATCHER_PC);
      FixupBranch pc_mismatch = B_CC(CC_NEQ);

      // Load entry point
      LDR(R14, R11, offsetof(JitBlockData, normalEntry));
      CMP(R14, 0);
      FixupBranch null_entry = B_CC(CC_EQ);

      // Jump to compiled code
	  LogRegFromJIT("GenerateAsm(): Jumping to compiled block for", R6);
      B(R14);

      // All failure paths lead to compilation
      SetJumpTarget(not_found);
      SetJumpTarget(pc_mismatch);
      SetJumpTarget(feature_flags_mismatch);
      SetJumpTarget(null_entry);
    }
  }

  // Write DISPATCHER_PC to PPCSTATE.pc for coherence before any C path
  LogRegFromJIT("PPC_REG before STR(pc)", PPC_REG);
  LogRegFromJIT("DISPATCHER_PC before STR(pc)", DISPATCHER_PC);

  STR(DISPATCHER_PC, PPC_REG, PPCSTATE_OFF(pc));
	LogRegFromJIT("GenerateAsm(): Writing DISPATCHER_PC before any c path:", DISPATCHER_PC);

  // ============================================================================
  // Cache Miss - Compile New Block
  // ============================================================================
  // Failure, fallback to the C++ dispatcher for calling the JIT.

  ResetStack();

  // Prepare arguments for JitArmTrampoline(JitBase& jit, u32 em_address)
  uintptr_t jit_ptr = reinterpret_cast<uintptr_t>(this);

  LogRegFromJIT("GenerateAsm(): Calling trampoline with PC", DISPATCHER_PC);

  // Load arguments
  // R0 = &m_jit (pointer to JitBase object)
  MOVI2R(R0, static_cast<u32>(jit_ptr), /*optimize=*/false);

  // R1 = DISPATCHER_PC (authoritative em address to compile)
  MOV(R1, DISPATCHER_PC);

  // Call trampoline to compile block
  MOVI2R(R14, (u32)&JitArmTrampoline, false);
  BLX(R14);

  {
	auto tmp_pc = gpr.GetScopedReg();
	LDR(tmp_pc, PPC_REG, PPCSTATE_OFF(pc));
	LogRegFromJIT("WriteExceptionExit: PC immediately after CheckExceptions", tmp_pc);
  }

  {
	auto tmp_pc = gpr.GetScopedReg();
	LDR(tmp_pc, PPC_REG, PPCSTATE_OFF(npc));
	LogRegFromJIT("WriteExceptionExit: NPC immediately after CheckExceptions", tmp_pc);
  }

  LogRegFromJIT("GenerateAsm(): Returned from trampoline, dispatch pc is currently before reload", DISPATCHER_PC);

  // After jitting, reload DISPATCHER_PC from PPCSTATE.pc and refresh membase
  LDR(DISPATCHER_PC, PPC_REG, PPCSTATE_OFF(pc));

  LogRegFromJIT("GenerateAsm(): Returned from trampoline, reloaded DISPATCHER_PC is", DISPATCHER_PC);

  EmitUpdateMembase();

  // Execute newly compiled block
  B(dispatcher_no_check);

  // ============================================================================
  // Timing Slice End - Handle Downcount Expiration
  // ============================================================================
  SetJumpTarget(bail);
  do_timing = GetCodePtr();

	// Write the current PC out to PPCSTATE
  static_assert(PPCSTATE_OFF(pc) <= 252);
  static_assert(PPCSTATE_OFF(pc) + 4 == PPCSTATE_OFF(npc));
  STR(DISPATCHER_PC, PPC_REG, PPCSTATE_OFF(pc));
  STR(DISPATCHER_PC, PPC_REG, PPCSTATE_OFF(npc));

  LogRegFromJIT("GenerateAsm(): Value at DISPATCHER_PC at Timing Slice End is", DISPATCHER_PC);

  {
	auto tmp2 = gpr.GetScopedReg();
	LDR(tmp2, PPC_REG, PPCSTATE_OFF(pc));
	LogRegFromJIT("GenerateAsm(): Value at PPCSTATE_OFF(pc) after Timing Slice End is", tmp2);
	auto tmp3 = gpr.GetScopedReg();
	LDR(tmp3, PPC_REG, PPCSTATE_OFF(npc));
	LogRegFromJIT("GenerateAsm(): Value at PPCSTATE_OFF(npc) after Timing Slice End is", tmp3);
  }

  // ----------------------------------------------------------------------------
  // CPU State Check - Exit if not running
  // ----------------------------------------------------------------------------
  MOVI2R(R11, (u32)m_system.GetCPU().GetStatePtr());
  LDR(R11, R11);         // Load CPU state
  MVN(R12, 0);           // R12 = 0xFFFFFFFF
  TST(R11, R12);         // Test if state != Running (Running == 0)
  FixupBranch Exit = B_CC(CC_NEQ);

  // ----------------------------------------------------------------------------
  // Start of timing slice: CoreTiming::GlobalAdvance
  // ----------------------------------------------------------------------------
  SetJumpTarget(to_start_of_timing_slice);
  QuickCallFunction(R12, (void*)&CoreTiming::GlobalAdvance);

  // When we've just entered the jit we need to update the membase
  // GlobalAdvance also checks exceptions after which we need to
  // update the membase so it makes sense to do this here.
  EmitUpdateMembase();

  // Load the PC back into DISPATCHER_PC (the exception handler might have changed it)
  LDR(DISPATCHER_PC, PPC_REG, PPCSTATE_OFF(pc));

  LogRegFromJIT("GenerateAsm(): Value at DISPATCHER_PC after GlobalAdvance is", DISPATCHER_PC);

  {
	auto tmp4 = gpr.GetScopedReg();
	LDR(tmp4, PPC_REG, PPCSTATE_OFF(pc));
	LogRegFromJIT("GenerateAsm(): Value at PPCSTATE_OFF(pc) after GlobalAdvance is", tmp4);
  }

  // We can safely assume that downcount >= 1
  B(dispatcher_no_check);

  // ============================================================================
  // Exit Path - Restore Registers and Return
  // ============================================================================
  SetJumpTarget(Exit);
  if (enable_debugging)
  {
    SetJumpTarget(dbg_exit);
    // Commit DISPATCHER_PC to both pc and npc at debug exit (ARM64 STP mirror)
    STR(DISPATCHER_PC, PPC_REG, PPCSTATE_OFF(pc));
    STR(DISPATCHER_PC, PPC_REG, PPCSTATE_OFF(npc));

	{
	  auto tmp5 = gpr.GetScopedReg();
	  LDR(tmp5, PPC_REG, PPCSTATE_OFF(pc));
	  LogRegFromJIT("GenerateAsm(): Value at PPCSTATE_OFF(pc) after Exit path is", tmp5);
	  auto tmp6 = gpr.GetScopedReg();
	  LDR(tmp6, PPC_REG, PPCSTATE_OFF(npc));
	  LogRegFromJIT("GenerateAsm(): Value at PPCSTATE_OFF(npc) after Exit path is", tmp6);
	}
  }

  // Restore stack alignment
  ADD(_SP, _SP, 4);

  // Pop callee-saved registers and return
  POP(9, R4, R5, R6, R7, R8, PPC_REG, R10, R11, _PC);

  // ============================================================================
  // Generate Common Routines (paired load/store, etc.)
  // ============================================================================
  GenerateCommonAsm();

  // ============================================================================
  // Flush Instruction Cache
  // ============================================================================
  FlushIcache();
}

void JitArm::GenerateCommonAsm()
{
	printf("Generating common routines for JIT ARM32\n");
	fflush(stdout);
	// R14 is LR
	// R12 is scratch
	// R11 is scale
	// R10 is the address
	Operand2 mask(3, 1); // ~(Memory::MEMVIEW32_MASK)
	Operand2 arghmask(3, 3); // 0x0C000000
	NEONXEmitter nemit(this);

	const u8* loadPairedIllegal = GetCodePtr();
	BKPT(0x10);

	const u8* loadPairedFloatTwo = GetCodePtr();
	{
		// Check alignment
		TST(R10, 0x7);
		FixupBranch not_aligned = B_CC(CC_NEQ);

		// ALIGNED PATH - Use NEON
		BIC(R10, R10, mask);
		ADD(R10, R10, R8);
		nemit.VLD1(I_32, D0, R10);
		nemit.VREV32(I_8, D0, D0);
		MOV(_PC, _LR);

		// UNALIGNED PATH - Use GPRs
		SetJumpTarget(not_aligned);
		BIC(R10, R10, mask);
		ADD(R10, R10, R8);

		LDR(R12, R10, 0);
		REV(R12, R12);
		VMOV(S0, R12);

		LDR(R12, R10, 4);
		REV(R12, R12);
		VMOV(S1, R12);

		MOV(_PC, _LR);
	}
	const u8* loadPairedFloatOne = GetCodePtr();
	{
		BIC(R10, R10, mask);
		ADD(R10, R10, R8);

		nemit.VLD1(I_32, D0, R10);
		nemit.VREV32(I_8, D0, D0);

		MOV(_PC, _LR);
	}
	const u8* loadPairedU8Two = GetCodePtr();
	{
		BIC(R10, R10, mask);
		ADD(R10, R10, R8);

		LDRB(R12, R10);
		VMOV(S0, R12);

		LDRB(R12, R10, 1);
		VMOV(S1, R12);

		MOVI2R(R10, (u32)&m_dequantizeTableS);
		ADD(R10, R10, R11);
		VLDR(S2, R10, 0);

		VCVT(S0, S0, TO_FLOAT);
		VCVT(S1, S1, TO_FLOAT);

		VMUL(S0, S0, S2);
		VMUL(S1, S1, S2);

		MOV(_PC, _LR);
	}
	const u8* loadPairedU8One = GetCodePtr();
	{
		BIC(R10, R10, mask);
		ADD(R10, R10, R8);

		LDRB(R12, R10);
		VMOV(S0, R12);

		MOVI2R(R10, (u32)&m_dequantizeTableS);
		ADD(R10, R10, R11);
		VLDR(S2, R10, 0);

		VCVT(S0, S0, TO_FLOAT);

		VMUL(S0, S0, S2);

		MOV(_PC, _LR);
	}
	const u8* loadPairedS8Two = GetCodePtr();
	{
		BIC(R10, R10, mask);
		ADD(R10, R10, R8);

		LDRSB(R12, R10);
		VMOV(S0, R12);

		LDRSB(R12, R10, 1);
		VMOV(S1, R12);

		MOVI2R(R10, (u32)&m_dequantizeTableS);
		ADD(R10, R10, R11);
		VLDR(S2, R10, 0);

		VCVT(S0, S0, TO_FLOAT | IS_SIGNED);
		VCVT(S1, S1, TO_FLOAT | IS_SIGNED);

		VMUL(S0, S0, S2);
		VMUL(S1, S1, S2);

		MOV(_PC, _LR);
	}
	const u8* loadPairedS8One = GetCodePtr();
	{
		BIC(R10, R10, mask);
		ADD(R10, R10, R8);

		LDRSB(R12, R10);
		VMOV(S0, R12);

		MOVI2R(R10, (u32)&m_dequantizeTableS);
		ADD(R10, R10, R11);
		VLDR(S2, R10, 0);

		VCVT(S0, S0, TO_FLOAT | IS_SIGNED);

		VMUL(S0, S0, S2);

		MOV(_PC, _LR);
	}
	const u8* loadPairedU16Two = GetCodePtr();
	{
		BIC(R10, R10, mask);
		ADD(R10, R10, R8);

		LDRH(R12, R10);
		REV16(R12, R12);
		VMOV(S0, R12);

		LDRH(R12, R10, 2);
		REV16(R12, R12);
		VMOV(S1, R12);

		MOVI2R(R10, (u32)&m_dequantizeTableS);
		ADD(R10, R10, R11);
		VLDR(S2, R10, 0);

		VCVT(S0, S0, TO_FLOAT);
		VCVT(S1, S1, TO_FLOAT);

		VMUL(S0, S0, S2);
		VMUL(S1, S1, S2);

		MOV(_PC, _LR);
	}
	const u8* loadPairedU16One = GetCodePtr();
	{
		BIC(R10, R10, mask);
		ADD(R10, R10, R8);

		LDRH(R12, R10);
		REV16(R12, R12);
		VMOV(S0, R12);

		MOVI2R(R10, (u32)&m_dequantizeTableS);
		ADD(R10, R10, R11);
		VLDR(S2, R10, 0);

		VCVT(S0, S0, TO_FLOAT);

		VMUL(S0, S0, S2);
		MOV(_PC, _LR);
	}
	const u8* loadPairedS16Two = GetCodePtr();
	{
		BIC(R10, R10, mask);
		ADD(R10, R10, R8);

		LDRH(R12, R10);
		REV16(R12, R12);
		SXTH(R12, R12);
		VMOV(S0, R12);

		LDRH(R12, R10, 2);
		REV16(R12, R12);
		SXTH(R12, R12);
		VMOV(S1, R12);

		MOVI2R(R10, (u32)&m_dequantizeTableS);
		ADD(R10, R10, R11);
		VLDR(S2, R10, 0);

		VCVT(S0, S0, TO_FLOAT | IS_SIGNED);
		VCVT(S1, S1, TO_FLOAT | IS_SIGNED);

		VMUL(S0, S0, S2);
		VMUL(S1, S1, S2);

		MOV(_PC, _LR);
	}
	const u8* loadPairedS16One = GetCodePtr();
	{
		BIC(R10, R10, mask);
		ADD(R10, R10, R8);

		LDRH(R12, R10);

		MOVI2R(R10, (u32)&m_dequantizeTableS);
		ADD(R10, R10, R11);
		VLDR(S2, R10, 0);

		REV16(R12, R12);
		SXTH(R12, R12);
		VMOV(S0, R12);
		VCVT(S0, S0, TO_FLOAT | IS_SIGNED);

		VMUL(S0, S0, S2);
		MOV(_PC, _LR);
	}

	paired_load_quantized = reinterpret_cast<const u8**>(const_cast<u8*>(AlignCode16()));
	ReserveCodeSpace(16 * sizeof(u8*));

	paired_load_quantized[0] = loadPairedFloatTwo;
	paired_load_quantized[1] = loadPairedIllegal;
	paired_load_quantized[2] = loadPairedIllegal;
	paired_load_quantized[3] = loadPairedIllegal;
	paired_load_quantized[4] = loadPairedU8Two;
	paired_load_quantized[5] = loadPairedU16Two;
	paired_load_quantized[6] = loadPairedS8Two;
	paired_load_quantized[7] = loadPairedS16Two;

	paired_load_quantized[8] = loadPairedFloatOne;
	paired_load_quantized[9] = loadPairedIllegal;
	paired_load_quantized[10] = loadPairedIllegal;
	paired_load_quantized[11] = loadPairedIllegal;
	paired_load_quantized[12] = loadPairedU8One;
	paired_load_quantized[13] = loadPairedU16One;
	paired_load_quantized[14] = loadPairedS8One;
	paired_load_quantized[15] = loadPairedS16One;

	// Stores
	const u8* storePairedIllegal = GetCodePtr();
		BKPT(0x21);
	const u8* storePairedFloat = GetCodePtr();
	{
		// Check for MMIO/uncached memory
		TST(R10, arghmask);
		FixupBranch argh = B_CC(CC_NEQ);

		// **FAST PATH: Check 8-byte alignment**
		TST(R10, 0x7);
		FixupBranch not_aligned = B_CC(CC_NEQ);

		// ============================================
		// ALIGNED PATH - Use NEON (fastest)
		// ============================================
		BIC(R10, R10, mask);
		ADD(R10, R10, R8);
		nemit.VREV32(I_8, D0, D0);
		nemit.VST1(I_32, D0, R10);  // 2-3 cycles
		MOV(_PC, _LR);

		// ============================================
		// UNALIGNED PATH - Use GPRs (still fast)
		// ============================================
		SetJumpTarget(not_aligned);
		BIC(R10, R10, mask);
		ADD(R10, R10, R8);

		// Extract and byte-swap using GPRs (~6-8 cycles)
		VMOV(R12, S0);
		REV(R12, R12);
		STR(R12, R10, 0);

		VMOV(R12, S1);
		REV(R12, R12);
		STR(R12, R10, 4);

		MOV(_PC, _LR);

		// ============================================
		// MMIO PATH - Use slow helper
		// ============================================
		SetJumpTarget(argh);
		PUSH(5, R0, R1, R2, R3, _LR);
		MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetMMU()));
		VMOV(R1, S0);
		VMOV(R2, S1);
		MOV(R3, R10);
		MOVI2R(R12, reinterpret_cast<u32>(&WriteDual32));
		BLX(R12);
		POP(5, R0, R1, R2, R3, _PC);
	}
	const u8* storePairedU8 = GetCodePtr();
	{
		// R10 is the addr
		// R11 is the scale
		// R12 is scratch
		// S0, S1 is the values
		PUSH(5, R0, R1, R2, R3, _LR);

		MOVI2R(R12, (u32)&m_quantizeTableS);
		ADD(R12, R12, R11);
		VLDR(S2, R12, 0);
		VMUL(S0, S0, S2);
		VMUL(S1, S1, S2);

		VCVT(S0, S0, TO_INT | ROUND_TO_ZERO);
		VCVT(S1, S1, TO_INT | ROUND_TO_ZERO);

		MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetMMU()));
		VMOV(R1, S0);
		VMOV(R2, S1);
		MOV(R3, R10);
		MOVI2R(R12, reinterpret_cast<u32>(&WriteDual8));
		BLX(R12);

		POP(5, R0, R1, R2, R3, _PC);

		//B(_LR); // ? TODO 32
	}
	const u8* storePairedS8 = GetCodePtr();
	{
		// R10 is the addr
		// R11 is the scale
		// R12 is scratch
		// S0, S1 is the values
		PUSH(5, R0, R1, R2, R3, _LR);

		MOVI2R(R12, (u32)&m_quantizeTableS);
		ADD(R12, R12, R11);
		VLDR(S2, R12, 0);
		VMUL(S0, S0, S2);
		VMUL(S1, S1, S2);

		VCVT(S0, S0, TO_INT | ROUND_TO_ZERO | IS_SIGNED);
		VCVT(S1, S1, TO_INT | ROUND_TO_ZERO | IS_SIGNED);

		MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetMMU()));
		VMOV(R1, S0);
		VMOV(R2, S1);
		MOV(R3, R10);
		MOVI2R(R12, reinterpret_cast<u32>(&WriteDual8));
		BLX(R12);

		POP(5, R0, R1, R2, R3, _PC);

		//B(_LR); // TODO 32
	}
	const u8* storePairedU16 = GetCodePtr();
	{
		PUSH(5, R0, R1, R2, R3, _LR);

		MOVI2R(R12, (u32)&m_quantizeTableS);
		ADD(R12, R12, R11);
		VLDR(S2, R12, 0);
		VMUL(S0, S0, S2);
		VMUL(S1, S1, S2);

		VCVT(S0, S0, TO_INT | ROUND_TO_ZERO);
		VCVT(S1, S1, TO_INT | ROUND_TO_ZERO);

		MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetMMU()));
		VMOV(R1, S0);
		VMOV(R2, S1);
		MOV(R3, R10);
		MOVI2R(R12, reinterpret_cast<u32>(&WriteDual16));
		BLX(R12);

		POP(5, R0, R1, R2, R3, _PC);

		//B(_LR); //TODO 32
	}
	const u8* storePairedS16 = GetCodePtr();
	{
		PUSH(5, R0, R1, R2, R3, _LR);

		MOVI2R(R12, (u32)&m_quantizeTableS);
		ADD(R12, R12, R11);
		VLDR(S2, R12, 0);
		VMUL(S0, S0, S2);
		VMUL(S1, S1, S2);

		VCVT(S0, S0, TO_INT | ROUND_TO_ZERO | IS_SIGNED);
		VCVT(S1, S1, TO_INT | ROUND_TO_ZERO | IS_SIGNED);

		MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetMMU()));
		VMOV(R1, S0);
		VMOV(R2, S1);
		MOV(R3, R10);
		MOVI2R(R12, reinterpret_cast<u32>(&WriteDual16));
		BLX(R12);

		POP(5, R0, R1, R2, R3, _PC);

		//B(_LR); // TODO webos
	}
	const u8* storeSingleIllegal = GetCodePtr();
	BKPT(0x27);
	const u8* storeSingleFloat = GetCodePtr();
	{
		TST(R10, arghmask);
		FixupBranch argh = B_CC(CC_NEQ);
		BIC(R10, R10, mask);
		ADD(R10, R10, R8);

		VMOV(R12, S0);
		REV(R12, R12);
		STR(R12, R10);
		MOV(_PC, _LR);

		SetJumpTarget(argh);

		PUSH(5, R0, R1, R2, R3, _LR);
		MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetMMU())); // MMU pointer
		VMOV(R1, S0); // value
		MOV(R2, R10); // address
		MOVI2R(R10, reinterpret_cast<u32>(&PowerPC::WriteFromJit<u32>));
		BLX(R10);

		POP(5, R0, R1, R2, R3, _PC);
	}
	const u8* storeSingleU8 = GetCodePtr();  // Used by MKWii
	{
		MOVI2R(R12, (u32)&m_quantizeTableS);
		ADD(R12, R12, R11);
		VLDR(S2, R12, 0);
		VMUL(S0, S0, S2);

		TST(R10, arghmask);
		FixupBranch argh = B_CC(CC_NEQ);
		BIC(R10, R10, mask);
		ADD(R10, R10, R8);

		VCVT(S0, S0, TO_INT | ROUND_TO_ZERO);
		VMOV(R12, S0);
		STRB(R12, R10);
		MOV(_PC, _LR);

		SetJumpTarget(argh);

		PUSH(5, R0, R1, R2, R3, _LR);
		MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetMMU())); // MMU pointer
		VMOV(R1, S0); // value
		MOV(R2, R10); // address
		MOVI2R(R10, reinterpret_cast<u32>(&PowerPC::WriteFromJit<u8>));
		BLX(R10);

		POP(5, R0, R1, R2, R3, _PC);
	}
	const u8* storeSingleS8 = GetCodePtr();
	{
		MOVI2R(R12, (u32)&m_quantizeTableS);
		ADD(R12, R12, R11);
		VLDR(S2, R12, 0);
		VMUL(S0, S0, S2);

		TST(R10, arghmask);
		FixupBranch argh = B_CC(CC_NEQ);
		BIC(R10, R10, mask);
		ADD(R10, R10, R8);

		VCVT(S0, S0, TO_INT | ROUND_TO_ZERO | IS_SIGNED);
		VMOV(R12, S0);
		STRB(R12, R10);
		MOV(_PC, _LR);

		SetJumpTarget(argh);

		PUSH(5, R0, R1, R2, R3, _LR);
		MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetMMU())); // MMU pointer
		VMOV(R1, S0); // value
		MOV(R2, R10); // address
		MOVI2R(R10, reinterpret_cast<u32>(&PowerPC::WriteFromJit<u8>));
		BLX(R10);

		POP(5, R0, R1, R2, R3, _PC);
	}
	const u8* storeSingleU16 = GetCodePtr();  // Used by MKWii
	{
		MOVI2R(R12, (u32)&m_quantizeTableS);
		ADD(R12, R12, R11);
		VLDR(S2, R12, 0);
		VMUL(S0, S0, S2);

		TST(R10, arghmask);
		FixupBranch argh = B_CC(CC_NEQ);
		BIC(R10, R10, mask);
		ADD(R10, R10, R8);

		VCVT(S0, S0, TO_INT | ROUND_TO_ZERO);
		VMOV(R12, S0);
		REV16(R12, R12);
		STRH(R12, R10);
		MOV(_PC, _LR);

		SetJumpTarget(argh);

		PUSH(5, R0, R1, R2, R3, _LR);
		MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetMMU())); // MMU pointer
		VMOV(R1, S0); // value
		MOV(R2, R10); // address
		MOVI2R(R10, reinterpret_cast<u32>(&PowerPC::WriteFromJit<u16>));
		BLX(R10);

		POP(5, R0, R1, R2, R3, _PC);
	}
	const u8* storeSingleS16 = GetCodePtr();
	{
		MOVI2R(R12, (u32)&m_quantizeTableS);
		ADD(R12, R12, R11);
		VLDR(S2, R12, 0);
		VMUL(S0, S0, S2);

		TST(R10, arghmask);
		FixupBranch argh = B_CC(CC_NEQ);
		BIC(R10, R10, mask);
		ADD(R10, R10, R8);

		VCVT(S0, S0, TO_INT | ROUND_TO_ZERO | IS_SIGNED);
		VMOV(R12, S0);
		REV16(R12, R12);
		STRH(R12, R10);
		MOV(_PC, _LR);

		SetJumpTarget(argh);

		PUSH(5, R0, R1, R2, R3, _LR);
		MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetMMU())); // MMU pointer
		VMOV(R1, S0); // value
		MOV(R2, R10); // address
		MOVI2R(R10, reinterpret_cast<u32>(&PowerPC::WriteFromJit<u16>));
		BLX(R10);

		POP(5, R0, R1, R2, R3, _PC);
	}

	paired_store_quantized = reinterpret_cast<const u8**>(const_cast<u8*>(AlignCode16()));
	ReserveCodeSpace(16 * sizeof(u8*));

	paired_store_quantized[0] = storePairedFloat;
	paired_store_quantized[1] = storePairedIllegal;
	paired_store_quantized[2] = storePairedIllegal;
	paired_store_quantized[3] = storePairedIllegal;
	paired_store_quantized[4] = storePairedU8;
	paired_store_quantized[5] = storePairedU16;
	paired_store_quantized[6] = storePairedS8;
	paired_store_quantized[7] = storePairedS16;

	paired_store_quantized[8] = storeSingleFloat;
	paired_store_quantized[9] = storeSingleIllegal;
	paired_store_quantized[10] = storeSingleIllegal;
	paired_store_quantized[11] = storeSingleIllegal;
	paired_store_quantized[12] = storeSingleU8;
	paired_store_quantized[13] = storeSingleU16;
	paired_store_quantized[14] = storeSingleS8;
	paired_store_quantized[15] = storeSingleS16;

	m_increment_profile_counter = AlignCode16();

	nemit.VLD1(I_64, D0, R0); // Start
	ADD(R0, R0, 8);
	nemit.VLD1(I_64, D1, R0); // End
	ADD(R0, R0, 8);
	nemit.VLD1(I_64, D2, R0); // Counter
	nemit.VSUB(I_64, D1, D1, D0);
	nemit.VADD(I_64, D2, D2, D1);
	nemit.VST1(I_64, D2, R0);
	MOV(_PC, _LR);
}

void JitArm::EmitUpdateMembase()
{
  auto W = gpr.GetScopedReg();

  auto& memory = m_system.GetMemory();

  // Default: logical base
  MOVI2R(MEM_REG,
         static_cast<u32>(reinterpret_cast<uintptr_t>(
           jo.fastmem ? memory.GetLogicalBase()
                      : memory.GetLogicalPageMappingsBase())));

  // Load MSR and test DR (bit 27)
  LDR(W, PPC_REG, PPCSTATE_OFF(msr));
  // Use ST_LSR, not ShiftType::LSR, and reuse W
  TST(W, Operand2(W, ST_LSR, 27));

  // If DR != 0, overwrite MEM_REG with physical base
  FixupBranch dr_clear = B_CC(CC_EQ);
  MOVI2R(MEM_REG,
         static_cast<u32>(reinterpret_cast<uintptr_t>(
           jo.fastmem ? memory.GetPhysicalBase()
                      : memory.GetPhysicalPageMappingsBase())));
  B_CC(CC_AL);

  SetJumpTarget(dr_clear);
  // DR == 0 → MEM_REG already holds logical base

  // Commit to PPCState
  STR(MEM_REG, PPC_REG, PPCSTATE_OFF(mem_ptr));
}
