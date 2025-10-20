// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

// ========================
// See comments in Jit.cpp.
// ========================

// Mystery: Capcom vs SNK 800aa278

// CR flags approach:
//   * Store that "N+Z flag contains CR0" or "S+Z flag contains CR3".
//   * All flag altering instructions flush this
//   * A flush simply does a conditional write to the appropriate CRx.
//   * If flag available, branch code can become absolutely trivial.

// Settings
// ----------
#pragma once

#include "Common/BitSet.h"
#include "Core/PowerPC/CPUCoreBase.h"
#include "Core/PowerPC/PPCAnalyst.h"
#include "Core/PowerPC/JitArm32/JitArmCache.h"
#include "Core/PowerPC/JitArm32/JitFPRCache.h"
#include "Core/PowerPC/JitArm32/JitRegCache.h"
#include "Core/PowerPC/JitArmCommon/BackPatch.h"
#include "Core/PowerPC/JitCommon/JitBase.h"
#include "Core/PowerPC/PowerPC.h"

#include <cstddef>
#include <Common/RangeSizeSet.h>

extern "C" void JitArmTrampoline(JitBase& jit, u32 em_address);
//extern "C" void LogRegHelper(const char* msg, u32 value);

// Enable or disable JIT debug logging here
#ifndef JIT_DEBUG
#define JIT_DEBUG 1
#endif

#ifdef JIT_DEBUG
  #ifndef JIT_LOG
  #define JIT_LOG(fmt, ...) \
    do { \
        printf(fmt "\n", ##__VA_ARGS__); \
        fflush(stdout); \
    } while (0)
  #endif
  #define JIT_LOG_NUM(msg, num) \
    do { \
        LogNumFromJIT(msg, num); \
    } while (0)
  #define JIT_LOG_MSG(msg) \
    do { \
        LogNumFromJIT(msg, 0xFFFFFFFF); \
    } while (0)
  #define JIT_LOG_REG(msg, reg) \
    do { \
        LogRegFromJIT(msg, reg); \
    } while (0)
#else
  #define JIT_LOG(fmt, ...)       do {} while (0)
  #define JIT_LOG_NUM(msg, num)   do {} while (0)
  #define JIT_LOG_MSG(msg)   	  do {} while (0)
  #define JIT_LOG_REG(msg, reg)   do {} while (0)
#endif

class HostDisassembler;

class JitArm : public JitBase, public ArmGen::ARMCodeBlock, public CommonAsmRoutinesBase
{
public:
  explicit JitArm(Core::System& system);

	~JitArm() override;

	void Init();
	void Shutdown();

	// Jit!

  void Jit(u32 em_address) override;
  void Jit(u32 em_address, bool clear_cache_and_retry_on_failure);
	//bool DoJit(u32 em_address, PPCAnalyst::CodeBuffer *code_buf, JitBlock *b);
	bool DoJit(u32 em_address, JitBlock *b, u32 nextPC);

	void EraseSingleBlock(const JitBlock& block) override;
	std::vector<MemoryStats> GetMemoryStats() const override;

	std::size_t DisassembleNearCode(const JitBlock& block, std::ostream& stream) const override;
	std::size_t DisassembleFarCode(const JitBlock& block, std::ostream& stream) const override;

	// Finds a free memory region
    // Returns false if no free memory region can be found.
    bool SetEmitterStateToFreeCodeRegion();

	JitBaseBlockCache *GetBlockCache() { return &blocks; }

	bool HandleFault(uintptr_t access_address, SContext* ctx) override;

	void Trace();

	void ClearCache();

	const char *GetName() const
	{
		return "JITARM";
	}

	CommonAsmRoutinesBase* GetAsmRoutines() override { return this; }

	// Run!
	void Run();
	void SingleStep();

	// Utilities for safer jumping > 32mb
	void SafeB(const void* fnptr, bool optimize = false);
	void SafeSetJumpTarget(const ArmGen::FixupBranch& branch);

	// Exits
	void
    WriteExit(u32 destination, bool LK = false, u32 exit_address_after_return = 0,
            ArmGen::ARMReg exit_address_after_return_reg = ArmGen::ARMReg::INVALID_REG);
    void
    WriteExit(ArmGen::ARMReg dest, bool LK = false, u32 exit_address_after_return = 0,
            ArmGen::ARMReg exit_address_after_return_reg = ArmGen::ARMReg::INVALID_REG);
	void WriteExitDestInR(ArmGen::ARMReg Reg);
	void WriteRfiExitDestInR(ArmGen::ARMReg Reg);
	void WriteConditionalExceptionExit(int exception, u32 increment_sp_on_exit = 0);
	void WriteConditionalExceptionExit(int exception, ArmGen::ARMReg temp_gpr, ArmGen::ARMReg /* temp_fpr */,
                                           u32 increment_sp_on_exit);
	void FakeLKExit(u32 exit_address_after_return, ArmGen::ARMReg exit_address_after_return_reg);
    void WriteExceptionExit(u32 destination, bool only_external = false,
		bool always_exception = false);
	void WriteExceptionExit(ArmGen::ARMReg dest, bool only_external = false,
			bool always_exception = false);
	void WriteBLRExit(ArmGen::ARMReg dest);
	void WriteCallInterpreter(UGeckoInstruction _inst);
	void Cleanup();

	void FreeRanges();
	void GenerateAsmAndResetFreeMemoryRanges();

	// AsmRoutines
  void GenerateAsm();
  void GenerateCommonAsm();
	void EmitUpdateMembase();

	void ComputeRC0(ArmGen::ARMReg reg);
	void ComputeRC0(u32 imm);

	void ComputeRC(ArmGen::ARMReg value, int cr = 0);
	void ComputeRC(s32 value, int cr);

	void ComputeCarry();
	void ComputeCarry(bool Carry);
	void GetCarryAndClear(ArmGen::ARMReg reg);
	void FinalizeCarry(ArmGen::ARMReg reg);
	void FlushCarry();

	void SafeStoreFromReg(s32 dest, u32 value, s32 offsetReg, int accessSize, s32 offset);
	void SafeLoadToReg(ArmGen::ARMReg dest, s32 addr, s32 offsetReg, int accessSize, s32 offset, bool signExtend, bool reverse, bool update);

	void CompileInstruction(PPCAnalyst::CodeOp& op);

	// OPCODES
	using Instruction = void (JitArm::*)(UGeckoInstruction);
	void FallBackToInterpreter(UGeckoInstruction _inst);
	void DoNothing(UGeckoInstruction _inst);
	void HLEFunction(UGeckoInstruction _inst);

	void DynaRunTable4(UGeckoInstruction _inst);
	void DynaRunTable19(UGeckoInstruction _inst);
	void DynaRunTable31(UGeckoInstruction _inst);
	void DynaRunTable59(UGeckoInstruction _inst);
	void DynaRunTable63(UGeckoInstruction _inst);

	// Breakin shit
	void Break(UGeckoInstruction _inst);
	// Branch
	void bx(UGeckoInstruction _inst);
	void bcx(UGeckoInstruction _inst);
	void bclrx(UGeckoInstruction _inst);
	void sc(UGeckoInstruction _inst);
	void rfi(UGeckoInstruction _inst);
	void bcctrx(UGeckoInstruction _inst);

	// Integer
	void arith(UGeckoInstruction _inst);
	void addic(UGeckoInstruction inst);

	void addix(UGeckoInstruction inst);
	void addex(UGeckoInstruction _inst);
	void subfic(UGeckoInstruction _inst);
	void cntlzwx(UGeckoInstruction _inst);
	void cmp (UGeckoInstruction _inst);
	void cmpl(UGeckoInstruction _inst);
	void cmpi(UGeckoInstruction _inst);
	void cmpli(UGeckoInstruction _inst);
	void negx(UGeckoInstruction _inst);
	void mulhwux(UGeckoInstruction _inst);
	void rlwimix(UGeckoInstruction _inst);
	void rlwinmx(UGeckoInstruction _inst);
	void rlwnmx(UGeckoInstruction _inst);
	void rlwinmx_internal(UGeckoInstruction inst, u32 sh);

	void srawix(UGeckoInstruction _inst);
	void extshx(UGeckoInstruction inst);
	void extsbx(UGeckoInstruction inst);

	// System Registers
	void mtmsr(UGeckoInstruction _inst);
	void mfmsr(UGeckoInstruction _inst);
	void mtspr(UGeckoInstruction _inst);
	void mfspr(UGeckoInstruction _inst);
	void mftb(UGeckoInstruction _inst);
	void mcrf(UGeckoInstruction _inst);
	void mtsr(UGeckoInstruction _inst);
	void mfsr(UGeckoInstruction _inst);
	void twx(UGeckoInstruction _inst);
	void mtfsfx(UGeckoInstruction inst);
	void UpdateFPExceptionSummary(ArmGen::ARMReg fpscr);
	void UpdateRoundingMode();

	// LoadStore
	void stX(UGeckoInstruction _inst);
	void lXX(UGeckoInstruction _inst);
	void lmw(UGeckoInstruction _inst);
	void stmw(UGeckoInstruction _inst);

	void icbi(UGeckoInstruction _inst);
	void dcbst(UGeckoInstruction _inst);

	// Floating point
	void fabsx(UGeckoInstruction _inst);
	void fnabsx(UGeckoInstruction _inst);
	void fnegx(UGeckoInstruction _inst);
	void faddsx(UGeckoInstruction _inst);
	void faddx(UGeckoInstruction _inst);
	void fsubsx(UGeckoInstruction _inst);
	void fsubx(UGeckoInstruction _inst);
	void fmulsx(UGeckoInstruction _inst);
	void fmulx(UGeckoInstruction _inst);
	void fmrx(UGeckoInstruction _inst);
	void fmaddsx(UGeckoInstruction _inst);
	void fmaddx(UGeckoInstruction _inst);
	void fctiwx(UGeckoInstruction _inst);
	void fctiwzx(UGeckoInstruction _inst);
	void fnmaddx(UGeckoInstruction _inst);
	void fnmaddsx(UGeckoInstruction _inst);
	void fresx(UGeckoInstruction _inst);
	void fselx(UGeckoInstruction _inst);
	void frsqrtex(UGeckoInstruction _inst);

	// Floating point loadStore
	void lfXX(UGeckoInstruction _inst);
	void stfXX(UGeckoInstruction _inst);

	// Paired Singles
	void ps_add(UGeckoInstruction _inst);
	void ps_div(UGeckoInstruction _inst);
	void ps_res(UGeckoInstruction _inst);
	void ps_sum0(UGeckoInstruction _inst);
	void ps_sum1(UGeckoInstruction _inst);
	void ps_madd(UGeckoInstruction _inst);
	void ps_nmadd(UGeckoInstruction _inst);
	void ps_msub(UGeckoInstruction _inst);
	void ps_nmsub(UGeckoInstruction _inst);
	void ps_madds0(UGeckoInstruction _inst);
	void ps_madds1(UGeckoInstruction _inst);
	void ps_sub(UGeckoInstruction _inst);
	void ps_mul(UGeckoInstruction _inst);
	void ps_muls0(UGeckoInstruction _inst);
	void ps_muls1(UGeckoInstruction _inst);
	void ps_merge00(UGeckoInstruction _inst);
	void ps_merge01(UGeckoInstruction _inst);
	void ps_merge10(UGeckoInstruction _inst);
	void ps_merge11(UGeckoInstruction _inst);
	void ps_mr(UGeckoInstruction _inst);
	void ps_neg(UGeckoInstruction _inst);
	void ps_abs(UGeckoInstruction _inst);
	void ps_nabs(UGeckoInstruction _inst);
	void ps_rsqrte(UGeckoInstruction _inst);
	void ps_sel(UGeckoInstruction _inst);

	// LoadStore paired
	void psq_l(UGeckoInstruction _inst);
	void psq_lx(UGeckoInstruction _inst);
	void psq_st(UGeckoInstruction _inst);
	void psq_stx(UGeckoInstruction _inst);

	template <bool condition>
	void WriteBranchWatch(u32 origin, u32 destination, UGeckoInstruction inst,
                              ArmGen::ARMReg reg_a, ArmGen::ARMReg reg_b,
                              BitSet32 /*gpr_caller_save*/, BitSet32 /*fpr_caller_save*/);

	void MMIOWriteRegToAddr(Core::System& system, MMIO::Mapping* mmio,
                        ArmGen::ARMXEmitter* emit,
                        ArmGen::ARMReg src_reg, u32 address, u32 flags);

	void LogRegFromJIT(const char* msg, ArmGen::ARMReg reg = ArmGen::INVALID_REG);
	void LogNumFromJIT(const char* msg, u32 value = -1);

protected:
	void SetBlockLinkingEnabled(bool enabled);
	void SetOptimizationEnabled(bool enabled);

  // Simple functions to switch between near and far code emitting
	void SwitchToFarCode()
	{
		// Save current (near) emitter state into m_near_code
		m_near_code.SetCodePtr(GetWritableCodePtr(),
													 GetWritableCodeEnd(),
													 HasWriteFailed());

		// Switch emitter to the far buffer state
		SetCodePtr(m_far_code.GetWritableCodePtr(),
							 m_far_code.GetWritableCodeEnd(),
							 m_far_code.HasWriteFailed());

		m_in_far_code = true;
	}

	void SwitchToNearCode()
	{
		// Save current (far) emitter state back into m_far_code
		m_far_code.SetCodePtr(GetWritableCodePtr(),
													GetWritableCodeEnd(),
													HasWriteFailed());

		// Restore emitter to the saved near buffer state
		SetCodePtr(m_near_code.GetWritableCodePtr(),
							 m_near_code.GetWritableCodeEnd(),
							 m_near_code.HasWriteFailed());

		m_in_far_code = false;
	}

	bool IsInFarCode() const { return m_in_far_code; }

	void DoDownCount();
	void ResetStack();
	void IntializeSpeculativeConstants();
	void MSRUpdated(ArmGen::ARMReg msr);

	void Helper_UpdateCR1(ArmGen::ARMReg fpscr, ArmGen::ARMReg temp);

	void SetFPException(ArmGen::ARMReg Reg, u32 Exception);

	ArmGen::FixupBranch JumpIfCRFieldBit(int field, int bit, bool jump_if_set);

	void BeginTimeProfile(JitBlock* b);
	void EndTimeProfile(JitBlock* b);

	bool BackPatch(SContext* ctx);
	bool DisasmLoadStore(const u8* ptr, u32* flags, ArmGen::ARMReg* rD, ArmGen::ARMReg* V1);
	// Initializes the information that backpatching needs
	// This is required so we know the backpatch routine sizes and trouble offsets
	void InitBackpatch();

	// This enum is used for selecting an implementation of EmitBackpatchRoutine.
	enum class MemAccessMode
	{
		// Always calls the slow C++ code. For performance reasons, should generally only be used if
		// the guest address is known in advance and IsOptimizableRAMAddress returns false for it.
		AlwaysSlowAccess,
		// Only emits fast access code. Must only be used if the guest address is known in advance
		// and IsOptimizableRAMAddress returns true for it, otherwise Dolphin will likely crash!
		AlwaysFastAccess,
		// Best in most cases. If backpatching is possible (!emitting_routine && jo.fastmem):
		// Tries to run fast access code, and if that fails, uses backpatching to replace the code
		// with a call to the slow C++ code. Otherwise: Checks whether the fast access code will work,
		// then branches to either the fast access code or the slow C++ code.
		Auto,
	};

	// Returns the trouble instruction offset
	// Zero if it isn't a fastmem routine
	u32 EmitBackpatchRoutine(ARMXEmitter* emit, u32 flags, MemAccessMode mode, bool do_padding,
		ArmGen::ARMReg RS, ArmGen::ARMReg V1 = ArmGen::ARMReg::INVALID_REG);

	// TODO: make void return
	u32 EmitBackpatchRoutine(u32 flags, MemAccessMode mode, ArmGen::ARMReg RS,
                              ArmGen::ARMReg V1 /* now addr */, BitSet32 gprs_to_push = BitSet32(0),
                              BitSet32 fprs_to_push = BitSet32(0), bool emitting_routine = false,
							  ARMXEmitter* emit = nullptr, bool do_padding = false);
	void FlushPPCStateBeforeSlowAccess(ArmGen::ARMReg temp_gpr, ArmGen::ARMReg temp_fpr);

	void ABI_PushCallerGPRsAndAdjustStack_BP(ARMXEmitter* emit);
	void ABI_PopCallerGPRsAndAdjustStack_BP(ARMXEmitter* emit);
	void ABI_PushCallerGPRsAndAdjustStack_Asm();
	void ABI_PopCallerGPRsAndAdjustStack_Asm();

	struct FastmemArea
	{
		const u8* fast_access_code;
		const u8* slow_access_code;
	};

	JitArmBlockCache blocks{*this};
	//JitArmBlockCache blocks;

	// <Fast path fault location, slow path handler location>
	std::map<const u8*, FastmemArea> m_fault_to_handler{};

	// TODO: Make arm specific versions of these, shouldn't be too hard to
	// make it so we allocate some space at the start(?) of code generation
	// and keep the registers in a cache. Will burn this bridge when we get to
	// it.
	ArmGPRCache gpr;
	ArmFPRCache fpr;

	const u8* m_increment_profile_counter;

	// The key is the backpatch flags
	std::map<u32, BackPatchInfo> m_backpatch_info;

	// Backed up when we switch to far code.
	ArmGen::ARMCodeBlock m_near_code;
	ArmGen::ARMCodeBlock m_far_code;
	u8* m_near_code_end = nullptr;
    bool m_near_code_write_failed = false;
	bool m_in_far_code = false;

	// Free‑range set for that buffer
	Common::RangeSizeSet<u8*> m_free_ranges_near;
	Common::RangeSizeSet<u8*> m_free_ranges_far;

  std::unique_ptr<HostDisassembler> m_disassembler;
};

struct JitLogEntry {
  const char* msg;
  u32 value;
};

extern JitLogEntry g_jit_log_buffer[256];
extern std::atomic<u32> g_jit_log_write_idx;
extern std::atomic<u32> g_jit_log_read_idx;
