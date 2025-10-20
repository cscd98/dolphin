// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <string>

#include "Common/CommonTypes.h"
#include "Common/StringUtil.h"

#include "Core/System.h"
#include "Core/HW/Memmap.h"
#include "Core/PowerPC/PowerPC.h"
#include "Core/PowerPC/JitArm32/Jit.h"

using namespace ArmGen;

// This generates some fairly heavy trampolines, but:
// 1) It's really necessary. We don't know anything about the context.
// 2) It doesn't really hurt. Only instructions that access I/O will get these, and there won't be
//    that many of them in a typical program/game.
bool JitArm::DisasmLoadStore(const u8* ptr, u32* flags, ARMReg* rD, ARMReg* V1)
{
	printf("JIT ARM32 Disassembling load/store instruction at %p\n", ptr);
	fflush(stdout);

	u32 inst = *(u32*)ptr;
	u32 prev_inst = *(u32*)(ptr - 4);
	u32 next_inst = *(u32*)(ptr + 4);
	u8 op = (inst >> 20) & 0xFF;
	*rD = (ARMReg)((inst >> 12) & 0xF);

	switch (op)
	{
		case 0b01011000: // STR(imm)
		case 0b01111000: // STR(register)
		{
			*flags |=
				BackPatchInfo::FLAG_STORE |
				BackPatchInfo::FLAG_SIZE_32;
			*rD = (ARMReg)(prev_inst & 0xF);
		}
		break;
		case 0b01011001: // LDR(imm)
		case 0b01111001: // LDR(register)
		{
			*flags |=
				BackPatchInfo::FLAG_LOAD |
				BackPatchInfo::FLAG_SIZE_32;
			// REV
			if ((next_inst & 0x0FFF0FF0) != 0x06BF0F30)
				*flags |= BackPatchInfo::FLAG_REVERSE;
		}
		break;
		case 0b00011101: // LDRH(imm)
		case 0b00011001: // LDRH(register)
		{
			*flags |=
				BackPatchInfo::FLAG_LOAD |
				BackPatchInfo::FLAG_SIZE_16;
			// REV16
			if((next_inst & 0x0FFF0FF0) != 0x06BF0FB0)
				*flags |= BackPatchInfo::FLAG_REVERSE;
		}
		break;
		case 0b01011101: // LDRB(imm)
		case 0b01111101: // LDRB(register)
		{
			*flags |=
				BackPatchInfo::FLAG_LOAD |
				BackPatchInfo::FLAG_SIZE_8;
		}
		break;
		case 0b01011100: // STRB(imm)
		case 0b01111100: // STRB(register)
		{
			*flags |=
				BackPatchInfo::FLAG_STORE |
				BackPatchInfo::FLAG_SIZE_8;
			*rD = (ARMReg)((inst >> 12) & 0xF);
		}
		break;
		case 0b00011100: // STRH(imm)
		case 0b00011000: // STRH(register)
		{
			*flags |=
				BackPatchInfo::FLAG_STORE |
				BackPatchInfo::FLAG_SIZE_16;
			*rD = (ARMReg)(prev_inst & 0xF);
		}
		break;
		default:
		{
			// Could be a floating point loadstore
			u8 op2 = (inst >> 24) & 0xF;
			switch (op2)
			{
			case 0xD: // VLDR/VSTR
			{
				bool load = (inst >> 20) & 1;
				bool single = !((inst >> 8) & 1);

				if (load)
					*flags |= BackPatchInfo::FLAG_LOAD;
				else
					*flags |= BackPatchInfo::FLAG_STORE;

				if (single)
					*flags |= BackPatchInfo::FLAG_SIZE_32;
				else
					*flags |= BackPatchInfo::FLAG_SIZE_64;
				if (single)
				{
					if (!load)
					{
						u32 vcvt = *(u32*)(ptr - 8);
						u32 src_register = vcvt & 0xF;
						src_register |= (vcvt >> 1) & 0x10;
						*rD = (ARMReg)(src_register + D0);
					}
				}
			}
			break;
			case 0x4: // VST1/VLD1
			{
				u32 size = (inst >> 6) & 0x3;
				bool load = (inst >> 21) & 1;
				if (load)
					*flags |= BackPatchInfo::FLAG_LOAD;
				else
					*flags |= BackPatchInfo::FLAG_STORE;


				if (size == 2) // 32bit
				{
					if (load)
					{
						// For 32bit loads we are loading to a temporary
						// So we need to read PC+8,PC+12 to get the two destination registers
						u32 vcvt_1 = *(u32*)(ptr + 8);
						u32 vcvt_2 = *(u32*)(ptr + 12);

						u32 dest_register_1 = (vcvt_1 >> 12) & 0xF;
						dest_register_1 |= (vcvt_1 >> 18) & 0x10;

						u32 dest_register_2 = (vcvt_2 >> 12) & 0xF;
						dest_register_2 |= (vcvt_2 >> 18) & 0x10;

						// Make sure to encode the destination register to something our emitter understands
						*rD = (ARMReg)(dest_register_1 + D0);
						*V1 = (ARMReg)(dest_register_2 + D0);
					}
					else
					{
						// For 32bit stores we are storing from a temporary
						// So we need to check the VCVT at PC-8 for the source register
						u32 vcvt = *(u32*)(ptr - 8);
						u32 src_register = vcvt & 0xF;
						src_register |= (vcvt >> 1) & 0x10;
						*rD = (ARMReg)(src_register + D0);
					}
					*flags |= BackPatchInfo::FLAG_SIZE_32;
				}
				else if (size == 3) // 64bit
				{
					if (load)
					{
						// For 64bit loads we load directly in to the VFP register
						u32 dest_register = (inst >> 12) & 0xF;
						dest_register |= (inst >> 18) & 0x10;
						// Make sure to encode the destination register to something our emitter understands
						*rD = (ARMReg)(dest_register + D0);
					}
					else
					{
						// For 64bit stores we are storing from a temporary
						// Check the previous VREV64 instruction for the real register
						u32 src_register = prev_inst & 0xF;
						src_register |= (prev_inst >> 1) & 0x10;
						*rD = (ARMReg)(src_register + D0);
					}
					*flags |= BackPatchInfo::FLAG_SIZE_64;
				}
			}
			break;
			default:
				printf("Op is 0x%02x\n", op);
				return false;
			break;
			}
		}
	}
	return true;
}

bool JitArm::HandleFault(uintptr_t access_address, SContext* ctx)
{
  printf("JIT ARM32 HandleFault called for access address 0x%08lx at PC 0x%08lx\n",
         (unsigned long)access_address, (unsigned long)ctx->CTX_PC);
  fflush(stdout);

  // Handle BLR stack guard faults
  const uintptr_t stack_guard = reinterpret_cast<uintptr_t>(m_stack_guard);
  if (access_address >= stack_guard && access_address < stack_guard + GUARD_SIZE)
    return HandleStackFault();

  auto& memory = m_system.GetMemory();

  if (memory.IsAddressInFastmemArea(reinterpret_cast<u8*>(access_address)))
  {
    const uintptr_t memory_base = reinterpret_cast<uintptr_t>(
        m_ppc_state.msr.DR ? memory.GetLogicalBase() : memory.GetPhysicalBase());

    if (access_address < memory_base || access_address >= memory_base + 0x1'0000'0000)
    {
      WARN_LOG_FMT(DYNA_REC,
                   "JitArm32 address calculation overflowed! "
                   "PC {:#010x}, access address {:#010x}, memory base {:#010x}, MSR.DR {}",
                   ctx->CTX_PC, access_address, memory_base, m_ppc_state.msr.DR);
    }

    return BackPatch(ctx);
  }

  return false;
}

/*bool JitArm::HandleFault(uintptr_t access_address, SContext* ctx)
{
	printf("JIT ARM32 HandleFault called for access address 0x%08lx at PC 0x%08lx\n",
		(unsigned long)access_address, (unsigned long)ctx->CTX_PC);
	fflush(stdout);

	// Check if this is a fastmem access
  auto& memory = m_system.GetMemory();
  if (!memory.IsAddressInFastmemArea(reinterpret_cast<const u8*>(access_address)))
  {
    printf("JIT ARM32: Not in fastmem area\n");
    fflush(stdout);
    return false;  // Not our fault
  }


	if (access_address < (uintptr_t)m_system.GetMemory().GetPhysicalBase())
		PanicAlertFmtT("Exception handler - access below memory space. 0x{0:08x}", access_address);

	return BackPatch(ctx);
}*/

bool JitArm::BackPatch(SContext* ctx)
{
	printf("JIT ARM32 Backpatch requested\n");
	fflush(stdout);
	printf("JIT ARM32 Backpatching at PC=0x%08lx\n", ctx->CTX_PC);
	fflush(stdout);

	// We need to get the destination register before we start
	u8* codePtr = reinterpret_cast<u8*>(ctx->CTX_PC);

  if (!IsInSpace(codePtr))
    return false;  // this will become a regular crash real soon after this

	u32 Value = *(u32*)codePtr;
	ARMReg rD = INVALID_REG;
	ARMReg V1 = INVALID_REG;
	u32 flags = 0;

	if (!DisasmLoadStore(codePtr, &flags, &rD, &V1))
	{
		WARN_LOG_FMT(DYNA_REC, "Invalid backpatch at PC 0x{:08x} (inst 0x{:08x})",
                     ctx->CTX_PC, Value);
		exit(0);
	}

	// Find the backpatch info for this instruction type
	auto it = m_backpatch_info.find(flags);
	if (it == m_backpatch_info.end())
	{
		ERROR_LOG_FMT(DYNA_REC, "No backpatch info for flags 0x{:08x}", flags);
		return false;
	}

	BackPatchInfo& info = it->second;

	// Calculate the start of the backpatchable sequence
	// trouble_inst_offset tells us how far into the sequence the faulting instruction is
	u8* sequence_start = codePtr - (info.m_fastmem_trouble_inst_offset * 4);

	printf("JIT ARM32: Backpatching sequence at %p (trouble offset=%u, size=%u/%u)\n",
		sequence_start, info.m_fastmem_trouble_inst_offset,
		info.m_fastmem_size, info.m_slowmem_size);
	fflush(stdout);

	// Create emitter at the start of the sequence
  ARMXEmitter emitter;
  emitter.SetCodePtrUnsafe(sequence_start, sequence_start + (info.m_fastmem_size * 4), false);

	// Generate slowmem code (no fastmem this time)
	EmitBackpatchRoutine(&emitter, flags, MemAccessMode::AlwaysSlowAccess, true, rD, V1);

	// Flush instruction cache for the modified code
	ARMXEmitter::FlushIcacheSection(sequence_start, sequence_start + (info.m_fastmem_size * 4));

	// Update PC to retry from the start of the sequence
	ctx->CTX_PC = (uintptr_t)sequence_start;

	printf("JIT ARM32: Backpatch complete, retrying from %p\n", sequence_start);
	fflush(stdout);

	return true;
}

u32 JitArm::EmitBackpatchRoutine(ARMXEmitter* emit, u32 flags, MemAccessMode mode, bool do_padding, ARMReg RS, ARMReg V1)
{
	if (m_accurate_cpu_cache_enabled || !jo.fastmem)
		mode = MemAccessMode::AlwaysSlowAccess;

	const bool emit_fast_access = mode != MemAccessMode::AlwaysSlowAccess;
	const bool emit_slow_access = mode != MemAccessMode::AlwaysFastAccess;

	printf("JIT ARM32 Emitting backpatch routine (fastmem=%d, flags=0x%08x, RS=%d, V1=%d)\n",
		emit_fast_access ? 1 : 0, flags, RS, V1);
	fflush(stdout);

	ARMReg addr = R12;
	ARMReg temp = R11;
	u32 trouble_offset = 0;
	const u8* code_base = emit->GetCodePtr();

	if (emit_fast_access)
	{
		ARMReg temp2 = R10;
		Operand2 mask(2, 1); // ~(Memory::MEMVIEW32_MASK)
		emit->BIC(temp, addr, mask);

		if (flags & BackPatchInfo::FLAG_STORE &&
		    flags & (BackPatchInfo::FLAG_SIZE_32 | BackPatchInfo::FLAG_SIZE_64))
		{
			emit->ADD(temp, temp, R8);
			NEONXEmitter nemit(emit);

			if (flags & BackPatchInfo::FLAG_SIZE_32)
			{
				// 32-bit float store - always use VSTR (no alignment issues)
				emit->VCVT(S0, RS, 0);
				nemit.VREV32(I_8, D0, D0);
				trouble_offset = (emit->GetCodePtr() - code_base) / 4;
				emit->VSTR(S0, temp, 0);
			}
			else
			{
				// 64-bit store - check alignment for optimal path
				emit->TST(temp, 0x7);  // Test 8-byte alignment
				ArmGen::FixupBranch aligned = emit->B_CC(CC_EQ);

				// UNALIGNED PATH - Use two 32-bit stores
				nemit.VREV64(I_8, D0, RS);  // Byte swap first
				emit->VMOV(temp2, S0);      // Extract low 32 bits
				trouble_offset = (emit->GetCodePtr() - code_base) / 4;
				emit->STR(temp2, temp, 0);  // Store low word
				emit->VMOV(temp2, S1);      // Extract high 32 bits
				emit->STR(temp2, temp, 4);  // Store high word
				ArmGen::FixupBranch done = emit->B();

				// ALIGNED PATH - Use fast NEON store
				emit->SetJumpTarget(aligned);
				nemit.VREV64(I_8, D0, RS);
				trouble_offset = (emit->GetCodePtr() - code_base) / 4;
				nemit.VST1(I_64, D0, temp);  // Fast 64-bit store

				emit->SetJumpTarget(done);
			}
		}
		else if (flags & BackPatchInfo::FLAG_LOAD &&
		         flags & (BackPatchInfo::FLAG_SIZE_32 | BackPatchInfo::FLAG_SIZE_64))
		{
			emit->ADD(temp, temp, R8);
			NEONXEmitter nemit(emit);

			if (flags & BackPatchInfo::FLAG_SIZE_32)
			{
				// 32-bit float load - no alignment issues with VLD1(F_32)
				trouble_offset = (emit->GetCodePtr() - code_base) / 4;
				nemit.VLD1(F_32, D0, temp);
				nemit.VREV32(I_8, D0, D0);
				emit->VCVT(RS, S0, 0);
				emit->VCVT(V1, S0, 0);
			}
			else
			{
				// 64-bit load - check alignment for optimal path
				emit->TST(temp, 0x7);  // Test 8-byte alignment
				ArmGen::FixupBranch aligned = emit->B_CC(CC_EQ);

				// UNALIGNED PATH - Use two 32-bit loads
				trouble_offset = (emit->GetCodePtr() - code_base) / 4;
				emit->LDR(temp2, temp, 0);   // Load low word
				emit->VMOV(S0, temp2);        // Move to VFP
				emit->LDR(temp2, temp, 4);    // Load high word
				emit->VMOV(S1, temp2);        // Move to VFP
				nemit.VREV64(I_8, RS, D0);    // Byte swap result
				ArmGen::FixupBranch done = emit->B();

				// ALIGNED PATH - Use fast NEON load
				emit->SetJumpTarget(aligned);
				trouble_offset = (emit->GetCodePtr() - code_base) / 4;
				nemit.VLD1(I_64, RS, temp);   // Fast 64-bit load
				nemit.VREV64(I_8, RS, RS);    // Byte swap result

				emit->SetJumpTarget(done);
			}
		}
		else if (flags & BackPatchInfo::FLAG_STORE)
		{
			if (flags & BackPatchInfo::FLAG_SIZE_32)
				emit->REV(temp2, RS);
			else if (flags & BackPatchInfo::FLAG_SIZE_16)
				emit->REV16(temp2, RS);

			trouble_offset = (emit->GetCodePtr() - code_base) / 4;

			if (flags & BackPatchInfo::FLAG_SIZE_32)
				emit->STR(temp2, R8, temp);
			else if (flags & BackPatchInfo::FLAG_SIZE_16)
				emit->STRH(temp2, R8, temp);
			else
				emit->STRB(RS, R8, temp);
		}
		else
		{
			trouble_offset = (emit->GetCodePtr() - code_base) / 4;

			if (flags & BackPatchInfo::FLAG_SIZE_32)
				emit->LDR(RS, R8, temp);
			else if (flags & BackPatchInfo::FLAG_SIZE_16)
				emit->LDRH(RS, R8, temp);
			else if (flags & BackPatchInfo::FLAG_SIZE_8)
				emit->LDRB(RS, R8, temp);

			if (!(flags & BackPatchInfo::FLAG_REVERSE))
			{
				if (flags & BackPatchInfo::FLAG_SIZE_32)
					emit->REV(RS, RS);
				else if (flags & BackPatchInfo::FLAG_SIZE_16)
					emit->REV16(RS, RS);
			}

			if (flags & BackPatchInfo::FLAG_EXTEND)
				emit->SXTH(RS, RS);
		}
	}
	else
	{
		// Slow access path - unchanged from your original
		if (flags & BackPatchInfo::FLAG_STORE &&
		    flags & (BackPatchInfo::FLAG_SIZE_32 | BackPatchInfo::FLAG_SIZE_64))
		{
			emit->PUSH(5, R0, R1, R2, R3, _LR);
			if (flags & BackPatchInfo::FLAG_SIZE_32)
			{
				emit->MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetMMU()));
				emit->MOV(R2, addr);
				emit->VCVT(S0, RS, 0);
				emit->VMOV(R1, S0);
				emit->MOVI2R(temp, (u32)&PowerPC::WriteFromJit<u32>);
				emit->BLX(temp);
			}
			else
			{
				emit->MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetMMU()));
				emit->MOVI2R(temp, (u32)&PowerPC::WriteFromJit<u64>);
#if !defined(__ARM_PCS_VFP)
				emit->VMOV_D_to_RR(R1, R2, RS);
				emit->MOV(R3, addr);
#else
				emit->VMOV(D0, RS);
				emit->MOV(R1, addr);
#endif
				emit->BLX(temp);
			}
			emit->POP(5, R0, R1, R2, R3, _LR);
		}
		else if (flags & BackPatchInfo::FLAG_LOAD &&
		         flags & (BackPatchInfo::FLAG_SIZE_32 | BackPatchInfo::FLAG_SIZE_64))
		{
			emit->PUSH(5, R0, R1, R2, R3, _LR);
			emit->MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetMMU()));
			if (flags & BackPatchInfo::FLAG_SIZE_32)
			{
				emit->MOV(R1, addr);
				emit->MOVI2R(temp, reinterpret_cast<u32>(&PowerPC::ReadFromJit<u32>));
				emit->BLX(temp);
				emit->VMOV(S0, R0);
				emit->VCVT(RS, S0, 0);
				emit->VCVT(V1, S0, 0);
			}
			else
			{
				emit->MOVI2R(temp, (u32)&PowerPC::ReadFromJit<u64>);
				emit->MOV(R1, addr);
				emit->BLX(temp);
#if !defined(__ARM_PCS_VFP)
				emit->VMOV_RR_to_D(RS, R0, R1);
#else
				emit->VMOV(RS, D0);
#endif
			}
			emit->POP(5, R0, R1, R2, R3, _LR);
		}
		else if (flags & BackPatchInfo::FLAG_STORE)
		{
			emit->PUSH(5, R0, R1, R2, R3, _LR);
			emit->MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetMMU()));
			emit->MOV(R1, RS);
			emit->MOV(R2, addr);

			if (flags & BackPatchInfo::FLAG_SIZE_32)
				emit->MOVI2R(temp, reinterpret_cast<u32>(&PowerPC::WriteFromJit<u32>));
			else if (flags & BackPatchInfo::FLAG_SIZE_16)
				emit->MOVI2R(temp, reinterpret_cast<u32>(&PowerPC::WriteFromJit<u16>));
			else
				emit->MOVI2R(temp, reinterpret_cast<u32>(&PowerPC::WriteFromJit<u8>));

			emit->BLX(temp);
			emit->POP(5, R0, R1, R2, R3, _LR);
		}
		else
		{
			emit->PUSH(5, R0, R1, R2, R3, _LR);
			emit->MOVI2R(R0, reinterpret_cast<u32>(&m_system.GetMMU()));
			emit->MOV(R1, addr);

			if (flags & BackPatchInfo::FLAG_SIZE_32)
				emit->MOVI2R(temp, reinterpret_cast<u32>(&PowerPC::ReadFromJit<u32>));
			else if (flags & BackPatchInfo::FLAG_SIZE_16)
				emit->MOVI2R(temp, reinterpret_cast<u32>(&PowerPC::ReadFromJit<u16>));
			else if (flags & BackPatchInfo::FLAG_SIZE_8)
				emit->MOVI2R(temp, reinterpret_cast<u32>(&PowerPC::ReadFromJit<u8>));

			emit->BLX(temp);
			emit->MOV(temp, R0);
			emit->POP(5, R0, R1, R2, R3, _LR);

			if (!(flags & BackPatchInfo::FLAG_REVERSE))
			{
				emit->MOV(RS, temp);
			}
			else
			{
				if (flags & BackPatchInfo::FLAG_SIZE_32)
					emit->REV(RS, temp);
				else if (flags & BackPatchInfo::FLAG_SIZE_16)
					emit->REV16(RS, temp);
			}
		}
	}

	if (do_padding)
	{
		BackPatchInfo& info = m_backpatch_info[flags];
		u32 num_insts_max = std::max(info.m_fastmem_size, info.m_slowmem_size);

		u32 code_size = emit->GetCodePtr() - code_base;
		code_size /= 4;

		emit->NOP(num_insts_max - code_size);
	}

	printf("membase r8=0x%08x\n", R8);
	printf("Finished EmitBackpatchRoutine\n");
	fflush(stdout);

	return trouble_offset;
}

void JitArm::InitBackpatch()
{
	printf("Initializing JIT ARM32 backpatch routines...\n");
	fflush(stdout);

	u32 flags = 0;
	BackPatchInfo info;
	u8* code_base = GetWritableCodePtr();
	u8* code_end;

	// Writes
	{
		// 8bit
		{
			flags =
				BackPatchInfo::FLAG_STORE |
				BackPatchInfo::FLAG_SIZE_8;
			EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysSlowAccess, false, R0);
			code_end = GetWritableCodePtr();
			info.m_slowmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			info.m_fastmem_trouble_inst_offset =
				EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysFastAccess, false, R0);
			code_end = GetWritableCodePtr();
			info.m_fastmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			m_backpatch_info[flags] = info;
		}
		// 16bit
		{
			flags =
				BackPatchInfo::FLAG_STORE |
				BackPatchInfo::FLAG_SIZE_16;
			EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysSlowAccess, false, R0);
			code_end = GetWritableCodePtr();
			info.m_slowmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			info.m_fastmem_trouble_inst_offset =
				EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysFastAccess, false, R0);
			code_end = GetWritableCodePtr();
			info.m_fastmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			m_backpatch_info[flags] = info;
		}
		// 32bit
		{
			flags =
				BackPatchInfo::FLAG_STORE |
				BackPatchInfo::FLAG_SIZE_32;
			EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysSlowAccess, false, R0);
			code_end = GetWritableCodePtr();
			info.m_slowmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			info.m_fastmem_trouble_inst_offset =
				EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysFastAccess, false, R0);
			code_end = GetWritableCodePtr();
			info.m_fastmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			m_backpatch_info[flags] = info;
		}
		// 32bit float
		{
			flags =
				BackPatchInfo::FLAG_STORE |
				BackPatchInfo::FLAG_SIZE_32;
			EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysSlowAccess, false, D0);
			code_end = GetWritableCodePtr();
			info.m_slowmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			info.m_fastmem_trouble_inst_offset =
				EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysFastAccess, false, D0);
			code_end = GetWritableCodePtr();
			info.m_fastmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			m_backpatch_info[flags] = info;
		}
		// 64bit float
		{
			flags =
				BackPatchInfo::FLAG_STORE |
				BackPatchInfo::FLAG_SIZE_64;
			EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysSlowAccess, false, D0);
			code_end = GetWritableCodePtr();
			info.m_slowmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			info.m_fastmem_trouble_inst_offset =
				EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysFastAccess, false, D0);
			code_end = GetWritableCodePtr();
			info.m_fastmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			m_backpatch_info[flags] = info;
		}

	}

	// Loads
	{
		// 8bit
		{
			flags =
				BackPatchInfo::FLAG_LOAD |
				BackPatchInfo::FLAG_SIZE_8;
			EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysSlowAccess, false, R0);
			code_end = GetWritableCodePtr();
			info.m_slowmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			info.m_fastmem_trouble_inst_offset =
				EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysFastAccess, false, R0);
			code_end = GetWritableCodePtr();
			info.m_fastmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			m_backpatch_info[flags] = info;
		}
		// 16bit
		{
			flags =
				BackPatchInfo::FLAG_LOAD |
				BackPatchInfo::FLAG_SIZE_16;
			EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysSlowAccess, false, R0);
			code_end = GetWritableCodePtr();
			info.m_slowmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			info.m_fastmem_trouble_inst_offset =
				EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysFastAccess, false, R0);
			code_end = GetWritableCodePtr();
			info.m_fastmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			m_backpatch_info[flags] = info;
		}
		// 32bit
		{
			flags =
				BackPatchInfo::FLAG_LOAD |
				BackPatchInfo::FLAG_SIZE_32;
			EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysSlowAccess, false, R0);
			code_end = GetWritableCodePtr();
			info.m_slowmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			info.m_fastmem_trouble_inst_offset =
				EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysFastAccess, false, R0);
			code_end = GetWritableCodePtr();
			info.m_fastmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			m_backpatch_info[flags] = info;
		}
		// 16bit - reverse
		{
			flags =
				BackPatchInfo::FLAG_LOAD |
				BackPatchInfo::FLAG_SIZE_16 |
				BackPatchInfo::FLAG_REVERSE;
			EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysSlowAccess, false, R0);
			code_end = GetWritableCodePtr();
			info.m_slowmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			info.m_fastmem_trouble_inst_offset =
				EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysFastAccess, false, R0);
			code_end = GetWritableCodePtr();
			info.m_fastmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			m_backpatch_info[flags] = info;
		}
		// 16bit - sign extend
		{
			flags =
				BackPatchInfo::FLAG_LOAD |
				BackPatchInfo::FLAG_SIZE_16 |
				BackPatchInfo::FLAG_EXTEND;
			EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysSlowAccess, false, R0);
			code_end = GetWritableCodePtr();
			info.m_slowmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			info.m_fastmem_trouble_inst_offset =
				EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysFastAccess, false, R0);
			code_end = GetWritableCodePtr();
			info.m_fastmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			m_backpatch_info[flags] = info;
		}
		// 32bit - reverse
		{
			flags =
				BackPatchInfo::FLAG_LOAD |
				BackPatchInfo::FLAG_SIZE_32 |
				BackPatchInfo::FLAG_REVERSE;
			EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysSlowAccess, false, R0);
			code_end = GetWritableCodePtr();
			info.m_slowmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			info.m_fastmem_trouble_inst_offset =
				EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysFastAccess, false, R0);
			code_end = GetWritableCodePtr();
			info.m_fastmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			m_backpatch_info[flags] = info;
		}
		// 32bit float
		{
			flags =
				BackPatchInfo::FLAG_LOAD |
				BackPatchInfo::FLAG_SIZE_32;
			EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysSlowAccess, false, D0, D1);
			code_end = GetWritableCodePtr();
			info.m_slowmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			info.m_fastmem_trouble_inst_offset =
				EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysFastAccess, false, D0, D1);
			code_end = GetWritableCodePtr();
			info.m_fastmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			m_backpatch_info[flags] = info;
		}
		// 64bit float
		{
			flags =
				BackPatchInfo::FLAG_LOAD |
				BackPatchInfo::FLAG_SIZE_64;
			EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysSlowAccess, false, D0);
			code_end = GetWritableCodePtr();
			info.m_slowmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			info.m_fastmem_trouble_inst_offset =
				EmitBackpatchRoutine(this, flags, MemAccessMode::AlwaysFastAccess, false, D0);
			code_end = GetWritableCodePtr();
			info.m_fastmem_size = (code_end - code_base) / 4;

			SetCodePtr(code_base);

			m_backpatch_info[flags] = info;
		}
	}
}
