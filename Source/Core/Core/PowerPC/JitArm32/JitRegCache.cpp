// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/PowerPC/JitArm32/Jit.h"
#include "Core/PowerPC/JitArm32/JitRegCache.h"

using namespace ArmGen;

// GPR Cache
constexpr size_t GUEST_GPR_COUNT = 32;
constexpr size_t GUEST_CR_COUNT = 8;
constexpr size_t GUEST_GPR_OFFSET = 0;
constexpr size_t GUEST_CR_OFFSET = GUEST_GPR_COUNT;

ArmRegCache::ArmRegCache()
{
	emit = nullptr;
}

void ArmRegCache::Init(ARMXEmitter *emitter)
{
	emit = emitter;
	ARMReg *PPCRegs = GetPPCAllocationOrder(NUMPPCREG);
	ARMReg *Regs = GetAllocationOrder(NUMARMREG);

	for (u8 a = 0; a < NUMPPCREG; ++a)
	{
		ArmCRegs[a].PPCReg = 33;
		ArmCRegs[a].Reg = PPCRegs[a];
		ArmCRegs[a].LastLoad = 0;
	}
	for (u8 a = 0; a < NUMARMREG; ++a)
	{
		ArmRegs[a].Reg = Regs[a];
		ArmRegs[a].free = !(ArmRegs[a].Reg == DISPATCHER_PC || ArmRegs[a].Reg == MEM_REG
			|| ArmRegs[a].Reg == PPC_REG);
	}
}

void ArmRegCache::ResetRegisters(BitSet32 regs_to_reset)
{
  for (size_t i : regs_to_reset)
  {
    if (i >= GUEST_GPR_COUNT)
      continue;

    if (regs[i].GetType() == REG_REG)
    {
      u32 regindex = regs[i].GetRegIndex();

			if (ArmCRegs[regindex].Reg == DISPATCHER_PC)
        continue;

      ArmCRegs[regindex].PPCReg = 33;
      ArmCRegs[regindex].LastLoad = 0;
    }

    regs[i].Flush();
  }
}

void ArmRegCache::ResetCRRegisters(BitSet8 cr_to_reset)
{
  for (int i : cr_to_reset)
  {
		// PPC GPRs are 0..31, CR fields follow at 32..39
    if (i < 0 || i >= 8)
      continue;

    int idx = GUEST_CR_OFFSET + i;

    if (regs[idx].GetType() == REG_REG)
    {
      u32 regindex = regs[idx].GetRegIndex();
      ArmCRegs[regindex].PPCReg = 33;
      ArmCRegs[regindex].LastLoad = 0;
    }

    regs[idx].Flush();
  }
}

ARMReg *ArmRegCache::GetPPCAllocationOrder(int &count)
{
	// This will return us the allocation order of the registers we can use on
	// the ppc side.
	static ARMReg allocationOrder[] =
	{
		R0, R1, R2, R3, R4, R5, R6 //, R7
	};
	count = sizeof(allocationOrder) / sizeof(const int);
	return allocationOrder;
}
ARMReg *ArmRegCache::GetAllocationOrder(int &count)
{
	// This will return us the allocation order of the registers we can use on
	// the host side.
	static ARMReg allocationOrder[] =
	{
		// See:
		// https://developer.arm.com/documentation/dui0056/d/using-the-procedure-call-standard/register-roles-and-names/register-names
		R12, R11, R10
		//R14 is LR
	};
	count = sizeof(allocationOrder) / sizeof(const int);
	return allocationOrder;
}

ARMReg ArmRegCache::GetReg(bool AutoLock)
{
	for (u8 a = 0; a < NUMARMREG; ++a)
	{
		if (ArmRegs[a].free)
		{
			ARMReg r = ArmRegs[a].Reg;
			ASSERT_MSG(DYNA_REC, (r != DISPATCHER_PC && r != MEM_REG && r != PPC_REG && r != R14),
				"Allocator tried to return reserved register!");
			// Alright, this one is free
			if (AutoLock)
				ArmRegs[a].free = false;
			return ArmRegs[a].Reg;
		}
	}

	// Uh Oh, we have all them locked....
	ASSERT_MSG(DYNA_REC, false, "All available registers are locked dumb dumb");
	return R0;
}

void ArmRegCache::Unlock(ARMReg R0, ARMReg R1, ARMReg R2, ARMReg R3)
{
	for (u8 RegNum = 0; RegNum < NUMARMREG; ++RegNum)
	{
		if (ArmRegs[RegNum].Reg == R0)
		{
			ASSERT_MSG(DYNA_REC, !ArmRegs[RegNum].free, "This register is already unlocked");
			ArmRegs[RegNum].free = true;
		}

		if (R1 != INVALID_REG && ArmRegs[RegNum].Reg == R1)
			ArmRegs[RegNum].free = true;

		if (R2 != INVALID_REG && ArmRegs[RegNum].Reg == R2)
			ArmRegs[RegNum].free = true;

		if (R3 != INVALID_REG && ArmRegs[RegNum].Reg == R3)
			ArmRegs[RegNum].free = true;
	}
}

u32 ArmRegCache::GetLeastUsedRegister(bool increment)
{
	u32 HighestUsed = 0;
	u8 lastRegIndex = 0;
	for (u8 a = 0; a < NUMPPCREG; ++a)
	{
		if (increment)
			++ArmCRegs[a].LastLoad;
		if (ArmCRegs[a].LastLoad > HighestUsed)
		{
			HighestUsed = ArmCRegs[a].LastLoad;
			lastRegIndex = a;
		}
	}
	return lastRegIndex;
}

bool ArmRegCache::FindFreeRegister(u32 &regindex)
{
	for (u8 a = 0; a < NUMPPCREG; ++a)
	{
		if (ArmCRegs[a].PPCReg == 33)
		{
			regindex = a;
			return true;
		}
	}
	return false;
}

ARMReg ArmRegCache::R(u32 preg)
{
	printf("ArmRegCache::R() preg %u\n", preg);

	if (regs[preg].GetType() == REG_IMM)
		return BindToRegister(preg, true, true);

	u32 lastRegIndex = GetLeastUsedRegister(true);

	// Check if already Loaded
	if (regs[preg].GetType() == REG_REG)
	{
		u8 a = regs[preg].GetRegIndex();
		ArmCRegs[a].LastLoad = 0;
		printf("ArmRegCache::R() cache hit %u\n", a);
		return ArmCRegs[a].Reg;
	}

	// Check if we have a free register
	u32 regindex;
	if (FindFreeRegister(regindex))
	{
		emit->LDR(ArmCRegs[regindex].Reg, PPC_REG, PPCSTATE_OFF_GPR(preg));
		ArmCRegs[regindex].PPCReg = preg;
		ArmCRegs[regindex].LastLoad = 0;

		regs[preg].LoadToReg(regindex);

		//printf("ArmRegCache::R() returning ARM reg index 0x%08x\n", ArmCRegs[index].Reg);

		return ArmCRegs[regindex].Reg;
	}

	// Alright, we couldn't get a free space, dump that least used register
	emit->STR(ArmCRegs[lastRegIndex].Reg, PPC_REG,
		PPCSTATE_OFF_GPR(ArmCRegs[lastRegIndex].PPCReg));
	emit->LDR(ArmCRegs[lastRegIndex].Reg, PPC_REG,
		PPCSTATE_OFF_GPR(preg));

	regs[ArmCRegs[lastRegIndex].PPCReg].Flush();

	ArmCRegs[lastRegIndex].PPCReg = preg;
	ArmCRegs[lastRegIndex].LastLoad = 0;

	regs[preg].LoadToReg(lastRegIndex);

	printf("ArmRegCache::R() spilling PPC reg %u\n", ArmCRegs[lastRegIndex].PPCReg);
	printf("ArmRegCache::R() loading PPC reg %u\n", preg);

	return ArmCRegs[lastRegIndex].Reg;
}

void ArmRegCache::BindToRegister(u32 preg, bool doLoad)
{
	BindToRegister(preg, doLoad, false);
}

ARMReg ArmRegCache::BindToRegister(u32 preg, bool doLoad, bool kill_imm)
{
	u32 lastRegIndex = GetLeastUsedRegister(false);
	u32 freeRegIndex;
	bool found_free = FindFreeRegister(freeRegIndex);
	if (regs[preg].GetType() == REG_IMM)
	{
		if (!kill_imm)
			return INVALID_REG;
		if (found_free)
		{
			if (doLoad)
				emit->MOVI2R(ArmCRegs[freeRegIndex].Reg, regs[preg].GetImm());
			ArmCRegs[freeRegIndex].PPCReg = preg;
			ArmCRegs[freeRegIndex].LastLoad = 0;
			regs[preg].LoadToReg(freeRegIndex);
			return ArmCRegs[freeRegIndex].Reg;
		}
		else
		{
			emit->STR(ArmCRegs[lastRegIndex].Reg, PPC_REG,
				PPCSTATE_OFF_GPR(ArmCRegs[lastRegIndex].PPCReg));
			if (doLoad)
				emit->MOVI2R(ArmCRegs[lastRegIndex].Reg, regs[preg].GetImm());

			regs[ArmCRegs[lastRegIndex].PPCReg].Flush();

			ArmCRegs[lastRegIndex].PPCReg = preg;
			ArmCRegs[lastRegIndex].LastLoad = 0;

			regs[preg].LoadToReg(lastRegIndex);
			return ArmCRegs[lastRegIndex].Reg;
		}
	}
	else if (regs[preg].GetType() == REG_NOTLOADED)
	{
		if (found_free)
		{
			if (doLoad)
				emit->LDR(ArmCRegs[freeRegIndex].Reg, PPC_REG,
					PPCSTATE_OFF_GPR(preg));

			ArmCRegs[freeRegIndex].PPCReg = preg;
			ArmCRegs[freeRegIndex].LastLoad = 0;
			regs[preg].LoadToReg(freeRegIndex);
			return ArmCRegs[freeRegIndex].Reg;
		}
		else
		{
			emit->STR(ArmCRegs[lastRegIndex].Reg, PPC_REG,
				PPCSTATE_OFF_GPR(ArmCRegs[lastRegIndex].PPCReg));

			if (doLoad)
				emit->LDR(ArmCRegs[lastRegIndex].Reg, PPC_REG,
					PPCSTATE_OFF_GPR(preg));

			regs[ArmCRegs[lastRegIndex].PPCReg].Flush();

			ArmCRegs[lastRegIndex].PPCReg = preg;
			ArmCRegs[lastRegIndex].LastLoad = 0;

			regs[preg].LoadToReg(lastRegIndex);
			return ArmCRegs[lastRegIndex].Reg;
		}
	}
	else
	{
		u8 a = regs[preg].GetRegIndex();
		ArmCRegs[a].LastLoad = 0;
		return ArmCRegs[a].Reg;
	}
}

void ArmRegCache::SetImmediate(u32 preg, u32 imm, bool dirty)
{
	if (regs[preg].GetType() == REG_REG)
	{
		// Dump real reg at this point
		u32 regindex = regs[preg].GetRegIndex();
		ArmCRegs[regindex].PPCReg = 33;
		ArmCRegs[regindex].LastLoad = 0;
	}
	regs[preg].LoadToImm(imm);
	regs[preg].SetDirty(dirty);
}

void ArmRegCache::Flush(FlushMode mode)
{
	for (u8 a = 0; a < 32; ++a)
	{
		if (regs[a].GetType() == REG_IMM)
		{
			if (mode == FLUSH_ALL)
			{
				// This changes the type over to a REG_REG and gets caught below.
				BindToRegister(a, true, true);
			}
			else
			{
				ARMReg tmp = GetReg();
				emit->MOVI2R(tmp, regs[a].GetImm());
				emit->STR(tmp, PPC_REG, PPCSTATE_OFF_GPR(a));
				Unlock(tmp);
			}
		}
		if (regs[a].GetType() == REG_REG)
		{
			u32 regindex = regs[a].GetRegIndex();
			emit->STR(ArmCRegs[regindex].Reg, PPC_REG, PPCSTATE_OFF_GPR(a));
			if (mode == FLUSH_ALL)
			{
				ArmCRegs[regindex].PPCReg = 33;
				ArmCRegs[regindex].LastLoad = 0;
				regs[a].Flush();
			}
		}
	}
}

void ArmRegCache::StoreFromRegister(u32 preg)
{
	if (regs[preg].GetType() == REG_IMM)
	{
		// This changes the type over to a REG_REG and gets caught below.
		BindToRegister(preg, true, true);
	}
	if (regs[preg].GetType() == REG_REG)
	{
		u32 regindex = regs[preg].GetRegIndex();
		emit->STR(ArmCRegs[regindex].Reg, PPC_REG, PPCSTATE_OFF_GPR(preg));

		ArmCRegs[regindex].PPCReg = 33;
		ArmCRegs[regindex].LastLoad = 0;
		regs[preg].Flush();
	}
}

void ArmRegCache::Lock(ArmGen::ARMReg R0,
                       ArmGen::ARMReg R1,
                       ArmGen::ARMReg R2,
                       ArmGen::ARMReg R3)
{
  for (ArmGen::ARMReg reg : {R0, R1, R2, R3})
  {
    if (reg == ArmGen::ARMReg::INVALID_REG)
      continue;

    // Flush any guest binding that might be using this host reg
    // (similar to FlushByHost in ARM64).
    for (int i = 0; i < NUMPPCREG; i++)
    {
      if (ArmCRegs[i].Reg == reg)
      {
        StoreFromRegister(i);
        regs[i].Flush();
        ArmCRegs[i].Reg = ArmGen::ARMReg::INVALID_REG;
      }
    }

    // Mark the host reg as not free
    for (int i = 0; i < NUMARMREG; i++)
    {
      if (ArmRegs[i].Reg == reg)
        ArmRegs[i].free = false;
    }
  }
}
