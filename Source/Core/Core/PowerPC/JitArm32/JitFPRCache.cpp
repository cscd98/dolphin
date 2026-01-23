// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/PowerPC/JitArm32/Jit.h"
#include "Core/PowerPC/JitArm32/JitFPRCache.h"

using namespace ArmGen;

ArmFPRCache::ArmFPRCache()
{
	emit = nullptr;
}

void ArmFPRCache::Init(ARMXEmitter *emitter)
{
	emit = emitter;
	ARMReg *PPCRegs = GetPPCAllocationOrder(NUMPPCREG);
	ARMReg *Regs = GetAllocationOrder(NUMARMREG);

	for (u8 a = 0; a < NUMPPCREG; ++a)
	{
		ArmCRegs[a].PPCReg = 33;
		ArmCRegs[a].Reg = PPCRegs[a];
		ArmCRegs[a].LastLoad = 0;
		ArmCRegs[a].PS1 = false;
	}
	for (u8 a = 0; a < NUMARMREG; ++a)
	{
		ArmRegs[a].Reg = Regs[a];
		ArmRegs[a].free = true;
	}
}

void ArmFPRCache::ResetRegisters(BitSet32 regs_to_reset)
{
  for (int i : regs_to_reset)
  {
    if (i < 0 || i >= 32)
      continue;

    for (int ps = 0; ps < 2; ++ps)
    {
      if (_regs[i][ps].GetType() == REG_REG)
      {
        u32 regindex = _regs[i][ps].GetRegIndex();
        ArmCRegs[regindex].PPCReg = 33;
        ArmCRegs[regindex].LastLoad = 0;
      }
      _regs[i][ps].Flush();
    }
  }
}

ARMReg *ArmFPRCache::GetPPCAllocationOrder(int &count)
{
	// This will return us the allocation order of the registers we can use on
	// the ppc side.
	static ARMReg allocationOrder[] =
	{
		D4, D5, D6, D7, D8, D9, D10, D11, D12, D13,
		D14, D15, D16,  D17, D18, D19, D20, D21, D22,
		D23, D24, D25, D26, D27, D28, D29, D30, D31
	};
	count = sizeof(allocationOrder) / sizeof(const int);
	return allocationOrder;
}
ARMReg *ArmFPRCache::GetAllocationOrder(int &count)
{
	// This will return us the allocation order of the registers we can use on
	// the host side.
	static ARMReg allocationOrder[] =
	{
		D0, D1, D2, D3
	};
	count = sizeof(allocationOrder) / sizeof(const int);
	return allocationOrder;
}

ARMReg ArmFPRCache::GetReg(bool AutoLock)
{
	for (u8 a = 0; a < NUMARMREG; ++a)
	{
		if (ArmRegs[a].free)
		{
			// Alright, this one is free
			if (AutoLock)
				ArmRegs[a].free = false;
			return ArmRegs[a].Reg;
		}
	}

	// Uh Oh, we have all them locked....
	ASSERT_MSG(DYNA_REC, false, "All available registers (FPRCache) are locked dumb dumb");
	return D31;
}
void ArmFPRCache::Unlock(ARMReg V0)
{
	for (u8 RegNum = 0; RegNum < NUMARMREG; ++RegNum)
	{
		if (ArmRegs[RegNum].Reg == V0)
		{
			ASSERT_MSG(DYNA_REC, !ArmRegs[RegNum].free, "This register is already unlocked");
			ArmRegs[RegNum].free = true;
		}
	}
}
u32 ArmFPRCache::GetLeastUsedRegister(bool increment)
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
bool ArmFPRCache::FindFreeRegister(u32 &regindex)
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

ARMReg ArmFPRCache::GetPPCReg(u32 preg, bool PS1, bool preLoad)
{
	u32 lastRegIndex = GetLeastUsedRegister(true);

	if (_regs[preg][PS1].GetType() != REG_NOTLOADED)
	{
		u8 a = _regs[preg][PS1].GetRegIndex();
		ArmCRegs[a].LastLoad = 0;
		return ArmCRegs[a].Reg;
	}

	u32 regindex;
	if (FindFreeRegister(regindex))
	{
		s16 offset = PPCSTATE_OFF_PS(preg) + (PS1 ? 8 : 0);

		ArmCRegs[regindex].PPCReg = preg;
		ArmCRegs[regindex].LastLoad = 0;
		ArmCRegs[regindex].PS1 = PS1;

		_regs[preg][PS1].LoadToReg(regindex);
		if (preLoad)
			emit->VLDR(ArmCRegs[regindex].Reg, PPC_REG, offset);
		return ArmCRegs[regindex].Reg;
	}

	// Alright, we couldn't get a free space, dump that least used register
	s16 offsetOld = PPCSTATE_OFF_PS(ArmCRegs[lastRegIndex].PPCReg) +
                (ArmCRegs[lastRegIndex].PS1 ? 8 : 0);
	s16 offsetNew = PPCSTATE_OFF_PS(preg) +
									(PS1 ? 8 : 0);

	emit->VSTR(ArmCRegs[lastRegIndex].Reg, PPC_REG, offsetOld);

	_regs[ArmCRegs[lastRegIndex].PPCReg][ArmCRegs[lastRegIndex].PS1].Flush();

	ArmCRegs[lastRegIndex].PPCReg = preg;
	ArmCRegs[lastRegIndex].LastLoad = 0;
	ArmCRegs[lastRegIndex].PS1 = PS1;

	_regs[preg][PS1].LoadToReg(lastRegIndex);
	if (preLoad)
		emit->VLDR(ArmCRegs[lastRegIndex].Reg, PPC_REG, offsetNew);
	return ArmCRegs[lastRegIndex].Reg;
}

ARMReg ArmFPRCache::R0(u32 preg, bool preLoad)
{
	return GetPPCReg(preg, false, preLoad);
}

ARMReg ArmFPRCache::R1(u32 preg, bool preLoad)
{
	return GetPPCReg(preg, true, preLoad);
}

void ArmFPRCache::Flush(FlushMode mode)
{
	for (u8 a = 0; a < 32; ++a)
	{
		if (_regs[a][0].GetType() != REG_NOTLOADED)
		{
			s16 offset = PPCSTATE_OFF_PS(a);
			u32 regindex = _regs[a][0].GetRegIndex();
			emit->VSTR(ArmCRegs[regindex].Reg, PPC_REG, offset);

			if (mode == FLUSH_ALL)
			{
				ArmCRegs[regindex].PPCReg = 33;
				ArmCRegs[regindex].LastLoad = 0;
				_regs[a][0].Flush();
			}
		}
		if (_regs[a][1].GetType() != REG_NOTLOADED)
		{
			s16 offset = PPCSTATE_OFF_PS(a);
			u32 regindex = _regs[a][1].GetRegIndex();
			emit->VSTR(ArmCRegs[regindex].Reg, PPC_REG, offset);

			if (mode == FLUSH_ALL)
			{
				ArmCRegs[regindex].PPCReg = 33;
				ArmCRegs[regindex].LastLoad = 0;
				_regs[a][1].Flush();
			}
		}
	}
}

void ArmFPRCache::StoreFromRegister(u32 preg)
{
	if (_regs[preg][0].GetType() != REG_NOTLOADED)
	{
		s16 offset = PPCSTATE_OFF_PS(preg);
		u32 regindex = _regs[preg][0].GetRegIndex();
		emit->VSTR(ArmCRegs[regindex].Reg, PPC_REG, offset);

		ArmCRegs[regindex].PPCReg = 33;
		ArmCRegs[regindex].LastLoad = 0;
		_regs[preg][0].Flush();
	}
	if (_regs[preg][1].GetType() != REG_NOTLOADED)
	{
		s16 offset = PPCSTATE_OFF_PS(preg);
		u32 regindex = _regs[preg][1].GetRegIndex();
		emit->VSTR(ArmCRegs[regindex].Reg, PPC_REG, offset);

		ArmCRegs[regindex].PPCReg = 33;
		ArmCRegs[regindex].LastLoad = 0;
		_regs[preg][1].Flush();
	}
}
