// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Common/Common.h"
#include "Common/ArmCommon.h"
#include "Core/HW/MMIO.h"
#include "Core/System.h"
#include "Core/PowerPC/JitArm32/Jit.h"

using namespace ArmGen;

// Thunks to invoke MMIO lambda handlers on ARM32 (AAPCS: R0=system, R1=address, R2=value(u32), R3=lambda)
static void MMIOWriteLambdaThunk_u8(Core::System* system, u32 address, u32 value_u32,
                                    const std::function<void(Core::System&, u32, u8)>* lambda)
{
  const u8 value = static_cast<u8>(value_u32);
  (*lambda)(*system, address, value);
}

static void MMIOWriteLambdaThunk_u16(Core::System* system, u32 address, u32 value_u32,
                                     const std::function<void(Core::System&, u32, u16)>* lambda)
{
  const u16 value = static_cast<u16>(value_u32);
  (*lambda)(*system, address, value);
}

static void MMIOWriteLambdaThunk_u32(Core::System* system, u32 address, u32 value_u32,
                                     const std::function<void(Core::System&, u32, u32)>* lambda)
{
  const u32 value = value_u32;
  (*lambda)(*system, address, value);
}

// ARM32 MMIO write code generator visitor (mirrors ARM64, but uses ARMXEmitter and no float emitter)
template <typename T>
class MMIOWriteCodeGenerator32 : public MMIO::WriteHandlingMethodVisitor<T>
{
public:
  MMIOWriteCodeGenerator32(Core::System* system, ARMXEmitter* emit, ARMReg src_reg, u32 address)
      : m_system(system), m_emit(emit), m_src_reg(src_reg), m_address(address) {}

  void VisitNop() override
  {
    // No-op
  }

  void VisitDirect(T* addr, u32 mask) override
  {
    WriteRegToAddr(sizeof(T) * 8, addr, mask);
  }

  void VisitComplex(const std::function<void(Core::System&, u32, T)>* lambda) override
  {
    CallLambda(sizeof(T) * 8, lambda);
  }

private:
  void StoreFromRegister(int sbits, ARMReg reg, s32 offset)
  {
    switch (sbits)
    {
    case 8:
      m_emit->STRB(reg, R0, Operand2(static_cast<u32>(offset)));
      break;
    case 16:
      m_emit->STRH(reg, R0, Operand2(static_cast<u32>(offset)));
      break;
    case 32:
      m_emit->STR(reg, R0, Operand2(static_cast<u32>(offset)));
      break;
    default:
      ASSERT_MSG(DYNA_REC, false, "Unknown size {} in MMIOWriteCodeGenerator32", sbits);
      break;
    }
  }

  void WriteRegToAddr(int sbits, const void* ptr, u32 mask)
  {
    const u32 base = reinterpret_cast<u32>(ptr);
    m_emit->MOVI2R(R0, base);

    const u32 all_ones = (sbits == 8 ? 0xFFu : sbits == 16 ? 0xFFFFu : 0xFFFFFFFFu);
    if ((all_ones & mask) == all_ones)
    {
      // No masking needed; store directly from source reg.
      StoreFromRegister(sbits, m_src_reg, 0);
    }
    else
    {
      // Mask the value before storing.
      Operand2 op_mask;
      if (TryMakeOperand2(mask, op_mask))
      {
        m_emit->AND(R11, m_src_reg, op_mask);
      }
      else
      {
        // Fallback: load mask into a register, then AND reg-reg.
        m_emit->MOVI2R(R12, mask);
        // AND(dest=R11, src=m_src_reg, op2=register form)
        m_emit->AND(R11, m_src_reg, Operand2(R12));
      }
      StoreFromRegister(sbits, R11, 0);
    }
  }

  void CallLambda(int /*sbits*/, const std::function<void(Core::System&, u32, T)>* lambda)
  {
    // R0 = Core::System*
    m_emit->MOVI2R(R0, reinterpret_cast<u32>(m_system));
    // R1 = address
    m_emit->MOVI2R(R1, m_address);
    // R2 = value (u32). If src_reg != R2, move it (reg-to-reg).
    if (m_src_reg != R2)
      m_emit->MOV(R2, m_src_reg);
    // R3 = lambda pointer
    m_emit->MOVI2R(R3, reinterpret_cast<u32>(lambda));

    void* thunk = nullptr;
    if constexpr (std::is_same<T, u8>::value)
      thunk = reinterpret_cast<void*>(&MMIOWriteLambdaThunk_u8);
    else if constexpr (std::is_same<T, u16>::value)
      thunk = reinterpret_cast<void*>(&MMIOWriteLambdaThunk_u16);
    else if constexpr (std::is_same<T, u32>::value)
      thunk = reinterpret_cast<void*>(&MMIOWriteLambdaThunk_u32);
    else
      ASSERT_MSG(DYNA_REC, false, "Unsupported MMIO write T");

    // Call thunk (uses R12 as scratch, does not clobber LR beyond call semantics)
    m_emit->QuickCallFunction(R12, thunk);
  }

  Core::System* m_system;
  ARMXEmitter* m_emit;
  ARMReg m_src_reg;
  u32 m_address;
};

// Public ARM32 helper, signature aligned to ARM32 usage in SafeStoreFromReg
void JitArm::MMIOWriteRegToAddr(Core::System& system, MMIO::Mapping* mmio,
                                ArmGen::ARMXEmitter* emit, ArmGen::ARMReg src_reg, u32 address, u32 flags)
{
  ASSERT(!(flags & BackPatchInfo::FLAG_FLOAT));

  // Byteswap before store if PPC side expects reversed vs host little-endian
  ARMReg store_reg = src_reg;

  if (flags & BackPatchInfo::FLAG_SIZE_32)
  {
    if (!(flags & BackPatchInfo::FLAG_REVERSE))
    {
      emit->REV(R12, src_reg);
      store_reg = R12;
    }
    MMIOWriteCodeGenerator32<u32> gen(&system, emit, store_reg, address);
    mmio->GetHandlerForWrite<u32>(address).Visit(gen);
  }
  else if (flags & BackPatchInfo::FLAG_SIZE_16)
  {
    if (!(flags & BackPatchInfo::FLAG_REVERSE))
    {
      emit->REV16(R12, src_reg);
      store_reg = R12;
    }
    MMIOWriteCodeGenerator32<u16> gen(&system, emit, store_reg, address);
    mmio->GetHandlerForWrite<u16>(address).Visit(gen);
  }
  else if (flags & BackPatchInfo::FLAG_SIZE_8)
  {
    // No byteswap for 8-bit
    MMIOWriteCodeGenerator32<u8> gen(&system, emit, store_reg, address);
    mmio->GetHandlerForWrite<u8>(address).Visit(gen);
  }
}
