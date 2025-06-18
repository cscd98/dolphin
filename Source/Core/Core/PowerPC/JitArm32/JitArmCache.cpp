// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

// Enable define below to enable oprofile integration. For this to work,
// it requires at least oprofile version 0.9.4, and changing the build
// system to link the Dolphin executable against libopagent.  Since the
// dependency is a little inconvenient and this is possibly a slight
// performance hit, it's not enabled by default, but it's useful for
// locating performance issues.

#include "Core/PowerPC/JitInterface.h"
#include "Core/PowerPC/JitArm32/Jit.h"
#include "Core/PowerPC/JitArm32/JitArmCache.h"

using namespace ArmGen;

JitArmBlockCache::JitArmBlockCache(JitBase& jit) : JitBaseBlockCache{jit}
{
}

void JitArmBlockCache::WriteLinkBlock(ArmGen::ARMXEmitter& emit,
                                      const JitBlock::LinkData& source, const JitBlock* dest)
{
  if (!dest)
  {
    // Load 32-bit immediate into R12
    emit.MOV(R12, Operand2(source.exitAddress & 0xFFFF));         // lower 16 bits
    emit.ORR(R12, R12, Operand2(source.exitAddress & 0xFFFF0000)); // upper bits

    if (source.call)
      emit.BL(m_jit.GetAsmRoutines()->dispatcher);
    else
      emit.B(m_jit.GetAsmRoutines()->dispatcher);

    return;
  }

  if (source.call)
  {
    // Conditional forward call (fast path)
    FixupBranch fast_link = emit.B(CC_PL);  // Positive result (e.g., check passes)

    // Checked entry call (fallback)
    emit.BL(dest->checkedEntry);

    // Link target for fast path
    emit.SetJumpTarget(fast_link);

    // Normal entry call
    emit.BL(dest->normalEntry);
    return;
  }

  // Relative distance for near jump (±32MB in ARM32)
  intptr_t current = reinterpret_cast<intptr_t>(emit.GetCodePtr());
  intptr_t target = reinterpret_cast<intptr_t>(dest->normalEntry);
  intptr_t distance = target - current;

  if (distance >= -0x02000000 && distance <= 0x01FFFFFF)
  {
    // Fast path conditional branch
    emit.B(CC_PL, dest->normalEntry);

    // Checked entry
    emit.B(dest->checkedEntry);

    // Breakpoint trap (debug safety)
    emit.BKPT(101);
    return;
  }

  // Fallback long branch sequence
  FixupBranch fast_link = emit.B(CC_PL);
  emit.B(dest->checkedEntry);
  emit.SetJumpTarget(fast_link);
  emit.B(dest->normalEntry);
}

void JitArmBlockCache::WriteDestroyBlock(const JitBlock& block)
{
  // Only clear the entry points as we might still be within this block.
  ARMXEmitter emit(block.checkedEntry);

  while (emit.GetWritableCodePtr() <= block.normalEntry)
    emit.BKPT(0x123);

  emit.FlushIcache();
}

void JitArmBlockCache::WriteLinkBlock(const JitBlock::LinkData& source, const JitBlock* dest)
{
  u8* location = source.exitPtrs;
  ARMXEmitter emit(location);
  emit.B(dest->checkedEntry);
  emit.FlushIcache();
}
