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

void JitArmBlockCache::Init()
{
  JitBaseBlockCache::Init();
  ClearRangesToFree();
}

void JitArmBlockCache::WriteLinkBlock(ArmGen::ARMXEmitter& emit,
                                      const JitBlock::LinkData& source,
                                      const JitBlock* dest)
{
  const u8* start = emit.GetCodePtr();

  printf("Linking block\n");
  fflush(stdout);

  if (!dest)
  {
    // No destination: set DISPATCHER_PC = exitAddress, then branch to dispatcher
    emit.MOVI2R(DISPATCHER_PC, source.exitAddress);

    printf("Updating DISPATCHER_PC with exitAddress: %08x\n", source.exitAddress);
    fflush(stdout);

    if (source.call)
    {
      if (emit.GetCodePtr() == start + BLOCK_LINK_FAST_BL_OFFSET - sizeof(u32))
        emit.NOP();
      DEBUG_ASSERT(emit.GetCodePtr() == start + BLOCK_LINK_FAST_BL_OFFSET || emit.HasWriteFailed());
      emit.BL(m_jit.GetAsmRoutines()->dispatcher);
    }
    else
    {
      emit.B(m_jit.GetAsmRoutines()->dispatcher);
    }
  }
  else
  {
    if (source.call)
    {
      // Call destination block directly, with farcode fallback
      if (emit.GetCodePtr() == start + BLOCK_LINK_FAST_BL_OFFSET - sizeof(u32))
        emit.NOP();
      DEBUG_ASSERT(emit.GetCodePtr() == start + BLOCK_LINK_FAST_BL_OFFSET || emit.HasWriteFailed());

      // First branch to farcode stub
      emit.B(source.exitFarcode);

      // Then emit the BL to the destination
      emit.BL(dest->normalEntry);
    }
    else
    {
      // Direct branch if within range, else fallback
      s64 block_distance = ((s64)dest->normalEntry - (s64)emit.GetCodePtr()) >> 2;
      if (block_distance >= -0x40000 && block_distance <= 0x3FFFF)
      {
        // Direct branch, then farcode fallback
        emit.B(dest->normalEntry);

        intptr_t byte_delta =
        reinterpret_cast<intptr_t>(source.exitFarcode) -
        (reinterpret_cast<intptr_t>(emit.GetCodePtr()) + 8);

        if (byte_delta > -0x2000000 && byte_delta <= 0x1FFFFFF)
        {
          emit.B(source.exitFarcode);
        }
        else
        {
          static_cast<JitArm&>(m_jit).SafeB(source.exitFarcode);
        }
      }
      else
      {
        // Out of range: branch to dest, then farcode
        emit.B(dest->normalEntry);
        emit.B(source.exitFarcode);
      }
    }
  }

  // Pad out to fixed size with BKPT
  const u8* end = start + BLOCK_LINK_SIZE;
  while (emit.GetCodePtr() < end)
  {
    emit.BKPT(0xAB); // ARM32 filler (analogous to ARM64 BRK(101))
    if (emit.HasWriteFailed())
      return;
  }
  ASSERT(emit.GetCodePtr() == end);
}

// Existing function now just sets up the emitter and calls the new helper
void JitArmBlockCache::WriteLinkBlock(const JitBlock::LinkData& source, const JitBlock* dest)
{
  if (!source.exitPtrs)
    return;

  printf("WriteLinkBlock (source, dest=%p)\n", (void*)dest);
  printf("  location = %p\n", source.exitPtrs);
  fflush(stdout);

  const Common::ScopedJITPageWriteAndNoExecute enable_jit_page_writes(source.exitPtrs);
  u8* location = source.exitPtrs;
  ARMXEmitter emit(location, location + BLOCK_LINK_SIZE);

  WriteLinkBlock(emit, source, dest);

  emit.FlushIcache();
}

void JitArmBlockCache::WriteDestroyBlock(const JitBlock& block)
{
  const u8* location = block.normalEntry;
  const u32 address = block.effectiveAddress;

  printf("WriteDestroyBlock (loc, address)\n");
  printf("  location = %p\n", location);
  printf("  address  = 0x%08x\n", address);
  printf("  STR target offset (pc) = %d\n", PPCSTATE_OFF(pc));
  fflush(stdout);

	ARMXEmitter emit((u8 *)location, nullptr); // TODO WEBOS: , m_jit.GetNearCode().GetCodeEnd());
  const Common::ScopedJITPageWriteAndNoExecute enable_jit_page_writes(block.normalEntry);
	emit.MOVI2R(R12, (u32)m_jit.GetAsmRoutines()->dispatcher);
	emit.B(R12);
	emit.FlushIcache();
}

void JitArmBlockCache::DestroyBlock(JitBlock& block)
{
  JitBaseBlockCache::DestroyBlock(block);

  if (block.near_begin != block.near_end)
    m_ranges_to_free_on_next_codegen_near.emplace_back(block.near_begin, block.near_end);
  if (block.far_begin != block.far_end)
    m_ranges_to_free_on_next_codegen_far.emplace_back(block.far_begin, block.far_end);
}

const std::vector<std::pair<u8*, u8*>>& JitArmBlockCache::GetRangesToFreeNear() const
{
  return m_ranges_to_free_on_next_codegen_near;
}

const std::vector<std::pair<u8*, u8*>>& JitArmBlockCache::GetRangesToFreeFar() const
{
  return m_ranges_to_free_on_next_codegen_far;
}

void JitArmBlockCache::ClearRangesToFree()
{
  m_ranges_to_free_on_next_codegen_near.clear();
  m_ranges_to_free_on_next_codegen_far.clear();
}
