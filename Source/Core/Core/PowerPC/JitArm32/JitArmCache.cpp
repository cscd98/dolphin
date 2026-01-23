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

  bool usesafeB = false;
  bool hasdest = dest != nullptr;
  bool hascall = source.call;

  //static_cast<JitArm&>(m_jit).LogNumFromJIT("JitArmBlockCache::WriteLinkBlock - start is ",
  //  static_cast<u32>(reinterpret_cast<uintptr_t>(start)));

  u32 temp = 0;
  u32 temp2 = 0;

  if (!dest)
  {
    // No destination: set DISPATCHER_PC = exitAddress, then branch to dispatcher
    // use optimize false to avoid too many instructions
    emit.MOVI2R(DISPATCHER_PC, source.exitAddress, false);

    //static_cast<JitArm&>(m_jit).LogNumFromJIT("WriteLinkBlock - No dest: Updating DISPATCHER_PC with exitAddress:", source.exitAddress);

    if (source.call)
    {
      //static_cast<JitArm&>(m_jit).LogNumFromJIT("WriteLinkBlock - No dest: has source.call\n");

      if (emit.GetCodePtr() == start + BLOCK_LINK_FAST_BL_OFFSET - sizeof(u32))
        emit.NOP();
      DEBUG_ASSERT(emit.GetCodePtr() == start + BLOCK_LINK_FAST_BL_OFFSET || emit.HasWriteFailed());
      emit.BL(m_jit.GetAsmRoutines()->dispatcher);
    }
    else
    {
      temp = static_cast<u32>(reinterpret_cast<uintptr_t>(emit.GetCodePtr()));
      emit.B(m_jit.GetAsmRoutines()->dispatcher);
      temp2 = static_cast<u32>(reinterpret_cast<uintptr_t>(emit.GetCodePtr()));
    }
  }
  else
  {
    if (source.call)
    {
     // static_cast<JitArm&>(m_jit).LogNumFromJIT("WriteLinkBlock - has source.call\n");

      // Call destination block directly, with farcode fallback
      if (emit.GetCodePtr() == start + BLOCK_LINK_FAST_BL_OFFSET - sizeof(u32))
        emit.NOP();
      DEBUG_ASSERT(emit.GetCodePtr() == start + BLOCK_LINK_FAST_BL_OFFSET || emit.HasWriteFailed());

      // First branch to farcode stub
      //static_cast<JitArm&>(m_jit).LogNumFromJIT(
      //  "WriteLinkBlock - Emitting branch to farcode",
      //  static_cast<u32>(reinterpret_cast<uintptr_t>(source.exitFarcode)));

      emit.B(source.exitFarcode);

      // Then emit the BL to the destination
     // static_cast<JitArm&>(m_jit).LogNumFromJIT(
      //  "WriteLinkBlock - Emitting BL to dest normalEntry",
     //   static_cast<u32>(reinterpret_cast<uintptr_t>(dest->normalEntry)));

      emit.BL(dest->normalEntry);
    }
    else
    {
      //static_cast<JitArm&>(m_jit).LogNumFromJIT("WriteLinkBlock - no source.call\n");

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
          usesafeB = true;
          static_cast<JitArm&>(m_jit).SafeB(source.exitFarcode, false);
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
  //static_cast<JitArm&>(m_jit).LogNumFromJIT(
  //  "JitArmBlockCache::WriteLinkBlock - End of block is", static_cast<u32>(reinterpret_cast<uintptr_t>(end)));

  // Use a fixed number of instructions so we have enough room for any patching needed later.
  // Must match BLOCK_LINK_SIZE (currently 3 instructions).
  int padded = 0;

  while (emit.GetCodePtr() < end)
  {
    emit.BKPT(0xAB);
    padded++;
    if (emit.HasWriteFailed())
      return;
  }
  ASSERT_MSG(DYNA_REC, emit.GetCodePtr() == end, "start {} emit.GetCodePtr() {} end {} BLOCK_LINK_SIZE {} padded {} safeB {} hasdest {} hascall {} temp {} temp2 {}",
    reinterpret_cast<uintptr_t>(start), reinterpret_cast<uintptr_t>(emit.GetCodePtr()), reinterpret_cast<uintptr_t>(end), BLOCK_LINK_SIZE,
    padded, usesafeB, hasdest, hascall, temp, temp2);
}

// Existing function now just sets up the emitter and calls the new helper
void JitArmBlockCache::WriteLinkBlock(const JitBlock::LinkData& source, const JitBlock* dest)
{
  if (!source.exitPtrs)
    return;

  static_cast<JitArm&>(m_jit).LogNumFromJIT("JitArmBlockCache::WriteLinkBlock (source, dest) = ", (u32)(uintptr_t)dest);
  static_cast<JitArm&>(m_jit).LogNumFromJIT("location = ", (u32)(uintptr_t)dest);

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

  static_cast<JitArm&>(m_jit).LogNumFromJIT("WriteDestroyBlock (loc, address)");
  static_cast<JitArm&>(m_jit).LogNumFromJIT("  location = ", (u32)(uintptr_t)location);
  static_cast<JitArm&>(m_jit).LogNumFromJIT("  address  = ", address);
  static_cast<JitArm&>(m_jit).LogNumFromJIT("  STR target offset (pc) = ", PPCSTATE_OFF(pc));

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
