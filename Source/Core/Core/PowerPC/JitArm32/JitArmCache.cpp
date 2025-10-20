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

  JIT_LOG("[WriteLinkBlock] start=%p dest=%p call=%d BLOCK_LINK_SIZE=%zu", start, dest, hascall, BLOCK_LINK_SIZE);

  u32 temp = 0;
  u32 temp2 = 0;

  if (!dest)
  {
    JIT_LOG("[WriteLinkBlock] No dest: exitAddress=0x%08x call=%d", source.exitAddress, source.call);

    // No destination: set DISPATCHER_PC = exitAddress, then branch to dispatcher
    // use optimize false to avoid too many instructions
    emit.MOVI2R(DISPATCHER_PC, source.exitAddress, false);

    //static_cast<JitArm&>(m_jit).JIT_LOG_NUM("WriteLinkBlock - No dest: Updating DISPATCHER_PC with exitAddress:", source.exitAddress);

    if (source.call)
    {
      //static_cast<JitArm&>(m_jit).JIT_LOG_NUM("WriteLinkBlock - No dest: has source.call\n");
      JIT_LOG("[WriteLinkBlock] No dest: emitting BL to dispatcher");

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
      JIT_LOG("[WriteLinkBlock] No dest: emitted B to dispatcher temp=0x%08x temp2=0x%08x", temp, temp2);
    }
  }
  else
  {
    if (source.call)
    {
        JIT_LOG("[WriteLinkBlock] Has dest and source.call, exitFarcode=%p normalEntry=%p",
              source.exitFarcode, dest->normalEntry);

        // The "fast" BL should be the last instruction for return address matching
        if (emit.GetCodePtr() == start + BLOCK_LINK_FAST_BL_OFFSET - sizeof(u32))
            emit.NOP();
        DEBUG_ASSERT(emit.GetCodePtr() == start + BLOCK_LINK_FAST_BL_OFFSET || emit.HasWriteFailed());

        // Conditional branch: skip to BL if downcount > 0
        FixupBranch fast = emit.B_CC(CC_GT);

        // Slow path: go to timing check
        emit.B(source.exitFarcode);

        // Fast path: call destination
        emit.SetJumpTarget(fast);
        emit.BL(dest->normalEntry);
    }
    else
    {
      //static_cast<JitArm&>(m_jit).JIT_LOG_NUM("WriteLinkBlock - no source.call\n");
      JIT_LOG("[WriteLinkBlock] Has dest, no source.call, normalEntry=%p exitFarcode=%p",
             dest->normalEntry, source.exitFarcode);

      // Direct branch if within range, else fallback
      s64 block_distance = ((s64)dest->normalEntry - (s64)emit.GetCodePtr()) >> 2;
      if (block_distance >= -0x40000 && block_distance <= 0x3FFFF)
      {
        // TODO: not required in JitArm64 - Load downcount and set flags
        //emit.LDR(R12, PPC_REG, PPCSTATE_OFF(downcount));
        //emit.CMP(R12, 0);

        emit.B_CC(CC_GT, dest->normalEntry);  // Branch to dest if downcount > 0

        // Otherwise go to timing check
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
          //static_cast<JitArm&>(m_jit).SafeB(source.exitFarcode, false);
          // Manually emit long branch to exitFarcode using R12 as scratch
          u32 addr = reinterpret_cast<u32>(source.exitFarcode);
          emit.MOVW(R12, addr & 0xFFFF);
          emit.MOVT(R12, addr >> 16);
          emit.BX(R12);

          JIT_LOG("[WriteLinkBlock] SafeB used for exitFarcode=%p", source.exitFarcode);
        }
      }
      else
      {
          FixupBranch slow = emit.B_CC(CC_LE);   // Branch to slow if downcount <= 0
          emit.B(dest->normalEntry);              // Fast path
          emit.SetJumpTarget(slow);               // Slow path starts here
          emit.B(source.exitFarcode);             // Go to timing
          JIT_LOG("[WriteLinkBlock] Out of range branch to normalEntry=%p and exitFarcode=%p",
               dest->normalEntry, source.exitFarcode);
      }
    }
  }

  // Pad out to fixed size with BKPT
  const u8* end = start + BLOCK_LINK_SIZE;
  //static_cast<JitArm&>(m_jit).JIT_LOG_NUM(
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
  JIT_LOG("[WriteLinkBlock] End reached start=%p end=%p padded=%d usesafeB=%d hasdest=%d hascall=%d temp=0x%08x temp2=0x%08x",
         start, end, padded, usesafeB, hasdest, hascall, temp, temp2);

  JIT_LOG("[WriteLinkBlock] before padding: ptr=%p, used=%zu bytes",
         emit.GetCodePtr(), emit.GetCodePtr() - start);

  ASSERT_MSG(DYNA_REC, emit.GetCodePtr() == end,
    "start {} emit.GetCodePtr() {} end {} BLOCK_LINK_SIZE {} padded {} safeB {} hasdest {} hascall {} temp {} temp2 {}",
    reinterpret_cast<uintptr_t>(start), reinterpret_cast<uintptr_t>(emit.GetCodePtr()),
    reinterpret_cast<uintptr_t>(end), BLOCK_LINK_SIZE,
    padded, usesafeB, hasdest, hascall, temp, temp2);
}

// Existing function now just sets up the emitter and calls the new helper
void JitArmBlockCache::WriteLinkBlock(const JitBlock::LinkData& source, const JitBlock* dest)
{
  if (!source.exitPtrs)
    return;

  // Added logging
  JIT_LOG("[WriteLinkBlock] source.exitPtrs=%p source.exitAddress=0x%08x dest=%p", source.exitPtrs,
    source.exitAddress, dest);

  static_cast<JitArm&>(m_jit).LogNumFromJIT("JitArmBlockCache::WriteLinkBlock (source, dest) = ", (u32)(uintptr_t)dest);
  static_cast<JitArm&>(m_jit).LogNumFromJIT("location = ", (u32)(uintptr_t)dest);

  const Common::ScopedJITPageWriteAndNoExecute enable_jit_page_writes(source.exitPtrs);
  u8* location = source.exitPtrs;

  // Added logging
  JIT_LOG("[WriteLinkBlock] location=%p", location);

  ARMXEmitter emit(location, location + BLOCK_LINK_SIZE);

  // Added logging
  JIT_LOG("[WriteLinkBlock] Emitting link block at %p (end=%p)", location, location + BLOCK_LINK_SIZE);

  WriteLinkBlock(emit, source, dest);

  // Added logging
  JIT_LOG("[WriteLinkBlock] Finished WriteLinkBlock emit");
  emit.FlushIcache();

  // Added logging
  JIT_LOG("[WriteLinkBlock] Flushed Icache for location=%p", location);
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
