// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include "Core/PowerPC/JitCommon/JitCache.h"
#include "Common/ArmEmitter.h"

class JitBase;

typedef void (*CompiledCode)();

class JitArmBlockCache : public JitBaseBlockCache
{
public:
  explicit JitArmBlockCache(JitBase& jit);

	void Init() override;

  void DestroyBlock(JitBlock& block) override;

	const std::vector<std::pair<u8*, u8*>>& GetRangesToFreeNear() const;
  const std::vector<std::pair<u8*, u8*>>& GetRangesToFreeFar() const;

  void ClearRangesToFree();

  void WriteLinkBlock(ArmGen::ARMXEmitter& emit,
    const JitBlock::LinkData& source,
    const JitBlock* dest = nullptr);

  // SafeB might increase to 6? instructions
  static constexpr size_t BLOCK_LINK_SIZE = 3 * sizeof(u32);
  static constexpr size_t BLOCK_LINK_FAST_BL_OFFSET = BLOCK_LINK_SIZE - sizeof(u32);

private:
  void WriteLinkBlock(const JitBlock::LinkData& source, const JitBlock* dest) override;
  void WriteDestroyBlock(const JitBlock& block) override;

	std::vector<std::pair<u8*, u8*>> m_ranges_to_free_on_next_codegen_near;
  std::vector<std::pair<u8*, u8*>> m_ranges_to_free_on_next_codegen_far;
};
