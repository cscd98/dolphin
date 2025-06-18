// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include "Common/ArmEmitter.h"
#include "Core/PowerPC/JitCommon/JitCache.h"

class JitBase;

typedef void (*CompiledCode)();

class JitArmBlockCache : public JitBaseBlockCache
{
public:
  explicit JitArmBlockCache(JitBase& jit);

  void WriteLinkBlock(ArmGen::ARMXEmitter& emit, const JitBlock::LinkData& source,
                      const JitBlock* dest = nullptr);

private:
  void WriteLinkBlock(const JitBlock::LinkData& source, const JitBlock* dest) override;
  void WriteDestroyBlock(const JitBlock& block) override;
};
