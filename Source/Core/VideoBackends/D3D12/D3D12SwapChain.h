// Copyright 2019 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <d3d12.h>
#include <memory>
#include <vector>

#include "Common/CommonTypes.h"
#include "Common/WindowSystemInfo.h"
#include "VideoBackends/D3D12/Common.h"
#include "VideoBackends/D3D12/DX12Texture.h"
#include "VideoBackends/D3DCommon/SwapChain.h"
#include "VideoCommon/TextureConfig.h"

namespace DX12
{
class SwapChain : public D3DCommon::SwapChain
{
public:
  SwapChain(const WindowSystemInfo& wsi, IDXGIFactory* dxgi_factory,
            ID3D12CommandQueue* d3d_command_queue);
  ~SwapChain() override;

  static std::unique_ptr<SwapChain> Create(const WindowSystemInfo& wsi);

  bool Present() override;

  DXTexture* GetCurrentTexture() const { return m_buffers[m_current_buffer].texture.get(); }
  DXFramebuffer* GetCurrentFramebuffer() const
  {
    return m_buffers[m_current_buffer].framebuffer.get();
  }

#ifdef __LIBRETRO__
  void AddTexture(std::unique_ptr<DX12::DXTexture> new_tex,
                  std::unique_ptr<DX12::DXFramebuffer> new_fb)
  {
    BufferResources buffer;
    buffer.texture = std::move(new_tex);
    buffer.framebuffer = std::move(new_fb);
    m_buffers.push_back(std::move(buffer));
  }
  void ResetCurrentBuffer() { m_current_buffer = 0; }
#endif

protected:
  bool CreateSwapChainBuffers() override;
  void DestroySwapChainBuffers() override;

private:
  struct BufferResources
  {
    std::unique_ptr<DXTexture> texture;
    std::unique_ptr<DXFramebuffer> framebuffer;
  };

  std::vector<BufferResources> m_buffers;
  u32 m_current_buffer = 0;
};

}  // namespace DX12
