#pragma once

#include <libretro.h>
#include "DolphinLibretro/Common/Globals.h"
#include "DolphinLibretro/Common/Options.h"
#include "VideoBackends/Null/NullGfx.h"
#include "VideoBackends/Software/SWOGLWindow.h"
#include "VideoBackends/Software/SWGfx.h"
#include "VideoBackends/Software/SWTexture.h"
#include "VideoCommon/VideoBackendBase.h"
#include "VideoCommon/VideoConfig.h"
#include "Common/Logging/Log.h"
#ifdef _WIN32
#include "VideoBackends/D3D/D3DBase.h"
#include "VideoBackends/D3D/D3DState.h"
#include "VideoBackends/D3D/DXShader.h"
#include "VideoBackends/D3D/DXTexture.h"
#include "VideoBackends/D3D/D3DSwapChain.h"
#include "VideoBackends/D3D12/D3D12Gfx.h"
#include "VideoBackends/D3D12/DX12Context.h"
#include "VideoBackends/D3D12/DX12Shader.h"
#include "VideoBackends/D3D12/DX12Texture.h"
#include "VideoBackends/D3D12/D3D12SwapChain.h"
#include "VideoBackends/D3D12/DescriptorHeapManager.h"
#endif
#ifdef HAS_VULKAN
#ifndef __APPLE__
#include "VideoBackends/Vulkan/VulkanLoader.h"
#endif
#include "VideoBackends/Vulkan/VulkanContext.h"
#include "DolphinLibretro/Vulkan.h"
#include <libretro_vulkan.h>
#endif

namespace Libretro
{
namespace Video
{
void Init(void);
bool Video_InitializeBackend();
bool SetHWRender(retro_hw_context_type type);
void ContextReset(void);
void ContextDestroy(void);

class SWGfx : public SW::SWGfx
{
public:
  SWGfx()
    : SW::SWGfx(SWOGLWindow::Create(
            WindowSystemInfo(WindowSystemType::Libretro, nullptr, nullptr, nullptr)))
  {
  }
  void ShowImage(const AbstractTexture* source_texture,
                 const MathUtil::Rectangle<int>& source_rc) override
  {
    SW::SWGfx::ShowImage(source_texture, source_rc);
    video_cb(
      static_cast<const SW::SWTexture*>(source_texture)->GetData(0, 0),
      source_rc.GetWidth(),
      source_rc.GetHeight(),
      source_texture->GetWidth() * 4
    );
    UpdateActiveConfig();
  }
};

class NullGfx : public Null::NullGfx
{
public:
  void ShowImage(const AbstractTexture* source_texture,
                 const MathUtil::Rectangle<int>& source_rc) override
  {
    video_cb(NULL, 512, 512, 512 * 4);
    UpdateActiveConfig();
  }
};

#ifdef _WIN32
class DX11SwapChain : public DX11::SwapChain
{
public:
  DX11SwapChain(const WindowSystemInfo& wsi, int width, int height,
                IDXGIFactory* dxgi_factory, ID3D11Device* d3d_device)
      : DX11::SwapChain(wsi, dxgi_factory, d3d_device)
  {
    m_width = width;
    m_height = height;
    m_stereo = WantsStereo();
    //CreateSwapChainBuffers();
  }

  bool Present() override
  {
    auto* tex = GetTexture();
    if (!tex)
    {
        ERROR_LOG_FMT(VIDEO, "Present aborted: no swap chain texture");
        return false;
    }

    auto* srv = tex->GetD3DSRV();
    if (!srv)
    {
        ERROR_LOG_FMT(VIDEO, "Present aborted: no SRV for swap chain texture");
        return false;
    }

    ID3D11RenderTargetView* nullView = nullptr;
    DX11::D3D::context->OMSetRenderTargets(1, &nullView, nullptr);
    DX11::D3D::context->PSSetShaderResources(0, 1, &srv);

    Libretro::Video::video_cb(RETRO_HW_FRAME_BUFFER_VALID,
                              m_width, m_height,
                              m_width);

    DX11::D3D::stateman->Restore();
    return true;
  }

protected:
  bool CreateSwapChainBuffers() override
  {
    TextureConfig config(m_width, m_height, 1, 1, 1,
                         AbstractTextureFormat::RGBA8,
                         AbstractTextureFlag_RenderTarget,
                         AbstractTextureType::Texture_2D);

    auto tex = DX11::DXTexture::Create(config, "LibretroSwapChainTexture");
    if (!tex)
    {
      ERROR_LOG_FMT(VIDEO, "Backbuffer texture creation failed");
      return false;
    }
    SetTexture(std::move(tex));

    auto fb = DX11::DXFramebuffer::Create(GetTexture(), nullptr, {});
    if (!fb)
    {
      ERROR_LOG_FMT(VIDEO, "Backbuffer framebuffer creation failed");
      return false;
    }
    SetFramebuffer(std::move(fb));

    return true;
  }
};

class DX12SwapChain : public DX12::SwapChain
{
public:
  DX12SwapChain(const WindowSystemInfo& wsi, int width, int height,
                IDXGIFactory* dxgi_factory, ID3D12CommandQueue* d3d_command_queue)
      : DX12::SwapChain(wsi, dxgi_factory, d3d_command_queue)
  {
    m_width = width;
    m_height = height;
    m_stereo = WantsStereo();
    CreateNullRTV();
    CreateSwapChainBuffers();
  }

  bool Present() override
  {
    auto* tex = GetCurrentTexture();
    if (!tex)
    {
        ERROR_LOG_FMT(VIDEO, "Present aborted: no swap chain texture");
        return false;
    }

    const DX12::DescriptorHandle& srv = tex->GetSRVDescriptor();
    if (!srv)
    {
        ERROR_LOG_FMT(VIDEO, "Present aborted: no SRV for swap chain texture");
        return false;
    }

    DX12::g_dx_context->GetCommandList()->OMSetRenderTargets(1, &m_null_rtv, FALSE, nullptr);
    auto* cmdlist = DX12::g_dx_context->GetCommandList();

    cmdlist->SetGraphicsRootDescriptorTable(DX12::ROOT_PARAMETER_PS_SRV, srv.gpu_handle);

    Libretro::Video::video_cb(RETRO_HW_FRAME_BUFFER_VALID,
                              m_width, m_height,
                              m_width);

    static_cast<DX12::Gfx*>(g_gfx.get())->ApplyState();

    return true;
  }

protected:
  bool CreateSwapChainBuffers() override
  {
    DX12::DXContext* ctx = DX12::g_dx_context.get();
    if (!ctx)
    {
      ERROR_LOG_FMT(VIDEO, "DX12 context is null");
      return false;
    }

    ID3D12Device* device = ctx->GetDevice();
    ID3D12CommandQueue* queue = ctx->GetCommandQueue();

    if (!device || !queue)
    {
      ERROR_LOG_FMT(VIDEO, "DX12 device or command queue is null");
      return false;
    }

    TextureConfig config(m_width, m_height, 1, 1, 1,
                         AbstractTextureFormat::RGBA8,
                         AbstractTextureFlag_RenderTarget,
                         AbstractTextureType::Texture_2D);

    for (u32 i = 0; i < SWAP_CHAIN_BUFFER_COUNT; i++)
    {
      auto tex = DX12::DXTexture::Create(config, "LibretroSwapChainTexture");
      ASSERT_MSG(VIDEO, tex != nullptr, "Failed to create swap chain buffer texture");
      if (!tex)
        return false;

      auto fb = DX12::DXFramebuffer::Create(tex.get(), nullptr, {});
      ASSERT_MSG(VIDEO, fb != nullptr,
                "Failed to create swap chain buffer framebuffer");
      if (!fb)
        return false;

      AddTexture(std::move(tex), std::move(fb));
    }

    ResetCurrentBuffer();

    return true;
  }
private:
  void CreateNullRTV()
  {
    if (!m_null_rtv_heap)
    {
      D3D12_DESCRIPTOR_HEAP_DESC desc = {};
      desc.NumDescriptors = 1;
      desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
      desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
      HRESULT hr = DX12::g_dx_context->GetDevice()->CreateDescriptorHeap(
        &desc, IID_PPV_ARGS(&m_null_rtv_heap));

      if (FAILED(hr))
      {
          ERROR_LOG_FMT(VIDEO, "CreateDescriptorHeap failed: 0x{:08X}",
                        static_cast<unsigned>(hr));
          return;
      }
    }

    m_null_rtv = m_null_rtv_heap->GetCPUDescriptorHandleForHeapStart();

    D3D12_RENDER_TARGET_VIEW_DESC null_desc = {};
    null_desc.Format = DXGI_FORMAT_B8G8R8A8_UNORM;
    null_desc.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2D;
    null_desc.Texture2D.MipSlice = 0;
    null_desc.Texture2D.PlaneSlice = 0;

    DX12::g_dx_context->GetDevice()->CreateRenderTargetView(nullptr, &null_desc, m_null_rtv);
  }

  Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> m_null_rtv_heap;
  D3D12_CPU_DESCRIPTOR_HANDLE m_null_rtv = {};
};

#endif

namespace Vk
{
const VkApplicationInfo* GetApplicationInfo(void);

#ifdef __APPLE__
VkInstance CreateInstance(PFN_vkGetInstanceProcAddr get_instance_proc_addr,
                          const VkApplicationInfo* app,
                          retro_vulkan_create_instance_wrapper_t create_instance_wrapper,
                          void* opaque);
#endif
bool CreateDevice(retro_vulkan_context* context, VkInstance instance, VkPhysicalDevice gpu,
                  VkSurfaceKHR surface, PFN_vkGetInstanceProcAddr get_instance_proc_addr,
                  const char** required_device_extensions,
                  unsigned num_required_device_extensions,
                  const char** required_device_layers, unsigned num_required_device_layers,
                  const VkPhysicalDeviceFeatures* required_features);
} // namespace Vk

}  // namespace Video
}  // namespace Libretro
