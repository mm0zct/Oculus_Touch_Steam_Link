
/************************************************************************************
Filename    :   Win32_DirectXAppUtil.h
Content     :   D3D11 application/Window setup functionality for RoomTiny
Created     :   October 20th, 2014
Author      :   Tom Heath
Copyright   :   Copyright (c) Facebook Technologies, LLC and its affiliates. All rights reserved.
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*************************************************************************************/
#ifndef OVR_Win32_DirectXAppUtil_h
#define OVR_Win32_DirectXAppUtil_h

#include <cstdint>
#include <string>
#include <vector>
#include "d3dcompiler.h"
#include "d3d11.h"
#include "stdio.h"
#include <new>
#if _MSC_VER > 1600
#include "DirectXMath.h"

using namespace DirectX;
#else
#include "xnamath.h"
#endif //_MSC_VER > 1600

#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "d3dcompiler.lib")

#ifndef VALIDATE
#define VALIDATE(x, msg) if (!(x)) { MessageBoxA(NULL, (msg), "OculusRoomTiny", MB_ICONERROR | MB_OK); exit(-1); }
#endif

// clean up member COM pointers
template<typename T> void Release(T *&obj)
{
    if (!obj) return;
    obj->Release();
    obj = nullptr;
}

//------------------------------------------------------------
struct DepthBuffer
{
    ID3D11DepthStencilView * TexDsv;

    DepthBuffer(ID3D11Device * Device, int sizeW, int sizeH, int sampleCount = 1)
    {
        DXGI_FORMAT format = DXGI_FORMAT_D32_FLOAT;
        D3D11_TEXTURE2D_DESC dsDesc;
        dsDesc.Width = sizeW;
        dsDesc.Height = sizeH;
        dsDesc.MipLevels = 1;
        dsDesc.ArraySize = 1;
        dsDesc.Format = format;
        dsDesc.SampleDesc.Count = sampleCount;
        dsDesc.SampleDesc.Quality = 0;
        dsDesc.Usage = D3D11_USAGE_DEFAULT;
        dsDesc.CPUAccessFlags = 0;
        dsDesc.MiscFlags = 0;
        dsDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
        ID3D11Texture2D * Tex;
        Device->CreateTexture2D(&dsDesc, NULL, &Tex);
        Device->CreateDepthStencilView(Tex, NULL, &TexDsv);
        Tex->Release();
    }
    ~DepthBuffer()
    {
        Release(TexDsv);
    }
};

//----------------------------------------------------------------
struct DataBuffer
{
    ID3D11Buffer * D3DBuffer;
    size_t         Size;

    DataBuffer(ID3D11Device * Device, D3D11_BIND_FLAG use, const void* buffer, size_t size) : Size(size)
    {
        D3D11_BUFFER_DESC desc;   memset(&desc, 0, sizeof(desc));
        desc.Usage = D3D11_USAGE_DYNAMIC;
        desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        desc.BindFlags = use;
        desc.ByteWidth = (unsigned)size;
        D3D11_SUBRESOURCE_DATA sr;
        sr.pSysMem = buffer;
        sr.SysMemPitch = sr.SysMemSlicePitch = 0;
        Device->CreateBuffer(&desc, buffer ? &sr : NULL, &D3DBuffer);
    }
    ~DataBuffer()
    {
        Release(D3DBuffer);
    }
};

//---------------------------------------------------------------------
struct DirectX11
{
	HWND                     Window;
	bool                     Running;
	bool                     Key[256];
	int                      WinSizeW;
	int                      WinSizeH;
	ID3D11Device           * Device;
	ID3D11DeviceContext    * Context;
	IDXGISwapChain         * SwapChain;
	DepthBuffer            * MainDepthBuffer;
	ID3D11Texture2D        * BackBuffer;
	ID3D11RenderTargetView * BackBufferRT;
    // Fixed size buffer for shader constants, before copied into buffer
    static const int         UNIFORM_DATA_SIZE = 2000;
	unsigned char            UniformData[UNIFORM_DATA_SIZE];
	DataBuffer             * UniformBufferGen;
    HINSTANCE                hInstance;

	static LRESULT CALLBACK WindowProc(_In_ HWND hWnd, _In_ UINT Msg, _In_ WPARAM wParam, _In_ LPARAM lParam)
	{
        auto p = reinterpret_cast<DirectX11 *>(GetWindowLongPtr(hWnd, 0));
        switch (Msg)
        {
        case WM_KEYDOWN:
            p->Key[wParam] = true;
            break;
        case WM_KEYUP:
            p->Key[wParam] = false;
            break;
        case WM_DESTROY:
            p->Running = false;
            break;
        default:
            return DefWindowProcW(hWnd, Msg, wParam, lParam);
        }
        if ((p->Key['Q'] && p->Key[VK_CONTROL]) || p->Key[VK_ESCAPE])
        {
            p->Running = false;
        }
        return 0;
	}

    DirectX11() :
        Window(nullptr),
        Running(false),
        WinSizeW(0),
        WinSizeH(0),
        Device(nullptr),
        Context(nullptr),
        SwapChain(nullptr),
        MainDepthBuffer(nullptr),
        BackBuffer(nullptr),
        BackBufferRT(nullptr),
        UniformBufferGen(nullptr),
        hInstance(nullptr)
    {
		// Clear input
		for (int i = 0; i < sizeof(Key)/sizeof(Key[0]); ++i)
            Key[i] = false;
    }

    ~DirectX11()
	{
        ReleaseDevice();
        CloseWindow();
    }

    bool InitWindow(HINSTANCE hinst, LPCWSTR title)
	{
        hInstance = hinst;
		Running = true;

		WNDCLASSW wc;
        memset(&wc, 0, sizeof(wc));
		wc.lpszClassName = L"App";
		wc.style = CS_OWNDC;
		wc.lpfnWndProc = WindowProc;
		wc.cbWndExtra = sizeof(this);
		RegisterClassW(&wc);

        // adjust the window size and show at InitDevice time
		Window = CreateWindowW(wc.lpszClassName, title, WS_OVERLAPPEDWINDOW, 0, 0, 0, 0, 0, 0, hinst, 0);
        if (!Window) return false;

		SetWindowLongPtr(Window, 0, LONG_PTR(this));

        return true;
	}

    void CloseWindow()
    {
        if (Window)
        {
	        DestroyWindow(Window);
	        Window = nullptr;
	        UnregisterClassW(L"App", hInstance);
        }
    }

    bool InitDevice(int vpW, int vpH, const LUID* pLuid, bool windowed = true)
	{
		WinSizeW = vpW;
		WinSizeH = vpH;

		RECT size = { 0, 0, vpW, vpH };
		AdjustWindowRect(&size, WS_OVERLAPPEDWINDOW, false);
        const UINT flags = SWP_NOMOVE | SWP_NOZORDER;
        if (!SetWindowPos(Window, nullptr, 0, 0, 0, 0, flags))
            return false;

		IDXGIFactory * DXGIFactory = nullptr;
		HRESULT hr = CreateDXGIFactory1(__uuidof(IDXGIFactory), (void**)(&DXGIFactory));
        VALIDATE((hr == ERROR_SUCCESS), "CreateDXGIFactory1 failed");

		IDXGIAdapter * Adapter = nullptr;
        for (UINT iAdapter = 0; DXGIFactory->EnumAdapters(iAdapter, &Adapter) != DXGI_ERROR_NOT_FOUND; ++iAdapter)
        {
            DXGI_ADAPTER_DESC adapterDesc;
            Adapter->GetDesc(&adapterDesc);
            if ((pLuid == nullptr) || memcmp(&adapterDesc.AdapterLuid, pLuid, sizeof(LUID)) == 0)
                break;
            Release(Adapter);
        }

        auto DriverType = Adapter ? D3D_DRIVER_TYPE_UNKNOWN : D3D_DRIVER_TYPE_HARDWARE;
		hr = D3D11CreateDevice(Adapter, DriverType, 0, 0, 0, 0, D3D11_SDK_VERSION, &Device, 0, &Context);
        Release(Adapter);
        VALIDATE((hr == ERROR_SUCCESS), "D3D11CreateDevice failed");

		// Create swap chain
		DXGI_SWAP_CHAIN_DESC scDesc;
        memset(&scDesc, 0, sizeof(scDesc));
		scDesc.BufferCount = 2;
		scDesc.BufferDesc.Width = WinSizeW;
		scDesc.BufferDesc.Height = WinSizeH;
		scDesc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
		scDesc.BufferDesc.RefreshRate.Denominator = 1;
		scDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
		scDesc.OutputWindow = Window;
		scDesc.SampleDesc.Count = 1;
		scDesc.Windowed = windowed;
		scDesc.SwapEffect = DXGI_SWAP_EFFECT_SEQUENTIAL;
		hr = DXGIFactory->CreateSwapChain(Device, &scDesc, &SwapChain);
        Release(DXGIFactory);
        VALIDATE((hr == ERROR_SUCCESS), "CreateSwapChain failed");

		// Create backbuffer
		SwapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), (void**)&BackBuffer);
		hr = Device->CreateRenderTargetView(BackBuffer, NULL, &BackBufferRT);
        VALIDATE((hr == ERROR_SUCCESS), "CreateRenderTargetView failed");

		// Main depth buffer
		MainDepthBuffer = new DepthBuffer(Device, WinSizeW, WinSizeH);
		Context->OMSetRenderTargets(1, &BackBufferRT, MainDepthBuffer->TexDsv);

		// Buffer for shader constants
        UniformBufferGen = new DataBuffer(Device, D3D11_BIND_CONSTANT_BUFFER, NULL, UNIFORM_DATA_SIZE);
		Context->VSSetConstantBuffers(0, 1, &UniformBufferGen->D3DBuffer);

		// Set max frame latency to 1
		IDXGIDevice1* DXGIDevice1 = nullptr;
	    hr = Device->QueryInterface(__uuidof(IDXGIDevice1), (void**)&DXGIDevice1);
        VALIDATE((hr == ERROR_SUCCESS), "QueryInterface failed");
        DXGIDevice1->SetMaximumFrameLatency(1);
		Release(DXGIDevice1);

        return true;
	}

	void SetAndClearRenderTarget(ID3D11RenderTargetView * rendertarget, ID3D11DepthStencilView * depthtarget, float R = 0, float G = 0, float B = 0, float A = 0)
	{
		float black[] = { R, G, B, A }; // Important that alpha=0, if want pixels to be transparent, for manual layers
        Context->OMSetRenderTargets(1, &rendertarget, depthtarget);
		Context->ClearRenderTargetView(rendertarget, black);
        if (depthtarget)
            Context->ClearDepthStencilView(depthtarget, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1, 0);
	}

    void SetAndClearRenderTarget(ID3D11RenderTargetView * rendertarget, struct DepthBuffer * depthbuffer, float R = 0, float G = 0, float B = 0, float A = 0) {
        SetAndClearRenderTarget(rendertarget, (depthbuffer ? depthbuffer->TexDsv : nullptr), R, G, B, A);
    }

	void SetViewport(float vpX, float vpY, float vpW, float vpH)
	{
		D3D11_VIEWPORT D3Dvp;
		D3Dvp.Width = vpW;    D3Dvp.Height = vpH;
		D3Dvp.MinDepth = 0;   D3Dvp.MaxDepth = 1;
		D3Dvp.TopLeftX = vpX; D3Dvp.TopLeftY = vpY;
		Context->RSSetViewports(1, &D3Dvp);
	}

	bool HandleMessages(void)
	{
		MSG msg;
		while (PeekMessage(&msg, NULL, 0U, 0U, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		// This is to provide a means to terminate after a maximum number of frames
		// to facilitate automated testing
        #ifdef MAX_FRAMES_ACTIVE
            if (maxFrames > 0)
            {
		        if (--maxFrames <= 0)
			        Running = false;
            }
        #endif
		return Running;
	}

    void Run(bool (*MainLoop)(bool retryCreate))
    {
        // false => just fail on any error
        VALIDATE(MainLoop(false), "Oculus Rift not detected.");
        while (HandleMessages())
        {
            // true => we'll attempt to retry for ovrError_DisplayLost
            if (!MainLoop(true))
                break;
            // Sleep a bit before retrying to reduce CPU load while the HMD is disconnected
            Sleep(10);
        }
    }

	void ReleaseDevice()
	{
        Release(BackBuffer);
        Release(BackBufferRT);
        if (SwapChain)
        {
            SwapChain->SetFullscreenState(FALSE, NULL);
            Release(SwapChain);
        }
        Release(Context);
        Release(Device);
        delete MainDepthBuffer;
        MainDepthBuffer = nullptr;
        delete UniformBufferGen;
        UniformBufferGen = nullptr;
	}
};

// global DX11 state
//extern struct DirectX11 DIRECTX;


#endif // OVR_Win32_DirectXAppUtil_h


