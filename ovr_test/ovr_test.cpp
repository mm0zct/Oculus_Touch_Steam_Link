// ovr_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>

#ifndef NOMINMAX
#define NOMINMAX
#endif

#define   OVR_D3D_VERSION 11
#pragma warning(disable: 4324)
#include "Win32_DirectXAppUtil.h" // DirectX Helper
#include <OVR_CAPI_D3D.h>

#include <Windows.h>
#include <mutex>

//#define MAX_HAPTICS

struct shared_buffer {
    ovrInputState input_state;
    uint32_t vrEvent_type;
    float vib_amplitude[2];
    float vib_frequency[2];
    float vib_duration_s[2];
    bool vib_valid[2];
};

DirectX11 DIRECTX;


class GuardianSystemDemo
{
public:
    void Start(HINSTANCE hinst);
    void InitRenderTargets(const ovrHmdDesc& hmdDesc);


    void  Render();

private:
    //    XMVECTOR mObjPosition[Scene::MAX_MODELS];                               // Objects cached position 
    //    XMVECTOR mObjVelocity[Scene::MAX_MODELS];                               // Objects velocity
    //    Scene mDynamicScene;                                                    // Scene graph

    ovrSession mSession = nullptr;
    //   high_resolution_clock mLastUpdateClock;                                 // Stores last update time
    //   float mGlobalTimeSec = 0;                                               // Game global time

    uint32_t mFrameIndex = 0;                                               // Global frame counter
    ovrPosef mHmdToEyePose[ovrEye_Count] = {};                              // Offset from the center of the HMD to each eye
    ovrRecti mEyeRenderViewport[ovrEye_Count] = {};                         // Eye render target viewport

    ovrLayerEyeFov mEyeRenderLayer = {};                                    // OVR  - Eye render layers description
    ovrTextureSwapChain mTextureChain[ovrEye_Count] = {};                   // OVR  - Eye render target swap chain
    ID3D11DepthStencilView* mEyeDepthTarget[ovrEye_Count] = {};             // DX11 - Eye depth view
    std::vector<ID3D11RenderTargetView*> mEyeRenderTargets[ovrEye_Count];   // DX11 - Eye render view

    bool mShouldQuit = false;
    //    bool mSlowMotionMode = false;                                           // Slow motion gets enabled when too close to the boundary
};


void GuardianSystemDemo::InitRenderTargets(const ovrHmdDesc& hmdDesc)
{
    // For each eye
    for (int i = 0; i < ovrEye_Count; ++i) {
        // Viewport
        const float kPixelsPerDisplayPixel = 1.0f;
        ovrSizei idealSize = ovr_GetFovTextureSize(mSession, (ovrEyeType)i, hmdDesc.DefaultEyeFov[i], kPixelsPerDisplayPixel);
        mEyeRenderViewport[i] = { 0, 0, idealSize.w, idealSize.h };

        // Create Swap Chain
        ovrTextureSwapChainDesc desc = {
            ovrTexture_2D, OVR_FORMAT_R8G8B8A8_UNORM_SRGB, 1, idealSize.w, idealSize.h, 1, 1,
            ovrFalse, ovrTextureMisc_DX_Typeless, ovrTextureBind_DX_RenderTarget
        };

        // Configure Eye render layers
        mEyeRenderLayer.Header.Type = ovrLayerType_EyeFov;
        mEyeRenderLayer.Viewport[i] = mEyeRenderViewport[i];
        mEyeRenderLayer.Fov[i] = hmdDesc.DefaultEyeFov[i];
        mHmdToEyePose[i] = ovr_GetRenderDesc(mSession, (ovrEyeType)i, hmdDesc.DefaultEyeFov[i]).HmdToEyePose;

        // DirectX 11 - Generate RenderTargetView from textures in swap chain
        // ----------------------------------------------------------------------
        ovrResult result = ovr_CreateTextureSwapChainDX(mSession, DIRECTX.Device, &desc, &mTextureChain[i]);
        if (!OVR_SUCCESS(result)) {
            printf("ovr_CreateTextureSwapChainDX failed"); exit(-1);
        }

        // Render Target, normally triple-buffered
        int textureCount = 0;
        ovr_GetTextureSwapChainLength(mSession, mTextureChain[i], &textureCount);
        for (int j = 0; j < textureCount; ++j) {
            ID3D11Texture2D* renderTexture = nullptr;
            ovr_GetTextureSwapChainBufferDX(mSession, mTextureChain[i], j, IID_PPV_ARGS(&renderTexture));

            D3D11_RENDER_TARGET_VIEW_DESC renderTargetViewDesc = {
                DXGI_FORMAT_R8G8B8A8_UNORM, D3D11_RTV_DIMENSION_TEXTURE2D
            };

            ID3D11RenderTargetView* renderTargetView = nullptr;
            DIRECTX.Device->CreateRenderTargetView(renderTexture, &renderTargetViewDesc, &renderTargetView);
            mEyeRenderTargets[i].push_back(renderTargetView);
            renderTexture->Release();
        }

        // DirectX 11 - Generate Depth
        // ----------------------------------------------------------------------
        D3D11_TEXTURE2D_DESC depthTextureDesc = {
            (UINT)idealSize.w, (UINT)idealSize.h, 1, 1, DXGI_FORMAT_D32_FLOAT, {1, 0},
            D3D11_USAGE_DEFAULT, D3D11_BIND_DEPTH_STENCIL, 0, 0
        };

        ID3D11Texture2D* depthTexture = nullptr;
        DIRECTX.Device->CreateTexture2D(&depthTextureDesc, NULL, &depthTexture);
        DIRECTX.Device->CreateDepthStencilView(depthTexture, NULL, &mEyeDepthTarget[i]);
        depthTexture->Release();
    }
}




void GuardianSystemDemo::Start(HINSTANCE hinst)
{
    hinst = hinst;
    ovrResult result;
    result = ovr_Initialize(nullptr);
    if (!OVR_SUCCESS(result)) {
        printf("ovr_Initialize failed"); exit(-1);
    }

    ovrGraphicsLuid luid;
    result = ovr_Create(&mSession, &luid);
    if (!OVR_SUCCESS(result)) {
        printf("ovr_Create failed"); exit(-1);
    }

    if (!DIRECTX.InitWindow(0/*hinst*/, L"GuardianSystemDemo")) {
        printf("DIRECTX.InitWindow failed"); exit(-1);
    }

    // Use HMD desc to initialize device
    ovrHmdDesc hmdDesc = ovr_GetHmdDesc(mSession);
    if (!DIRECTX.InitDevice(hmdDesc.Resolution.w / 2, hmdDesc.Resolution.h / 2, reinterpret_cast<LUID*>(&luid))) {
        printf("DIRECTX.InitDevice failed"); exit(-1);
    }

    // Use FloorLevel tracking origin
    ovr_SetTrackingOriginType(mSession, ovrTrackingOrigin_FloorLevel);

    InitRenderTargets(hmdDesc);
    //InitSceneGraph();
    //mLastUpdateClock = std::chrono::high_resolution_clock::now();
    HANDLE hMapFile;
    shared_buffer* comm_buffer;

#if 1
    hMapFile = CreateFileMapping(
        INVALID_HANDLE_VALUE,    // use paging file
        NULL,                    // default security
        PAGE_READWRITE,          // read/write access
        0,                       // maximum object size (high-order DWORD)
        sizeof(shared_buffer) ,                // maximum object size (low-order DWORD)
        L"Local\\oculus_steamvr_touch_controller_data_channel");                 // name of mapping object
#else
hMapFile = OpenFileMapping(
    FILE_MAP_ALL_ACCESS,   // read/write access
    FALSE,                 // do not inherit the name
    L"Global\\oculus_steamvr_touch_controller_data_channel");               // name of mapping object
#endif
    if (hMapFile == NULL)
    {
        std::cout << "Could not open file mapping object " << GetLastError() << std::endl;
        return;
    }

    comm_buffer = new(MapViewOfFile(hMapFile, // handle to map object
        FILE_MAP_ALL_ACCESS,  // read/write permission
        0,
        0,
        sizeof(shared_buffer)))shared_buffer();


    if (comm_buffer == NULL)
    {
        std::cout << "Could not map view of file " << GetLastError() << std::endl;

        CloseHandle(hMapFile);

        return;
    }
    HANDLE comm_mutex = CreateMutex(0, true, L"Local\\oculus_steamvr_touch_controller_mutex");
    //MessageBox(NULL, pBuf, TEXT("Process2"), MB_OK);
    
    WaitForSingleObject(
        comm_mutex,    // handle to mutex
        INFINITE);  // no time-out interval
    ReleaseMutex(comm_mutex);
 
    // Main Loop
    uint64_t counter = 0;
    Render();
    uint64_t frame_count = 0;
    uint8_t buf[128] = { 0 };// , 255, 0, 0, 255, 255, 0, 0, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 0, 0, 255, 255, 255, 255, 0, 0, 0, 0, 255, 255, 255, 255, 0, 0, 0, 0, 255, 255, 255, 255, 0, 0, 0, 0     };

    ovrHapticsBuffer vibuffer;
    vibuffer.Samples = buf;
    vibuffer.SamplesCount = sizeof(buf);
    vibuffer.SubmitMode = ovrHapticsBufferSubmit_Enqueue;

    for (int i = 0; i <( sizeof(buf)/* / 2*/); i++) {
        buf[i/* *2*/] = 255;
    }

    while (DIRECTX.HandleMessages() && !mShouldQuit)
    {
        ovrSessionStatus sessionStatus;
        ovr_GetSessionStatus(mSession, &sessionStatus);
        if (sessionStatus.ShouldQuit)
            break;
        //if((counter &0xf)== 0 )   Render();
        //counter++;

        ovrTrackingState ss = ovr_GetTrackingState(mSession, 0, false);
     
        WaitForSingleObject(comm_mutex, INFINITE);
        if (comm_buffer->vrEvent_type) {
            //std::cout << "VR Event 0x" << comm_buffer->vrEvent_type << std::endl;
            comm_buffer->vrEvent_type = 0;
        }
        for (int i = 0; i < 2; i++) {

            ovrInputState inputState;
            ovrResult input_res;
            if (i == 1) {
                input_res = ovr_GetInputState(mSession, ovrControllerType::ovrControllerType_RTouch, &inputState);
                comm_buffer->input_state.Buttons &= ~ovrButton_RMask;
                comm_buffer->input_state.Buttons |= (ovrButton_RMask & inputState.Buttons);
                comm_buffer->input_state.Touches &= ~(ovrTouch_RButtonMask | ovrTouch_RPoseMask);
                comm_buffer->input_state.Touches |= ((ovrTouch_RButtonMask | ovrTouch_RPoseMask)& inputState.Touches);
            }
            else {
                input_res = ovr_GetInputState(mSession, ovrControllerType::ovrControllerType_LTouch, &inputState);
                comm_buffer->input_state.Buttons &= ~ovrButton_LMask;
                comm_buffer->input_state.Buttons |= (ovrButton_LMask & inputState.Buttons);
                comm_buffer->input_state.Touches &= ~(ovrTouch_LButtonMask | ovrTouch_LPoseMask);
                comm_buffer->input_state.Touches |= ((ovrTouch_LButtonMask | ovrTouch_LPoseMask) & inputState.Touches);
            }
            comm_buffer->input_state.HandTrigger[i] = inputState.HandTrigger[i];
            comm_buffer->input_state.IndexTrigger[i] = inputState.IndexTrigger[i];
            comm_buffer->input_state.Thumbstick[i] = inputState.Thumbstick[i];

            if (comm_buffer->vib_valid[i]) {
                int duration = comm_buffer->vib_duration_s[i] * 320;
                if (duration > 128) duration = 128;
                if (duration < 4) duration = 4;
                vibuffer.SamplesCount = duration;
#ifndef MAX_HAPTICS                
                uint32_t ratio = 1;
                if (comm_buffer->vib_frequency[i] > 80/*300*/) {
                    ratio = 4;
                } else if (comm_buffer->vib_frequency[i] > 66/*230*/) {
                    ratio = 3;
                } else if (comm_buffer->vib_frequency[i] > 40/*160*/) {
                    ratio = 2;
                } else {
                    ratio = 1;
                }
                for (int i = 0; i < duration; i++) {                    
                    if ((i & 3) < ratio) {
                        float v = (0.5f + (comm_buffer->vib_amplitude[i] * 0.55f)) * 255.0f;
                        uint8_t vb=v;
                        if (v > 255.0f) vb = 255;
                        else if (v < 0.5f) vb = 255 / 2;
                        
                        buf[i] = vb; //(comm_buffer->vib_amplitude[i] < 0.5000001) ? 127 : 255;
                    } else {
                        buf[i] = 0;
                    }
                }
#endif
                ovr_SubmitControllerVibration(mSession, (i>0) ? ovrControllerType_RTouch : ovrControllerType_LTouch, &vibuffer);
                comm_buffer->vib_valid[i] = 0;
               if (0) {
                    float v = (0.5f + (comm_buffer->vib_amplitude[i] * 0.55f)) * 255.0f;
                    uint8_t vb = v;
                    if (v > 255.0f) vb = 255;
                    else if (v < 0.5) vb = 255 / 2;
                    std::cout << " BUZZ " << i << " : " << comm_buffer->vib_amplitude[i] << " " << comm_buffer->vib_duration_s[i] << "s " << comm_buffer->vib_frequency[i] << "Hz " << (uint32_t)vb<< " : " << v << " amplitude mapping, ratio = " << ratio << std::endl;
                }
            }
            ReleaseMutex(comm_mutex);
            if ((frame_count & 0x7FF) == 0) {
                std::cout.precision(4);

                std::cout << (i == 0 ? "lhand" : "rhand") << " 0x" << std::fixed <<
                    std::hex << ss.HandStatusFlags[i] << " x " <<
                    ss.HandPoses[i].ThePose.Position.x << " y " <<
                    ss.HandPoses[i].ThePose.Position.y << " z " <<
                    ss.HandPoses[i].ThePose.Position.z << " button 0x" << inputState.Buttons << " touch 0x" << inputState.Touches << " g|t " << inputState.HandTrigger[i] <<"|"<<inputState.IndexTrigger[i] << " res 0x" << input_res;
                //ovr_SetControllerVibration(mSession, i ? ovrControllerType_RTouch : ovrControllerType_LTouch, 1, 255);
                //std::cout << " " << 
                Render();

                //ovrErrorInfo errorInfo;
               // ovr_GetLastErrorInfo(&errorInfo);
               // std::cout << " " << errorInfo.ErrorString;
               /* if (i == 0) {
                    std::cout << " : ";
                }else{ */
                        std::cout << std::endl;
                /*} */
            }
        }

        frame_count++;
        Sleep(1);

    }

    ovr_Shutdown();
    UnmapViewOfFile(comm_buffer);

    CloseHandle(hMapFile);
    CloseHandle(comm_mutex);
}



void GuardianSystemDemo::Render()
{
    // Get current eye pose for rendering
    double eyePoseTime = 0;
    ovrPosef eyePose[ovrEye_Count] = {};
    ovr_GetEyePoses(mSession, mFrameIndex, ovrTrue, mHmdToEyePose, eyePose, &eyePoseTime);

    // Render each eye
    for (int i = 0; i < ovrEye_Count; ++i) {
        int renderTargetIndex = 0;
        ovr_GetTextureSwapChainCurrentIndex(mSession, mTextureChain[i], &renderTargetIndex);
        ID3D11RenderTargetView* renderTargetView = mEyeRenderTargets[i][renderTargetIndex];
        ID3D11DepthStencilView* depthTargetView = mEyeDepthTarget[i];

        // Clear and set render/depth target and viewport
        DIRECTX.SetAndClearRenderTarget(renderTargetView, depthTargetView, 0.0f, 0.0f, 0.0f, 1.0f);  // THE SCREEN RENDER COLOUR
        DIRECTX.SetViewport((float)mEyeRenderViewport[i].Pos.x, (float)mEyeRenderViewport[i].Pos.y,
            (float)mEyeRenderViewport[i].Size.w, (float)mEyeRenderViewport[i].Size.h);

        // Eye
        XMVECTOR eyeRot = XMVectorSet(eyePose[i].Orientation.x, eyePose[i].Orientation.y,
            eyePose[i].Orientation.z, eyePose[i].Orientation.w);
        XMVECTOR eyePos = XMVectorSet(eyePose[i].Position.x, eyePose[i].Position.y, eyePose[i].Position.z, 0);
        XMVECTOR eyeForward = XMVector3Rotate(XMVectorSet(0, 0, -1, 0), eyeRot);

        // Matrices
        XMMATRIX viewMat = XMMatrixLookAtRH(eyePos, XMVectorAdd(eyePos, eyeForward),
            XMVector3Rotate(XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f), eyeRot));
        ovrMatrix4f proj = ovrMatrix4f_Projection(mEyeRenderLayer.Fov[i], 0.001f, 1000.0f, ovrProjection_None);
        XMMATRIX projMat = XMMatrixTranspose(XMMATRIX(&proj.M[0][0]));
        XMMATRIX viewProjMat = XMMatrixMultiply(viewMat, projMat);

        // Render and commit to swap chain
        //mDynamicScene.Render(&viewProjMat, 1.0f, 1.0f, 1.0f, 1.0f, true);
        ovr_CommitTextureSwapChain(mSession, mTextureChain[i]);

        // Update eye layer
        mEyeRenderLayer.ColorTexture[i] = mTextureChain[i];
        mEyeRenderLayer.RenderPose[i] = eyePose[i];
        mEyeRenderLayer.SensorSampleTime = eyePoseTime;
    }

    // Submit frames
    ovrLayerHeader* layers = &mEyeRenderLayer.Header;
    ovrResult result = ovr_SubmitFrame(mSession, mFrameIndex++, nullptr, &layers, 1);
    if (!OVR_SUCCESS(result)) {
        printf("ovr_SubmitFrame failed"); exit(-1);
    }
}





int main( int argc, char** argsv)
{
    std::cout << "Hello World! "<<argc<<std::endl;
#if 0
    GuardianSystemDemo* instance = new (_aligned_malloc(sizeof(GuardianSystemDemo), 16)) GuardianSystemDemo();
    instance->Start(0);
    delete instance;
    return 0;
#endif
    ovrSession mSession = nullptr;
  //  ovrSession hmd2 = nullptr;
    ovrGraphicsLuid luid{};
  //  ovrGraphicsLuid luid2{};
    ovrInitParams initParams = { ovrInit_RequestVersion | ovrInit_FocusAware | ((argc>1)?0:ovrInit_Invisible), OVR_MINOR_VERSION, NULL, 0, 0 };
    /*ovrResult result = ovr_Initialize(&initParams);   */
    if (!mSession) {
#if 0
        if (OVR_FAILURE(ovr_Initialize(nullptr))) std::cout << "ovr_Initialize error" << std::endl;

        if (OVR_FAILURE(ovr_Create(&hmd2, &luid2)))  std::cout << "ovr_Create error" << std::endl;
#endif          
        if (OVR_FAILURE(ovr_Initialize(&initParams)/*ovr_Initialize(nullptr)*/)) std::cout << "ovr_Initialize error" << std::endl;

        if (OVR_FAILURE(ovr_Create(&mSession, &luid)))  std::cout << "ovr_Create error" << std::endl;

        if (OVR_FAILURE(ovr_SetTrackingOriginType(mSession, ovrTrackingOrigin_FloorLevel)))  std::cout << "ovr_SetTrackingOriginType error" << std::endl;
    }
    if (argc > 1) {
        while (1)
            Sleep(1000);
        return 0;
    }

    HANDLE hMapFile;
    shared_buffer* comm_buffer;

#if 1
    hMapFile = CreateFileMapping(
        INVALID_HANDLE_VALUE,    // use paging file
        NULL,                    // default security
        PAGE_READWRITE,          // read/write access
        0,                       // maximum object size (high-order DWORD)
        sizeof(shared_buffer),                // maximum object size (low-order DWORD)
        L"Local\\oculus_steamvr_touch_controller_data_channel");                 // name of mapping object
#else
    hMapFile = OpenFileMapping(
        FILE_MAP_ALL_ACCESS,   // read/write access
        FALSE,                 // do not inherit the name
        L"Global\\oculus_steamvr_touch_controller_data_channel");               // name of mapping object
#endif
    if (hMapFile == NULL)
    {
        std::cout << "Could not open file mapping object " << GetLastError() << std::endl;
        return -1;
    }

    comm_buffer = new(MapViewOfFile(hMapFile, // handle to map object
        FILE_MAP_ALL_ACCESS,  // read/write permission
        0,
        0,
        sizeof(shared_buffer)))shared_buffer();


    if (comm_buffer == NULL)
    {
        std::cout << "Could not map view of file " << GetLastError() << std::endl;

        CloseHandle(hMapFile);

        return -1;
    }
    HANDLE comm_mutex = CreateMutex(0, true, L"Local\\oculus_steamvr_touch_controller_mutex");
    //MessageBox(NULL, pBuf, TEXT("Process2"), MB_OK);

    WaitForSingleObject(
        comm_mutex,    // handle to mutex
        INFINITE);  // no time-out interval
    ReleaseMutex(comm_mutex);

    // Main Loop
    uint64_t counter = 0;
    uint64_t frame_count = 0;
    uint8_t buf[128] = { 0 };// , 255, 0, 0, 255, 255, 0, 0, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 0, 0, 255, 255, 255, 255, 0, 0, 0, 0, 255, 255, 255, 255, 0, 0, 0, 0, 255, 255, 255, 255, 0, 0, 0, 0     };

    ovrHapticsBuffer vibuffer;
    vibuffer.Samples = buf;
    vibuffer.SamplesCount = sizeof(buf);
    vibuffer.SubmitMode = ovrHapticsBufferSubmit_Enqueue;

    for (int i = 0; i < (sizeof(buf)/* / 2*/); i++) {
        buf[i/* *2*/] = 255;
    }

    //ovr_RecenterTrackingOrigin(hmd);
    while (1) {


        ovrSessionStatus sessionStatus;
        ovr_GetSessionStatus(mSession, &sessionStatus);
        if (sessionStatus.ShouldQuit)
            break;
        //if((counter &0xf)== 0 )   Render();
        //counter++;

        ovrTrackingState ss = ovr_GetTrackingState(mSession, 0, false);

        WaitForSingleObject(comm_mutex, INFINITE);
        if (comm_buffer->vrEvent_type) {
            //std::cout << "VR Event 0x" << comm_buffer->vrEvent_type << std::endl;
            comm_buffer->vrEvent_type = 0;
        }
        for (int i = 0; i < 2; i++) {

            ovrInputState inputState;
            ovrResult input_res;
            if (i == 1) {
                input_res = ovr_GetInputState(mSession, ovrControllerType::ovrControllerType_RTouch, &inputState);
                comm_buffer->input_state.Buttons &= ~ovrButton_RMask;
                comm_buffer->input_state.Buttons |= (ovrButton_RMask & inputState.Buttons);
                comm_buffer->input_state.Touches &= ~(ovrTouch_RButtonMask | ovrTouch_RPoseMask);
                comm_buffer->input_state.Touches |= ((ovrTouch_RButtonMask | ovrTouch_RPoseMask) & inputState.Touches);
            }
            else {
                input_res = ovr_GetInputState(mSession, ovrControllerType::ovrControllerType_LTouch, &inputState);
                comm_buffer->input_state.Buttons &= ~ovrButton_LMask;
                comm_buffer->input_state.Buttons |= (ovrButton_LMask & inputState.Buttons);
                comm_buffer->input_state.Touches &= ~(ovrTouch_LButtonMask | ovrTouch_LPoseMask);
                comm_buffer->input_state.Touches |= ((ovrTouch_LButtonMask | ovrTouch_LPoseMask) & inputState.Touches);
            }
            comm_buffer->input_state.HandTrigger[i] = inputState.HandTrigger[i];
            comm_buffer->input_state.IndexTrigger[i] = inputState.IndexTrigger[i];
            comm_buffer->input_state.Thumbstick[i] = inputState.Thumbstick[i];

            if (comm_buffer->vib_valid[i]) {
                int duration = comm_buffer->vib_duration_s[i] * 320;
                if (duration > 128) duration = 128;
                if (duration < 4) duration = 4;
                vibuffer.SamplesCount = duration;
#ifndef MAX_HAPTICS                
                uint32_t ratio = 1;
                if (comm_buffer->vib_frequency[i] > 80/*300*/) {
                    ratio = 4;
                }
                else if (comm_buffer->vib_frequency[i] > 66/*230*/) {
                    ratio = 3;
                }
                else if (comm_buffer->vib_frequency[i] > 40/*160*/) {
                    ratio = 2;
                }
                else {
                    ratio = 1;
                }
                for (int i = 0; i < duration; i++) {
                    if ((i & 3) < ratio) {
                        float v = (0.25f + (comm_buffer->vib_amplitude[i] * 0.8f)) * 255.0f;
                        uint8_t vb = v;
                        if (v > 255.0f) vb = 255;
                        else if (v < 0.25f) vb = 256 / 4;

                        buf[i] = vb; //(comm_buffer->vib_amplitude[i] < 0.5000001) ? 127 : 255;
                    }
                    else {
                        buf[i] = 0;
                    }
                }
#endif
                ovr_SubmitControllerVibration(mSession, (i > 0) ? ovrControllerType_RTouch : ovrControllerType_LTouch, &vibuffer);
                comm_buffer->vib_valid[i] = 0;
                if (0) {
                    float v = (0.5f + (comm_buffer->vib_amplitude[i] * 0.55f)) * 255.0f;
                    uint8_t vb = v;
                    if (v > 255.0f) vb = 255;
                    else if (v < 0.5) vb = 255 / 2;
                    std::cout << " BUZZ " << i << " : " << comm_buffer->vib_amplitude[i] << " " << comm_buffer->vib_duration_s[i] << "s " << comm_buffer->vib_frequency[i] << "Hz " << (uint32_t)vb << " : " << v << " amplitude mapping, ratio = " << ratio << std::endl;
                }
            }
            ReleaseMutex(comm_mutex);
            if ((frame_count & 0x7FF) == 0) {
                std::cout.precision(4);

                std::cout << (i == 0 ? "lhand" : "rhand") << " 0x" << std::fixed <<
                    std::hex << ss.HandStatusFlags[i] << " x " <<
                    ss.HandPoses[i].ThePose.Position.x << " y " <<
                    ss.HandPoses[i].ThePose.Position.y << " z " <<
                    ss.HandPoses[i].ThePose.Position.z << " button 0x" << inputState.Buttons << " touch 0x" << inputState.Touches << " g|t " << inputState.HandTrigger[i] << "|" << inputState.IndexTrigger[i] << " res 0x" << input_res;
                //ovr_SetControllerVibration(mSession, i ? ovrControllerType_RTouch : ovrControllerType_LTouch, 1, 255);
                //std::cout << " " << 
  
                //ovrErrorInfo errorInfo;
               // ovr_GetLastErrorInfo(&errorInfo);
               // std::cout << " " << errorInfo.ErrorString;
               /* if (i == 0) {
                    std::cout << " : ";
                }else{ */
                std::cout << std::endl;
                /*} */
            }
        }

        frame_count++;
        Sleep(1);
    }
    ovr_Destroy(mSession);
    ovr_Shutdown();

}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
