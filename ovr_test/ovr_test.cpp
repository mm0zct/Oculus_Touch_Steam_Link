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
#include <thread>

//#define MAX_HAPTICS

struct shared_buffer {
    ovrInputState input_state;
    uint32_t vrEvent_type;
    float vib_amplitude[2];
    float vib_frequency[2];
    float vib_duration_s[2];
    bool vib_valid[2];
    uint32_t vr_universe;
    bool perform_prediction;
    bool be_objects;
    float extra_prediction_ms;
    char tracking_space_name[128]; //oculus or Oculus
    char manufacturer_name[128]; //Oculus or Oculus_link
    char logging_buffer[1024];
    uint64_t logging_offset;
    bool external_tracking;
    ovrTrackingState tracking_state;
    uint32_t num_objects;
    ovrPoseStatef object_poses[4];
    bool track_hmd;
    bool update_offsets;
};

DirectX11 DIRECTX;

uint32_t future_vib_buffer_index[2] = { 0 };
double vib_buf_time[2] = { 0 };
//uint8_t future_vib_buffer[1024] = { 0 };
uint8_t future_vib_buffer[2][1024] = { {0},{0} };

void add_vibration(bool isRightHand, float amplitude, float frequency, float duration);

bool just_updated;
void main_loop(ovrSession mSession, HANDLE comm_mutex, shared_buffer* comm_buffer, uint64_t frame_count, ovrHapticsBuffer& vibuffer, uint8_t* buf, unsigned int sizeof_buf) {
    
    if ((GetKeyState(VK_RETURN) < 0 || GetKeyState(VK_SPACE) < 0) && GetForegroundWindow() == GetConsoleWindow()) { // kinda scuffed
        if (!just_updated)
            comm_buffer->update_offsets = true;
        just_updated = true;
    }
    else
    {
        just_updated = false;
    }

    ovrTrackingState ss = ovr_GetTrackingState(mSession, 0, false);

    WaitForSingleObject(comm_mutex, INFINITE);
    if (comm_buffer->logging_offset) {
        for (int i = 0; i < comm_buffer->logging_offset; i++) {
            putc(comm_buffer->logging_buffer[i], stdout);
        }
        comm_buffer->logging_offset = 0;
    }
    if (comm_buffer->vrEvent_type) {
        //std::cout << "VR Event 0x" << comm_buffer->vrEvent_type << std::endl;
        comm_buffer->vrEvent_type = 0;
    }

    if (comm_buffer->external_tracking) {
        comm_buffer->tracking_state = ovr_GetTrackingState(mSession, (ovr_GetTimeInSeconds() + (comm_buffer->extra_prediction_ms * 0.001)), ovrTrue);
    }

    for (int i = 0; i < comm_buffer->num_objects; i++) {
        ovrTrackedDeviceType deviceType = (ovrTrackedDeviceType)(ovrTrackedDevice_Object0 + i);
        ovrPoseStatef ovr_pose;

        ovr_GetDevicePoses(mSession, &deviceType, 1, (ovr_GetTimeInSeconds() + (comm_buffer->extra_prediction_ms * 0.001)), &ovr_pose);
        if ((ovr_pose.ThePose.Orientation.x != 0) && (ovr_pose.ThePose.Orientation.y != 0) && (ovr_pose.ThePose.Orientation.z != 0)){
            comm_buffer->object_poses[i] = ovr_pose;
        }
        if ((frame_count & 0x7FF) == 0) {
            std::cout.precision(4);

            std::cout << "Object" << i << std::dec << " x " <<
                ovr_pose.ThePose.Position.x << " y " <<
                ovr_pose.ThePose.Position.y << " z " <<
                ovr_pose.ThePose.Position.z << std::endl;
        }
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
            add_vibration(i == 1, comm_buffer->vib_frequency[i], comm_buffer->vib_amplitude[i], comm_buffer->vib_duration_s[i]);
            comm_buffer->vib_valid[i] = 0;
            //printf("vib[%u] dur %f=%f | freq %f | amp %f\n", i, comm_buffer->vib_duration_s[i], comm_buffer->vib_duration_s[i]*320, comm_buffer->vib_frequency[i], comm_buffer->vib_amplitude[i]);
#if 0
            int duration = comm_buffer->vib_duration_s[i] * 320;
            if (duration > 128) duration = 128;
            if (duration < 2) duration = 2;
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
            for (int j = 0; j < duration; j++) {
                if ((j & 3) < ratio) {
                    float v = (0.1f + (comm_buffer->vib_amplitude[i] * 0.9f)) * 255.0f;
                    uint8_t vb = v;
                    if (v > 255.0f) vb = 255;
                    else if (v < 0.1f) vb = 256 / 10;

                    buf[j] = vb; //(comm_buffer->vib_amplitude[i] < 0.5000001) ? 127 : 255;
                }
                else {
                    buf[j] = 0;
                }
            }
#endif
            ovr_SubmitControllerVibration(mSession, (i > 0) ? ovrControllerType_RTouch : ovrControllerType_LTouch, &vibuffer);
            comm_buffer->vib_valid[i] = 0;
            for (int j = 0; j < sizeof_buf; j++) {
                buf[j] = 255;
            }
#endif
        }
        ReleaseMutex(comm_mutex);
        if ((frame_count & 0x7FF) == 0) {
            std::cout.precision(4);

            std::cout << (i == 0 ? "lhand" : "rhand") << " 0x" << std::fixed <<
                std::hex << ss.HandStatusFlags[i] << std::dec << " x " <<
                ss.HandPoses[i].ThePose.Position.x << " y " <<
                ss.HandPoses[i].ThePose.Position.y << " z " <<
                ss.HandPoses[i].ThePose.Position.z << std::hex << " button 0x" << inputState.Buttons << " touch 0x" << inputState.Touches << " g|t " << inputState.HandTrigger[i] << "|" << inputState.IndexTrigger[i] << " res 0x" << input_res;
            //ovr_SetControllerVibration(mSession, i ? ovrControllerType_RTouch : ovrControllerType_LTouch, 1, 255);
            //std::cout << " " << 

            //ovrErrorInfo errorInfo;
           // ovr_GetLastErrorInfo(&errorInfo);
           // std::cout << " " << errorInfo.ErrorString;
           /* if (i == 0) {
                std::cout << " : ";
            }else{ */
            std::cout <<std::dec << std::endl;
            /*} */
        }
    }

}

class GuardianSystemDemo
{
public:
    void Start(HINSTANCE hinst, shared_buffer* comm_buffer, HANDLE comm_mutex);
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




void GuardianSystemDemo::Start(HINSTANCE hinst, shared_buffer* comm_buffer, HANDLE comm_mutex)
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
 
    comm_buffer->num_objects = (ovr_GetConnectedControllerTypes(mSession) >> 8) & 0xf;
    
    WaitForSingleObject(
        comm_mutex,    // handle to mutex
        INFINITE);  // no time-out interval
    ReleaseMutex(comm_mutex);
 
    // Main Loop
    uint64_t counter = 0;
    Render();
    uint64_t frame_count = 0;
    uint8_t buf[128] = { 0 };// , 255, 0, 0, 255, 255, 0, 0, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 0, 0, 255, 255, 255, 255, 0, 0, 0, 0, 255, 255, 255, 255, 0, 0, 0, 0, 255, 255, 255, 255, 0, 0, 0, 0     };
    unsigned int sizeof_buf = sizeof(buf);
    ovrHapticsBuffer vibuffer;
    vibuffer.Samples = buf;
    vibuffer.SamplesCount = sizeof_buf;
    vibuffer.SubmitMode = ovrHapticsBufferSubmit_Enqueue;

    for (int i = 0; i <(sizeof_buf/* / 2*/); i++) {
        buf[i/* *2*/] = 255;
    }

    while (DIRECTX.HandleMessages() && !mShouldQuit)
    {
        ovrSessionStatus sessionStatus;
        ovr_GetSessionStatus(mSession, &sessionStatus);
        if (sessionStatus.ShouldQuit)
            break;

        main_loop(mSession, comm_mutex, comm_buffer, frame_count, vibuffer, buf, sizeof_buf);
        
        if ((frame_count & 0xF) == 0) { // 1000/16 ~= 60Hz, render black frame at 60fps to keep oculus happy
            Render();
        }

        frame_count++;
        Sleep(1);

    }

    ovr_Shutdown();
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

void vibration_thread(ovrSession mSession);

void no_graphics_start(shared_buffer* comm_buffer, HANDLE comm_mutex) {
    ovrSession mSession = nullptr;
    //  ovrSession hmd2 = nullptr;
    ovrGraphicsLuid luid{};
    //  ovrGraphicsLuid luid2{};
    ovrInitParams initParams = { ovrInit_RequestVersion | ovrInit_FocusAware | ovrInit_Invisible, OVR_MINOR_VERSION, NULL, 0, 0 };
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


    comm_buffer->num_objects = (ovr_GetConnectedControllerTypes(mSession) >> 8) & 0xf;

    std::thread vib_thread(vibration_thread, mSession);
    //vibration_thread(mSession);

    // Main Loop
    uint64_t counter = 0;
    uint64_t frame_count = 0;
    uint8_t buf[128] = { 0 };
    unsigned int sizeof_buf = sizeof(buf);
    // , 255, 0, 0, 255, 255, 0, 0, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 0, 0, 255, 255, 255, 255, 0, 0, 0, 0, 255, 255, 255, 255, 0, 0, 0, 0, 255, 255, 255, 255, 0, 0, 0, 0     };

    ovrHapticsBuffer vibuffer;
    vibuffer.Samples = buf;
    vibuffer.SamplesCount = sizeof_buf;
    vibuffer.SubmitMode = ovrHapticsBufferSubmit_Enqueue;

    for (int i = 0; i < (sizeof_buf); i++) {
        buf[i] = 255;
    }

    //ovr_RecenterTrackingOrigin(hmd);
    while (1) {


        ovrSessionStatus sessionStatus;
        ovr_GetSessionStatus(mSession, &sessionStatus);
        if (sessionStatus.ShouldQuit)
            break;
        //if((counter &0xf)== 0 )   Render();
        //counter++;

        main_loop(mSession, comm_mutex, comm_buffer, frame_count, vibuffer, buf, sizeof_buf);

        frame_count++;
        Sleep(1);
    }
    ovr_Destroy(mSession);
    ovr_Shutdown();
}


int main(int argc, char** argsv)
{
    std::cout << "Welcome to oculus_touch_link, this program provides the input and haptic link to the stream driver, as well as passing confuguration data" << std::endl;
    std::cout << "You can provide this program with arguments to specify:" << std::endl;
    std::cout << "Render to Oculus headset y/n  (\"n\" must be use with ovr_dummy.exe)" << std::endl;
    std::cout << "VR \"Universe\" ID, 0=Invalid, 1=Oculus, 31=Recommended with VirtualDesktop/ALVR" << std::endl;
    std::cout << "Manufacturer name: Oculus or Oculus_link (or HTC for trackers)" << std::endl;
    std::cout << "Tracking system name: oculus or oculus_link (or lighthouse for trackers)" << std::endl;
    std::cout << "perform tracking prediction manually in-driver (n= ask oculus do the prediction) y/n" << std::endl;
    std::cout << "Extra prediction time (ms) for example 11.1 for 1 frame at 90fps" << std::endl;
    std::cout << "All controllers are tracked objects instead of controllers y/n" << std::endl;
    std::cout << "Perform tracking in ovr_test instead of steamvr driver y/n" << std::endl;
    std::cout << "Track the headset as a tracking object y/n" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "This program is super dumb and expects all of the arguments or none (for defaults), suggested invocations:" << std::endl;
    std::cout << "ovr_test.exe n 1 Oculus oculus n 10 n n(must be use with ovr_dummy.exe)" << std::endl;
    std::cout << "ovr_test.exe y 1 Oculus oculus y 10 n n" << std::endl;
    std::cout << "ovr_test.exe y 31 Oculus_link oculus_link n 10 n y n" << std::endl;
    std::cout << "ovr_test.exe n 31 Oculus_link oculus_link n 10 n n y(default)" << std::endl;


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
    comm_buffer->logging_offset = 0;
    bool do_rendering = false;
    if (argc < 9) {
        std::cout << " <9 arguments, using defaults: n 31 Oculus_link oculus_link n 16 n n y" << std::endl;
        do_rendering = false;
        comm_buffer->vr_universe = 31;
        strncpy_s(comm_buffer->manufacturer_name, "Oculus_link", 127);
        strncpy_s(comm_buffer->tracking_space_name, "oculus_link", 127);
        comm_buffer->perform_prediction = false;
        comm_buffer->extra_prediction_ms = 16.0f;
        comm_buffer->be_objects = false;
        comm_buffer->external_tracking = false;
        comm_buffer->track_hmd = true;
    }
    else {
        do_rendering = (std::string(argsv[1]) == "y");
        comm_buffer->vr_universe = atoi(argsv[2]);
        strncpy_s(comm_buffer->manufacturer_name, argsv[3], 127);
        strncpy_s(comm_buffer->tracking_space_name, argsv[4], 127);
        comm_buffer->perform_prediction = (std::string(argsv[5]) == "y");
        comm_buffer->extra_prediction_ms = atof(argsv[6]);
        comm_buffer->be_objects = (std::string(argsv[7]) == "y");
        comm_buffer->external_tracking = (std::string(argsv[8]) == "y");
        comm_buffer->track_hmd = (std::string(argsv[9]) == "y");
    }

    comm_buffer->update_offsets = true;

    HANDLE comm_mutex = CreateMutex(0, true, L"Local\\oculus_steamvr_touch_controller_mutex");
    //MessageBox(NULL, pBuf, TEXT("Process2"), MB_OK);

    WaitForSingleObject(
        comm_mutex,    // handle to mutex
        INFINITE);  // no time-out interval
    ReleaseMutex(comm_mutex);


    if (do_rendering) {
        GuardianSystemDemo* instance = new (_aligned_malloc(sizeof(GuardianSystemDemo), 16)) GuardianSystemDemo();
        instance->Start(0, comm_buffer, comm_mutex);
        delete instance;

    } else {
        no_graphics_start(comm_buffer, comm_mutex);
    }
     UnmapViewOfFile(comm_buffer);

     CloseHandle(hMapFile);
     CloseHandle(comm_mutex);
     return 0;
}



void add_vib_sample(bool isRightHand, uint8_t sample, uint32_t offset);
std::vector<uint8_t> pulse_patterns[17] = {
    {},
    {255},
    {255,0},
    {},
    {196,255,128,0},
    {},
    {196,255,196,128,0,0,0},
    {},
    {196,255,255,128,64,0,0,0},
    {},
    {196,255,255,196,128,0,0,0,0,0},
    {},
    {},
    {},
    {},
    {},
    {64,128,196,255,255,128,64,32,0,0,0,0,0,0,0,0}
};

void add_vibration(bool isRightHand, float amplitude, float frequency, float duration) {

    std::cout << " adding haptic for amplitude " << amplitude << " frequency " << frequency << " duration " << duration << std::endl;
    if (duration < 0) duration = 0;
    float amp = amplitude/100.0f;
    float freq = frequency * 320.0f;
    if (amplitude >= 100.0f) amp = 1.0f;
    if (amp < 128.0/256) amp = 128.0/256;
    uint32_t requested_duration = duration * 320; // 320 Hz processing rate
    uint32_t min_duration = 1;
    uint32_t pulse_width = 1;
    if (freq < 160) {
        min_duration = 2;
        pulse_width = 1;
    }
    if (freq < 80) {
        min_duration = 4;
        pulse_width = 2;
    }
    if (freq < 60) {
        min_duration = 6;
        pulse_width = 3;
    }
    if (freq < 40) {
        min_duration = 8;
        pulse_width = 4;
    }
    if (freq < 32) {
        min_duration = 10;
        pulse_width = 5;
    }
    if (freq < 20) {
        min_duration = 16;
        pulse_width = 8;
    }
  
    if (min_duration > requested_duration) requested_duration = min_duration;

  //  std::cout << " adding haptic for amplitude " << amplitude << " -> " << amp << " frequency " << frequency << " -> " << freq << " duration " << duration << " ->" << requested_duration
    //    <<" pulse_width " << pulse_width << std::endl;
  
    uint64_t vib_buffer_sample_delta = ((ovr_GetTimeInSeconds() - vib_buf_time[isRightHand]) * 320);
      std::cout << " adding haptic for amplitude " << amplitude << " -> " << amp << " frequency " << frequency << " -> " << freq << " duration " << duration << " ->" << requested_duration
      <<" pulse_width " << pulse_width << " delta " << vib_buffer_sample_delta << std::endl;
    for (int i = 0; i < requested_duration; i++) {
        add_vib_sample(isRightHand, pulse_patterns[min_duration][(future_vib_buffer_index[isRightHand] + vib_buffer_sample_delta + i) % min_duration] * amp, i + vib_buffer_sample_delta);
    }

}

void add_vib_sample(bool isRightHand, uint8_t sample, uint32_t offset) {
    future_vib_buffer[isRightHand][(future_vib_buffer_index[isRightHand] + offset)%1024] = sample;
}

void vibration_thread(ovrSession mSession) {
    unsigned char buf[8];
    ovrHapticsBuffer vibuffer;
    vibuffer.SamplesCount = 8;
    vibuffer.Samples = buf;
    vibuffer.SubmitMode = ovrHapticsBufferSubmit_Enqueue;
    uint64_t counter = 0;
    while (1) {

        Sleep(1000/40);
        for (int hand = 0; hand < 2; hand++) {
            for (int i = 0; i < 8; i++) {
                buf[i] = future_vib_buffer[hand][future_vib_buffer_index[hand] + i];
                future_vib_buffer[hand][future_vib_buffer_index[hand] + i] = 0;
            }
            ovrTouchHapticsDesc desc = ovr_GetTouchHapticsDesc(mSession, (hand==0)?ovrControllerType_LTouch: ovrControllerType_RTouch);
            //std::cout << "SampleRate " << desc.SampleRateHz << " SampleSize " << desc.SampleSizeInBytes << " MaxSamples " << desc.SubmitMaxSamples << " MinSamples " << desc.SubmitMinSamples << " OptimalSamples " << desc.SubmitOptimalSamples << std::endl;

            ovrHapticsPlaybackState pbState;
            ovr_GetControllerVibrationState(mSession, (hand == 0) ? ovrControllerType_LTouch : ovrControllerType_RTouch, &pbState);
            //std::cout << (counter & 0xff) << " - queue state before  space: " << pbState.RemainingQueueSpace << "   samples: " << pbState.SamplesQueued << " / " << desc.QueueMinSizeToAvoidStarvation << std::endl;
            ovr_SubmitControllerVibration(mSession, (hand == 0) ? ovrControllerType_LTouch : ovrControllerType_RTouch, &vibuffer);
            ovr_GetControllerVibrationState(mSession, (hand == 0) ? ovrControllerType_LTouch : ovrControllerType_RTouch, &pbState);

            //std::cout << (counter & 0xff) << " - queue state after   space: " << pbState.RemainingQueueSpace << "   samples: " << pbState.SamplesQueued << " / " << desc.QueueMinSizeToAvoidStarvation << std::endl;
            future_vib_buffer_index[hand] += 8;
            vib_buf_time[hand] = ovr_GetTimeInSeconds();
            if (future_vib_buffer_index[hand] >= 1024)future_vib_buffer_index[hand] = 0;
        }
        counter++;

    }
}