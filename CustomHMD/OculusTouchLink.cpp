
//============ Copyright (c) Valve Corporation, All rights reserved. ============
#include "Common.h"
#include "TouchControllerDriver.h"
#include "TouchTrackerDriver.h"
#include "SensorDriver.h"
#include "HeadsetTrackerDriver.h"
//-----------------------------------------------------------------------------

class CWatchdogDriver_Sample : public IVRWatchdogProvider
{
public:
    CWatchdogDriver_Sample()
    {
        m_pWatchdogThread = nullptr;
    }
    
    virtual EVRInitError Init(vr::IVRDriverContext* pDriverContext);
    virtual void Cleanup();

private:
    std::thread* m_pWatchdogThread;
};

CWatchdogDriver_Sample g_watchdogDriverNull;


bool g_bExiting = false;



void WatchdogThreadFunction()
{
    while (!g_bExiting)
    {
#if 0 
        defined( _WINDOWS )
        // on windows send the event when the Y key is pressed.
        if ((0x01 & GetAsyncKeyState('Y')) != 0)
        {
            // Y key was pressed. 
            vr::VRWatchdogHost()->WatchdogWakeUp(vr::TrackedDeviceClass_HMD);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
#else
        // for the other platforms, just send one every five seconds
        std::this_thread::sleep_for(std::chrono::seconds(5));
        vr::VRWatchdogHost()->WatchdogWakeUp(vr::TrackedDeviceClass_Controller);
#endif
    }
}

EVRInitError CWatchdogDriver_Sample::Init(vr::IVRDriverContext* pDriverContext)
{
    VR_INIT_WATCHDOG_DRIVER_CONTEXT(pDriverContext);
    InitDriverLog(vr::VRDriverLog());
    DriverLog("Started OculusTouchLink driver\n");
    // Watchdog mode on Windows starts a thread that listens for the 'Y' key on the keyboard to 
    // be pressed. A real driver should wait for a system button event or something else from the 
    // the hardware that signals that the VR system should start up.
    g_bExiting = false;
    m_pWatchdogThread = new std::thread(WatchdogThreadFunction);
    if (!m_pWatchdogThread)
    {
        DriverLog("Unable to create watchdog thread\n");
        return VRInitError_Driver_Failed;
    }

    return VRInitError_None;
}


void CWatchdogDriver_Sample::Cleanup()
{
    g_bExiting = true;
    if (m_pWatchdogThread)
    {
        m_pWatchdogThread->join();
        delete m_pWatchdogThread;
        m_pWatchdogThread = nullptr;
    }

    CleanupDriverLog();
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CServerDriver_OVRTL : public IServerTrackedDeviceProvider
{
public:
    virtual EVRInitError Init(vr::IVRDriverContext* pDriverContext);
    virtual void Cleanup();
    virtual const char* const* GetInterfaceVersions() { return vr::k_InterfaceVersions; }
    virtual void RunFrame();
    virtual bool ShouldBlockStandbyMode() { return false; }
    virtual void EnterStandby() {}
    virtual void LeaveStandby() {}



private:

    bool Setup();
    bool setup_complete = false;
    //std::thread setup_thread;
    void InitRenderTargets(const ovrHmdDesc& hmdDesc);
    void  Render();
    ovrSession mSession = nullptr;
    ovrGraphicsLuid luid{};
#if ADD_SENSORS
    std::vector<CTouchSensorDriver*> sensors;
#endif
#if ADD_HMD
    CTouchHeadsetTrackerDriver* m_pNullHmdLatest = nullptr;
#endif
    CTouchControllerDriver* m_pLController = nullptr;
    CTouchControllerDriver* m_pRController = nullptr;
    std::vector<CTouchTrackerDriver*> trackers;
    HANDLE hMapFile;

#if DRAW_FRAME
    uint32_t mFrameIndex = 0;                                               // Global frame counter
    ovrPosef mHmdToEyePose[ovrEye_Count] = {};                              // Offset from the center of the HMD to each eye
    ovrRecti mEyeRenderViewport[ovrEye_Count] = {};                         // Eye render target viewport

    ovrLayerEyeFov mEyeRenderLayer = {};                                    // OVR  - Eye render layers description
    ovrTextureSwapChain mTextureChain[ovrEye_Count] = {};                   // OVR  - Eye render target swap chain
    ID3D11DepthStencilView* mEyeDepthTarget[ovrEye_Count] = {};             // DX11 - Eye depth view
    std::vector<ID3D11RenderTargetView*> mEyeRenderTargets[ovrEye_Count];   // DX11 - Eye render view
    DirectX11 DIRECTX;
    bool mShouldQuit = false;
#endif
};

CServerDriver_OVRTL g_serverDriverNull;


void CServerDriver_OVRTL::Render()
{
    log_to_buffer(__func__);
#if DRAW_FRAME
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
        DIRECTX.SetAndClearRenderTarget(renderTargetView, depthTargetView, 0.2f, 0.2f, 0.2f, 1.0f);
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
#endif
}
void CServerDriver_OVRTL::InitRenderTargets(const ovrHmdDesc& hmdDesc)
{
    log_to_buffer(__func__);
#if DRAW_FRAME
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
#endif
}
ovrQuatf ToQuaternion(double x, double y, double z)
{
    // Abbreviations for the various angular functions
    double cx = cos(z * 0.5);
    double sx = sin(z * 0.5);
    double cy = cos(x * 0.5);
    double sy = sin(x * 0.5);
    double cz = cos(y * 0.5);
    double sz = sin(y * 0.5);

    ovrQuatf q;
    q.w = cz * cy * cx + sz * sy * sx;
    q.x = sz * cy * cx - cz * sy * sx;
    q.y = cz * sy * cx + sz * cy * sx;
    q.z = cz * cy * sx - sz * sy * cx;

    return q;
}

EVRInitError CServerDriver_OVRTL::Init(vr::IVRDriverContext* pDriverContext)
{


    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
    InitDriverLog(vr::VRDriverLog());
    //setup_thread = std::thread([this, pDriverContext]() {this->Setup(pDriverContext); });
    return VRInitError_None;

}


bool CServerDriver_OVRTL::Setup()
{




#if EXPERIMENTAL_OFFSET_CALIBRATION
    ovrVector3f overall_offset{ -0.01 * 30.57033738,-0.01 * 46.98443338, 0.01 * 54.50133916 };
    ovrQuatf overall_rotation{ 0 };

    overall_rotation = ToQuaternion(0, (30.28197291 / 180.0) * 3.141592, 0);
#endif

    //map the shared memory buffer for communicating button and vibration data

#if USE_SHARE_MEM_BUFFER
#if 0         // this switch toggles who creates the buffer, 1 = this, 0 = ovr_test external component, 0 means that this driver will fail to load if the external part is not running, which could be desirable
    hMapFile = CreateFileMapping(
        INVALID_HANDLE_VALUE,    // use paging file
        NULL,                    // default security
        PAGE_READWRITE,          // read/write access
        0,                       // maximum object size (high-order DWORD)
        sizeof(shared_buffer),                // maximum object size (low-order DWORD)
        L"Global\\oculus_steamvr_touch_controller_data_channel");                 // name of mapping object
#else

    hMapFile = OpenFileMapping(
        FILE_MAP_ALL_ACCESS,   // read/write access
        FALSE,                 // do not inherit the name
        L"Local\\oculus_steamvr_touch_controller_data_channel");               // name of mapping object
    
#endif
    if (hMapFile == NULL)
    {
        std::cout << "Could not create file mapping object " << GetLastError() << std::endl;
        return false;
    }
    comm_buffer = (shared_buffer*)MapViewOfFile(hMapFile,   // handle to map object
        FILE_MAP_ALL_ACCESS, // read/write permission
        0,
        0,
        sizeof(shared_buffer));

    if (comm_buffer == NULL)
    {
        std::cout << "Could not map view of file" << GetLastError() << std::endl;

        CloseHandle(hMapFile);

        return false;
    }
#endif

    log_to_buffer(__func__);
    // get a handle to the inter-process mutex used for accessing the shared data structure
#if USE_MUTEX
    comm_mutex = CreateMutex(0, false, L"Local\\oculus_steamvr_touch_controller_mutex");
    if (comm_mutex == NULL)
    {
        std::cout << "Could not open mutex" << GetLastError() << std::endl;
        return false;
    }
#endif


    if ((!comm_buffer->config.external_tracking) && !mSession) {
#if DRAW_FRAME
        ovrInitParams initParams = { ovrInit_RequestVersion, OVR_MINOR_VERSION, NULL, 0, 0 };

#else
        ovrInitParams initParams = { ovrInit_RequestVersion | ovrInit_FocusAware | ovrInit_Invisible, OVR_MINOR_VERSION, NULL, 0, 0 };
#endif
        log_to_buffer("ovr_initialize\n");
        if (OVR_FAILURE(ovr_Initialize(&initParams)))
        {
            ovrErrorInfo err;
            ovr_GetLastErrorInfo(&err);
            log_to_buffer("ovr_Initialize Failed! " + std::string(err.ErrorString));
            return false;
        }
        log_to_buffer("ovr_Create\n");
        if (OVR_FAILURE(ovr_Create(&mSession, &luid)))
        {
            log_to_buffer("ovr_Create Failed!");
            return false;
        }

#if DRAW_FRAME
        if (!DIRECTX.InitWindow(0/*hinst*/, L"GuardianSystemDemo")) {
            printf("DIRECTX.InitWindow failed");
        }
        ovrHmdDesc hmdDesc = ovr_GetHmdDesc(mSession);
        if (!DIRECTX.InitDevice(hmdDesc.Resolution.w / 2, hmdDesc.Resolution.h / 2, reinterpret_cast<LUID*>(&luid))) {
            printf("DIRECTX.InitDevice failed");
        }
        InitRenderTargets(hmdDesc);
#endif
        log_to_buffer("ovr_SetTrackingOriginType\n");
        if (OVR_FAILURE(ovr_SetTrackingOriginType(mSession, ovrTrackingOrigin_FloorLevel)))
            return false;
    }
#ifdef ADD_HMD
    if (comm_buffer->config.track_hmd) {
        m_pNullHmdLatest = new CTouchHeadsetTrackerDriver(mSession);
        vr::VRServerDriverHost()->TrackedDeviceAdded(m_pNullHmdLatest->GetSerialNumber().c_str(), vr::TrackedDeviceClass_GenericTracker, m_pNullHmdLatest);
    }
#endif
#if ADD_SENSORS
    //log_to_buffer("ADD_SENSORS");
    for (int i = 0; i < comm_buffer->num_sensors; i++)
    {
        log_to_buffer("Creating tracking reference");
        sensors.push_back(new CTouchSensorDriver(mSession, i));
        vr::VRServerDriverHost()->TrackedDeviceAdded(sensors.back()->GetSerialNumber().c_str(), vr::TrackedDeviceClass_TrackingReference, sensors.back());
    }
#endif
#if CREATE_CONTROLLERS
    log_to_buffer("CREATE_CONTROLLERS\n");
    if(!comm_buffer->config.disable_controllers){
        if (!comm_buffer->config.disable_left_controller) {
            m_pLController = new CTouchControllerDriver(mSession, false/*, overall_offset, overall_rotation*/);
            vr::VRServerDriverHost()->TrackedDeviceAdded(m_pLController->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pLController);
        }

        if (!comm_buffer->config.disable_right_controller) {
            m_pRController = new CTouchControllerDriver(mSession, true/*, overall_offset, overall_rotation*/);
            vr::VRServerDriverHost()->TrackedDeviceAdded(m_pRController->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pRController);
        }
    }
#endif

    //for(int tracker = 0; tracker < ((ovr_GetConnectedControllerTypes(mSession) >> 8 )&0xf); tracker++){
    for (int tracker = 0; tracker < comm_buffer->config.num_objects; tracker++) {
        log_to_buffer("Creating object tracker");
        trackers.push_back(new CTouchTrackerDriver(mSession, tracker/*(ovrTrackedDeviceType)(tracker<<8)*/, true));
        vr::VRServerDriverHost()->TrackedDeviceAdded(trackers.back()->GetSerialNumber().c_str(), vr::TrackedDeviceClass_GenericTracker, trackers.back());
    }

#if EXPERIMENTAL_OFFSET_CALLIBRATION
    ovrPosef pose;
    pose.Position = overall_offset;
    pose.Orientation = overall_rotation;
    ovr_SpecifyTrackingOrigin(mSession, pose);
#endif

#if DRAW_FRAME
    Render();
    DIRECTX.HandleMessages();
    Render();
#endif
    log_to_buffer("setup complete");

    setup_complete = true;
    return true;
}

void CServerDriver_OVRTL::Cleanup()
{
    log_to_buffer(__func__);
    CleanupDriverLog();
#ifdef ADD_HMD
    if (m_pNullHmdLatest) {
        delete m_pNullHmdLatest;
        m_pNullHmdLatest = NULL;
    }
#endif
#if ADD_SENSORS
    sensors.clear();
#endif
#if CREATE_CONTROLLERS
    if (m_pLController) delete m_pLController;
    m_pLController = NULL;
    if (m_pRController) delete m_pRController;
    m_pRController = NULL;
    for (CTouchTrackerDriver* t : trackers) delete t;
    trackers.clear();
#endif
    if (mSession) {
        ovr_Destroy(mSession);
        ovr_Shutdown();
    }
#if USE_SHARE_MEM_BUFFER
    UnmapViewOfFile(comm_buffer);

    CloseHandle(hMapFile);
#endif
#if USE_MUTEX
    CloseHandle(comm_mutex);
#endif
}


void CServerDriver_OVRTL::RunFrame()
{
    

    if (!setup_complete) {
        log_to_buffer("RunFrame ! setup");
        vr::VREvent_t vrEvent;
        while (vr::VRServerDriverHost()->PollNextEvent(&vrEvent, sizeof(vrEvent))); // process all the events

        if (!Setup()) return;
        log_to_buffer("RunFrame done Setup");
    }

#ifdef ADD_HMD
    if (m_pNullHmdLatest && (m_pNullHmdLatest->m_unObjectId != vr::k_unTrackedDeviceIndexInvalid))
    {
        m_pNullHmdLatest->RunFrame();
    }
#endif
#if ADD_SENSORS
    for (CTouchSensorDriver* sensor : sensors) sensor->RunFrame();
#endif
    if (m_pLController && (m_pLController->m_unObjectId != vr::k_unTrackedDeviceIndexInvalid))
    {
        m_pLController->RunFrame();
    }
    if (m_pRController && (m_pRController->m_unObjectId != vr::k_unTrackedDeviceIndexInvalid))
    {
        m_pRController->RunFrame();
    }
    for (CTouchTrackerDriver* t : trackers) if((t->m_unObjectId != vr::k_unTrackedDeviceIndexInvalid)) t->RunFrame();

    vr::VREvent_t vrEvent;
    while (vr::VRServerDriverHost()->PollNextEvent(&vrEvent, sizeof(vrEvent)))
    {
        /*
#if USE_SHARE_MEM_BUFFER
#if USE_MUTEX
        if (!WaitForSingleObject(comm_mutex, 100)) {
            comm_buffer->vrEvent_type = vrEvent.eventType;
            ReleaseMutex(comm_mutex);
        }
#else
        comm_buffer->vrEvent_type = vrEvent.eventType;
#endif
#endif
*/
       // log_to_buffer("Processing Event");
        if (m_pLController)
        {
            m_pLController->ProcessEvent(vrEvent);
        }
        if (m_pRController)
        {
            m_pRController->ProcessEvent(vrEvent);
        }
        for (CTouchTrackerDriver* t : trackers) t->ProcessEvent(vrEvent);
    }
#if DRAW_FRAME
    //Render();
    DIRECTX.HandleMessages();
#endif
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode)
{
    log_to_buffer(__func__);
    if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName))
    {
        return &g_serverDriverNull;
    }
    if (0 == strcmp(IVRWatchdogProvider_Version, pInterfaceName))
    {
        return &g_watchdogDriverNull;
    }

    if (pReturnCode)
        *pReturnCode = VRInitError_Init_InterfaceNotFound;

    return NULL;
}
