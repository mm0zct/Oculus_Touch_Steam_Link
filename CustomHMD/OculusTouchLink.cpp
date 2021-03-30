
//============ Copyright (c) Valve Corporation, All rights reserved. ============

#include <openvr_driver.h>
#include "driverlog.h"
//#define DRAW_FRAME 1
//#define DRAW_FRAME 1

#define USE_MUTEX 1

#define CREATE_CONTROLLERS 1

#define USE_SHARE_MEM_BUFFER 1

#define DO_SKELETON 1       //enable the common skeleton code, and the right hand skeleton
#define DO_LSKELETON 1      // enable the left hand skeleton, depends on the above, but allows simultanously comparing legacy and skeleton input by disabling the left hand skeleton

#if DRAW_FRAME
#ifndef NOMINMAX
#define NOMINMAX
#endif

#define   OVR_D3D_VERSION 11
#pragma warning(disable: 4324)
#include "Win32_DirectXAppUtil.h" // DirectX Helper
#include "OVR_CAPI_D3D.h" // Oculus SDK




#else
#include <OVR_CAPI.h>
#endif
#include <fstream>
#include <vector>
#include <thread>
#include <chrono>
#include <iostream>


#if defined( _WINDOWS )
#include <windows.h>
#endif

using namespace vr;
#if USE_MUTEX
HANDLE comm_mutex;
#endif

struct shared_buffer {
    ovrInputState input_state;
    uint32_t vrEvent_type;
    float vib_amplitude[2];
    float vib_frequency[2];
    float vib_duration_s[2];
    bool vib_valid[2];
};


#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

inline HmdQuaternion_t HmdQuaternion_Init(double w, double x, double y, double z)
{
    HmdQuaternion_t quat;
    quat.w = w;
    quat.x = x;
    quat.y = y;
    quat.z = z;
    return quat;
}

inline void HmdMatrix_SetIdentity(HmdMatrix34_t* pMatrix)
{
    pMatrix->m[0][0] = 1.f;
    pMatrix->m[0][1] = 0.f;
    pMatrix->m[0][2] = 0.f;
    pMatrix->m[0][3] = 0.f;
    pMatrix->m[1][0] = 0.f;
    pMatrix->m[1][1] = 1.f;
    pMatrix->m[1][2] = 0.f;
    pMatrix->m[1][3] = 0.f;
    pMatrix->m[2][0] = 0.f;
    pMatrix->m[2][1] = 0.f;
    pMatrix->m[2][2] = 1.f;
    pMatrix->m[2][3] = 0.f;
}


// keys for use with the settings API
static const char* const k_pch_Sample_Section = "driver_sample";
static const char* const k_pch_Sample_SerialNumber_String = "serialNumber";
static const char* const k_pch_Sample_ModelNumber_String = "modelNumber";
static const char* const k_pch_Sample_WindowX_Int32 = "windowX";
static const char* const k_pch_Sample_WindowY_Int32 = "windowY";
static const char* const k_pch_Sample_WindowWidth_Int32 = "windowWidth";
static const char* const k_pch_Sample_WindowHeight_Int32 = "windowHeight";
static const char* const k_pch_Sample_RenderWidth_Int32 = "renderWidth";
static const char* const k_pch_Sample_RenderHeight_Int32 = "renderHeight";
static const char* const k_pch_Sample_SecondsFromVsyncToPhotons_Float = "secondsFromVsyncToPhotons";
static const char* const k_pch_Sample_DisplayFrequency_Float = "displayFrequency";

//-----------------------------------------------------------------------------
// Purpose:
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

#if ADD_HMD    //we're not doing anything for the HMD at the moment
//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CSampleDeviceDriver : public vr::ITrackedDeviceServerDriver, public vr::IVRDisplayComponent
{
public:
    CSampleDeviceDriver()
    {
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
        m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

        DriverLog("Using settings values\n");
        m_flIPD = vr::VRSettings()->GetFloat(k_pch_SteamVR_Section, k_pch_SteamVR_IPD_Float);

        char buf[1024];
        vr::VRSettings()->GetString(k_pch_Sample_Section, k_pch_Sample_SerialNumber_String, buf, sizeof(buf));
        m_sSerialNumber = buf;

        vr::VRSettings()->GetString(k_pch_Sample_Section, k_pch_Sample_ModelNumber_String, buf, sizeof(buf));
        m_sModelNumber = buf;

        m_nWindowX = vr::VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_WindowX_Int32);
        m_nWindowY = vr::VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_WindowY_Int32);
        m_nWindowWidth = vr::VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_WindowWidth_Int32);
        m_nWindowHeight = vr::VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_WindowHeight_Int32);
        m_nRenderWidth = vr::VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_RenderWidth_Int32);
        m_nRenderHeight = vr::VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_RenderHeight_Int32);
        m_flSecondsFromVsyncToPhotons = vr::VRSettings()->GetFloat(k_pch_Sample_Section, k_pch_Sample_SecondsFromVsyncToPhotons_Float);
        m_flDisplayFrequency = vr::VRSettings()->GetFloat(k_pch_Sample_Section, k_pch_Sample_DisplayFrequency_Float);

        DriverLog("driver_null: Serial Number: %s\n", m_sSerialNumber.c_str());
        DriverLog("driver_null: Model Number: %s\n", m_sModelNumber.c_str());
        DriverLog("driver_null: Window: %d %d %d %d\n", m_nWindowX, m_nWindowY, m_nWindowWidth, m_nWindowHeight);
        DriverLog("driver_null: Render Target: %d %d\n", m_nRenderWidth, m_nRenderHeight);
        DriverLog("driver_null: Seconds from Vsync to Photons: %f\n", m_flSecondsFromVsyncToPhotons);
        DriverLog("driver_null: Display Frequency: %f\n", m_flDisplayFrequency);
        DriverLog("driver_null: IPD: %f\n", m_flIPD);
    }

    virtual ~CSampleDeviceDriver()
    {
    }


    virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
    {
        m_unObjectId = unObjectId;
        m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);


        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_ModelNumber_String, m_sModelNumber.c_str());
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, m_sModelNumber.c_str());
        vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_UserIpdMeters_Float, m_flIPD);
        vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_UserHeadToEyeDepthMeters_Float, 0.f);
        vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_DisplayFrequency_Float, m_flDisplayFrequency);
        vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_SecondsFromVsyncToPhotons_Float, m_flSecondsFromVsyncToPhotons);

        // return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
        vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 1);

        // avoid "not fullscreen" warnings from vrmonitor
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false);

        // Icons can be configured in code or automatically configured by an external file "drivername\resources\driver.vrresources".
        // Icon properties NOT configured in code (post Activate) are then auto-configured by the optional presence of a driver's "drivername\resources\driver.vrresources".
        // In this manner a driver can configure their icons in a flexible data driven fashion by using an external file.
        //
        // The structure of the driver.vrresources file allows a driver to specialize their icons based on their HW.
        // Keys matching the value in "Prop_ModelNumber_String" are considered first, since the driver may have model specific icons.
        // An absence of a matching "Prop_ModelNumber_String" then considers the ETrackedDeviceClass ("HMD", "Controller", "GenericTracker", "TrackingReference")
        // since the driver may have specialized icons based on those device class names.
        //
        // An absence of either then falls back to the "system.vrresources" where generic device class icons are then supplied.
        //
        // Please refer to "bin\drivers\sample\resources\driver.vrresources" which contains this sample configuration.
        //
        // "Alias" is a reserved key and specifies chaining to another json block.
        //
        // In this sample configuration file (overly complex FOR EXAMPLE PURPOSES ONLY)....
        //
        // "Model-v2.0" chains through the alias to "Model-v1.0" which chains through the alias to "Model-v Defaults".
        //
        // Keys NOT found in "Model-v2.0" would then chase through the "Alias" to be resolved in "Model-v1.0" and either resolve their or continue through the alias.
        // Thus "Prop_NamedIconPathDeviceAlertLow_String" in each model's block represent a specialization specific for that "model".
        // Keys in "Model-v Defaults" are an example of mapping to the same states, and here all map to "Prop_NamedIconPathDeviceOff_String".
        //
        bool bSetupIconUsingExternalResourceFile = true;
        if (!bSetupIconUsingExternalResourceFile)
        {
            // Setup properties directly in code.
            // Path values are of the form {drivername}\icons\some_icon_filename.png
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{OculusTouchLink}/icons/headset_sample_status_off.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, "{OculusTouchLink}/icons/headset_sample_status_searching.gif");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{OculusTouchLink}/icons/headset_sample_status_searching_alert.gif");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{OculusTouchLink}/icons/headset_sample_status_ready.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{OculusTouchLink}/icons/headset_sample_status_ready_alert.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, "{OculusTouchLink}/icons/headset_sample_status_error.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, "{OculusTouchLink}/icons/headset_sample_status_standby.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, "{OculusTouchLink}/icons/headset_sample_status_ready_low.png");
        }

        return VRInitError_None;
    }

    virtual void Deactivate()
    {
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
    }

    virtual void EnterStandby()
    {
    }

    void* GetComponent(const char* pchComponentNameAndVersion)
    {
        if (!_stricmp(pchComponentNameAndVersion, vr::IVRDisplayComponent_Version))
        {
            return (vr::IVRDisplayComponent*)this;
        }

        // override this to add a component to a driver
        return NULL;
    }

    virtual void PowerOff()
    {
    }

    /** debug request from a client */
    virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
    {
        if (unResponseBufferSize >= 1)
            pchResponseBuffer[0] = 0;
    }

    virtual void GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
    {
        *pnX = m_nWindowX;
        *pnY = m_nWindowY;
        *pnWidth = m_nWindowWidth;
        *pnHeight = m_nWindowHeight;
    }

    virtual bool IsDisplayOnDesktop()
    {
        return true;
    }

    virtual bool IsDisplayRealDisplay()
    {
        return false;
    }

    virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight)
    {
        *pnWidth = m_nRenderWidth;
        *pnHeight = m_nRenderHeight;
    }

    virtual void GetEyeOutputViewport(EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
    {
        *pnY = 0;
        *pnWidth = m_nWindowWidth / 2;
        *pnHeight = m_nWindowHeight;

        if (eEye == Eye_Left)
        {
            *pnX = 0;
        }
        else
        {
            *pnX = m_nWindowWidth / 2;
            *pnX = m_nWindowWidth / 2;
        }
    }

    virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom)
    {
        *pfLeft = -1.0;
        *pfRight = 1.0;
        *pfTop = -1.0;
        *pfBottom = 1.0;
    }

    virtual DistortionCoordinates_t ComputeDistortion(EVREye eEye, float fU, float fV)
    {
        DistortionCoordinates_t coordinates;
        coordinates.rfBlue[0] = fU;
        coordinates.rfBlue[1] = fV;
        coordinates.rfGreen[0] = fU;
        coordinates.rfGreen[1] = fV;
        coordinates.rfRed[0] = fU;
        coordinates.rfRed[1] = fV;
        return coordinates;
    }

    virtual DriverPose_t GetPose()
    {
        DriverPose_t pose = { 0 };
        pose.poseIsValid = true;
        pose.result = TrackingResult_Running_OK;
        pose.deviceIsConnected = true;

        pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
        pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);


        return pose;
    }


    void RunFrame()
    {
        // In a real driver, this should happen from some pose tracking thread.
        // The RunFrame interval is unspecified and can be very irregular if some other
        // driver blocks it for some periodic task.
        if (m_unObjectId != vr::k_unTrackedDeviceIndexInvalid)
        {
            vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, GetPose(), sizeof(DriverPose_t));
        }
    }

    std::string GetSerialNumber() const { return m_sSerialNumber; }

private:
    vr::TrackedDeviceIndex_t m_unObjectId;
    vr::PropertyContainerHandle_t m_ulPropertyContainer;

    std::string m_sSerialNumber;
    std::string m_sModelNumber;

    int32_t m_nWindowX;
    int32_t m_nWindowY;
    int32_t m_nWindowWidth;
    int32_t m_nWindowHeight;
    int32_t m_nRenderWidth;
    int32_t m_nRenderHeight;
    float m_flSecondsFromVsyncToPhotons;
    float m_flDisplayFrequency;
    float m_flIPD;
};
#endif
//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CSampleControllerDriver : public vr::ITrackedDeviceServerDriver                                               
{                                                                                                                                             // hand_offset({ 0.01071,0.04078,-0.04731 }), hand_offset2({-0.003,-0.101,0.0089 })
public:                                                                                                                                      //x = 0.00571 y = 0.04078 z = -0.03531 x2 =-0.000999998 y2 = -0.1 z = 0.0019
    CSampleControllerDriver(ovrSession mSession, bool isRightHand, shared_buffer *comm_buffer/*, ovrVector3f overall_offset, ovrQuatf overall_rotation*/): mSession(mSession),isRightHand(isRightHand), comm_buffer(comm_buffer), hand_offset({ 0.00571,0.04078,-0.03531 }), hand_offset2({ -0.000999998,-0.1, 0.0019 })/*, overall_offset(overall_offset), overall_rotation(overall_rotation)*/
    {
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
        m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;
        if (isRightHand) {
            m_sSerialNumber = "WMHD316J600000_Controller_Right";
            m_sModelNumber = "Oculus Rift CV1(Right Controller)";
        } else {
            m_sSerialNumber = "WMHD316J600000_Controller_Left";
            m_sModelNumber = "Oculus Rift CV1(Left Controller)";
        }
    }

    virtual ~CSampleControllerDriver()
    {
    }


    virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
    {

  
        m_unObjectId = unObjectId;
        m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);
        /** initializes the driver. This will be called before any other methods are called.
* If Init returns anything other than VRInitError_None the driver DLL will be unloaded.
*
* pDriverHost will never be NULL, and will always be a pointer to a IServerDriverHost interface
*
* pchUserDriverConfigDir - The absolute path of the directory where the driver should store user
*	config files.
* pchDriverInstallDir - The absolute path of the root directory for the driver.
*/
        
       /* vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_ModelNumber_String, m_sModelNumber.c_str());  */
      /*  vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, m_sModelNumber.c_str());    */


       /* VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_ControllerType_String, "oculus_touch");       */

        ovrHmdDesc hmd_desc = ovr_GetHmdDesc(mSession);
        switch (hmd_desc.Type)
        {
            case ovrHmd_CV1:
                if (isRightHand) {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_cv1_controller_right");
                }
                else {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_cv1_controller_left");
                }
            break;
            case ovrHmd_RiftS:
                if (isRightHand) {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_rifts_controller_right");
                }
                else {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_rifts_controller_left");
                }
            case ovrHmd_Quest:
                if (isRightHand) {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_quest_controller_right");
                }
                else {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_quest_controller_left");
                }
            case ovrHmd_Quest2:
                if (isRightHand) {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_quest2_controller_right");
                }
                else {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_quest2_controller_left");
                }
            default:
                if (isRightHand) {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_cv1_controller_right");
                }
                else {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_cv1_controller_left");
                }
        }
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_TrackingSystemName_String, "oculus");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, m_sModelNumber.c_str());
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_SerialNumber_String, m_sSerialNumber.c_str());
       /* vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, (isRightHand) ? "oculus_cv1_controller_right" : "oculus_cv1_controller_left");    */
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "Oculus");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_HardwareRevision_String, "14");
        vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_HardwareRevision_Uint64, 14U);
        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);
        //vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_ParentDriver_Uint64, 8589934599U); // Strange value from dump
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ResourceRoot_String, "oculus");


        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RegisteredDeviceType_String, (isRightHand)?"oculus/WMHD316J600000_Controller_Right": "oculus/WMHD316J600000_Controller_Left");

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_InputProfilePath_String, "{oculus}/input/touch_profile.json");
        vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_SupportedButtons_Uint64, 30064771207U);
        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_Axis0Type_Int32, vr::k_eControllerAxis_Joystick);
        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_Axis1Type_Int32, vr::k_eControllerAxis_Trigger);
        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_Axis2Type_Int32, vr::k_eControllerAxis_Trigger);
        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, (isRightHand) ? vr::TrackedControllerRole_RightHand : vr::TrackedControllerRole_LeftHand);
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, "oculus_touch");
        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerHandSelectionPriority_Int32, 0);

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_off.png" : "{oculus}/icons/cv1_right_controller_off.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_searching.gif" : "{oculus}/icons/cv1_right_controller_searching.gif");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_alert_searching.gif" : "{oculus}/icons/cv1_right_controller_alert_searching.gif");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_ready.png" : "{oculus}/icons/cv1_right_controller_ready.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_ready_alert.png" : "{oculus}/icons/cv1_right_controller_ready_alert.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_error.png" : "{oculus}/icons/cv1_right_controller_error.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_standby.png" : "{oculus}/icons/cv1_right_controller_standby.png");

        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasDisplayComponent_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasCameraComponent_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasDriverDirectModeComponent_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasVirtualDisplayComponent_Bool, false);


        // return a constant that's not 0 (invalid), 1 is reserved for Oculus, so let's use that ;)
        vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 1);


        if (isRightHand) {
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/a/click", &m_compAc);
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/b/click", &m_compBc);
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/a/touch", &m_compAt);
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/b/touch", &m_compBt);
#if DO_SKELETON
            vr::VRDriverInput()->CreateSkeletonComponent(m_ulPropertyContainer, "/input/skeleton/right", "/skeleton/hand/right", "/pose/raw", VRSkeletalTracking_Estimated, nullptr, 0, &m_compSkel);
#endif
        } else {
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &m_compSysc);
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/x/click", &m_compXc);
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/y/click", &m_compYc);
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/x/touch", &m_compXt);
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/y/touch", &m_compYt);
#if DO_LSKELETON
            vr::VRDriverInput()->CreateSkeletonComponent(m_ulPropertyContainer, "/input/skeleton/left", "/skeleton/hand/left", "/pose/raw", VRSkeletalTracking_Estimated, nullptr, 0, &m_compSkel);
#endif

        }
            
            
        vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/grip/value", &m_compGripv, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/button", &m_compGripb);
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/touch"    , &m_compGript);
        vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trigger/value" , &m_compTrigv, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trigger/touch" , &m_compTrigt);
        vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/joystick/x"    , &m_compJoyx, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
        vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/joystick/y"    , &m_compJoyy, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/joystick/click", &m_compJoyc);
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/joystick/touch", &m_compJoyt);

       

        // create our haptic component
        vr::VRDriverInput()->CreateHapticComponent(m_ulPropertyContainer, "/output/haptic", &m_compHaptic);


        if (isRightHand)
        {
            // Transformation inversion along 0YZ plane
            for (size_t i = 1U; i < NUM_BONES; i++)
            {
                right_open_hand_pose[i].position.v[0] *= -1.f;
                right_fist_pose[i].position.v[0] *= -1.f;
                switch (i)
                {
                case HSB_Wrist:
                {
                    right_open_hand_pose[i].orientation.y *= -1.f;
                    right_fist_pose[i].orientation.y *= -1.f;
                    right_open_hand_pose[i].orientation.z *= -1.f;
                    right_fist_pose[i].orientation.z *= -1.f;
                } break;

                case HSB_Thumb0:
                case HSB_IndexFinger0:
                case HSB_MiddleFinger0:
                case HSB_RingFinger0:
                case HSB_PinkyFinger0:
                {
                    right_open_hand_pose[i].orientation.z *= -1.f;
                    right_fist_pose[i].orientation.z *= -1.f;
                    std::swap(right_open_hand_pose[i].orientation.x, right_open_hand_pose[i].orientation.w);
                    std::swap(right_fist_pose[i].orientation.x, right_fist_pose[i].orientation.w);;
                    right_open_hand_pose[i].orientation.w *= -1.f;
                    right_fist_pose[i].orientation.w *= -1.f;
                    std::swap(right_open_hand_pose[i].orientation.y, right_open_hand_pose[i].orientation.z);
                    std::swap(right_fist_pose[i].orientation.y, right_fist_pose[i].orientation.z);
                } break;
                }
            }
        }


        return VRInitError_None;
    }
    enum HandSkeletonBone : size_t
    {
        HSB_Root = 0U,
        HSB_Wrist,
        HSB_Thumb0,
        HSB_Thumb1,
        HSB_Thumb2,
        HSB_Thumb3, // Last, no effect
        HSB_IndexFinger0,
        HSB_IndexFinger1,
        HSB_IndexFinger2,
        HSB_IndexFinger3,
        HSB_IndexFinger4, // Last, no effect
        HSB_MiddleFinger0,
        HSB_MiddleFinger1,
        HSB_MiddleFinger2,
        HSB_MiddleFinger3,
        HSB_MiddleFinger4, // Last, no effect
        HSB_RingFinger0,
        HSB_RingFinger1,
        HSB_RingFinger2,
        HSB_RingFinger3,
        HSB_RingFinger4, // Last, no effect
        HSB_PinkyFinger0,
        HSB_PinkyFinger1,
        HSB_PinkyFinger2,
        HSB_PinkyFinger3,
        HSB_PinkyFinger4, // Last, no effect
        HSB_Aux_Thumb,
        HSB_Aux_IndexFinger,
        HSB_Aux_MiddleFinger,
        HSB_Aux_RingFinger,
        HSB_Aux_PinkyFinger,

        HSB_Count
    };
    virtual void Deactivate()
    {
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
    }

    virtual void EnterStandby()
    {
    }

    void* GetComponent(const char* pchComponentNameAndVersion)
    {
        // override this to add a component to a driver
        return NULL;
    }

    virtual void PowerOff()
    {
    }

    /** debug request from a client */
    virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
    {
        if (unResponseBufferSize >= 1)
            pchResponseBuffer[0] = 0;
    }


    ovrQuatf ovrQuatfmul(ovrQuatf q1, ovrQuatf q2) {
        ovrQuatf result = { 0 };
        result.x = q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
        result.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
        result.z = q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
        result.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
        return result;
    }
    ovrVector3f rotateVector(const ovrVector3f _V, ovrQuatf q)const {
        ovrVector3f vec;   // any constructor will do
        float r, i, j, k;
        r = q.w;
        i = q.x;
        j = q.y;
        k = q.z;
        vec.x = 2 * (r * _V.z * j + i * _V.z * k - r * _V.y * k + i * _V.y * j) + _V.x * (r * r + i * i - j * j - k * k);
        vec.y = 2 * (r * _V.x * k + i * _V.x * j - r * _V.z * i + j * _V.z * k) + _V.y * (r * r - i * i + j * j - k * k);
        vec.z = 2 * (r * _V.y * i - r * _V.x * j + i * _V.x * k + j * _V.y * k) + _V.z * (r * r - i * i - j * j + k * k);
        return vec;
    }

    ovrVector3f crossProduct(const ovrVector3f v, ovrVector3f p) const
               {
        return ovrVector3f{ v.y * p.z - v.z * p.y, v.z * p.x - v.x * p.z, v.x * p.y - v.y * p.x };
               }

    ovrVector3f rotateVector2 ( ovrVector3f v, ovrQuatf q)
        {
            // nVidia SDK implementation
        
        ovrVector3f uv, uuv;
        ovrVector3f qvec{ q.x, q.y, q.z };
            uv = crossProduct(qvec,v);
            uuv = crossProduct(qvec,uv);
            uv.x *= (2.0f * q.w);
            uv.y *= (2.0f * q.w);
            uv.z *= (2.0f * q.w);
            uuv.x *= 2.0f;
            uuv.y *= 2.0f;
            uuv.z *= 2.0f;
        
            return ovrVector3f{ v.x + uv.x + uuv.x, v.y + uv.y + uuv.y, v.z + uv.z + uuv.z};
        }
    virtual DriverPose_t GetPose()
    {
        m_last_pose.poseTimeOffset = m_time_of_last_pose - ovr_GetTimeInSeconds();
        return this->m_last_pose;
    }
    virtual DriverPose_t CalculatePose()
    {

        ovrTrackingState ss = ovr_GetTrackingState(mSession, 0, false);
        m_time_of_last_pose = ss.HandPoses[isRightHand].TimeInSeconds;
        DriverPose_t pose = { 0 };
        pose.poseIsValid = true;
        pose.result = TrackingResult_Running_OK;
        pose.deviceIsConnected = true;
    
        ovrQuatf hand_qoffset = { 0.3420201, 0, 0, 0.9396926 };
        ovrQuatf hand_input = ss.HandPoses[isRightHand].ThePose.Orientation;
        ovrQuatf hand_result = ovrQuatfmul(hand_input, hand_qoffset);
        ovrVector3f hand_voffset={ 0,0,0 };
        if (isRightHand) {
            hand_voffset = rotateVector2(hand_offset, hand_input);
        } else {
            ovrVector3f left_hand_offset = hand_offset;
            left_hand_offset.x = -left_hand_offset.x;
            hand_voffset = rotateVector2(left_hand_offset, hand_input);
        }
    

        //hand_result = ovrQuatfmul(overall_rotation, hand_result);
        pose.qRotation.w = hand_result.w;
        pose.qRotation.x = hand_result.x;
        pose.qRotation.y = hand_result.y;
        pose.qRotation.z = hand_result.z;
        ovrVector3f position;
        position.x = ss.HandPoses[isRightHand].ThePose.Position.x + hand_voffset.x + hand_offset2.x;
        position.y = ss.HandPoses[isRightHand].ThePose.Position.y + hand_voffset.y + hand_offset2.y;
        position.z = ss.HandPoses[isRightHand].ThePose.Position.z + hand_voffset.z + hand_offset2.z;
        //position = rotateVector2(position, overall_rotation);
        pose.vecPosition[0] = position.x;// +overall_offset.x;
        pose.vecPosition[1] = position.y;// +overall_offset.y;
        pose.vecPosition[2] = position.z;// +overall_offset.z;
     


        ovrVector3f linAcc = (ss.HandPoses[isRightHand].LinearAcceleration);
        ovrVector3f linVel = (ss.HandPoses[isRightHand].LinearVelocity);
        ovrQuatf hand_nqoffset = { 0.3420201, 0, 0, -0.9396926 };
        /*linAcc = rotateVector2(linAcc, hand_qoffset);  
        linVel = rotateVector2(linVel, hand_nqoffset);*/    //do not do this


        pose.vecAcceleration[0] = linAcc.x;
        pose.vecAcceleration[1] = linAcc.y;
        pose.vecAcceleration[2] = linAcc.z;

        pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
        pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

        pose.vecVelocity[0] = linVel.x;
        pose.vecVelocity[1] = linVel.y;
        pose.vecVelocity[2] = linVel.z;

        pose.vecAngularVelocity[0] = ss.HandPoses[isRightHand].AngularVelocity.x;
        pose.vecAngularVelocity[1] = ss.HandPoses[isRightHand].AngularVelocity.y;
        pose.vecAngularVelocity[2] = ss.HandPoses[isRightHand].AngularVelocity.z;

        pose.poseTimeOffset = -0.01;
        return pose;
    }

                                   
    const static int NUM_BONES = 31;

    const VRBoneTransform_t left_open_hand_pose[NUM_BONES] = {
    { { 0.000000f,  0.000000f,  0.000000f,  1.000000f}, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
    { {-0.034038f,  0.036503f,  0.164722f,  1.000000f}, {-0.055147f, -0.078608f, -0.920279f,  0.379296f} },
    { {-0.012083f,  0.028070f,  0.025050f,  1.000000f}, { 0.464112f,  0.567418f,  0.272106f,  0.623374f} },
    { { 0.040406f,  0.000000f, -0.000000f,  1.000000f}, { 0.994838f,  0.082939f,  0.019454f,  0.055130f} },
    { { 0.032517f,  0.000000f,  0.000000f,  1.000000f}, { 0.974793f, -0.003213f,  0.021867f, -0.222015f} },
    { { 0.030464f, -0.000000f, -0.000000f,  1.000000f}, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
    { { 0.000632f,  0.026866f,  0.015002f,  1.000000f}, { 0.644251f,  0.421979f, -0.478202f,  0.422133f} },
    { { 0.074204f, -0.005002f,  0.000234f,  1.000000f}, { 0.995332f,  0.007007f, -0.039124f,  0.087949f} },
    { { 0.043930f, -0.000000f, -0.000000f,  1.000000f}, { 0.997891f,  0.045808f,  0.002142f, -0.045943f} },
    { { 0.028695f,  0.000000f,  0.000000f,  1.000000f}, { 0.999649f,  0.001850f, -0.022782f, -0.013409f} },
    { { 0.022821f,  0.000000f, -0.000000f,  1.000000f}, { 1.000000f, -0.000000f,  0.000000f, -0.000000f} },
    { { 0.002177f,  0.007120f,  0.016319f,  1.000000f}, { 0.546723f,  0.541276f, -0.442520f,  0.460749f} },
    { { 0.070953f,  0.000779f,  0.000997f,  1.000000f}, { 0.980294f, -0.167261f, -0.078959f,  0.069368f} },
    { { 0.043108f,  0.000000f,  0.000000f,  1.000000f}, { 0.997947f,  0.018493f,  0.013192f,  0.059886f} },
    { { 0.033266f,  0.000000f,  0.000000f,  1.000000f}, { 0.997394f, -0.003328f, -0.028225f, -0.066315f} },
    { { 0.025892f, -0.000000f,  0.000000f,  1.000000f}, { 0.999195f, -0.000000f,  0.000000f,  0.040126f} },
    { { 0.000513f, -0.006545f,  0.016348f,  1.000000f}, { 0.516692f,  0.550143f, -0.495548f,  0.429888f} },
    { { 0.065876f,  0.001786f,  0.000693f,  1.000000f}, { 0.990420f, -0.058696f, -0.101820f,  0.072495f} },
    { { 0.040697f,  0.000000f,  0.000000f,  1.000000f}, { 0.999545f, -0.002240f,  0.000004f,  0.030081f} },
    { { 0.028747f, -0.000000f, -0.000000f,  1.000000f}, { 0.999102f, -0.000721f, -0.012693f,  0.040420f} },
    { { 0.022430f, -0.000000f,  0.000000f,  1.000000f}, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
    { {-0.002478f, -0.018981f,  0.015214f,  1.000000f}, { 0.526918f,  0.523940f, -0.584025f,  0.326740f} },
    { { 0.062878f,  0.002844f,  0.000332f,  1.000000f}, { 0.986609f, -0.059615f, -0.135163f,  0.069132f} },
    { { 0.030220f,  0.000000f,  0.000000f,  1.000000f}, { 0.994317f,  0.001896f, -0.000132f,  0.106446f} },
    { { 0.018187f,  0.000000f,  0.000000f,  1.000000f}, { 0.995931f, -0.002010f, -0.052079f, -0.073526f} },
    { { 0.018018f,  0.000000f, -0.000000f,  1.000000f}, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
    { {-0.006059f,  0.056285f,  0.060064f,  1.000000f}, { 0.737238f,  0.202745f,  0.594267f,  0.249441f} },
    { {-0.040416f, -0.043018f,  0.019345f,  1.000000f}, {-0.290331f,  0.623527f, -0.663809f, -0.293734f} },
    { {-0.039354f, -0.075674f,  0.047048f,  1.000000f}, {-0.187047f,  0.678062f, -0.659285f, -0.265683f} },
    { {-0.038340f, -0.090987f,  0.082579f,  1.000000f}, {-0.183037f,  0.736793f, -0.634757f, -0.143936f} },
    { {-0.031806f, -0.087214f,  0.121015f,  1.000000f}, {-0.003659f,  0.758407f, -0.639342f, -0.126678f} },
    };


    const VRBoneTransform_t left_fist_pose[NUM_BONES] =
    {
    { { 0.000000f,  0.000000f,  0.000000f,  1.000000f}, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
    { {-0.034038f,  0.036503f,  0.164722f,  1.000000f}, {-0.055147f, -0.078608f, -0.920279f,  0.379296f} },
    { {-0.016305f,  0.027529f,  0.017800f,  1.000000f}, { 0.225703f,  0.483332f,  0.126413f,  0.836342f} },
    { { 0.040406f,  0.000000f, -0.000000f,  1.000000f}, { 0.894335f, -0.013302f, -0.082902f,  0.439448f} },
    { { 0.032517f,  0.000000f,  0.000000f,  1.000000f}, { 0.842428f,  0.000655f,  0.001244f,  0.538807f} },
    { { 0.030464f, -0.000000f, -0.000000f,  1.000000f}, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
    { { 0.003802f,  0.021514f,  0.012803f,  1.000000f}, { 0.617314f,  0.395175f, -0.510874f,  0.449185f} },
    { { 0.074204f, -0.005002f,  0.000234f,  1.000000f}, { 0.737291f, -0.032006f, -0.115013f,  0.664944f} },
    { { 0.043287f, -0.000000f, -0.000000f,  1.000000f}, { 0.611381f,  0.003287f,  0.003823f,  0.791321f} },
    { { 0.028275f,  0.000000f,  0.000000f,  1.000000f}, { 0.745388f, -0.000684f, -0.000945f,  0.666629f} },
    { { 0.022821f,  0.000000f, -0.000000f,  1.000000f}, { 1.000000f, -0.000000f,  0.000000f, -0.000000f} },
    { { 0.005787f,  0.006806f,  0.016534f,  1.000000f}, { 0.514203f,  0.522315f, -0.478348f,  0.483700f} },
    { { 0.070953f,  0.000779f,  0.000997f,  1.000000f}, { 0.723653f, -0.097901f,  0.048546f,  0.681458f} },
    { { 0.043108f,  0.000000f,  0.000000f,  1.000000f}, { 0.637464f, -0.002366f, -0.002831f,  0.770472f} },
    { { 0.033266f,  0.000000f,  0.000000f,  1.000000f}, { 0.658008f,  0.002610f,  0.003196f,  0.753000f} },
    { { 0.025892f, -0.000000f,  0.000000f,  1.000000f}, { 0.999195f, -0.000000f,  0.000000f,  0.040126f} },
    { { 0.004123f, -0.006858f,  0.016563f,  1.000000f}, { 0.489609f,  0.523374f, -0.520644f,  0.463997f} },
    { { 0.065876f,  0.001786f,  0.000693f,  1.000000f}, { 0.759970f, -0.055609f,  0.011571f,  0.647471f} },
    { { 0.040331f,  0.000000f,  0.000000f,  1.000000f}, { 0.664315f,  0.001595f,  0.001967f,  0.747449f} },
    { { 0.028489f, -0.000000f, -0.000000f,  1.000000f}, { 0.626957f, -0.002784f, -0.003234f,  0.779042f} },
    { { 0.022430f, -0.000000f,  0.000000f,  1.000000f}, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
    { { 0.001131f, -0.019295f,  0.015429f,  1.000000f}, { 0.479766f,  0.477833f, -0.630198f,  0.379934f} },
    { { 0.062878f,  0.002844f,  0.000332f,  1.000000f}, { 0.827001f,  0.034282f,  0.003440f,  0.561144f} },
    { { 0.029874f,  0.000000f,  0.000000f,  1.000000f}, { 0.702185f, -0.006716f, -0.009289f,  0.711903f} },
    { { 0.017979f,  0.000000f,  0.000000f,  1.000000f}, { 0.676853f,  0.007956f,  0.009917f,  0.736009f} },
    { { 0.018018f,  0.000000f, -0.000000f,  1.000000f}, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
    { { 0.019716f,  0.002802f,  0.093937f,  1.000000f}, { 0.377286f, -0.540831f,  0.150446f, -0.736562f} },
    { { 0.000171f,  0.016473f,  0.096515f,  1.000000f}, {-0.006456f,  0.022747f, -0.932927f, -0.359287f} },
    { { 0.000448f,  0.001536f,  0.116543f,  1.000000f}, {-0.039357f,  0.105143f, -0.928833f, -0.353079f} },
    { { 0.003949f, -0.014869f,  0.130608f,  1.000000f}, {-0.055071f,  0.068695f, -0.944016f, -0.317933f} },
    { { 0.003263f, -0.034685f,  0.139926f,  1.000000f}, { 0.019690f, -0.100741f, -0.957331f, -0.270149f} },
    };

    VRBoneTransform_t right_open_hand_pose[NUM_BONES] = {
    { { 0.000000f,  0.000000f,  0.000000f,  1.000000f}, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
    { {-0.034038f,  0.036503f,  0.164722f,  1.000000f}, {-0.055147f, -0.078608f, -0.920279f,  0.379296f} },
    { {-0.012083f,  0.028070f,  0.025050f,  1.000000f}, { 0.464112f,  0.567418f,  0.272106f,  0.623374f} },
    { { 0.040406f,  0.000000f, -0.000000f,  1.000000f}, { 0.994838f,  0.082939f,  0.019454f,  0.055130f} },
    { { 0.032517f,  0.000000f,  0.000000f,  1.000000f}, { 0.974793f, -0.003213f,  0.021867f, -0.222015f} },
    { { 0.030464f, -0.000000f, -0.000000f,  1.000000f}, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
    { { 0.000632f,  0.026866f,  0.015002f,  1.000000f}, { 0.644251f,  0.421979f, -0.478202f,  0.422133f} },
    { { 0.074204f, -0.005002f,  0.000234f,  1.000000f}, { 0.995332f,  0.007007f, -0.039124f,  0.087949f} },
    { { 0.043930f, -0.000000f, -0.000000f,  1.000000f}, { 0.997891f,  0.045808f,  0.002142f, -0.045943f} },
    { { 0.028695f,  0.000000f,  0.000000f,  1.000000f}, { 0.999649f,  0.001850f, -0.022782f, -0.013409f} },
    { { 0.022821f,  0.000000f, -0.000000f,  1.000000f}, { 1.000000f, -0.000000f,  0.000000f, -0.000000f} },
    { { 0.002177f,  0.007120f,  0.016319f,  1.000000f}, { 0.546723f,  0.541276f, -0.442520f,  0.460749f} },
    { { 0.070953f,  0.000779f,  0.000997f,  1.000000f}, { 0.980294f, -0.167261f, -0.078959f,  0.069368f} },
    { { 0.043108f,  0.000000f,  0.000000f,  1.000000f}, { 0.997947f,  0.018493f,  0.013192f,  0.059886f} },
    { { 0.033266f,  0.000000f,  0.000000f,  1.000000f}, { 0.997394f, -0.003328f, -0.028225f, -0.066315f} },
    { { 0.025892f, -0.000000f,  0.000000f,  1.000000f}, { 0.999195f, -0.000000f,  0.000000f,  0.040126f} },
    { { 0.000513f, -0.006545f,  0.016348f,  1.000000f}, { 0.516692f,  0.550143f, -0.495548f,  0.429888f} },
    { { 0.065876f,  0.001786f,  0.000693f,  1.000000f}, { 0.990420f, -0.058696f, -0.101820f,  0.072495f} },
    { { 0.040697f,  0.000000f,  0.000000f,  1.000000f}, { 0.999545f, -0.002240f,  0.000004f,  0.030081f} },
    { { 0.028747f, -0.000000f, -0.000000f,  1.000000f}, { 0.999102f, -0.000721f, -0.012693f,  0.040420f} },
    { { 0.022430f, -0.000000f,  0.000000f,  1.000000f}, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
    { {-0.002478f, -0.018981f,  0.015214f,  1.000000f}, { 0.526918f,  0.523940f, -0.584025f,  0.326740f} },
    { { 0.062878f,  0.002844f,  0.000332f,  1.000000f}, { 0.986609f, -0.059615f, -0.135163f,  0.069132f} },
    { { 0.030220f,  0.000000f,  0.000000f,  1.000000f}, { 0.994317f,  0.001896f, -0.000132f,  0.106446f} },
    { { 0.018187f,  0.000000f,  0.000000f,  1.000000f}, { 0.995931f, -0.002010f, -0.052079f, -0.073526f} },
    { { 0.018018f,  0.000000f, -0.000000f,  1.000000f}, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
    { {-0.006059f,  0.056285f,  0.060064f,  1.000000f}, { 0.737238f,  0.202745f,  0.594267f,  0.249441f} },
    { {-0.040416f, -0.043018f,  0.019345f,  1.000000f}, {-0.290331f,  0.623527f, -0.663809f, -0.293734f} },
    { {-0.039354f, -0.075674f,  0.047048f,  1.000000f}, {-0.187047f,  0.678062f, -0.659285f, -0.265683f} },
    { {-0.038340f, -0.090987f,  0.082579f,  1.000000f}, {-0.183037f,  0.736793f, -0.634757f, -0.143936f} },
    { {-0.031806f, -0.087214f,  0.121015f,  1.000000f}, {-0.003659f,  0.758407f, -0.639342f, -0.126678f} },
    };


    VRBoneTransform_t right_fist_pose[NUM_BONES] =
    {
    { { 0.000000f,  0.000000f,  0.000000f,  1.000000f}, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
    { {-0.034038f,  0.036503f,  0.164722f,  1.000000f}, {-0.055147f, -0.078608f, -0.920279f,  0.379296f} },
    { {-0.016305f,  0.027529f,  0.017800f,  1.000000f}, { 0.225703f,  0.483332f,  0.126413f,  0.836342f} },
    { { 0.040406f,  0.000000f, -0.000000f,  1.000000f}, { 0.894335f, -0.013302f, -0.082902f,  0.439448f} },
    { { 0.032517f,  0.000000f,  0.000000f,  1.000000f}, { 0.842428f,  0.000655f,  0.001244f,  0.538807f} },
    { { 0.030464f, -0.000000f, -0.000000f,  1.000000f}, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
    { { 0.003802f,  0.021514f,  0.012803f,  1.000000f}, { 0.617314f,  0.395175f, -0.510874f,  0.449185f} },
    { { 0.074204f, -0.005002f,  0.000234f,  1.000000f}, { 0.737291f, -0.032006f, -0.115013f,  0.664944f} },
    { { 0.043287f, -0.000000f, -0.000000f,  1.000000f}, { 0.611381f,  0.003287f,  0.003823f,  0.791321f} },
    { { 0.028275f,  0.000000f,  0.000000f,  1.000000f}, { 0.745388f, -0.000684f, -0.000945f,  0.666629f} },
    { { 0.022821f,  0.000000f, -0.000000f,  1.000000f}, { 1.000000f, -0.000000f,  0.000000f, -0.000000f} },
    { { 0.005787f,  0.006806f,  0.016534f,  1.000000f}, { 0.514203f,  0.522315f, -0.478348f,  0.483700f} },
    { { 0.070953f,  0.000779f,  0.000997f,  1.000000f}, { 0.723653f, -0.097901f,  0.048546f,  0.681458f} },
    { { 0.043108f,  0.000000f,  0.000000f,  1.000000f}, { 0.637464f, -0.002366f, -0.002831f,  0.770472f} },
    { { 0.033266f,  0.000000f,  0.000000f,  1.000000f}, { 0.658008f,  0.002610f,  0.003196f,  0.753000f} },
    { { 0.025892f, -0.000000f,  0.000000f,  1.000000f}, { 0.999195f, -0.000000f,  0.000000f,  0.040126f} },
    { { 0.004123f, -0.006858f,  0.016563f,  1.000000f}, { 0.489609f,  0.523374f, -0.520644f,  0.463997f} },
    { { 0.065876f,  0.001786f,  0.000693f,  1.000000f}, { 0.759970f, -0.055609f,  0.011571f,  0.647471f} },
    { { 0.040331f,  0.000000f,  0.000000f,  1.000000f}, { 0.664315f,  0.001595f,  0.001967f,  0.747449f} },
    { { 0.028489f, -0.000000f, -0.000000f,  1.000000f}, { 0.626957f, -0.002784f, -0.003234f,  0.779042f} },
    { { 0.022430f, -0.000000f,  0.000000f,  1.000000f}, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
    { { 0.001131f, -0.019295f,  0.015429f,  1.000000f}, { 0.479766f,  0.477833f, -0.630198f,  0.379934f} },
    { { 0.062878f,  0.002844f,  0.000332f,  1.000000f}, { 0.827001f,  0.034282f,  0.003440f,  0.561144f} },
    { { 0.029874f,  0.000000f,  0.000000f,  1.000000f}, { 0.702185f, -0.006716f, -0.009289f,  0.711903f} },
    { { 0.017979f,  0.000000f,  0.000000f,  1.000000f}, { 0.676853f,  0.007956f,  0.009917f,  0.736009f} },
    { { 0.018018f,  0.000000f, -0.000000f,  1.000000f}, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
    { { 0.019716f,  0.002802f,  0.093937f,  1.000000f}, { 0.377286f, -0.540831f,  0.150446f, -0.736562f} },
    { { 0.000171f,  0.016473f,  0.096515f,  1.000000f}, {-0.006456f,  0.022747f, -0.932927f, -0.359287f} },
    { { 0.000448f,  0.001536f,  0.116543f,  1.000000f}, {-0.039357f,  0.105143f, -0.928833f, -0.353079f} },
    { { 0.003949f, -0.014869f,  0.130608f,  1.000000f}, {-0.055071f,  0.068695f, -0.944016f, -0.317933f} },
    { { 0.003263f, -0.034685f,  0.139926f,  1.000000f}, { 0.019690f, -0.100741f, -0.957331f, -0.270149f} },
    };


    void RunFrame()
    {
        //#if defined( _WINDOWS )
                // Your driver would read whatever hardware state is associated with its input components and pass that
                // in to UpdateBooleanComponent. This could happen in RunFrame or on a thread of your own that's reading USB
                // state. There's no need to update input state unless it changes, but it doesn't do any harm to do so.
#if USE_MUTEX
        if (!WaitForSingleObject(comm_mutex, 10)) {
#else
        {
#endif

            ovrInputState& inputState(comm_buffer->input_state);
            if (isRightHand) {
                //ovr_GetInputState(mSession, ovrControllerType::ovrControllerType_RTouch, &inputState);
                vr::VRDriverInput()->UpdateBooleanComponent(m_compAc, inputState.Buttons & ovrButton_A, 0);
                vr::VRDriverInput()->UpdateBooleanComponent(m_compBc, inputState.Buttons & ovrButton_B, 0);
                vr::VRDriverInput()->UpdateBooleanComponent(m_compJoyc, inputState.Buttons & ovrButton_RThumb, 0);
                vr::VRDriverInput()->UpdateBooleanComponent(m_compAt, inputState.Touches & ovrTouch_A, 0);
                vr::VRDriverInput()->UpdateBooleanComponent(m_compBt, inputState.Touches & ovrTouch_B, 0);
                vr::VRDriverInput()->UpdateBooleanComponent(m_compTrigt, inputState.Touches & ovrTouch_RIndexTrigger, 0);
                vr::VRDriverInput()->UpdateBooleanComponent(m_compJoyt, inputState.Touches & ovrTouch_RThumb, 0);
#if DO_SKELETON
                
                VRBoneTransform_t active_hand_pose[HSB_Count];

                if (inputState.HandTrigger[isRightHand] > 0.01) {
                    for (int i = 0; i < HSB_Count; i++) active_hand_pose[i] = right_fist_pose[i];
                }
                else {
                    for (int i = 0; i < HSB_Count; i++) active_hand_pose[i] = right_open_hand_pose[i];
                }

                if ((inputState.Touches & ovrTouch_RIndexPointing) || (inputState.IndexTrigger[isRightHand]<0.01))
                {
                    for (int i = HSB_IndexFinger0; i <= HSB_IndexFinger4; i++) active_hand_pose[i] = right_open_hand_pose[i];
                }
                if (inputState.Touches & ovrTouch_RThumbUp)
                {
                    for (int i = HSB_Thumb0; i <= HSB_Thumb3; i++) active_hand_pose[i] = right_open_hand_pose[i];
                }
                vr::VRDriverInput()->UpdateSkeletonComponent(
                    m_compSkel,
                    vr::VRSkeletalMotionRange_WithoutController,
                    active_hand_pose,      
                    NUM_BONES);
#endif
            }
            else {
                //ovr_GetInputState(mSession, ovrControllerType::ovrControllerType_LTouch, &inputState);
                vr::VRDriverInput()->UpdateBooleanComponent(m_compXc, inputState.Buttons & ovrButton_X, 0);
                vr::VRDriverInput()->UpdateBooleanComponent(m_compYc, inputState.Buttons & ovrButton_Y, 0);
                vr::VRDriverInput()->UpdateBooleanComponent(m_compJoyc, inputState.Buttons & ovrButton_LThumb, 0);
                vr::VRDriverInput()->UpdateBooleanComponent(m_compXt, inputState.Touches & ovrTouch_X, 0);
                vr::VRDriverInput()->UpdateBooleanComponent(m_compYt, inputState.Touches & ovrTouch_Y, 0);
                vr::VRDriverInput()->UpdateBooleanComponent(m_compTrigt, inputState.Touches & ovrTouch_LIndexTrigger, 0);
                vr::VRDriverInput()->UpdateBooleanComponent(m_compJoyt, inputState.Touches & ovrTouch_LThumb, 0);

                vr::VRDriverInput()->UpdateBooleanComponent(m_compSysc, inputState.Buttons & ovrButton_Enter, 0);
#if DO_LSKELETON
                VRBoneTransform_t active_hand_pose[HSB_Count];

                if (inputState.HandTrigger[isRightHand] > 0.01) {
                    for (int i = 0; i < HSB_Count; i++) active_hand_pose[i] = left_fist_pose[i];
                }
                else {
                    for (int i = 0; i < HSB_Count; i++) active_hand_pose[i] = left_open_hand_pose[i];
                }

                if ((inputState.Touches & ovrTouch_RIndexPointing) || (inputState.IndexTrigger[isRightHand] < 0.01))
                {
                    for (int i = HSB_IndexFinger0; i <= HSB_IndexFinger4; i++) active_hand_pose[i] = left_open_hand_pose[i];
                }
                if (inputState.Touches & ovrTouch_RThumbUp)
                {
                    for (int i = HSB_Thumb0; i <= HSB_Thumb3; i++) active_hand_pose[i] = left_open_hand_pose[i];
                }
                vr::VRDriverInput()->UpdateSkeletonComponent(
                    m_compSkel,
                    vr::VRSkeletalMotionRange_WithoutController,
                    active_hand_pose,
                    NUM_BONES);
#endif
            }

            vr::VRDriverInput()->UpdateScalarComponent(m_compTrigv, inputState.IndexTrigger[isRightHand], 0);
            vr::VRDriverInput()->UpdateBooleanComponent(m_compGripv, inputState.HandTrigger[isRightHand]>0.5, 0);
            vr::VRDriverInput()->UpdateScalarComponent(m_compGripv, inputState.HandTrigger[isRightHand], 0);
            vr::VRDriverInput()->UpdateScalarComponent(m_compJoyx, inputState.Thumbstick[isRightHand].x, 0);
            vr::VRDriverInput()->UpdateScalarComponent(m_compJoyy, inputState.Thumbstick[isRightHand].y, 0);

#if USE_MUTEX
            ReleaseMutex(comm_mutex);
#endif
        }

        //this block of code lets you manipulate the pre and post rotation offsets of the controllers, the final value used in the constructor was determined by manual calibration using this code
        /*
        if (inputState.Buttons & ovrTouch_RThumb) {
            if (inputState.Thumbstick[isRightHand].x > 0.5) hand_offset.x += 0.001f;
            if (inputState.Thumbstick[isRightHand].x < -0.5) hand_offset.x -= 0.001f;

            if (inputState.Thumbstick[isRightHand].y > 0.5) hand_offset.y += 0.001f;
            if (inputState.Thumbstick[isRightHand].y < -0.5) hand_offset.y -= 0.001f;

            if (inputState.HandTrigger[isRightHand] > 0.5) hand_offset.z -= 0.001f;
            if (inputState.IndexTrigger[isRightHand] > 0.5) hand_offset.z += 0.001f;
        }
        else {
            if (inputState.Thumbstick[isRightHand].x > 0.5) hand_offset2.x += 0.001f;
            if (inputState.Thumbstick[isRightHand].x < -0.5) hand_offset2.x -= 0.001f;

            if (inputState.Thumbstick[isRightHand].y > 0.5) hand_offset2.y += 0.001f;
            if (inputState.Thumbstick[isRightHand].y < -0.5) hand_offset2.y -= 0.001f;

            if (inputState.HandTrigger[isRightHand] > 0.5) hand_offset2.z -= 0.001f;
            if (inputState.IndexTrigger[isRightHand] > 0.5) hand_offset2.z += 0.001f;
        }
        
        std::fstream tmp_out;
        tmp_out.open((isRightHand?"c:/test/ovr_offset_r.txt": "c:/test/ovr_offset_l.txt"));
        tmp_out << "x = " << hand_offset.x << " y = " << hand_offset.y << " z = " << hand_offset.z << " x2 =" << hand_offset2.x << " y2 = " << hand_offset2.y << " z = " << hand_offset2.z << std::endl;
        tmp_out.close();
        tmp_out.open((isRightHand ? "ovr_offset_r.txt" : "ovr_offset_l.txt"));
        tmp_out << "x = " << hand_offset.x << " y = " << hand_offset.y << " z = " << hand_offset.z << " x2 =" << hand_offset2.x << " y2 = " << hand_offset2.y << " z = " << hand_offset2.z << std::endl;
        tmp_out.close();
        std::cout << "x = " << hand_offset.x << " y = " << hand_offset.y << " z = " << hand_offset.z << " x2 =" << hand_offset2.x << " y2 = " << hand_offset2.y << " z = " << hand_offset2.z << std::endl;
        std::cerr << "x = " << hand_offset.x << " y = " << hand_offset.y << " z = " << hand_offset.z << " x2 =" << hand_offset2.x << " y2 = " << hand_offset2.y << " z = " << hand_offset2.z << std::endl; 
        */

        m_last_pose = this->CalculatePose();
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, m_last_pose, sizeof(DriverPose_t));


    }

    void ProcessEvent(const vr::VREvent_t& vrEvent)
    {

        //if(vrEvent.trackedDeviceIndex == m_unObjectId)
        switch (vrEvent.eventType)
        {
        case vr::VREvent_Input_HapticVibration:
        {
            if (vrEvent.data.hapticVibration.componentHandle == m_compHaptic)
            {

 
#if USE_MUTEX                
                if (!WaitForSingleObject(comm_mutex, 10)) {
#endif
                    comm_buffer->vib_duration_s[isRightHand] = vrEvent.data.hapticVibration.fDurationSeconds;
                    comm_buffer->vib_amplitude[isRightHand] = vrEvent.data.hapticVibration.fAmplitude;
                    comm_buffer->vib_frequency[isRightHand] = vrEvent.data.hapticVibration.fFrequency;
                    comm_buffer->vib_valid[isRightHand] = true;
#if USE_MUTEX
                    ReleaseMutex(comm_mutex);
                }
#endif
                
   
            }
        }
        break;
        }
    }


    std::string GetSerialNumber() const { return m_sSerialNumber; }

private:
    vr::TrackedDeviceIndex_t m_unObjectId;
    vr::PropertyContainerHandle_t m_ulPropertyContainer;

    vr::VRInputComponentHandle_t m_compAc;
    vr::VRInputComponentHandle_t m_compBc;
    vr::VRInputComponentHandle_t m_compXc;
    vr::VRInputComponentHandle_t m_compYc;
    vr::VRInputComponentHandle_t m_compAt;
    vr::VRInputComponentHandle_t m_compBt;
    vr::VRInputComponentHandle_t m_compXt;
    vr::VRInputComponentHandle_t m_compYt;
    vr::VRInputComponentHandle_t m_compTrigv;
    vr::VRInputComponentHandle_t m_compTrigt;
    vr::VRInputComponentHandle_t m_compGripv;
    vr::VRInputComponentHandle_t m_compGripb;
    vr::VRInputComponentHandle_t m_compGript;
    vr::VRInputComponentHandle_t m_compJoyx;
    vr::VRInputComponentHandle_t m_compJoyy;
    vr::VRInputComponentHandle_t m_compJoyc;
    vr::VRInputComponentHandle_t m_compJoyt;
    vr::VRInputComponentHandle_t m_compSysc;
    vr::VRInputComponentHandle_t m_compSkel;

    vr::VRInputComponentHandle_t m_compHaptic;
    shared_buffer* comm_buffer;
    std::string m_sSerialNumber;
    std::string m_sModelNumber;
    ovrSession mSession;
    bool isRightHand;
    ovrVector3f hand_offset;
    ovrVector3f hand_offset2;


    DriverPose_t m_last_pose;
    float m_time_of_last_pose;


    /*std::chrono::time_point<std::chrono::steady_clock>  haptic_end;
    float  haptic_strength;
    float  haptic_frequency; */
    //uint8_t hap_buf[24];// = { 255,255,0,0,255,255,0,0,0,0,255,255,0,0,255,255,0,0,0,0,255,255,0,0,255,255,0,0,0,0,255,255,255,255,0,0,0,0,255,255,255,255,0,0,0,0,255,255,255,255,0,0,0,0 };
    //ovrHapticsBuffer vibuffer;
   /* ovrVector3f overall_offset{ 0 };
    ovrQuatf overall_rotation{ 0 };         */
};

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
    void InitRenderTargets(const ovrHmdDesc& hmdDesc);
    void  Render();
    ovrSession mSession = nullptr;
    ovrGraphicsLuid luid{};
#if ADD_HMD
    CSampleDeviceDriver* m_pNullHmdLatest = nullptr;
#endif
    CSampleControllerDriver* m_pLController = nullptr;
    CSampleControllerDriver* m_pRController = nullptr;
    HANDLE hMapFile;
    shared_buffer* comm_buffer;
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

#if EXPERIMENTAL_OFFSET_CALIBRATION
    ovrVector3f overall_offset{ -0.01*30.57033738,-0.01*46.98443338, 0.01* 54.50133916 };
    ovrQuatf overall_rotation{ 0 };

    overall_rotation = ToQuaternion(0, (30.28197291 / 180.0) * 3.141592, 0);
#endif

    //map the shared memory buffer for communicating button and vibration data

#if USE_SHARE_MEM_BUFFER 1    
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
        std::cout<<"Could not create file mapping object "<< GetLastError() << std::endl;
        return VRInitError_Init_Internal;
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

        return VRInitError_Init_Internal;
    }
#endif

    // get a handle to the inter-process mutex used for accessing the shared data structure
#if USE_MUTEX
    comm_mutex = CreateMutex(0, false, L"Local\\oculus_steamvr_touch_controller_mutex");
    if (comm_mutex == NULL)
    {
        std::cout << "Could notopen mutex" << GetLastError() << std::endl;

        return VRInitError_Init_Internal;
    }
#endif


    if (!mSession) {
#if DRAW_FRAME
        ovrInitParams initParams = { ovrInit_RequestVersion, OVR_MINOR_VERSION, NULL, 0, 0 };

#else
        ovrInitParams initParams = { ovrInit_RequestVersion | ovrInit_FocusAware | ovrInit_Invisible, OVR_MINOR_VERSION, NULL, 0, 0 };
#endif
        if (OVR_FAILURE(ovr_Initialize(&initParams)))
            return VRInitError_Init_Internal;

        if (OVR_FAILURE(ovr_Create(&mSession, &luid)))
            return VRInitError_Init_Internal;

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
        if (OVR_FAILURE(ovr_SetTrackingOriginType(mSession, ovrTrackingOrigin_FloorLevel)))
            return VRInitError_Init_Internal;
    }
 #ifdef ADD_HMD
    m_pNullHmdLatest = new CSampleDeviceDriver();
    vr::VRServerDriverHost()->TrackedDeviceAdded(m_pNullHmdLatest->GetSerialNumber().c_str(), vr::TrackedDeviceClass_HMD, m_pNullHmdLatest);
#endif
#if CREATE_CONTROLLERS
    m_pLController = new CSampleControllerDriver(mSession, false, comm_buffer/*, overall_offset, overall_rotation*/);
    vr::VRServerDriverHost()->TrackedDeviceAdded(m_pLController->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pLController);

    m_pRController = new CSampleControllerDriver(mSession, true, comm_buffer/*, overall_offset, overall_rotation*/);
    vr::VRServerDriverHost()->TrackedDeviceAdded(m_pRController->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pRController);
#endif

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
    return VRInitError_None;
}

void CServerDriver_OVRTL::Cleanup()
{
    CleanupDriverLog();
#ifdef ADD_HMD
    delete m_pNullHmdLatest;
    m_pNullHmdLatest = NULL;
#endif
#if CREATE_CONTROLLERS
    delete m_pLController;
    m_pLController = NULL;
    delete m_pRController;
    m_pRController = NULL;
#endif
    ovr_Destroy(mSession);
    ovr_Shutdown();
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
#ifdef ADD_HMD
    if (m_pNullHmdLatest)
    {
        m_pNullHmdLatest->RunFrame();
    }
#endif
    if (m_pLController)
    {
        m_pLController->RunFrame();
    }
    if (m_pRController)
    {
        m_pRController->RunFrame();
    }

    vr::VREvent_t vrEvent;
    while (vr::VRServerDriverHost()->PollNextEvent(&vrEvent, sizeof(vrEvent)))
    {
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

        if (m_pLController)
        {
            m_pLController->ProcessEvent(vrEvent);
        }
        if (m_pRController)
        {
            m_pRController->ProcessEvent(vrEvent);
        }
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


