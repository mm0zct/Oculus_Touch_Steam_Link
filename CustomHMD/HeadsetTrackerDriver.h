#pragma once
#include "Common.h"


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CTouchHeadsetTrackerDriver : public vr::ITrackedDeviceServerDriver
{
public:
    CTouchHeadsetTrackerDriver(ovrSession mSession) : mSession(mSession)
    {
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
        m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;
        m_sSerialNumber = "OTL-00000099";
        m_sModelNumber = "Oculus Rift CV1(HMD)";
        log_to_buffer(__func__);
    }

    virtual ~CTouchHeadsetTrackerDriver()
    {
    }


    virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
    {

        log_to_buffer(__func__);
        m_unObjectId = unObjectId;
        m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_TrackingSystemName_String, comm_buffer->config.tracking_space_name);
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "generic_hmd");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, m_sModelNumber.c_str());
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_SerialNumber_String, m_sSerialNumber.c_str());
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, comm_buffer->config.manufacturer_name);
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_HardwareRevision_String, "1");
        vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_HardwareRevision_Uint64, 1U);
        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_GenericTracker);
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ResourceRoot_String, "oculus");


        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RegisteredDeviceType_String, "htc/vive_trackerLHR-OCULUS_HMD");

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_InputProfilePath_String, "{htc}/input/vive_tracker_profile.json");
        // vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_SupportedButtons_Uint64, 30064771207U);
        // vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_Axis0Type_Int32, vr::k_eControllerAxis_Joystick);
        // vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_Axis1Type_Int32, vr::k_eControllerAxis_Trigger);
        // vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_Axis2Type_Int32, vr::k_eControllerAxis_Trigger);
        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_Invalid);
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, "vive_tracker");

        // vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_off.png" : "{oculus}/icons/cv1_right_controller_off.png");
        // vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_searching.gif" : "{oculus}/icons/cv1_right_controller_searching.gif");
        // vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_alert_searching.gif" : "{oculus}/icons/cv1_right_controller_alert_searching.gif");
        // vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_ready.png" : "{oculus}/icons/cv1_right_controller_ready.png");
        // vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_ready_alert.png" : "{oculus}/icons/cv1_right_controller_ready_alert.png");
        // vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_error.png" : "{oculus}/icons/cv1_right_controller_error.png");
        // vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_standby.png" : "{oculus}/icons/cv1_right_controller_standby.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{oculus}/icons/cv1_headset_off.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, "{oculus}/icons/cv1_headset_searching.gif");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{oculus}/icons/cv1_headset_searching_alert.gif");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{oculus}/icons/cv1_headset_ready.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{oculus}/icons/cv1_headset_ready_alert.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, "{oculus}/icons/cv1_headset_error.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, "{oculus}/icons/cv1_headset_standby.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, "{oculus}/icons/cv1_headset_ready_low.png");

        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasDisplayComponent_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasCameraComponent_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasDriverDirectModeComponent_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasVirtualDisplayComponent_Bool, false);

        // return a constant that's not 0 (invalid), 1 is reserved for Oculus, so let's use that ;)
        vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, comm_buffer->config.vr_universe);

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


    virtual DriverPose_t GetPose()
    {
        m_last_pose = CalculatePose();
        return this->m_last_pose;
    }
    virtual DriverPose_t CalculatePose()
    {

        ovrTrackedDeviceType deviceType = ovrTrackedDevice_HMD;
        ovrPoseStatef ovr_pose;
        //ovr_GetDevicePoses(mSession, &deviceType, 1, ovr_GetTimeInSeconds(), &ovr_pose);

        ovrTrackingState ss;
        if (comm_buffer->config.external_tracking) {
            ss = comm_buffer->tracking_state;
        }
        else {
            ss = ovr_GetTrackingState(mSession, (ovr_GetTimeInSeconds() + (comm_buffer->config.extra_prediction_ms * 0.001)),
                ovrTrue);
        }
        ovr_pose = ss.HeadPose;
        DriverPose_t pose = { 0 };
        pose.poseIsValid = true;
        pose.result = TrackingResult_Running_OK;
        pose.deviceIsConnected = true;

        pose.qRotation.w = ovr_pose.ThePose.Orientation.w;
        pose.qRotation.x = ovr_pose.ThePose.Orientation.x;
        pose.qRotation.y = ovr_pose.ThePose.Orientation.y;
        pose.qRotation.z = ovr_pose.ThePose.Orientation.z;

        pose.vecPosition[0] = ovr_pose.ThePose.Position.x;
        pose.vecPosition[1] = ovr_pose.ThePose.Position.y;
        pose.vecPosition[2] = ovr_pose.ThePose.Position.z;

        ovrVector3f linAcc = (ovr_pose.LinearAcceleration);
        ovrVector3f linVel = (ovr_pose.LinearVelocity);


        pose.vecAcceleration[0] = linAcc.x;
        pose.vecAcceleration[1] = linAcc.y;
        pose.vecAcceleration[2] = linAcc.z;

        pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
        pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

        pose.vecVelocity[0] = linVel.x;
        pose.vecVelocity[1] = linVel.y;
        pose.vecVelocity[2] = linVel.z;


        pose.poseTimeOffset = 0;  // let's let Oculus do it


        pose.vecAngularAcceleration[0] = ovr_pose.AngularAcceleration.x;
        pose.vecAngularAcceleration[1] = ovr_pose.AngularAcceleration.y;
        pose.vecAngularAcceleration[2] = ovr_pose.AngularAcceleration.z;

        pose.vecAngularVelocity[0] = ovr_pose.AngularVelocity.x;
        pose.vecAngularVelocity[1] = ovr_pose.AngularVelocity.y;
        pose.vecAngularVelocity[2] = ovr_pose.AngularVelocity.z;


        return pose;
    }




    void RunFrame()
    {
        m_last_pose = this->CalculatePose();
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, m_last_pose, sizeof(DriverPose_t));
    }

    void ProcessEvent(const vr::VREvent_t& vrEvent)
    {

    }


    std::string GetSerialNumber() const { log_to_buffer(__func__); return m_sSerialNumber; }

private:
    vr::TrackedDeviceIndex_t m_unObjectId;
    vr::PropertyContainerHandle_t m_ulPropertyContainer;

    std::string m_sSerialNumber;
    std::string m_sModelNumber;
    ovrSession mSession;


    DriverPose_t m_last_pose;


};


