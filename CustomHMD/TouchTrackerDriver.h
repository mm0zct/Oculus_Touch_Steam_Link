#pragma once

#include "Common.h"

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CTouchTrackerDriver : public vr::ITrackedDeviceServerDriver
{                                                                                                                                             // hand_offset({ 0.01071,0.04078,-0.04731 }), hand_offset2({-0.003,-0.101,0.0089 })
public:                                                                                                                                      //x = 0.00571 y = 0.04078 z = -0.03531 x2 =-0.000999998 y2 = -0.1 z = 0.0019
    CTouchTrackerDriver(ovrSession mSession, /*ovrTrackedDeviceType */ unsigned int object_index, bool isRightHand = true/*, ovrVector3f overall_offset, ovrQuatf overall_rotation*/) : mSession(mSession), m_object_index(object_index), isRightHand(isRightHand), hand_offset({ 0.00571,0.04078,-0.03531 }), hand_offset2({ -0.000999998,-0.1, 0.0019 })/*, overall_offset(overall_offset), overall_rotation(overall_rotation)*/
    {
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
        m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;
        m_sSerialNumber = "ODT-0000000" + std::to_string(object_index + 5);
        m_sModelNumber = "Oculus Rift CV1 Tracker " + std::to_string(object_index);
        log_to_buffer(__func__);
    }

    virtual ~CTouchTrackerDriver()
    {
    }


    virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
    {

        log_to_buffer(__func__);
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
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_TrackingSystemName_String, comm_buffer->config.tracking_space_name);
        VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "{htc}vr_tracker_vive_1_0");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, "Vive Tracker Pro MV");
        // vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_SerialNumber_String, m_sSerialNumber.c_str());
       /* vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, (isRightHand) ? "oculus_cv1_controller_right" : "oculus_cv1_controller_left");    */
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, comm_buffer->config.manufacturer_name);
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_HardwareRevision_String, "14");
        vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_HardwareRevision_Uint64, 14U);
        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_GenericTracker);
        //vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_ParentDriver_Uint64, 8589934599U); // Strange value from dump
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ResourceRoot_String, "htc");


        //vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RegisteredDeviceType_String, (isRightHand) ? "htc/vive_trackerLHR-OCULUS_RIGHT" : "htc/vive_trackerLHR-OCULUS_LEFT");

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_InputProfilePath_String, "{htc}/input/vive_tracker_profile.json");
        // vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_SupportedButtons_Uint64, 30064771207U);
        // vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_Axis0Type_Int32, vr::k_eControllerAxis_Joystick);
        // vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_Axis1Type_Int32, vr::k_eControllerAxis_Trigger);
        // vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_Axis2Type_Int32, vr::k_eControllerAxis_Trigger);
        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_Invalid);
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, "vive_tracker");

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, m_sModelNumber.c_str()); // not sure if this is needed
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_SerialNumber_String, m_sSerialNumber.c_str());

        // vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_off.png" : "{oculus}/icons/cv1_right_controller_off.png");
        // vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_searching.gif" : "{oculus}/icons/cv1_right_controller_searching.gif");
        // vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_alert_searching.gif" : "{oculus}/icons/cv1_right_controller_alert_searching.gif");
        // vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_ready.png" : "{oculus}/icons/cv1_right_controller_ready.png");
        // vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_ready_alert.png" : "{oculus}/icons/cv1_right_controller_ready_alert.png");
        // vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_error.png" : "{oculus}/icons/cv1_right_controller_error.png");
        // vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_standby.png" : "{oculus}/icons/cv1_right_controller_standby.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{htc}/icons/tracker_status_off.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, "{htc}/icons/tracker_status_searching.gif");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{htc}/icons/tracker_status_searching_alert.gif");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{htc}/icons/tracker_status_ready.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{htc}/icons/tracker_status_ready_alert.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, "{htc}/icons/tracker_status_error.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, "{htc}/icons/tracker_status_standby.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, "{htc}/icons/tracker_status_ready_low.png");

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
        /*ovrTrackedDeviceType deviceType = m_object_index;
        ovrPoseStatef ovr_pose;
        ovr_GetDevicePoses(mSession, &deviceType, 1, ovr_GetTimeInSeconds(), &ovr_pose);*/
        ovrPoseStatef ovr_pose = comm_buffer->object_poses[this->m_object_index];
        DriverPose_t pose = { 0 };
        pose.poseIsValid = true;
        pose.result = TrackingResult_Running_OK;
        pose.deviceIsConnected = true;
        float delta_t = (comm_buffer->config.extra_prediction_ms * 0.001f) + (ovr_GetTimeInSeconds() - ovr_pose.TimeInSeconds);

        ovrQuatf hand_qoffset = { 0.3420201, 0, 0, 0.9396926 };
        ovrQuatf hand_input = ovr_pose.ThePose.Orientation;
        ovrQuatf hand_result = ovrQuatfmul(hand_input, hand_qoffset);
        ovrVector3f hand_voffset = { 0,0,0 };

#if CORRECT_OBJECT_TRACKING
        if (isRightHand) {
            hand_voffset = rotateVector2(hand_offset, hand_input);
        }
        else {
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
        position.x = ovr_pose.ThePose.Position.x + hand_voffset.x + hand_offset2.x;
        position.y = ovr_pose.ThePose.Position.y + hand_voffset.y + hand_offset2.y;
        position.z = ovr_pose.ThePose.Position.z + hand_voffset.z + hand_offset2.z;
        //position = rotateVector2(position, overall_rotation);
        pose.vecPosition[0] = position.x;// +overall_offset.x;
        pose.vecPosition[1] = position.y;// +overall_offset.y;
        pose.vecPosition[2] = position.z;// +overall_offset.z;
#else
        pose.qRotation.w = ovr_pose.ThePose.Orientation.w;
        pose.qRotation.x = ovr_pose.ThePose.Orientation.x;
        pose.qRotation.y = ovr_pose.ThePose.Orientation.y;
        pose.qRotation.z = ovr_pose.ThePose.Orientation.z;

        pose.vecPosition[0] = ovr_pose.ThePose.Position.x;
        pose.vecPosition[1] = ovr_pose.ThePose.Position.y + 0.09;
        pose.vecPosition[2] = ovr_pose.ThePose.Position.z;
#endif
        ovrVector3f linAcc = (ovr_pose.LinearAcceleration);
        ovrVector3f linVel = (ovr_pose.LinearVelocity);

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


        pose.poseTimeOffset = 0;  // let's let Oculus do it

        pose.vecAngularAcceleration[0] = ovr_pose.AngularAcceleration.x;
        pose.vecAngularAcceleration[1] = ovr_pose.AngularAcceleration.y;
        pose.vecAngularAcceleration[2] = ovr_pose.AngularAcceleration.z;

        pose.vecAngularVelocity[0] = ovr_pose.AngularVelocity.x;
        pose.vecAngularVelocity[1] = ovr_pose.AngularVelocity.y;
        pose.vecAngularVelocity[2] = ovr_pose.AngularVelocity.z;

        //pose.poseTimeOffset = -0.01;
        if (comm_buffer->config.do_world_transformation) {
            pose.qWorldFromDriverRotation = comm_buffer->config.world_orientation_q * pose.qWorldFromDriverRotation;

            vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(comm_buffer->config.world_orientation_q, pose.vecWorldFromDriverTranslation);
            pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + comm_buffer->config.world_translation[0];
            pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + comm_buffer->config.world_translation[1];
            pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + comm_buffer->config.world_translation[2];

        }
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

    vr::TrackedDeviceIndex_t m_unObjectId;
private:
    vr::PropertyContainerHandle_t m_ulPropertyContainer;

    std::string m_sSerialNumber;
    std::string m_sModelNumber;
    ovrSession mSession;
    /*ovrTrackedDeviceType*/ unsigned int m_object_index;
    bool isRightHand;
    ovrVector3f hand_offset;
    ovrVector3f hand_offset2;


    DriverPose_t m_last_pose;
    float m_time_of_last_pose;


};

