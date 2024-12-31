#pragma once
#include "Common.h"


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CTouchSensorDriver : public vr::ITrackedDeviceServerDriver
{
public:
    CTouchSensorDriver(ovrSession mSession, /*ovrTrackedDeviceType */ unsigned int object_index) :
        mSession(mSession),
        m_object_index(object_index)
    {
        //m_unObjectId = vr::k_unTrackedDeviceIndexInvalid; //Crashes OpenVR-SpaceCalibrator on start
        m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;
        m_sSerialNumber = "ODS-000000" + std::to_string(object_index + 10);
        m_sModelNumber = "Oculus Rift CV1 Sensor " + std::to_string(object_index);
        log_to_buffer(__func__);
    }

    virtual ~CTouchSensorDriver()
    {
        
    }

    virtual EVRInitError Activate( vr::TrackedDeviceIndex_t unObjectId )
    {
        log_to_buffer(__func__);
        m_unObjectId = unObjectId;
        m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_TrackingSystemName_String, comm_buffer->config.tracking_space_name);
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "rift_camera");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, comm_buffer->config.manufacturer_name);
        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_TrackingReference);

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, m_sModelNumber.c_str()); // not sure if this is needed
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_SerialNumber_String, m_sSerialNumber.c_str());

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{oculus}/icons/cv1_camera_off.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, "{oculus}/icons/cv1_camera_searching.gif");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{oculus}/icons/cv1_camera_searching_alert.gif");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{oculus}/icons/cv1_camera_ready.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{oculus}/icons/cv1_camera_ready_alert.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, "{oculus}/icons/cv1_camera_error.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, "{oculus}/icons/cv1_camera_off.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, "{oculus}/icons/cv1_camera_ready_alert.png");

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
        ovrTrackerPose ovr_pose = comm_buffer->config.external_tracking ? comm_buffer->sensor_poses[this->m_object_index] : ovr_GetTrackerPose(mSession, this->m_object_index);

        DriverPose_t pose = { 0 };
        pose.poseIsValid = true;
        pose.result = TrackingResult_Running_OK;
        pose.deviceIsConnected = true;

        pose.qRotation.w = ovr_pose.LeveledPose.Orientation.w;
        pose.qRotation.x = ovr_pose.LeveledPose.Orientation.x;
        pose.qRotation.y = ovr_pose.LeveledPose.Orientation.y;
        pose.qRotation.z = ovr_pose.LeveledPose.Orientation.z;

        pose.vecPosition[0] = ovr_pose.LeveledPose.Position.x;
        pose.vecPosition[1] = ovr_pose.LeveledPose.Position.y;
        pose.vecPosition[2] = ovr_pose.LeveledPose.Position.z;

        pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
        pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

        pose.poseTimeOffset = 0;  // let's let Oculus do it

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
    //vr::VRInputComponentHandle_t m_compHaptic;

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
