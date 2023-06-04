#pragma once

#include "Common.h"

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CTouchControllerDriver : public vr::ITrackedDeviceServerDriver
{                                                                                                                                             // hand_offset({ 0.01071,0.04078,-0.04731 }), hand_offset2({-0.003,-0.101,0.0089 })
public:                                                                                                                                      //x = 0.00571 y = 0.04078 z = -0.03531 x2 =-0.000999998 y2 = -0.1 z = 0.0019
    CTouchControllerDriver(ovrSession mSession, bool isRightHand/*, ovrVector3f overall_offset, ovrQuatf overall_rotation*/) :
        mSession(mSession), isRightHand(isRightHand)
        , hand_offset({ 0.00571,0.04078,-0.03531 })
        , hand_offset2({ -0.000999998,-0.1, 0.0019 }
        )/*, overall_offset(overall_offset), overall_rotation(overall_rotation)*/
    {
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
        m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;
        if (isRightHand) {
            m_sSerialNumber = "ODT-00000003";
            m_sModelNumber = "Oculus Rift CV1(Right Controller)";
        }
        else {
            m_sSerialNumber = "ODT-00000004";
            m_sModelNumber = "Oculus Rift CV1(Left Controller)";
        }
        log_to_buffer(__func__);
    }

    virtual ~CTouchControllerDriver()
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
  

        vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, comm_buffer->config.vr_universe);

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, m_sModelNumber.c_str());
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_SerialNumber_String, m_sSerialNumber.c_str());

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_TrackingSystemName_String, comm_buffer->config.tracking_space_name);
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, comm_buffer->config.manufacturer_name);

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_HardwareRevision_String, "14");
        vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_HardwareRevision_Uint64, 14U);
        //vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_ParentDriver_Uint64, 8589934599U); // Strange value from dump
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasDisplayComponent_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasCameraComponent_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasDriverDirectModeComponent_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasVirtualDisplayComponent_Bool, false);

        if (comm_buffer->config.be_objects) {
            VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "{htc}vr_tracker_vive_1_0");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, "Vive Tracker Pro MV");
            vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_GenericTracker);
            //vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_ParentDriver_Uint64, 8589934599U); // Strange value from dump
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ResourceRoot_String, "htc");

            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RegisteredDeviceType_String, (isRightHand) ? "htc/vive_trackerLHR-OCULUS_RIGHT" : "htc/vive_trackerLHR-OCULUS_LEFT");

            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_InputProfilePath_String, "{htc}/input/vive_tracker_profile.json");
            // vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_SupportedButtons_Uint64, 30064771207U);
            vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_Invalid);
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, "vive_tracker");

            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{htc}/icons/tracker_status_off.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, "{htc}/icons/tracker_status_searching.gif");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{htc}/icons/tracker_status_searching_alert.gif");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{htc}/icons/tracker_status_ready.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{htc}/icons/tracker_status_ready_alert.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, "{htc}/icons/tracker_status_error.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, "{htc}/icons/tracker_status_standby.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, "{htc}/icons/tracker_status_ready_low.png");
        } else {
            vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ResourceRoot_String, "oculus");

            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RegisteredDeviceType_String, (isRightHand) ? "oculus/WMHD316J600000_Controller_Right" : "oculus/WMHD316J600000_Controller_Left");

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
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_searching_alert.gif" : "{oculus}/icons/cv1_right_controller_searching_alert.gif");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_ready.png" : "{oculus}/icons/cv1_right_controller_ready.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_ready_alert.png" : "{oculus}/icons/cv1_right_controller_ready_alert.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_error.png" : "{oculus}/icons/cv1_right_controller_error.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_searching.gif" : "{oculus}/icons/cv1_right_controller_searching.gif");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_ready_low.png" : "{oculus}/icons/cv1_right_controller_ready_low.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandbyAlert_String, (!isRightHand) ? "{oculus}/icons/cv1_left_controller_standby.png" : "{oculus}/icons/cv1_right_controller_standby.png");


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
   
            // Register our input bindings
            if (isRightHand) {
                vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/a/click", &m_compAc);
                vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/b/click", &m_compBc);
                vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/a/touch", &m_compAt);
                vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/b/touch", &m_compBt);
                vr::VRDriverInput()->CreateSkeletonComponent(m_ulPropertyContainer, "/input/skeleton/right", "/skeleton/hand/right", "/pose/raw", VRSkeletalTracking_Estimated, nullptr, 0, &m_compSkel);
            }
            else {
                vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &m_compSysc);
                vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/x/click", &m_compXc);
                vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/y/click", &m_compYc);
                vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/x/touch", &m_compXt);
                vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/y/touch", &m_compYt);
                vr::VRDriverInput()->CreateSkeletonComponent(m_ulPropertyContainer, "/input/skeleton/left", "/skeleton/hand/left", "/pose/raw", VRSkeletalTracking_Estimated, nullptr, 0, &m_compSkel);
            }

            vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/grip/value", &m_compGripv, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/button", &m_compGripb);
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/touch", &m_compGript);
            vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trigger/value", &m_compTrigv, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
            vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trigger/touch", &m_compTrigt);
            vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/joystick/x", &m_compJoyx, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
            vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/joystick/y", &m_compJoyy, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
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

    virtual DriverPose_t GetPose()
    {
        m_last_pose = CalculatePose();
        return this->m_last_pose;
    }
    virtual DriverPose_t CalculatePose()
    {
        ovrTrackingState ss;
        if (comm_buffer->config.external_tracking) {
            ss = comm_buffer->tracking_state;
        }
        else {
            ss = ovr_GetTrackingState(mSession,
                (ovr_GetTimeInSeconds() + (comm_buffer->config.extra_prediction_ms * 0.001)),
                ovrTrue);
        }
        m_time_of_last_pose = ovr_GetTimeInSeconds();// ss.HandPoses[isRightHand].TimeInSeconds;
        DriverPose_t pose = { 0 };
        pose.poseIsValid = true;
        pose.result = TrackingResult_Running_OK;
        pose.deviceIsConnected = true;

        ovrQuatf hand_qoffset = { 0.3420201, 0, 0, 0.9396926 };
        ovrQuatf hand_input = ss.HandPoses[isRightHand].ThePose.Orientation;
        ovrQuatf hand_result = hand_input;// ovrQuatfmul(hand_input, hand_qoffset);
        ovrVector3f hand_voffset = { 0,0,0 };
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
        if (comm_buffer->config.be_objects) {
            position.x = ss.HandPoses[isRightHand].ThePose.Position.x;
            position.y = ss.HandPoses[isRightHand].ThePose.Position.y;
            position.z = ss.HandPoses[isRightHand].ThePose.Position.z;
        }
        else {
            position.x = ss.HandPoses[isRightHand].ThePose.Position.x + hand_voffset.x + hand_offset2.x;
            position.y = ss.HandPoses[isRightHand].ThePose.Position.y + hand_voffset.y + hand_offset2.y;
            position.z = ss.HandPoses[isRightHand].ThePose.Position.z + hand_voffset.z + hand_offset2.z;
        }
        //position = rotateVector2(position, overall_rotation);
        pose.vecPosition[0] = position.x;// +overall_offset.x;
        pose.vecPosition[1] = position.y;// +overall_offset.y;
        pose.vecPosition[2] = position.z;// +overall_offset.z;



        ovrVector3f linAcc = (ss.HandPoses[isRightHand].LinearAcceleration);
        ovrVector3f linVel = (ss.HandPoses[isRightHand].LinearVelocity);
  //      ovrQuatf hand_nqoffset = { 0.3420201, 0, 0, -0.9396926 };
        /*linAcc = rotateVector2(linAcc, hand_qoffset);
        linVel = rotateVector2(linVel, hand_nqoffset);*/    //do not do this


        pose.vecAcceleration[0] = linAcc.x;
        pose.vecAcceleration[1] = linAcc.y;
        pose.vecAcceleration[2] = linAcc.z;

        pose.qWorldFromDriverRotation = HmdQuaternion_Init(hand_qoffset.w, hand_qoffset.x, hand_qoffset.y, hand_qoffset.z);
        pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

        pose.vecVelocity[0] = linVel.x;
        pose.vecVelocity[1] = linVel.y;
        pose.vecVelocity[2] = linVel.z;


        pose.poseTimeOffset = 0;  // let's let Oculus do it

        pose.vecAngularAcceleration[0] = ss.HandPoses[isRightHand].AngularAcceleration.x;
        pose.vecAngularAcceleration[1] = ss.HandPoses[isRightHand].AngularAcceleration.y;
        pose.vecAngularAcceleration[2] = ss.HandPoses[isRightHand].AngularAcceleration.z;

        pose.vecAngularVelocity[0] = ss.HandPoses[isRightHand].AngularVelocity.x;
        pose.vecAngularVelocity[1] = ss.HandPoses[isRightHand].AngularVelocity.y;
        pose.vecAngularVelocity[2] = ss.HandPoses[isRightHand].AngularVelocity.z;

        if (comm_buffer->config.do_world_transformation) {
            pose.qWorldFromDriverRotation = comm_buffer->config.world_orientation_q * pose.qWorldFromDriverRotation;

            vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(comm_buffer->config.world_orientation_q, pose.vecWorldFromDriverTranslation);
            pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + comm_buffer->config.world_translation[0];
            pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + comm_buffer->config.world_translation[1];
            pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + comm_buffer->config.world_translation[2];
                
        }

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

    vr::HmdQuaternionf_t scaler_quat_mult(vr::HmdQuaternionf_t q, float s) {
        vr::HmdQuaternionf_t qr{ q.w * s, q.x * s, q.y * s, q.z * s };
        return qr;
    }
    vr::HmdQuaternionf_t add_quat(vr::HmdQuaternionf_t q1, vr::HmdQuaternionf_t q2) {
        vr::HmdQuaternionf_t qr{ q1.w + q2.w, q1.x + q2.x, q1.y + q2.y, q1.z + q2.z };
        return qr;
    }

    HmdQuaternionf_t Nlerp(HmdQuaternionf_t qa, HmdQuaternionf_t qb, float t2) {
        HmdQuaternionf_t qm;
        float t1 = 1.0f - t2;
        qm = add_quat(scaler_quat_mult(qa, t1), scaler_quat_mult(qb, t2));
        float len = sqrtf(qm.x * qm.x + qm.y * qm.y + qm.z * qm.z + qm.w * qm.w);

        return scaler_quat_mult(qm, 1.0 / len);
    }

    HmdVector4_t v4_scalar_mult(HmdVector4_t v, float s) {
        HmdVector4_t result{ v.v[0] * s, v.v[1] * s, v.v[2] * s, v.v[3] * s };
        return result;
    }

    HmdVector4_t v4_add(HmdVector4_t v1, HmdVector4_t v2) {
        HmdVector4_t r{ v1.v[0] + v2.v[0], v1.v[1] + v2.v[1], v1.v[2] + v2.v[2], v1.v[3] + v2.v[3] };
        return r;
    }

    VRBoneTransform_t blend_bones(VRBoneTransform_t b1, VRBoneTransform_t b2, float f) {
        VRBoneTransform_t br;
        br.position = v4_add(v4_scalar_mult(b1.position, (1.0 - f)), v4_scalar_mult(b2.position, f));
        br.orientation = Nlerp(b1.orientation, b2.orientation, f);
        return br;
    }

    void RunFrame()
    {
        //#if defined( _WINDOWS )
                // Your driver would read whatever hardware state is associated with its input components and pass that
                // in to UpdateBooleanComponent. This could happen in RunFrame or on a thread of your own that's reading USB
                // state. There's no need to update input state unless it changes, but it doesn't do any harm to do so.
        if (!comm_buffer->config.be_objects) {
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
                        float hand_blend_fraction = inputState.HandTrigger[isRightHand];
                        float finger_bend_fraction = inputState.IndexTrigger[isRightHand];
                        if (!(inputState.Touches & ovrTouch_RThumbUp)) {
                            if (hand_blend_fraction < 0.1) hand_blend_fraction = 0.1;
                            if (!(inputState.Touches & ovrTouch_RIndexPointing)) {
                                if (finger_bend_fraction < 0.1) finger_bend_fraction = 0.1;
                            }
                        }

                        for (int i = 0; i < HSB_Count; i++) active_hand_pose[i] = blend_bones(right_open_hand_pose[i], right_fist_pose[i], hand_blend_fraction);

                        for (int i = HSB_IndexFinger0; i <= HSB_IndexFinger4; i++) active_hand_pose[i] = blend_bones(right_open_hand_pose[i], right_fist_pose[i], finger_bend_fraction);

                        if (finger_bend_fraction > hand_blend_fraction)
                            for (int i = HSB_Thumb0; i <= HSB_Thumb3; i++) active_hand_pose[i] = blend_bones(right_open_hand_pose[i], right_fist_pose[i], finger_bend_fraction);

                        if (inputState.Touches & ovrTouch_RThumbUp)
                        {
                            for (int i = HSB_Thumb0; i <= HSB_Thumb3; i++) active_hand_pose[i] = right_open_hand_pose[i];
                        }
                        vr::VRDriverInput()->UpdateSkeletonComponent(
                            m_compSkel,
                            vr::VRSkeletalMotionRange_WithoutController,
                            active_hand_pose,
                            NUM_BONES);
                        vr::VRDriverInput()->UpdateSkeletonComponent(
                            m_compSkel,
                            vr::VRSkeletalMotionRange_WithController,
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

                        float hand_blend_fraction = inputState.HandTrigger[isRightHand];
                        float finger_bend_fraction = inputState.IndexTrigger[isRightHand];
                        if (!(inputState.Touches & ovrTouch_LThumbUp)) {
                            if (hand_blend_fraction < 0.1) hand_blend_fraction = 0.1;
                            if (!(inputState.Touches & ovrTouch_LIndexPointing)) {
                                if (finger_bend_fraction < 0.1) finger_bend_fraction = 0.1;
                            }
                        }

                        for (int i = 0; i < HSB_Count; i++) active_hand_pose[i] = blend_bones(left_open_hand_pose[i], left_fist_pose[i], hand_blend_fraction);

                        for (int i = HSB_IndexFinger0; i <= HSB_IndexFinger4; i++) active_hand_pose[i] = blend_bones(left_open_hand_pose[i], left_fist_pose[i], finger_bend_fraction);

                        if (finger_bend_fraction > hand_blend_fraction)
                            for (int i = HSB_Thumb0; i <= HSB_Thumb3; i++) active_hand_pose[i] = blend_bones(left_open_hand_pose[i], left_fist_pose[i], finger_bend_fraction);

                        if (inputState.Touches & ovrTouch_LThumbUp)
                        {
                            for (int i = HSB_Thumb0; i <= HSB_Thumb3; i++) active_hand_pose[i] = left_open_hand_pose[i];
                        }
                        vr::VRDriverInput()->UpdateSkeletonComponent(
                            m_compSkel,
                            vr::VRSkeletalMotionRange_WithoutController,
                            active_hand_pose,
                            NUM_BONES);
                        vr::VRDriverInput()->UpdateSkeletonComponent(
                            m_compSkel,
                            vr::VRSkeletalMotionRange_WithController,
                            active_hand_pose,
                            NUM_BONES);
#endif
                    }

                    vr::VRDriverInput()->UpdateScalarComponent(m_compTrigv, inputState.IndexTrigger[isRightHand], 0);
                    vr::VRDriverInput()->UpdateBooleanComponent(m_compGripv, inputState.HandTrigger[isRightHand] > 0.5, 0);
                    vr::VRDriverInput()->UpdateScalarComponent(m_compGripv, inputState.HandTrigger[isRightHand], 0);
                    vr::VRDriverInput()->UpdateScalarComponent(m_compJoyx, inputState.Thumbstick[isRightHand].x, 0);
                    vr::VRDriverInput()->UpdateScalarComponent(m_compJoyy, inputState.Thumbstick[isRightHand].y, 0);

#if USE_MUTEX
                    ReleaseMutex(comm_mutex);
#endif
                }
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

    void ProcessEvent(const vr::VREvent_t & vrEvent)
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
