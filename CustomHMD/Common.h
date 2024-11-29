#pragma once

#include <openvr_driver.h>
#include "driverlog.h"
//#define DRAW_FRAME 1
//#define DRAW_FRAME 1

//#define USE_MUTEX 1
#define ADD_HMD 1
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
#include <atomic>


#if defined( _WINDOWS )
#include <windows.h>
#endif

using namespace vr;
#if USE_MUTEX
HANDLE comm_mutex = NULL;
#endif


struct config_data {
    uint32_t vr_universe;
    bool be_objects;
    float extra_prediction_ms;
    char tracking_space_name[128]; //oculus or Oculus
    char manufacturer_name[128]; //Oculus or Oculus_link
    uint32_t num_objects;
    bool external_tracking;
    bool track_hmd;
    bool disable_controllers;
    uint8_t min_amplitude;
    float amplitude_scale;
    bool sqrt_pre_filter;
    bool sqrt_post_filter;
    bool do_rendering;
    bool do_world_transformation;
    double world_translation[3];
    vr::HmdQuaternion_t world_orientation_q;
    double world_orientation_euler[3];
    double skeleton_smoothing;
    bool disable_left_controller;
    bool disable_right_controller;
};

struct vib_sample {
    float amplitude;
    float freqency;
    float duration;
    double timestamp;
};


struct vib_sample_buffer {
    vib_sample buf[128];
    std::atomic<unsigned int> head;
    std::atomic<unsigned int> tail;
    void reset() {
        head = tail = 0;
    }
    bool empty() {
        return head == tail;
    }
    bool full() {
        return (((128 + tail) - head) % 128) == 1;
    }
    void push(vib_sample sample) {
        if (!full()) {
            buf[head] = sample;
            int new_head = (head + 1) % 128;
            head = new_head;
        }
    }
    bool pop(vib_sample& sample) {
        if (empty()) return false;
        sample = buf[tail];
        unsigned int new_tail = (tail + 1) % 128;
        tail = new_tail;
        return true;
    }
};

struct shared_buffer {
    config_data config;
    ovrInputState input_state;
    ovrTrackingState tracking_state;
    ovrPoseStatef object_poses[4];
    uint32_t vrEvent_type;
    vib_sample_buffer vib_buffers[2];
    uint64_t logging_offset;
    char logging_buffer[1024];
};


shared_buffer* comm_buffer = 0;

void log_to_buffer(std::string s) {
    if (!comm_buffer) return;
    //WaitForSingleObject(comm_mutex, INFINITE);
    for (int i = 0; i < s.size(); i++) {
        if (comm_buffer->logging_offset < 1023) {
            comm_buffer->logging_buffer[comm_buffer->logging_offset] = s.c_str()[i];
            comm_buffer->logging_offset++;
        }
    }
    comm_buffer->logging_buffer[comm_buffer->logging_offset] = '\n';
    comm_buffer->logging_offset ++;
    //ReleaseMutex(comm_mutex);
    return;
}

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



ovrQuatf ovrQuatfmul(ovrQuatf q1, ovrQuatf q2) {
    ovrQuatf result = { 0 };
    result.x = q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
    result.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
    result.z = q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
    result.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
    return result;
}
ovrVector3f rotateVector(const ovrVector3f _V, ovrQuatf q) {
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

inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t& lhs, const vr::HmdQuaternion_t& rhs) {
    return {
        (lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
        (lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
        (lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
        (lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)
    };
}

inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const double(&vector)[3]) {
    vr::HmdQuaternion_t vectorQuat = { 0.0, vector[0], vector[1] , vector[2] };
    vr::HmdQuaternion_t conjugate = { quat.w, -quat.x, -quat.y, -quat.z };
    auto rotatedVectorQuat = quat * vectorQuat * conjugate;
    return { rotatedVectorQuat.x, rotatedVectorQuat.y, rotatedVectorQuat.z };
}

ovrVector3f crossProduct(const ovrVector3f v, ovrVector3f p) 
{
    return ovrVector3f{ v.y * p.z - v.z * p.y, v.z * p.x - v.x * p.z, v.x * p.y - v.y * p.x };
}

ovrVector3f rotateVector2(ovrVector3f v, ovrQuatf q)
{
    // nVidia SDK implementation

    ovrVector3f uv, uuv;
    ovrVector3f qvec{ q.x, q.y, q.z };
    uv = crossProduct(qvec, v);
    uuv = crossProduct(qvec, uv);
    uv.x *= (2.0f * q.w);
    uv.y *= (2.0f * q.w);
    uv.z *= (2.0f * q.w);
    uuv.x *= 2.0f;
    uuv.y *= 2.0f;
    uuv.z *= 2.0f;

    return ovrVector3f{ v.x + uv.x + uuv.x, v.y + uv.y + uuv.y, v.z + uv.z + uuv.z };
}