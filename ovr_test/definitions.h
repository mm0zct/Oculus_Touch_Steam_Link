#pragma once


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

namespace vr {
    struct HmdQuaternion_t
    {
        double w, x, y, z;
    };
}
struct config_data {
    uint32_t vr_universe;
    bool be_objects;
    float extra_prediction_ms;
    char tracking_space_name[128]; //oculus or Oculus
    char manufacturer_name[128]; //Oculus or Oculus_link
    uint32_t num_objects;
    bool external_tracking;
    bool track_hmd;
    uint8_t min_amplitude;
    float amplitude_scale;
    bool sqrt_pre_filter;
    bool sqrt_post_filter;
    bool do_rendering;
    bool do_world_transformation;
    double world_translation[3];
    vr::HmdQuaternion_t world_orientation_q;
};

struct shared_buffer {
    config_data config;
    ovrInputState input_state;
    ovrTrackingState tracking_state;
    ovrPoseStatef object_poses[4];
    uint32_t vrEvent_type;
    float vib_amplitude[2];
    float vib_frequency[2];
    float vib_duration_s[2];
    bool vib_valid[2];
    uint64_t logging_offset;
    char logging_buffer[1024];
};

extern shared_buffer* comm_buffer;




class GUI_Manager {

public:
    static GUI_Manager* self;
    GUI_Manager(shared_buffer* comm_buffer);
    void handle_loop();
    LRESULT CALLBACK GUIWndProc(HWND window, UINT msg, WPARAM wp, LPARAM lp);
    private:

    
    
    shared_buffer* comm_buffer;
    HWND window;

};

void main_loop(ovrSession mSession, HANDLE comm_mutex, shared_buffer* comm_buffer, uint64_t frame_count, ovrHapticsBuffer& vibuffer, uint8_t* buf, unsigned int sizeof_buf);

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


