// ovr_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//


#include "definitions.h"
#include <fstream>
#include <mutex>
#include <atomic>

//#define MAX_HAPTICS

std::mutex vib_mtx;

const bool haptics_direct = false;
ovrSession mSession;
uint32_t future_vib_buffer_index[2] = { 0 };
double vib_buf_time[2] = { 0 };
//uint8_t future_vib_buffer[1024] = { 0 };
uint8_t future_vib_buffer[2][1024] = { {0},{0} };
bool future_vib_buffer_used[2][1024] = { {0},{0} };
uint8_t future_vib_buffer_freq[2][1024] = { {0},{0} };
uint8_t future_vib_buffer_freq_sample[2][1024] = { {0},{0} };


void vibration_thread(ovrSession mSession);
void add_vibration(bool isRightHand, float amplitude, float frequency, float duration);
void main_loop(ovrSession mSession, HANDLE comm_mutex, shared_buffer* comm_buffer, uint64_t frame_count, ovrHapticsBuffer& vibuffer, uint8_t* buf, unsigned int sizeof_buf) {

    ovrTrackingState ss = ovr_GetTrackingState(mSession, 0, false);

//    WaitForSingleObject(comm_mutex, INFINITE);
    if (comm_buffer->logging_offset) {
        for (int i = 0;  ((i < 1024) && (i < comm_buffer->logging_offset)); i++) {
            putc(comm_buffer->logging_buffer[i], stdout);
        }
        comm_buffer->logging_offset = 0;
    }
    if (comm_buffer->vrEvent_type) {
        //std::cout << "VR Event 0x" << comm_buffer->vrEvent_type << std::endl;
        comm_buffer->vrEvent_type = 0;
    }

    if (comm_buffer->config.external_tracking) {
        comm_buffer->tracking_state = ovr_GetTrackingState(mSession, (ovr_GetTimeInSeconds() + (comm_buffer->config.extra_prediction_ms * 0.001)), ovrTrue);
    }

    for (int i = 0; i < comm_buffer->config.num_objects; i++) {
        ovrTrackedDeviceType deviceType = (ovrTrackedDeviceType)(ovrTrackedDevice_Object0 + i);
        ovrPoseStatef ovr_pose;

        ovr_GetDevicePoses(mSession, &deviceType, 1, (ovr_GetTimeInSeconds() + (comm_buffer->config.extra_prediction_ms * 0.001)), &ovr_pose);
        if ((ovr_pose.ThePose.Orientation.x != 0) || (ovr_pose.ThePose.Orientation.y != 0) || (ovr_pose.ThePose.Orientation.z != 0)){
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

    for (int i = 0; i < comm_buffer->num_sensors; i++) comm_buffer->sensor_poses[i] = ovr_GetTrackerPose(mSession, i);

    ovrResult input_res[2];
    ovrInputState inputState[2];
    for (int i = 0; i < 2; i++) {

        if (i == 1) {
            input_res[i] = ovr_GetInputState(mSession, ovrControllerType::ovrControllerType_RTouch, &inputState[i]);
            comm_buffer->input_state.Buttons &= ~ovrButton_RMask;
            comm_buffer->input_state.Buttons |= (ovrButton_RMask & inputState[i].Buttons);
            comm_buffer->input_state.Touches &= ~(ovrTouch_RButtonMask | ovrTouch_RPoseMask);
            comm_buffer->input_state.Touches |= ((ovrTouch_RButtonMask | ovrTouch_RPoseMask) & inputState[i].Touches);
        }
        else {
            input_res[i] = ovr_GetInputState(mSession, ovrControllerType::ovrControllerType_LTouch, &inputState[i]);
            comm_buffer->input_state.Buttons &= ~ovrButton_LMask;
            comm_buffer->input_state.Buttons |= (ovrButton_LMask & inputState[i].Buttons);
            comm_buffer->input_state.Touches &= ~(ovrTouch_LButtonMask | ovrTouch_LPoseMask);
            comm_buffer->input_state.Touches |= ((ovrTouch_LButtonMask | ovrTouch_LPoseMask) & inputState[i].Touches);
        }
        comm_buffer->input_state.HandTrigger[i] = inputState[i].HandTrigger[i];
        comm_buffer->input_state.IndexTrigger[i] = inputState[i].IndexTrigger[i];
        comm_buffer->input_state.Thumbstick[i] = inputState[i].Thumbstick[i];
    }
//    ReleaseMutex(comm_mutex);

    for( int i =0; i<2; i++){
    

        if ((frame_count & 0x7FF) == 0) {
            std::cout.precision(4);

            std::cout << (i == 0 ? "lhand" : "rhand") << " 0x" << std::fixed <<
                std::hex << ss.HandStatusFlags[i] << std::dec << " x " <<
                ss.HandPoses[i].ThePose.Position.x << " y " <<
                ss.HandPoses[i].ThePose.Position.y << " z " <<
                ss.HandPoses[i].ThePose.Position.z << std::hex << " button 0x" << inputState[i].Buttons << " touch 0x" << inputState[i].Touches << " g|t " << comm_buffer->input_state.HandTrigger[i] << "|" << comm_buffer->input_state.IndexTrigger[i] << " res 0x" << input_res[i];
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


void no_graphics_start(shared_buffer* comm_buffer, HANDLE comm_mutex) {
    mSession = nullptr;
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
        if (OVR_FAILURE(ovr_Initialize(&initParams)/*ovr_Initialize(nullptr)*/)) {
            std::cout << "ovr_Initialize error" << std::endl;
            return;
        }

        if (OVR_FAILURE(ovr_Create(&mSession, &luid))) {
            std::cout << "ovr_Create error" << std::endl;
            return;
        }

        if (OVR_FAILURE(ovr_SetTrackingOriginType(mSession, ovrTrackingOrigin_FloorLevel))) {
            std::cout << "ovr_SetTrackingOriginType error" << std::endl;
            return;
        }
    }


    comm_buffer->config.num_objects = (ovr_GetConnectedControllerTypes(mSession) >> 8) & 0xf;
    comm_buffer->num_sensors = ovr_GetTrackerCount(mSession);

    std::thread vib_thread;
    if(!haptics_direct)
        vib_thread = std::thread(vibration_thread, mSession);
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


shared_buffer* comm_buffer;


GUI_Manager* p_gui_manager = nullptr;

void reset_config_settings(config_data& config) {
    config.vr_universe = 31;
    config.be_objects = false;
    config.extra_prediction_ms = 5.0f;
    strncpy_s(comm_buffer->config.manufacturer_name, "Oculus_link", 127);
    strncpy_s(comm_buffer->config.tracking_space_name, "oculus_link", 127);
    comm_buffer->config.num_objects = (ovr_GetConnectedControllerTypes(mSession) >> 8) & 0xf;
    comm_buffer->num_sensors = ovr_GetTrackerCount(mSession);
    config.external_tracking = true;
    config.track_hmd = false;
    config.show_sensors_steam = true;
    config.disable_controllers = false;
    config.disable_left_controller = false;
    config.disable_right_controller = false;
    config.min_amplitude = 0;
    config.amplitude_scale = 1.0;
    config.sqrt_pre_filter = false;
    config.sqrt_post_filter = true;
    config.do_rendering = false;
    config.do_world_transformation = false;
    config.world_translation[0] = 0.0;
    config.world_translation[1] = 0.0;
    config.world_translation[2] = 0.0;
    config.world_orientation_q.w = 1.0;
    config.world_orientation_q.x = 0.0;
    config.world_orientation_q.y = 0.0;
    config.world_orientation_q.z = 0.0;
    config.world_orientation_euler[0] = 0.0;
    config.world_orientation_euler[1] = 0.0;
    config.world_orientation_euler[2] = 0.0;
    config.skeleton_smoothing = 0.2;
}

void save_config_to_file(config_data& config) {
    char exe_filename[512];
    auto str_len = GetModuleFileNameA( NULL, (LPSTR) exe_filename, 512);
    std::cout << "exe_filename = " << exe_filename << std::endl;
    while (str_len && (exe_filename[str_len] != '\\')) str_len--;
    exe_filename[str_len] = '\0';
    std::cout << "exe_filepath = " << exe_filename << std::endl;
    char config_filename[] = "\\config.dat";
    for (size_t i = 0; i < sizeof(config_filename); i++) {
        exe_filename[str_len] = config_filename[i];
        str_len++;
    }
    exe_filename[str_len] = '\0';
    std::cout << "config filename = " << exe_filename << std::endl;

    std::ofstream ofs;
    ofs.open(exe_filename);

    ofs << config.vr_universe << std::endl;
    ofs << config.be_objects << std::endl;
    ofs << config.extra_prediction_ms << std::endl;
    ofs << config.tracking_space_name << std::endl;
    ofs << config.manufacturer_name << std::endl;
    ofs << config.num_objects << std::endl;
    ofs << config.external_tracking << std::endl;
    ofs << config.track_hmd << std::endl;
    ofs << config.show_sensors_steam << std::endl;
    ofs << (unsigned)config.min_amplitude << std::endl;
    ofs << config.amplitude_scale << std::endl;
    ofs << config.sqrt_pre_filter << std::endl;
    ofs << config.sqrt_post_filter << std::endl;
    ofs << config.do_rendering << std::endl;
    ofs << config.do_world_transformation << std::endl;
    ofs << config.world_translation[0] << std::endl;
    ofs << config.world_translation[1] << std::endl;
    ofs << config.world_translation[2] << std::endl;
    ofs << config.world_orientation_q.w << std::endl;
    ofs << config.world_orientation_q.x << std::endl;
    ofs << config.world_orientation_q.y << std::endl;
    ofs << config.world_orientation_q.z << std::endl;
    ofs << config.world_orientation_euler[0] << std::endl;
    ofs << config.world_orientation_euler[1] << std::endl;
    ofs << config.world_orientation_euler[2] << std::endl;
    ofs << config.disable_controllers << std::endl;
    ofs << config.skeleton_smoothing << std::endl;
    ofs << config.disable_left_controller << std::endl;
    ofs << config.disable_right_controller << std::endl;
    ofs.close();
}


void load_config_from_file(config_data& config) {
    char exe_filename[512];
    auto str_len = GetModuleFileNameA(NULL, (LPSTR)exe_filename, 512);
    std::cout << "exe_filename = " << exe_filename << std::endl;
    while (str_len && (exe_filename[str_len] != '\\')) str_len--;
    exe_filename[str_len] = '\0';
    std::cout << "exe_filepath = " << exe_filename << std::endl;
    char config_filename[] = "\\config.dat";
    for (size_t i = 0; i < sizeof(config_filename); i++) {
        exe_filename[str_len] = config_filename[i];
        str_len++;
    }
    exe_filename[str_len] = '\0';
    std::cout << "config filename = " << exe_filename << std::endl;

    std::ifstream ifs;
    ifs.open(exe_filename);
    if (ifs.is_open()) {
        ifs >> config.vr_universe;
        ifs >> config.be_objects;
        ifs >> config.extra_prediction_ms;
        ifs >> config.tracking_space_name;
        ifs >> config.manufacturer_name;
        ifs >> config.num_objects;
        ifs >> config.external_tracking;
        ifs >> config.track_hmd;
        ifs >> config.show_sensors_steam;
        unsigned min_amp;
        ifs >> min_amp;
        config.min_amplitude = min_amp;
        
        ifs >> config.amplitude_scale;
        ifs >> config.sqrt_pre_filter;
        ifs >> config.sqrt_post_filter;
        ifs >> config.do_rendering;
        ifs >> config.do_world_transformation;
        ifs >> config.world_translation[0];
        ifs >> config.world_translation[1];
        ifs >> config.world_translation[2];
        ifs >> config.world_orientation_q.w;
        ifs >> config.world_orientation_q.x;
        ifs >> config.world_orientation_q.y;
        ifs >> config.world_orientation_q.z;
        ifs >> config.world_orientation_euler[0];
        ifs >> config.world_orientation_euler[1];
        ifs >> config.world_orientation_euler[2];

        ifs >> config.disable_controllers;
        ifs >> config.skeleton_smoothing;

        ifs >> config.disable_left_controller;
        ifs >> config.disable_right_controller;
        ifs.close();
    }
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
    std::cout << "Minumim haptic amplitude 0-255 (64)" << std::endl;
    std::cout << "haptic scale multiplier 0-inf (1.0)" << std::endl;
    std::cout << "Show Oculus Vr sensors in steam?  y/n" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "This program is super dumb and expects all of the arguments or none (for defaults), suggested invocations:" << std::endl;
    std::cout << "ovr_test.exe n 1 Oculus oculus n 10 n n 64 1.0(must be use with ovr_dummy.exe)" << std::endl;
    std::cout << "ovr_test.exe y 1 Oculus oculus y 10 n n 64 1.0" << std::endl;
    std::cout << "ovr_test.exe y 31 Oculus_link oculus_link n 10 n y n 64 1.0" << std::endl;
    std::cout << "ovr_test.exe n 31 Oculus_link oculus_link n 10 n n y 64 1.0(default)" << std::endl;


    HANDLE hMapFile;
 
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
    comm_buffer->vib_buffers[0].reset();
    comm_buffer->vib_buffers[1].reset();
    reset_config_settings(comm_buffer->config);
    if (argc != 12) {
        std::cout << " <12 arguments, using defaults: n 31 Oculus_link oculus_link n 5 n n y n" << std::endl;
        load_config_from_file(comm_buffer->config);
    } else {
        comm_buffer->config.do_rendering = (std::string(argsv[1]) == "y");
        comm_buffer->config.vr_universe = atoi(argsv[2]);
        strncpy_s(comm_buffer->config.manufacturer_name, argsv[3], 127);
        strncpy_s(comm_buffer->config.tracking_space_name, argsv[4], 127);
        comm_buffer->config.extra_prediction_ms = atof(argsv[6]);
        comm_buffer->config.be_objects = (std::string(argsv[7]) == "y");
        comm_buffer->config.external_tracking = (std::string(argsv[8]) == "y");
        comm_buffer->config.track_hmd = (std::string(argsv[9]) == "y");
        comm_buffer->config.min_amplitude = strtoul(argsv[10],0, 10);
        comm_buffer->config.amplitude_scale = strtof(argsv[11],0);
        comm_buffer->config.show_sensors_steam = (std::string(argsv[12]) == "y");
    }

    HANDLE comm_mutex = CreateMutex(0, true, L"Local\\oculus_steamvr_touch_controller_mutex");
    //MessageBox(NULL, pBuf, TEXT("Process2"), MB_OK);
//    ReleaseMutex(comm_mutex);
//    WaitForSingleObject(
//        comm_mutex,    // handle to mutex
//        INFINITE);  // no time-out interval
//    ReleaseMutex(comm_mutex);


       
    std::thread gui_thread([&]() {
        p_gui_manager = new GUI_Manager(comm_buffer);
        p_gui_manager->handle_loop();
        });
   
#if 1
    if (comm_buffer->config.do_rendering) {
        GuardianSystemDemo* instance = new (_aligned_malloc(sizeof(GuardianSystemDemo), 16)) GuardianSystemDemo();
        instance->Start(0, comm_buffer, comm_mutex);
        delete instance;

    } else {
        no_graphics_start(comm_buffer, comm_mutex);
    }
#endif
    gui_thread.join();

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
    {255,0,0},
    //{196,255,128,0},
    {255,0,0,0},
    {255,0,0,0,0},
    //{196,255,196,128,0,0,0},
    {255,0,0,0,0,0},
    {255,0,0,0,0,0,0},
    //{196,255,255,128,64,0,0,0},
    {255,0,0,0,0,0,0,0},
    {255,0,0,0,0,0,0,0,0},
    {255,0,0,0,0,0,0,0,0,0},
    //{196,255,255,196,128,0,0,0,0,0},
    {255,0,0,0,0,0,0,0,0,0,0},
    {255,0,0,0,0,0,0,0,0,0,0,0},
    {255,0,0,0,0,0,0,0,0,0,0,0,0},
    {255,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {255,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {255,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    //{64,128,196,255,255,128,64,32,0,0,0,0,0,0,0,0}
};


uint8_t clamp_scale(uint8_t sample, float amplitude) {
    if (comm_buffer->config.sqrt_pre_filter) amplitude = sqrtf(amplitude);
    float scale = amplitude * (comm_buffer->config.amplitude_scale);
    if (comm_buffer->config.sqrt_post_filter) scale = sqrtf(scale);
    uint64_t output = static_cast<uint64_t> ( static_cast<float>(sample) * scale );
    if (output > 255) output = 255;
   // if (output < comm_buffer->config.min_amplitude) output = comm_buffer->config.min_amplitude;
    return static_cast<uint8_t>(output & 0xFF);
}


std::atomic<unsigned int> vib_interval[2] = { 0 };
std::atomic<float> vib_amplitude[2] = { 0 };
std::atomic<double> sample_end_time[2] = { 0 };
void add_vibration(bool isRightHand, float frequency, float amplitude, float duration) {
    double now = ovr_GetTimeInSeconds();
    ovrTouchHapticsDesc desc = ovr_GetTouchHapticsDesc(mSession, ovrControllerType_LTouch);
   

}

void vibration_thread(ovrSession mSession) {
    unsigned char buf[32];
    ovrHapticsBuffer vibuffer;
    vibuffer.SamplesCount = 8;
    vibuffer.Samples = buf;
    vibuffer.SubmitMode = ovrHapticsBufferSubmit_Enqueue;
    uint64_t counter = 0;
    unsigned int last_vib_interval[2] = { 0 };
    unsigned int vib_interval_counter[2] = { 0 };
    ovrTouchHapticsDesc desc = ovr_GetTouchHapticsDesc(mSession, ovrControllerType_LTouch);
    while (1) {

       
        //Sleep((1000 * desc.SubmitOptimalSamples) / desc.SampleRateHz);
        //Sleep(1000/40);
        for (int hand = 0; hand < 2; hand++) {
            vib_sample s;

   

            double now = ovr_GetTimeInSeconds();
            while (comm_buffer->vib_buffers[hand].pop(s)) {
               // if(s.amplitude)
               //     printf("vib[%u] dur %f=%f | freq %f | amp %f | time %lums  = %lf\n", hand, s.duration, s.duration * desc.SampleRateHz, s.frequency, s.amplitude, (unsigned long)(now * 1000), now);
               // add_vibration(hand == 1, s.frequency, s.amplitude, s.duration);

                if (s.duration <= (8.0 / desc.SampleRateHz)) s.duration = (8.0 / desc.SampleRateHz);
                sample_end_time[hand] = now + s.duration;
                if (s.amplitude < 0) s.amplitude = -s.amplitude;
                if (s.frequency < 0) s.frequency = -s.frequency;
                vib_amplitude[hand] = s.amplitude;
                vib_interval[hand] = lround(desc.SampleRateHz / s.frequency);
                vib_interval_counter[hand] = 0;
                //if (s.amplitude)
                //    printf("vib[%u] dur %f=%f | freq %f | interval %u | amp %f | time %lums  = %lf\n", hand, s.duration, s.duration * desc.SampleRateHz, s.frequency, vib_interval[hand].load(), s.amplitude, (unsigned long)(now * 1000), now);

            }
            double time_left =  sample_end_time[hand] - now;
            if (time_left<=0) {
                vib_interval_counter[hand] = 0;
                continue;
            }
            //std::cout << "sample has " << time_left << "s remaining   now = " << now << "sample end = " << sample_end_time[hand] << std::endl;
            //std::cout << (hand == 0 ? "Left" : "Right") << std::endl;
            //std::lock_guard<std::mutex> lk(vib_mtx);
            if (last_vib_interval[hand] != vib_interval[hand]) {
                last_vib_interval[hand] = vib_interval[hand];
                vib_interval_counter[hand] = 0;
            }
            
            //ovrTouchHapticsDesc desc = ovr_GetTouchHapticsDesc(mSession, (hand==0)?ovrControllerType_LTouch: ovrControllerType_RTouch);
            //std::cout << "SampleRate " << desc.SampleRateHz << " SampleSize " << desc.SampleSizeInBytes << " MaxSamples " << desc.SubmitMaxSamples << " MinSamples " << desc.SubmitMinSamples << " OptimalSamples " << desc.SubmitOptimalSamples << std::endl;
            ovrHapticsPlaybackState pbState;
            ovr_GetControllerVibrationState(mSession, (hand == 0) ? ovrControllerType_LTouch : ovrControllerType_RTouch, &pbState);
            //std::cout << (counter & 0xff) << " - queue state before space: " << pbState.RemainingQueueSpace << " samples: " << pbState.SamplesQueued << " / min desired samples: " << desc.QueueMinSizeToAvoidStarvation << std::endl;

            uint32_t need_count = 0;
            /*desc.SubmitOptimalSamples*/
            if(pbState.SamplesQueued < 16 ) need_count =  16 - pbState.SamplesQueued;
            uint64_t samples_remaining = lround(time_left * desc.SampleRateHz);
            //std::cout << "want " << need_count << " remaining " << samples_remaining << std::endl;
            if (need_count > samples_remaining) {
                need_count = samples_remaining;
            }
            if (need_count) {
                //if (vib_amplitude[hand])std::cout << (counter & 0xff) << " - queue state before space: " << pbState.RemainingQueueSpace << " samples: " << pbState.SamplesQueued << " / min desired samples: " << desc.QueueMinSizeToAvoidStarvation << std::endl;
                //std::cout << "need " << need_count << std::endl;
                for (unsigned int i = 0; i < need_count; i++) {
                    unsigned int amp = clamp_scale(255, vib_amplitude[hand]);
                    if (vib_interval_counter[hand] == 0) {
                        //if (amp > 255) amp = 255;
                        buf[i] = amp;
                        if (last_vib_interval[hand] > 0) {
                            vib_interval_counter[hand] = last_vib_interval[hand] - 1;
                        }
                    }
                    else {
                        vib_interval_counter[hand]--;
                        buf[i] = 0;// ((last_vib_interval[hand] - vib_interval_counter[hand]) * amp) / last_vib_interval[hand];
                    }
                    //if(vib_amplitude[hand])
                    //    std::cout << " " << (unsigned int)buf[i];
                }
                //if(vib_amplitude[hand])std::cout << std::endl;
                vibuffer.Samples = buf;
                vibuffer.SamplesCount = need_count;
                vibuffer.SubmitMode = ovrHapticsBufferSubmit_Enqueue;
                ovr_SubmitControllerVibration(mSession, (hand == 0) ? ovrControllerType_LTouch : ovrControllerType_RTouch, &vibuffer);
                ovr_GetControllerVibrationState(mSession, (hand == 0) ? ovrControllerType_LTouch : ovrControllerType_RTouch, &pbState);
            }
            // std::cout << (counter & 0xff) << " - queue state after space: " << pbState.RemainingQueueSpace << " samples: " << pbState.SamplesQueued << " / min desired samples: " << desc.QueueMinSizeToAvoidStarvation << std::endl;

        }

        
        counter++;

    }
}