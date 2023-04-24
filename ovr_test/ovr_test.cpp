// ovr_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//


#include "definitions.h"
#include <fstream>

//#define MAX_HAPTICS


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

    if (comm_buffer->config.external_tracking) {
        comm_buffer->tracking_state = ovr_GetTrackingState(mSession, (ovr_GetTimeInSeconds() + (comm_buffer->config.extra_prediction_ms * 0.001)), ovrTrue);
    }

    for (int i = 0; i < comm_buffer->config.num_objects; i++) {
        ovrTrackedDeviceType deviceType = (ovrTrackedDeviceType)(ovrTrackedDevice_Object0 + i);
        ovrPoseStatef ovr_pose;

        ovr_GetDevicePoses(mSession, &deviceType, 1, (ovr_GetTimeInSeconds() + (comm_buffer->config.extra_prediction_ms * 0.001)), &ovr_pose);
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


shared_buffer* comm_buffer;


GUI_Manager* p_gui_manager = nullptr;

void reset_config_settings(config_data& config) {
    config.vr_universe = 31;
    config.be_objects = false;
    config.extra_prediction_ms = 5.0f;
    strncpy_s(comm_buffer->config.manufacturer_name, "Oculus_link", 127);
    strncpy_s(comm_buffer->config.tracking_space_name, "oculus_link", 127);
    comm_buffer->config.num_objects = (ovr_GetConnectedControllerTypes(mSession) >> 8) & 0xf;
    config.external_tracking = false;
    config.track_hmd = false;
    config.min_amplitude = 64;
    config.amplitude_scale = 10.0;
    config.sqrt_pre_filter = false;
    config.sqrt_post_filter = false;
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
    config.ipv4[0] = 127;
    config.ipv4[1] = 0;
    config.ipv4[2] = 0;
    config.ipv4[3] = 1;
    config.net_type = net_local;

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

    for(int i = 0; i<4; i++){
        ofs << (unsigned)config.ipv4[i] << std::endl;
    }

    ofs << (unsigned)config.net_type << std::endl;

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
        if (ifs.good()) ifs >> config.vr_universe;
        if (ifs.good())ifs >> config.be_objects;
        if (ifs.good())ifs >> config.extra_prediction_ms;
        if (ifs.good())ifs >> config.tracking_space_name;
        if (ifs.good()) ifs >> config.manufacturer_name;
        if (ifs.good()) ifs >> config.num_objects;
        if (ifs.good()) ifs >> config.external_tracking;
        if (ifs.good()) ifs >> config.track_hmd;
        unsigned min_amp = 64;
        if (ifs.good()) ifs >> min_amp;
        config.min_amplitude = min_amp;
        
        if (ifs.good()) ifs >> config.amplitude_scale;
        if (ifs.good()) ifs >> config.sqrt_pre_filter;
        if (ifs.good()) ifs >> config.sqrt_post_filter;
        if (ifs.good()) ifs >> config.do_rendering;
        if (ifs.good())ifs >> config.do_world_transformation;
        if (ifs.good()) ifs >> config.world_translation[0];
        if (ifs.good()) ifs >> config.world_translation[1];
        if (ifs.good()) ifs >> config.world_translation[2];
        if (ifs.good()) ifs >> config.world_orientation_q.w;
        if (ifs.good()) ifs >> config.world_orientation_q.x;
        if (ifs.good()) ifs >> config.world_orientation_q.y;
        if (ifs.good()) ifs >> config.world_orientation_q.z;
        if (ifs.good()) ifs >> config.world_orientation_euler[0];
        if (ifs.good()) ifs >> config.world_orientation_euler[1];
        if (ifs.good()) ifs >> config.world_orientation_euler[2];
        if(ifs.good())
        for (int i = 0; i < 4; i++) {
            unsigned ip = 0;
            if (ifs.good()) ifs >> ip;
            config.ipv4[i] = ip;
        }
        unsigned temp = 0;
        if (ifs.good()) ifs >> temp;
        if (temp >= net_max_connection_types) temp = 0;
        config.net_type = (e_connection_type)temp;


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
    reset_config_settings(comm_buffer->config);
    if (argc != 11) {
        std::cout << " <11 arguments, using defaults: n 31 Oculus_link oculus_link n 5 n n y n" << std::endl;
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
    }

    HANDLE comm_mutex = CreateMutex(0, true, L"Local\\oculus_steamvr_touch_controller_mutex");
    //MessageBox(NULL, pBuf, TEXT("Process2"), MB_OK);

    WaitForSingleObject(
        comm_mutex,    // handle to mutex
        INFINITE);  // no time-out interval
    ReleaseMutex(comm_mutex);


       
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


uint8_t clamp_scale(uint8_t sample, float amplitude) {
    if (comm_buffer->config.sqrt_pre_filter) amplitude = sqrtf(amplitude);
    float scale = amplitude * (comm_buffer->config.amplitude_scale);
    if (comm_buffer->config.sqrt_post_filter) scale = sqrtf(scale);
    uint64_t output = static_cast<uint64_t> ( static_cast<float>(sample) * scale );
    if (output > 255) output = 255;
    if (output < comm_buffer->config.min_amplitude) output = comm_buffer->config.min_amplitude;
    return static_cast<uint8_t>(output & 0xFF);
}


void add_vib_sample(bool isRightHand, uint8_t sample, uint32_t offset, uint8_t pulse_pattern_index, uint8_t pulse_pattern_offset) {
    future_vib_buffer[isRightHand][offset % 1024] = sample;
    future_vib_buffer_freq[isRightHand][offset % 1024] = pulse_pattern_index;
    future_vib_buffer_freq_sample[isRightHand][offset % 1024] = pulse_pattern_offset;
}

void add_vib_sample_part1(bool isRightHand, uint8_t pulse_pattern_index, uint32_t sample_count, float amplitude) {

    uint64_t sample_offset = 0;
    uint64_t vib_buffer_sample_delta = ((ovr_GetTimeInSeconds() - vib_buf_time[isRightHand]) * 320);
    uint64_t previous_index = (future_vib_buffer_index[isRightHand] + vib_buffer_sample_delta +(1024-1)) % 1024;
    uint64_t next_index = (future_vib_buffer_index[isRightHand] + vib_buffer_sample_delta) % 1024;
    if (future_vib_buffer_used[isRightHand][previous_index]) {
        if (future_vib_buffer_freq[isRightHand][previous_index] == pulse_pattern_index) {
            sample_offset = future_vib_buffer_freq_sample[isRightHand][previous_index];
        }
    }
    
    
    for (int i = 0; i < sample_count; i++) {
        uint8_t sample_value = pulse_patterns[pulse_pattern_index][(sample_offset + i) % pulse_pattern_index];
        add_vib_sample(isRightHand, clamp_scale(sample_value, amplitude), next_index + i, pulse_pattern_index, (sample_offset + i) % pulse_pattern_index);

    }

}

void add_vibration(bool isRightHand, float amplitude, float frequency, float duration) {

    //std::cout << " adding haptic for amplitude " << amplitude << " frequency " << frequency << " duration " << duration << std::endl;

    if ((amplitude <= 0) || (frequency <= 0) /*|| (duration <= 0)*/) return;

    float amp = amplitude/100.0f;
    float freq = frequency * 320.0f;
    uint32_t requested_duration = duration * 320; // 320 Hz processing rate
    if (requested_duration < 2) requested_duration = 2;
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
  

      std::cout << " adding haptic for amplitude " << amplitude << " -> " << amp << " frequency " << frequency << " -> " << freq << " duration " << duration << " ->" << requested_duration
      <<" pulse_width " << pulse_width << std::endl;
      
      add_vib_sample_part1(isRightHand, min_duration, requested_duration, amp);

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
                future_vib_buffer_freq[hand][future_vib_buffer_index[hand] + i] = 0;
                future_vib_buffer_freq_sample[hand][future_vib_buffer_index[hand] + i] = 0;
                future_vib_buffer_used[hand][future_vib_buffer_index[hand] + i] = false;
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