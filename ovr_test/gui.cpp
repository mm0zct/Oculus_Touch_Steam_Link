#include "definitions.h"
#include <vector>
#include <atomic>
#include <functional>
#include <map>
#include <windowsx.h>
#include <tuple>
#define _USE_MATH_DEFINES
#include <math.h>

#define DEBUG_VIB 1

class unique_window_id {
public:
    unique_window_id() {
        window_id = allocation_counter.fetch_add(1);
    }
    uint32_t get_id() { return window_id; }
private:
    uint32_t window_id;
    static std::atomic<uint32_t> allocation_counter;
};

std::atomic<uint32_t> unique_window_id::allocation_counter = 0;

vr::HmdQuaternion_t euler_to_quat(double yaw, double pitch, double roll) {
    //https://math.stackexchange.com/questions/2975109/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
    vr::HmdQuaternion_t result;
    
    yaw *= M_PI / 180.0;
    pitch *= M_PI / 180.0;
    roll *= M_PI / 180.0;
    result.x = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    result.y = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    result.z = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    result.w = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
        
    return result;
}

std::tuple<double, double, double> quaternion_to_euler(vr::HmdQuaternion_t q){
    //https://math.stackexchange.com/questions/2975109/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
    double t0 = 2.0 * (q.w * q.x + q.y * q.z);
    double t1 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    double roll = atan2(t0, t1);
    double t2 = 2.0 * (q.w * q.y - q.z * q.x);
    if (t2 > 1.0) t2 = 1.0;
    if (t2 < -1.0) t2 = -1.0;
    double pitch = asin(t2);
    double t3 = +2.0 * (q.w * q.z + q.x * q.y);
    double t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw = atan2(t3, t4);
    return { yaw *180.0 / M_PI, pitch * 180.0 / M_PI, roll * 180.0 / M_PI };
 }

class config_window_object;
typedef std::function<void(HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer)> wm_command_prototype;
typedef std::function<void(config_window_object* self, HWND parent, shared_buffer* comm_buffer)> init_prototype;

class config_window_object {

public:
    unique_window_id id;
    HWND wnd = 0;
    HWND parent = 0;
    DWORD style;
    std::wstring name;
    std::wstring window_type;
    config_window_object(LPCWSTR name, LPCWSTR window_type, DWORD style,
        wm_command_prototype wm_command,
        init_prototype init
    ) : name(name), window_type(window_type), style(style), wm_command(wm_command), init(init) {}
    wm_command_prototype wm_command;
    init_prototype init;
};

config_window_object* orient_q_wp = nullptr;
config_window_object* orient_euler_wp = nullptr;
bool internal_angle_update = false;

wm_command_prototype default_wm_command = [](HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer) { return; };
init_prototype default_init = [](config_window_object* self, HWND parent, shared_buffer* comm_buffer) { self->parent = parent; return; };

std::vector<config_window_object> config_windows = { 

   {L"Render In Headset (Applies on restart)",L"BUTTON", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT | BS_CHECKBOX, 
   [](HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer) {
        BOOL checked = IsDlgButtonChecked(window, wp);
        comm_buffer->config.do_rendering = !checked;
        CheckDlgButton(window, wp, checked ? BST_UNCHECKED : BST_CHECKED);
        return;
    }, [](config_window_object* self, HWND parent, shared_buffer* comm_buffer) {
         self->parent = parent;
        CheckDlgButton(parent, self->id.get_id(), comm_buffer->config.do_rendering ? BST_CHECKED : BST_UNCHECKED);
        return;
    } }
,  {L"Enable External Tracking",L"BUTTON", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT | BS_CHECKBOX,
    [](HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer) {
        BOOL checked = IsDlgButtonChecked(window, wp);
        comm_buffer->config.external_tracking = !checked;
        CheckDlgButton(window, wp, checked? BST_UNCHECKED:BST_CHECKED);
        return;
    }, [](config_window_object* self, HWND parent, shared_buffer* comm_buffer) {
         self->parent = parent;
        CheckDlgButton(parent, self->id.get_id(), comm_buffer->config.external_tracking ? BST_CHECKED : BST_UNCHECKED);
        return;
    } }
,  {L"Controller are Trackers/Objects",L"BUTTON", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT | BS_CHECKBOX,
    [](HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer) {
        BOOL checked = IsDlgButtonChecked(window, wp);
        comm_buffer->config.be_objects = !checked;
        CheckDlgButton(window, wp, checked ? BST_UNCHECKED : BST_CHECKED);
        return;
    }, [](config_window_object* self, HWND parent, shared_buffer* comm_buffer) {
        self->parent = parent;
        CheckDlgButton(parent, self->id.get_id(), comm_buffer->config.be_objects ? BST_CHECKED : BST_UNCHECKED);
        return;
    } }
,  {L"HMD as Tracker/Object",L"BUTTON", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT | BS_CHECKBOX,
    [](HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer) {
        BOOL checked = IsDlgButtonChecked(window, wp);
        comm_buffer->config.track_hmd = !checked;
        CheckDlgButton(window, wp, checked ? BST_UNCHECKED : BST_CHECKED);
        return;
    }, [](config_window_object* self, HWND parent, shared_buffer* comm_buffer) {
        self->parent = parent;
        CheckDlgButton(parent, self->id.get_id(), comm_buffer->config.track_hmd ? BST_CHECKED : BST_UNCHECKED);
        return;
    } }
,  {L"Universe ID",L"EDIT", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT | ES_READONLY,
    default_wm_command, default_init }
,  {L"31",L"EDIT", WS_VISIBLE | WS_CHILD | ES_NUMBER,
    [](HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer) {
        switch (HIWORD(wp))
        {
        case EN_CHANGE:
            {
                WCHAR Buffer[256];
                GetWindowText((HWND)lp, Buffer, 256);
                std::wcout << L"control text is: " << Buffer << std::endl;
                auto universe_id = _wtol(Buffer);
                std::cout << "parsed universe_id " << universe_id << std::endl;
                comm_buffer->config.vr_universe = universe_id;
            }
        }
        return;
    }, [](config_window_object* self, HWND parent, shared_buffer* comm_buffer) {
        self->parent = parent;
        const size_t len = 32;
        WCHAR CompBuffer[len];
        swprintf_s(CompBuffer, len, L"%u", comm_buffer->config.vr_universe);
        SetWindowText((HWND)self->wnd, CompBuffer);
        return;
    }  }
,  {L"Tracker Space Name",L"EDIT", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT | ES_READONLY,
    default_wm_command, default_init}
,  {L"oculus_link",L"EDIT", WS_VISIBLE | WS_CHILD ,
    [](HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer) {
        switch (HIWORD(wp))
        {
        case EN_CHANGE:
            {
                WCHAR Buffer[256];
                GetWindowText((HWND)lp, Buffer, 256);
                std::wcout << L"control text is: " << Buffer << std::endl;
                size_t converted_length;
                wcstombs_s(&converted_length, comm_buffer->config.tracking_space_name, Buffer, sizeof(comm_buffer->config.tracking_space_name)-1);
                std::cout << "parsed tracking space name " << comm_buffer->config.tracking_space_name << std::endl;
            }
        }
        return;
    }, [](config_window_object* self, HWND parent, shared_buffer* comm_buffer) {
        self->parent = parent;
        WCHAR Buffer[256];
        size_t converted_length;
        mbstowcs_s(&converted_length,  Buffer, comm_buffer->config.tracking_space_name, sizeof(comm_buffer->config.tracking_space_name) - 1);
        SetWindowText((HWND)self->wnd, Buffer);
        return;
    } }
,  {L"Manufacturer Name",L"EDIT", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT | ES_READONLY,
    default_wm_command, default_init }
,  {L"Oculus_link",L"EDIT", WS_VISIBLE | WS_CHILD ,
    [](HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer) {
        switch (HIWORD(wp))
        {
        case EN_CHANGE:
            {
                WCHAR Buffer[256];
                GetWindowText((HWND)lp, Buffer, 256);
                std::wcout << L"control text is: " << Buffer << std::endl;
                size_t converted_length;
                wcstombs_s(&converted_length, comm_buffer->config.manufacturer_name, Buffer, sizeof(comm_buffer->config.manufacturer_name) - 1);
                std::cout << "parsed manufacturer name " << comm_buffer->config.manufacturer_name << std::endl;
            }
        }
        return;
    }, [](config_window_object* self, HWND parent, shared_buffer* comm_buffer) {
        self->parent = parent;
        WCHAR Buffer[256];
        size_t converted_length;
        mbstowcs_s(&converted_length,  Buffer, comm_buffer->config.manufacturer_name, sizeof(comm_buffer->config.manufacturer_name) - 1);
        SetWindowText((HWND)self->wnd, Buffer);
        return;
    } }
,  {L"Prediction latency (ms)",L"EDIT", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT | ES_READONLY, 
    default_wm_command, default_init }
,  {L"10",L"EDIT", WS_VISIBLE | WS_CHILD | ES_NUMBER,
    [](HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer) {
        switch (HIWORD(wp))
        {
        case EN_CHANGE:
            {
                static bool being_changed = false;
                if (!being_changed) {
                    const size_t len = 32;
                    WCHAR Buffer[len];
                    WCHAR CompBuffer[len];
                    GetWindowText((HWND)lp, Buffer, len);
                    //std::wcout << L"control text is: " << Buffer << std::endl;
                    auto latency = _wtof(Buffer);
                    std::cout << "parsed latency is " << latency << std::endl;
                    swprintf_s(CompBuffer, len, L"%.1f", latency);
                    if (wcsncmp(Buffer, CompBuffer, len)) {
                        being_changed = true;
                        DWORD pos;
                        SendMessage((HWND)lp, EM_GETSEL, (WPARAM)&pos, NULL);
                        SetWindowText((HWND)lp, CompBuffer);
                        SendMessage((HWND)lp, EM_SETSEL, pos, pos);
                        being_changed = false;
                    }
                    comm_buffer->config.extra_prediction_ms = latency;
                }
            }
        }
        return;
    },[](config_window_object* self, HWND parent, shared_buffer* comm_buffer) {
        self->parent = parent;
        const size_t len = 32;
        WCHAR CompBuffer[len];
        swprintf_s(CompBuffer, len, L"%.1f", comm_buffer->config.extra_prediction_ms);
        SetWindowText((HWND)self->wnd, CompBuffer);
        return;
    } }
,   {L"Haptic Scale Factor",L"EDIT", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT | ES_READONLY,
     default_wm_command, default_init }
,   {L"1.00",L"EDIT", WS_VISIBLE | WS_CHILD | ES_NUMBER,
    [](HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer) {
        switch (HIWORD(wp))
        {
        case EN_CHANGE:
            {
                static bool being_changed = false;
                if (!being_changed) {
                    const size_t len = 32;
                    WCHAR Buffer[len];
                    WCHAR CompBuffer[len];
                    GetWindowText((HWND)lp, Buffer, len);
                    std::wcout << L"control text is: " << Buffer << std::endl;
                    auto amplitude_scale = _wtof(Buffer);
                    std::cout << "parsed scale factor is " << amplitude_scale << std::endl;
                    swprintf_s(CompBuffer, len, L"%.2f", amplitude_scale);
                    if (wcsncmp(Buffer, CompBuffer, len)) {
                        being_changed = true;
                        DWORD pos;
                        SendMessage((HWND)lp, EM_GETSEL, (WPARAM)&pos, NULL);
                        SetWindowText((HWND)lp, CompBuffer);
                        SendMessage((HWND)lp, EM_SETSEL, pos, pos);
                        being_changed = false;
                    }
                    comm_buffer->config.amplitude_scale = amplitude_scale;
                }
            }
        }
        return;
    },  [](config_window_object* self, HWND parent, shared_buffer* comm_buffer) {
        self->parent = parent;
        const size_t len = 32;
        WCHAR CompBuffer[len];
        swprintf_s(CompBuffer, len, L"%.2f", comm_buffer->config.amplitude_scale);
        SetWindowText((HWND)self->wnd, CompBuffer);
        return;
    } }
, { L"Haptic Minimum Value",L"EDIT", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT | ES_READONLY,
        default_wm_command, default_init }
, { L"64",L"EDIT", WS_VISIBLE | WS_CHILD | ES_NUMBER,
    [](HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer) {
        switch (HIWORD(wp))
        {
        case EN_CHANGE:
            {
                static bool being_changed = false;
                if (!being_changed) {
                    const size_t len = 32;
                    WCHAR Buffer[len];
                    WCHAR CompBuffer[len];
                    GetWindowText((HWND)lp, Buffer, len);
                    std::wcout << L"control text is: " << Buffer << std::endl;
                    auto amplitude = _wtol(Buffer);
                    if (amplitude < 0) amplitude = -amplitude;
                    std::cout << "parsed amplitude minimum is " << amplitude << std::endl;
                    swprintf_s(CompBuffer, len, L"%u", amplitude);
                    if (wcsncmp(Buffer, CompBuffer, len)) {
                        being_changed = true;
                        DWORD pos;
                        SendMessage((HWND)lp, EM_GETSEL, (WPARAM)&pos, NULL);
                        SetWindowText((HWND)lp, CompBuffer);
                        SendMessage((HWND)lp, EM_SETSEL, pos, pos);
                        being_changed = false;
                    }
                    comm_buffer->config.min_amplitude = amplitude;
                }
            }
        }
        return;
    },  [](config_window_object* self, HWND parent, shared_buffer* comm_buffer) {
        self->parent = parent;
        const size_t len = 32;
        WCHAR CompBuffer[len];
        swprintf_s(CompBuffer, len, L"%u", comm_buffer->config.min_amplitude);
        SetWindowText((HWND)self->wnd, CompBuffer);
        return;
    } }
, { L"Haptics sqrt pre-filter",L"BUTTON", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT | BS_CHECKBOX,
    [](HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer) {
        BOOL checked = IsDlgButtonChecked(window, wp);
        comm_buffer->config.sqrt_pre_filter = !checked;
        CheckDlgButton(window, wp, checked ? BST_UNCHECKED : BST_CHECKED);
        return;
    }, [](config_window_object* self, HWND parent, shared_buffer* comm_buffer) {
        self->parent = parent;
        CheckDlgButton(parent, self->id.get_id(), comm_buffer->config.sqrt_pre_filter ? BST_CHECKED : BST_UNCHECKED);
        return;
    } }
, { L"Haptics sqrt post-filter",L"BUTTON", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT | BS_CHECKBOX,
    [](HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer) {
        BOOL checked = IsDlgButtonChecked(window, wp);
        comm_buffer->config.sqrt_post_filter = !checked;
        CheckDlgButton(window, wp, checked ? BST_UNCHECKED : BST_CHECKED);
        return;
    }, [](config_window_object* self, HWND parent, shared_buffer* comm_buffer) {
        self->parent = parent;
        CheckDlgButton(parent, self->id.get_id(), comm_buffer->config.sqrt_post_filter ? BST_CHECKED : BST_UNCHECKED);
        return;
    } }
, { L"Perform World Translation",L"BUTTON", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT | BS_CHECKBOX,
    [](HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer) {
        BOOL checked = IsDlgButtonChecked(window, wp);
        comm_buffer->config.do_world_transformation = !checked;
        CheckDlgButton(window, wp, checked ? BST_UNCHECKED : BST_CHECKED);
        return;
    }, [](config_window_object* self, HWND parent, shared_buffer* comm_buffer) {
         self->parent = parent;
        CheckDlgButton(parent, self->id.get_id(), comm_buffer->config.do_world_transformation ? BST_CHECKED : BST_UNCHECKED);
        return;
    } }
 , { L"World Position Offset X Y Z",L"EDIT", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT | ES_READONLY,
     default_wm_command, default_init }
 , { L"0.0000 0.0000 0.0000",L"EDIT", WS_VISIBLE | WS_CHILD ,
            [](HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer) {
                switch (HIWORD(wp))
                {
                case EN_CHANGE:
                    {
                        static bool being_changed = false;
                        if (!being_changed) {
                            const size_t len = 64;
                            WCHAR Buffer[len];
                            WCHAR CompBuffer[len];
                            double offset[3];
                            GetWindowText((HWND)lp, Buffer, len);
                            swscanf_s(Buffer, L"%lf %lf %lf", &offset[0], &offset[1], &offset[2]);
                            std::wcout << L"control text is: " << Buffer << std::endl;
                            std::cout << "parsed world offset " << offset[0] << " " << offset[1] << " " << offset[2] << std::endl;
                            swprintf_s(CompBuffer, len, L"%.4lf %.4lf %.4lf", offset[0], offset[1], offset[2]);
                            if (wcsncmp(Buffer, CompBuffer, len)) {
                                DWORD pos;
                                SendMessage((HWND)lp, EM_GETSEL, (WPARAM)&pos, NULL);
                                std::cout << "got caret result = " << pos << std::endl;
                                being_changed = true;
                                std::wcout << L"reformatting text to: " << CompBuffer << std::endl;
                                SetWindowText((HWND)lp, CompBuffer);
                                SendMessage((HWND)lp, EM_SETSEL, pos, pos);
                                being_changed = false;
                            }
                            comm_buffer->config.world_translation[0] = offset[0] * 0.01;
                            comm_buffer->config.world_translation[1] = offset[1] * 0.01;
                            comm_buffer->config.world_translation[2] = offset[2] * 0.01;
                        }
                    }
                }
                return;
            },  [](config_window_object* self, HWND parent, shared_buffer* comm_buffer) {
                self->parent = parent;
                const size_t len = 32;
                WCHAR CompBuffer[len];
                swprintf_s(CompBuffer, len, L"%.4lf %.4lf %.4lf", 100.0*comm_buffer->config.world_translation[0], 100.0 * comm_buffer->config.world_translation[1], 100.0 * comm_buffer->config.world_translation[2]);
                SetWindowText((HWND)self->wnd, CompBuffer);
                return;
            } }

, { L"World Rotation Offset Euler Yaw Pitch Roll",L"EDIT", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT | ES_READONLY,
        default_wm_command, default_init }
, { L"0.0000 0.0000 0.0000",L"EDIT", WS_VISIBLE | WS_CHILD | ES_NUMBER,
            [](HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer) {
                switch (HIWORD(wp))
                {
                case EN_CHANGE:
                    {
                        if (!internal_angle_update) {
                            internal_angle_update = true;
                            const size_t len = 64;
                            WCHAR Buffer[len];
                            WCHAR CompBuffer[len];
                            double offset[3];
                            GetWindowText((HWND)lp, Buffer, len);
                            swscanf_s(Buffer, L"%lf %lf %lf", &offset[0], &offset[1], &offset[2]);
                            std::wcout << L"control text is: " << Buffer << std::endl;
                            std::cout << "parsed world rotation offset " << offset[0] << " " << offset[1] << " " << offset[2] << std::endl;
                            swprintf_s(CompBuffer, len, L"%.4lf %.4lf %.4lf", offset[0], offset[1], offset[2]);
                            if (wcsncmp(Buffer, CompBuffer, len)) {
                                DWORD pos;
                                SendMessage((HWND)lp, EM_GETSEL, (WPARAM)&pos, NULL);
                                std::cout << "got caret result = " << pos << std::endl;
                                std::wcout << L"reformatting text to: " << CompBuffer << std::endl;
                                SetWindowText((HWND)lp, CompBuffer);
                                SendMessage((HWND)lp, EM_SETSEL, pos, pos);

                            }
                            comm_buffer->config.world_orientation_euler[1] = offset[0];
                            comm_buffer->config.world_orientation_euler[2] = offset[1];
                            comm_buffer->config.world_orientation_euler[0] = offset[2];
                            if (orient_q_wp) {
                                comm_buffer->config.world_orientation_q = euler_to_quat(offset[2], offset[0], offset[1]);
                                orient_q_wp->init(orient_q_wp, window, comm_buffer);
                            }
                            internal_angle_update = false;

                        }
                    }
                }
                return;
            },  [](config_window_object* self, HWND parent, shared_buffer* comm_buffer) {
                self->parent = parent;
                orient_euler_wp = self;
                const size_t len = 64;
                WCHAR CompBuffer[len];
                swprintf_s(CompBuffer, len, L"%.4lf %.4lf %.4lf", comm_buffer->config.world_orientation_euler[1], comm_buffer->config.world_orientation_euler[2], comm_buffer->config.world_orientation_euler[0]);
                SetWindowText((HWND)self->wnd, CompBuffer);
                return;
            } }
, { L"World Rotation Offset Quaternion X Y Z W",L"EDIT", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT | ES_READONLY,
        default_wm_command, default_init }
, { L"0.0000 0.0000 0.0000 1.0000",L"EDIT", WS_VISIBLE | WS_CHILD | ES_READONLY,
        [](HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer) {
            switch (HIWORD(wp))
            {
            case EN_CHANGE:
                if (false) { // we've made this read only, we don't want to be updating the euler representation again when this is re-loaded
                
                    if (!internal_angle_update) {
                        internal_angle_update = true;
                        const size_t len = 64;
                        WCHAR Buffer[len];
                        WCHAR CompBuffer[len];
                        double offset[4];
                        GetWindowText((HWND)lp, Buffer, len);
                        swscanf_s(Buffer, L"%lf %lf %lf %lf", &offset[0], &offset[1], &offset[2], &offset[3]);
                        std::wcout << L"control text is: " << Buffer << std::endl;
                        std::cout << "parsed world rotation offset " << offset[0] << " " << offset[1] << " " << offset[2] << " " << offset[3] << std::endl;
                        swprintf_s(CompBuffer, len, L"%.4lf %.4lf %.4lf %.4lf", offset[0], offset[1], offset[2], offset[3]);
                        if (wcsncmp(Buffer, CompBuffer, len)) {
                            DWORD pos;
                            SendMessage((HWND)lp, EM_GETSEL, (WPARAM)&pos, NULL);
                            std::cout << "got caret result = " << pos << std::endl;
                            std::wcout << L"reformatting text to: " << CompBuffer << std::endl;
                            SetWindowText((HWND)lp, CompBuffer);
                            SendMessage((HWND)lp, EM_SETSEL, pos, pos);

                        }
                        comm_buffer->config.world_orientation_q.x = offset[0];
                        comm_buffer->config.world_orientation_q.y = offset[1];
                        comm_buffer->config.world_orientation_q.z = offset[2];
                        comm_buffer->config.world_orientation_q.w = offset[3];
                        if (orient_euler_wp) {
                            std::tie(comm_buffer->config.world_orientation_euler[0],
                                comm_buffer->config.world_orientation_euler[1],
                                comm_buffer->config.world_orientation_euler[2])
                                = quaternion_to_euler(comm_buffer->config.world_orientation_q);
                            orient_euler_wp->init(orient_euler_wp, window, comm_buffer);
                            }
                        internal_angle_update = false;
                    }
                }
            }
            return;
        },  [](config_window_object* self, HWND parent, shared_buffer* comm_buffer) {
            self->parent = parent;
            orient_q_wp = self;
            const size_t len = 64;
            WCHAR CompBuffer[len];
            swprintf_s(CompBuffer, len, L"%.4lf %.4lf %.4lf %.4lf", comm_buffer->config.world_orientation_q.x, comm_buffer->config.world_orientation_q.y, comm_buffer->config.world_orientation_q.z, comm_buffer->config.world_orientation_q.w);
            SetWindowText((HWND)self->wnd, CompBuffer);
            return;
        } }
, { L"Reset",L"BUTTON", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT ,
    [](HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer) {
        reset_config_settings(comm_buffer->config);
        GUI_Manager::self->reset_settings_window();
        return;
    }, default_init }
#ifdef DEBUG_VIB
 , { L"frequency amplitude duration",L"EDIT", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT | ES_READONLY,
                    default_wm_command, default_init }
        , { L"0.0000 0.0000 0.0000",L"EDIT", WS_VISIBLE | WS_CHILD | ES_NUMBER,
                    [](HWND window, WPARAM wp, LPARAM lp, shared_buffer* comm_buffer) {
                        switch (HIWORD(wp))
                        {
                        case EN_CHANGE:
                            {
                                const size_t len = 64;
                                WCHAR Buffer[len];
                                double offset[3];
                                GetWindowText((HWND)lp, Buffer, len);
                                swscanf_s(Buffer, L"%lf %lf %lf", &offset[0], &offset[1], &offset[2]);
                                std::wcout << L"control text is: " << Buffer << std::endl;
                                std::cout << "parsed vib " << offset[0] << " " << offset[1] << " " << offset[2] << std::endl;
  
                                comm_buffer->vib_frequency[0] = offset[0];
                                comm_buffer->vib_amplitude[0] = offset[1];
                                comm_buffer->vib_duration_s[0] = offset[2];
                                comm_buffer->vib_valid[0] = true;

                                comm_buffer->vib_frequency[1] = offset[0];
                                comm_buffer->vib_amplitude[1] = offset[1];
                                comm_buffer->vib_duration_s[1] = offset[2];
                                comm_buffer->vib_valid[1] = true;
                            }
                        }
                        return;
                    }, default_init }
#endif
};

std::map<HWND, config_window_object*> config_window_map;


extern shared_buffer* comm_buffer;

GUI_Manager* GUI_Manager::self = nullptr;

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam) {
    if (GUI_Manager::self) {
        return GUI_Manager::self->GUIWndProc(hWnd, message, wParam, lParam);
    }
    else return -1;
}


GUI_Manager::GUI_Manager(shared_buffer* comm_buffer)
    :comm_buffer(comm_buffer) {
    self = this;

    window = 0;
    WNDCLASSEX wndclass = { sizeof(WNDCLASSEX), CS_DBLCLKS, WndProc,
    0, 0, GetModuleHandle(0), LoadIcon(0,IDI_APPLICATION),
    LoadCursor(0,IDC_ARROW), HBRUSH(COLOR_WINDOW + 1),
    0, L"MyWindowsApp", LoadIcon(0,IDI_APPLICATION) };
    if (RegisterClassEx(&wndclass))
    {
        window = CreateWindowEx(0, L"MyWindowsApp", L"title",
            WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT,
            800, 600, 0, 0, GetModuleHandle(0), 0);

    }

    if (window)
    {
        ShowWindow(window, SW_SHOWDEFAULT);
    }

}

    void GUI_Manager::handle_loop() {
        MSG msg;
        while (GetMessage(&msg, 0, 0, 0)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
        std::cout << "GUI thread exiting" << std::endl;
    }

void GUI_Manager::reset_settings_window() {
    for (auto& config : config_windows) config.init(&config, config.parent, comm_buffer);
}

LRESULT CALLBACK GUI_Manager::GUIWndProc(HWND window, UINT msg, WPARAM wp, LPARAM lp)
{
    switch (msg)
    {
    case WM_DESTROY:
        std::cout << "\nWM_DESTROY" << std::endl;
        PostQuitMessage(0);
        return 0L;
    case WM_CLOSE:
        PostQuitMessage(0);
        break;

    case WM_CREATE:
 //       std::cout << "WM_CREATE window 0x" << std::hex << window << " wp 0x" << wp << " lp 0x" << lp << std::dec << std::endl;
        //HWND hEdit = CreateWindowEx(WS_EX_CLIENTEDGE, L"BUTTON", L"Enable External Tracking",
//    WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT | BS_CHECKBOX,
//    10, 10, 15, 15, window, (HMENU)ExternalTrackingID, GetModuleHandle(NULL), NULL);
        for (auto& config : config_windows) {
            config.wnd = CreateWindowEx(NULL, config.window_type.c_str(), config.name.c_str(),
                config.style,
                10, 20*config.id.get_id(), 300, 20, window, (HMENU)config.id.get_id(), (HINSTANCE)GetWindowLongPtr(window, GWLP_HINSTANCE), NULL);
  //          std::cout << std::hex << "adding window 0x" << config.wnd << " to the map" << std::endl;
            config_window_map[config.wnd] = &config;
            Edit_Enable(config.wnd, true);
            EnableWindow(config.wnd, true);
            config.init(&config, window, comm_buffer);
        }
#if 0
        hwnd_ExternalTracking = CreateWindowEx(NULL, L"BUTTON", L"Enable External Tracking",
            WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_FLAT | BS_TEXT | BS_CHECKBOX,
            10, 10, 300, 15, window, (HMENU)ExternalTrackingID, GetModuleHandle(NULL), NULL);
        if (hwnd_ExternalTracking == NULL)
            return -1;
#endif

        break;

    case WM_COMMAND:
 //       std::cout << "WM_COMMAND window 0x" << std::hex << window << " wp 0x" << wp << " lp 0x" << lp << std::dec << std::endl;
        if (config_window_map.find((HWND)lp) != config_window_map.end()) {
            config_window_map[(HWND)lp]->wm_command(window, wp, lp, comm_buffer);
            save_config_to_file(comm_buffer->config);
        }

#if 0
        BOOL checked = IsDlgButtonChecked(window, ExternalTrackingID);
        comm_buffer->external_tracking = !checked;
        if (checked) {
            std::cout << "checked" << std::endl;
            CheckDlgButton(window, ExternalTrackingID, BST_UNCHECKED);
            //SetWindowText(hwnd_ExternalTracking, TEXT(""));
        }
        else {
            std::cout << "unchecked" << std::endl;
            CheckDlgButton(window, ExternalTrackingID, BST_CHECKED);
            //SetWindowText(hwnd_ExternalTracking, title);
        }
#endif
        break;


    case WM_LBUTTONDOWN:
        //    std::cout << "\nmouse left button down at (" << LOWORD(lp)
        //        << ',' << HIWORD(lp) << ")\n";
            // fall thru
    default:
     //   std::cout << std::hex << "window 0x" << window << " msg 0x" << msg << " wp 0x" << wp << " lp 0x" << lp << std::dec << std::endl;
        return DefWindowProc(window, msg, wp, lp);
    }
}