// ovr_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>

#ifndef NOMINMAX
#define NOMINMAX
#endif

#define   OVR_D3D_VERSION 11
#pragma warning(disable: 4324)
#include "Win32_DirectXAppUtil.h" // DirectX Helper
#include <OVR_CAPI_D3D.h>

#include <Windows.h>



int main()
{
    std::cout << "Hello World! "<<std::endl;
#if 0
    GuardianSystemDemo* instance = new (_aligned_malloc(sizeof(GuardianSystemDemo), 16)) GuardianSystemDemo();
    instance->Start(0);
    delete instance;
    return 0;
#endif
    ovrSession mSession = nullptr;
  //  ovrSession hmd2 = nullptr;
    ovrGraphicsLuid luid{};
  //  ovrGraphicsLuid luid2{};
    ovrInitParams initParams = { ovrInit_RequestVersion | ovrInit_FocusAware, OVR_MINOR_VERSION, NULL, 0, 0 };
    /*ovrResult result = ovr_Initialize(&initParams);   */
         
    if (OVR_FAILURE(ovr_Initialize(&initParams)/*ovr_Initialize(nullptr)*/)) std::cout << "ovr_Initialize error" << std::endl;

    if (OVR_FAILURE(ovr_Create(&mSession, &luid)))  std::cout << "ovr_Create error" << std::endl;

    if (OVR_FAILURE(ovr_SetTrackingOriginType(mSession, ovrTrackingOrigin_FloorLevel)))  std::cout << "ovr_SetTrackingOriginType error" << std::endl;

    int temp;
    std::cin >> temp;

    ovr_Destroy(mSession);
    ovr_Shutdown();

}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
