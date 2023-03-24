# Oculus_Touch_Steam_Link
SteamVR driver to enable Oculus Touch controllers with other headsets

Guide copied from PumkinSpice's nice writeup
https://github.com/PumkinSpice/MixedVR/wiki/MixedVR-CV1-Touch

Prerequisites:
This SteamVR Driver

ODTKRA https://github.com/DeltaNeverUsed/ODTKRA - this will keep the Oculus runtime and headset awake

A full CV1 touch kit (HMD, two controllers, at least two sensors)

The HMD you want to use. This will work for WMR, Lighthouse HMDs such as Index and Vive (you need BOTH lighthouses and CV1 sensor for lighthouse HMDs), and Quest (Quest will ONLY WORK WITH ALVR OR VIRTUAL DESKTOP). It is possible to do this without the HMD's native controllers by calibrating the CV1 touch controller against the HMD you are using.

OpenVR Space Calibrator https://github.com/pushrax/OpenVR-SpaceCalibrator/releases

OpenVR Advanced Settings from Steam https://store.steampowered.com/app/1009850/OVR_Advanced_Settings/ (for creating a proper steamVR chaperone after setting up the mixedVR system -- required for certain games! Note you can also get this from GitHub https://github.com/OpenVR-Advanced-Settings/OpenVR-AdvancedSettings/releases but Steam will get you the latest stable release and integrate better)

Steps:

Have the HMD you want to use all set up with its native home (WMR Portal, Quest's Home, etc), controllers and all. If you're using WMR, follow step 1 & 2 from here https://github.com/PumkinSpice/MixedVR/wiki/ReadMe for general setup guidelines. Guidelines for Quest are similar, but you need to additionally make sure the app you're using to connect to steamVR wirelessly is using 'Staged Tracking' (it's in the 'streaming' section of settings in Virtual Desktop and you will have to look for it on ALVR as I am not familiar with this program).

Once you have everything set up in its native environment and have installed SteamVR and anything extra such as WMR for steamVR, start steamVR. Make sure everything works. You will likely have to disable SteamVR home if you're on a G2.

Exit SteamVR and steam (completely close out of steam!). Install openVR Space Calibrator.

Close out of your native HMD's environment. If on a G2, Index, or Vive, disconnect the power from its link box/HMD cable. If on other WMR HMD, disconnect its USB from the PC.

Set up your CV1 kit as if you were going to use that as your main VR. If you have two sensors, put them diagonal from each other for fairly decent 360 coverage (the more sensors you have, the better!). Go through the entire setup: install oculus home, set up the guardian, make sure your floor is correct, everything. Also keep in mind that your Oculus HMD will need the proximity sensor triggered, but we can use ODKTRA to keep it awake now!

Close oculus home for now.

Unzip the touch-link SteamVR driver from the latest release and copy the whole folder to your steamVR path. The default path is "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers" then open a PowerShell command line and run:

C:\'Program Files (x86)'\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg adddriver C:\'Program Files (x86)'\Steam\steamapps\common\SteamVR\drivers\OculusTouchLink

(change the path to your own steam/steamVR install path, instead of SteamVR you may have OpenVR if you were very early to the VR scene)

If using WMR: Plug the power back into the HMD (or plug USB back into the PC) and wait for WMR portal to come up and start working. Turn on your WMR controllers.
If using Quest: connect to your PC via ALVR/Virtual Desktop but do nothing else yet except make sure you have staged tracking on. Proceed to step 9.
If using Vive or Index: Do nothing yet. Proceed to step 9.
Start ODTKRA https://github.com/DeltaNeverUsed/ODTKRA , ovr_test.exe and ovr_dummy.exe. Give your oculus HMD a little wiggle for good measure. Pick up an oculus controller and watch the ovr_test.exe console window, make sure the state for the controller turns to 0xf. If it doesn't, move the oculus headset around a bit and make sure a sensor can see it.

Put on your main HMD and start steamVR from its native environment

On the space calibrator screen (either in Vr or on your desktop), the left side should show your native HMD devices and the right side should show your CV1 oculus devices. Both sides have a drop-down menu and sometimes you have to manually pick the universe you want to use from there. Pick a native HMD controller on the left and a CV1 touch controller on the right. You should pick the same ‘hand’ (there’s an ‘identify’ button that makes the controllers you’ve picked vibrate). Click ‘copy chaperone bounds’ on the bottom of the space calibrator menu. NOTE: sometimes the 'wrong' controller wants to be the input in which case you may need to contort yourself a bit to pick everything with the not yet calibrated touch controllers, or just use the desktop with the mouse.

With the native HMD and CV1 Touch controller you’ve picked in the same hand (and make sure the tracking rings on the native controller can be seen by the native HMD) choose "Very slow calibration" (to be honest, I just use fastest and I'm happy with it), and then click "Start Calibration". Now start slowly moving the hand holding both controllers around in a figure 8 pattern, back and forth and in and out. The HMD needs to be able to see its native controller while doing this so don’t swing wildly or like over your head or behind your back. It also helps to walk around your room and spin slow circles while doing this. After 30-40 seconds the touch controller should pop into place over the native controller and the space calibrator window should read ‘calibration complete’.

If you do not have controllers for your primary HMD, then instead of a controller on the left, select the HMD, when calibrating hold the CV1 controller against your forehead and swing your head in loops (this is known as "the unicorn method").

At this point you’re all calibrated and it should save! You only need to and should only "calibrate" one controller (this is because you're not actually calibrating controllers, you're calibrating the native HMD and SteamVR play spaces using a controller). It is likely only one of the controllers shows up when outside of the steamVR menu and/or you can't click any buttons on the touch controllers. Just exit out of steamVR, turn off your native controllers (if on Quest, go into the 'input' section of Virtual Desktop and tell it to have the Quest controllers emulate a regular joypad), and go back into SteamVR.

From here, set up a proper steamVR chaperone so everything runs correctly: https://github.com/PumkinSpice/MixedVR/wiki/ReadMe#using-steamvr-chaperone-instead-of-wmr-bounds
