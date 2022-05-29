# Oculus_Touch_Steam_Link
SteamVR driver to enable Oculus Touch controllers with other headsets

```
You can provide this program with arguments to specify:
Render to Oculus headset y/n  ("n" must be use with ovr_dummy.exe)
VR "Universe" ID, 0=Invalid, 1=Oculus, 31=Recommended with VirtualDesktop/ALVR
Manufacturer name: Oculus or Oculus_link (or HTC for trackers)
Tracking system name: oculus or oculus_link (or lighthouse for trackers)
perform tracking prediction manually in-driver (n= ask oculus do the prediction) y/n
Extra prediction time (ms) for example 11.1 for 1 frame at 90fps
All controllers are tracked objects instead of controllers y/n
Perform tracking in ovr_test instead of steamvr driver y/n
Track the headset as a tracking object y/n

This program is super dumb and expects all of the arguments or none (for defaults), suggested invocations:
ovr_test.exe n 1 Oculus oculus n 10 n n(must be use with ovr_dummy.exe)
ovr_test.exe y 1 Oculus oculus y 10 n n
ovr_test.exe y 31 Oculus_link oculus_link n 10 n y n
ovr_test.exe n 31 Oculus_link oculus_link n 10 n n y(default)
```

### Setting the auto-launcher arguments:
Inside the `args.txt` file all you need to do is set the arguments as your normally would on the command line.  
For example: `n 31 Oculus_link oculus_link n 10 n n y`