ovr_test.exe n 31 HTC lighthouse n 10 y y y 64 10.0



;;Welcome to oculus_touch_link, this program provides the input and haptic link to the stream driver, as well as passing confuguration data
;;You can provide this program with arguments to specify:
;;Render to Oculus headset y/n  ("n" must be use with ovr_dummy.exe)
;;VR "Universe" ID, 0=Invalid, 1=Oculus, 31=Recommended with VirtualDesktop/ALVR
;;Manufacturer name: Oculus or Oculus_link (or HTC for trackers)
;;Tracking system name: oculus or oculus_link (or lighthouse for trackers)
;;perform tracking prediction manually in-driver (n= ask oculus do the prediction) y/n
;;Extra prediction time (ms) for example 11.1 for 1 frame at 90fps
;;All controllers are tracked objects instead of controllers y/n
;;Perform tracking in ovr_test instead of steamvr driver y/n
;;Track the headset as a tracking object y/n
;;minimum haptic value to be generated (0-255)
;;scale multiplier on input haptic signal 

;;This program is super dumb and expects all of the arguments or none (for defaults), suggested invocations:
;;ovr_test.exe n 1 Oculus oculus n 10 n n n  64 10.0(must be use with ovr_dummy.exe)
;;ovr_test.exe y 1 Oculus oculus y 10 n n n 64 10.0
;;ovr_test.exe y 31 Oculus_link oculus_link n 10 n y n 64 10.0
;;ovr_test.exe n 31 Oculus_link oculus_link n 10 n n n  64 10.0(default)
;;defaults: n 31 Oculus_link oculus_link n 10 n n n 64 10.0