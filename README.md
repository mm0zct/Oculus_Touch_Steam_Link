# Oculus_Touch_Steam_Link
SteamVR driver to enable Oculus Touch controllers with other headsets

```
This program is super dumb and expects all of the arguments or none (for defaults), suggested invocations:
ovr_test.exe n 1 Oculus oculus n 10 n n(must be use with ovr_dummy.exe)
ovr_test.exe y 1 Oculus oculus y 10 n n
ovr_test.exe y 31 Oculus_link oculus_link n 10 n y n
ovr_test.exe n 31 Oculus_link oculus_link n 10 n n y(default)
```

### Setting the auto-launcher arguments:
Inside the `args.txt` file all you need to do is set the arguments as your normally would on the command line.  
For example: `n 31 Oculus_link oculus_link n 10 n n y`