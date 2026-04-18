#
# Wine PE spec for libtrueforce's Windows-side stub
# (trueforce_sdk_x64.dll, v1.3.11).
#
# Ordinals and names are taken verbatim from winedump -j export of the
# original DLL (see sdk/trueforce_1_3_11/exports_x64.txt). The 16
# C++-mangled symbols (ordinals 4-16) are internal to the Windows
# implementation and are exposed as stubs here; games should not call
# them directly.
#

1  stdcall dllClose() shim_dllClose
2  stdcall dllOpen() shim_dllOpen
3  stdcall logiTrueForceAvailable(long) shim_logiTrueForceAvailable

# C++-mangled internals - stubs
4  stub ?z05c197a23e@@YAHH@Z
5  stub ?z138bf1c956@@YA?AW4zfa07dd3623@@H@Z
6  stub ?z3786eeb338@@YAHHAEBV?$map@W4zc021e26d66@@_NU?$less@W4zc021e26d66@@@std@@V?$allocator@U?$pair@$$CBW4zc021e26d66@@_N@std@@@3@@std@@@Z
7  stub ?z54f2cda717@@YA_NHW4zc021e26d66@@@Z
8  stub ?z691c1b7d2b@@YANH@Z
9  stub ?z6f21c28add@@YAHHPEAN00@Z
10 stub ?z96447731d5@@YAHHW4zc021e26d66@@_N@Z
11 stub ?za03e7a40ba@@YA_NHW4zc021e26d66@@@Z
12 stub ?za9366b2029@@YAHHPEAIPEAN111PEAG21@Z
13 stub ?zada3b7386d@@YAHHPEAW4z869e698237@@PEAIPEAN2@Z
14 stub ?zc4b84aae0c@@YAHHW4z869e698237@@INN@Z
15 stub ?zed5afbbef2@@YAHHNNN@Z
16 stub ?zf32f9e9aba@@YAHHGG@Z

17 stdcall DllRegisterServer() shim_DllRegisterServer
18 stdcall DllUnregisterServer() shim_DllUnregisterServer
19 stdcall logiAdvancedGetThreadHandles(long ptr long) shim_logiAdvancedGetThreadHandles

20 stdcall logiTrueForceClearKF(long) shim_logiTrueForceClearKF
21 stdcall logiTrueForceClearTF(long) shim_logiTrueForceClearTF
22 stdcall logiTrueForceGetAngleDegrees(long) shim_logiTrueForceGetAngleDegrees
23 stdcall logiTrueForceGetAngleRadians(long) shim_logiTrueForceGetAngleRadians
24 stdcall logiTrueForceGetAngularVelocityDegrees(long) shim_logiTrueForceGetAngularVelocityDegrees
25 stdcall logiTrueForceGetAngularVelocityRadians(long) shim_logiTrueForceGetAngularVelocityRadians
26 stdcall logiTrueForceGetDamping(long) shim_logiTrueForceGetDamping
27 stdcall logiTrueForceGetDampingMax(long) shim_logiTrueForceGetDampingMax
28 stdcall logiTrueForceGetGainKF(long) shim_logiTrueForceGetGainKF
29 stdcall logiTrueForceGetGainTF(long) shim_logiTrueForceGetGainTF
30 stdcall logiTrueForceGetHapticRate(long) shim_logiTrueForceGetHapticRate
31 stdcall logiTrueForceGetHapticThreadStatus(long) shim_logiTrueForceGetHapticThreadStatus
32 stdcall logiTrueForceGetMaxContinuousTorqueKF(long) shim_logiTrueForceGetMaxContinuousTorqueKF
33 stdcall logiTrueForceGetMaxPeakTorqueKF(long) shim_logiTrueForceGetMaxPeakTorqueKF
34 stdcall logiTrueForceGetReconstructionFilterKF(long) shim_logiTrueForceGetReconstructionFilterKF
35 stdcall logiTrueForceGetTorqueKF(long) shim_logiTrueForceGetTorqueKF
36 stdcall logiTrueForceGetTorqueTF(long) shim_logiTrueForceGetTorqueTF
37 stdcall logiTrueForceGetTorqueTFRateBounds(long ptr ptr) shim_logiTrueForceGetTorqueTFRateBounds
38 stdcall logiTrueForceIsPaused(long) shim_logiTrueForceIsPaused
39 stdcall logiTrueForcePause(long) shim_logiTrueForcePause
40 stdcall logiTrueForceResume(long) shim_logiTrueForceResume
41 stdcall logiTrueForceSetDamping(long double) shim_logiTrueForceSetDamping
42 stdcall logiTrueForceSetGainKF(long double) shim_logiTrueForceSetGainKF
43 stdcall logiTrueForceSetGainTF(long double) shim_logiTrueForceSetGainTF
44 stdcall logiTrueForceSetReconstructionFilterKF(long long) shim_logiTrueForceSetReconstructionFilterKF
45 stdcall logiTrueForceSetStreamTF(long ptr long) shim_logiTrueForceSetStreamTF
46 stdcall logiTrueForceSetTorqueKF(long double) shim_logiTrueForceSetTorqueKF
47 stdcall logiTrueForceSetTorqueKFPiecewise(long ptr long) shim_logiTrueForceSetTorqueKFPiecewise
48 stdcall logiTrueForceSetTorqueTFdouble(long ptr long) shim_logiTrueForceSetTorqueTFdouble
49 stdcall logiTrueForceSetTorqueTFfloat(long ptr long) shim_logiTrueForceSetTorqueTFfloat
50 stdcall logiTrueForceSetTorqueTFint16(long ptr long) shim_logiTrueForceSetTorqueTFint16
51 stdcall logiTrueForceSetTorqueTFint32(long ptr long) shim_logiTrueForceSetTorqueTFint32
52 stdcall logiTrueForceSetTorqueTFint8(long ptr long) shim_logiTrueForceSetTorqueTFint8
53 stdcall logiTrueForceSupported(long) shim_logiTrueForceSupported
54 stdcall logiTrueForceSupportedByDirectInputA(ptr) shim_logiTrueForceSupportedByDirectInputA
55 stdcall logiTrueForceSupportedByDirectInputW(ptr) shim_logiTrueForceSupportedByDirectInputW
56 stdcall logiTrueForceSync(long) shim_logiTrueForceSync
57 stdcall logiWheelClose(long) shim_logiWheelClose
58 stdcall logiWheelGetCoreLibraryVersion(ptr ptr ptr) shim_logiWheelGetCoreLibraryVersion
59 stdcall logiWheelGetForceMode(long) shim_logiWheelGetForceMode
60 stdcall logiWheelGetOperatingRangeBoundsDegrees(long ptr ptr) shim_logiWheelGetOperatingRangeBoundsDegrees
61 stdcall logiWheelGetOperatingRangeBoundsRadians(long ptr ptr) shim_logiWheelGetOperatingRangeBoundsRadians
62 stdcall logiWheelGetOperatingRangeDegrees(long) shim_logiWheelGetOperatingRangeDegrees
63 stdcall logiWheelGetOperatingRangeRadians(long) shim_logiWheelGetOperatingRangeRadians
64 stdcall logiWheelGetRpmLedCaps(long ptr) shim_logiWheelGetRpmLedCaps
65 stdcall logiWheelGetVersion(long ptr ptr ptr) shim_logiWheelGetVersion
66 stdcall logiWheelOpenByDirectInputA(ptr) shim_logiWheelOpenByDirectInputA
67 stdcall logiWheelOpenByDirectInputW(ptr) shim_logiWheelOpenByDirectInputW
68 stdcall logiWheelPlayLeds(long double double double) shim_logiWheelPlayLeds
69 stdcall logiWheelSdkHasControl(long) shim_logiWheelSdkHasControl
70 stdcall logiWheelSetForceMode(long long) shim_logiWheelSetForceMode
71 stdcall logiWheelSetOperatingRangeDegrees(long double) shim_logiWheelSetOperatingRangeDegrees
72 stdcall logiWheelSetOperatingRangeRadians(long double) shim_logiWheelSetOperatingRangeRadians
73 stdcall logiWheelSetRpmLeds(long long) shim_logiWheelSetRpmLeds
74 stdcall logiWheelSupportedByDirectInputA(ptr) shim_logiWheelSupportedByDirectInputA
75 stdcall logiWheelSupportedByDirectInputW(ptr) shim_logiWheelSupportedByDirectInputW
