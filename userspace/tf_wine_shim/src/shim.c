/*
 * Wine PE shim for the Windows Trueforce SDK ABI.
 *
 * Proton games load trueforce_sdk_x64.dll inside their Wine prefix.
 * When WINEDLLOVERRIDES puts "trueforce_sdk=n,b" in effect, this
 * shim DLL (.dll.so as built by winegcc) is loaded in place of the
 * genuine PE. Each exported function here forwards to the native
 * libtrueforce.so which speaks to the RS50 over hidraw + evdev.
 *
 * Shim functions are named shim_<export> to avoid colliding with
 * libtrueforce's ELF symbols of the same name. The .spec file maps
 * the Windows-side export back to the shim_ name.
 *
 * DirectInput handles are opaque pointers in the Windows ABI. We
 * don't need DirectInput state; we just need a wheel identity.
 * For single-wheel setups (the common case) we map every open to
 * controller index 0. Multi-wheel mapping via GUID_Product is a
 * Phase 23.x follow-up.
 */

#define WIN32_LEAN_AND_MEAN
#include <windef.h>
#include <winbase.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Native Linux library header. Only uses stdint / stdbool / stddef. */
#include <trueforce.h>

/* ------------------------------------------------------------------ */
/*  DirectInput handle -> controller-index map (single-threaded)      */
/* ------------------------------------------------------------------ */

#define SHIM_MAX_MAPS 4

static struct {
	const void *di_handle;
	int index;
} g_map[SHIM_MAX_MAPS];

static int shim_register(const void *di_handle)
{
	for (int i = 0; i < SHIM_MAX_MAPS; i++) {
		if (!g_map[i].di_handle) {
			g_map[i].di_handle = di_handle;
			g_map[i].index = i;
			return i;
		}
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/*  Module lifecycle                                                  */
/* ------------------------------------------------------------------ */

int WINAPI shim_dllOpen(void)               { return dllOpen(); }
int WINAPI shim_dllClose(void)              { return dllClose(); }
int WINAPI shim_DllRegisterServer(void)     { return DllRegisterServer(); }
int WINAPI shim_DllUnregisterServer(void)   { return DllUnregisterServer(); }

/* ------------------------------------------------------------------ */
/*  Availability                                                      */
/* ------------------------------------------------------------------ */

BOOL WINAPI shim_logiTrueForceAvailable(int index)
{
	return logiTrueForceAvailable(index);
}
BOOL WINAPI shim_logiTrueForceSupported(int index)
{
	return logiTrueForceSupported(index);
}
BOOL WINAPI shim_logiTrueForceSupportedByDirectInputA(const void *h)
{
	(void)h;
	return logiTrueForceAvailable(0);
}
BOOL WINAPI shim_logiTrueForceSupportedByDirectInputW(const void *h)
{
	return shim_logiTrueForceSupportedByDirectInputA(h);
}
BOOL WINAPI shim_logiWheelSupportedByDirectInputA(const void *h)
{
	return shim_logiTrueForceSupportedByDirectInputA(h);
}
BOOL WINAPI shim_logiWheelSupportedByDirectInputW(const void *h)
{
	return shim_logiTrueForceSupportedByDirectInputA(h);
}

/* ------------------------------------------------------------------ */
/*  Session                                                           */
/* ------------------------------------------------------------------ */

int WINAPI shim_logiWheelOpenByDirectInputA(const void *h) { return shim_register(h); }
int WINAPI shim_logiWheelOpenByDirectInputW(const void *h) { return shim_register(h); }
int WINAPI shim_logiWheelClose(int idx)                     { return logiWheelClose(idx); }
BOOL WINAPI shim_logiWheelSdkHasControl(int idx)            { return logiWheelSdkHasControl(idx); }

/* ------------------------------------------------------------------ */
/*  Versioning                                                        */
/* ------------------------------------------------------------------ */

int WINAPI shim_logiWheelGetCoreLibraryVersion(int *maj, int *min, int *bld)
{
	return logiWheelGetCoreLibraryVersion(maj, min, bld);
}
int WINAPI shim_logiWheelGetVersion(int idx, int *maj, int *min, int *bld)
{
	return logiWheelGetVersion(idx, maj, min, bld);
}

/* ------------------------------------------------------------------ */
/*  Operating range                                                   */
/* ------------------------------------------------------------------ */

int    WINAPI shim_logiWheelGetForceMode(int idx)                                    { return logiWheelGetForceMode(idx); }
int    WINAPI shim_logiWheelSetForceMode(int idx, int m)                             { return logiWheelSetForceMode(idx, m); }
double WINAPI shim_logiWheelGetOperatingRangeDegrees(int idx)                        { return logiWheelGetOperatingRangeDegrees(idx); }
double WINAPI shim_logiWheelGetOperatingRangeRadians(int idx)                        { return logiWheelGetOperatingRangeRadians(idx); }
int    WINAPI shim_logiWheelGetOperatingRangeBoundsDegrees(int idx, double *lo, double *hi) { return logiWheelGetOperatingRangeBoundsDegrees(idx, lo, hi); }
int    WINAPI shim_logiWheelGetOperatingRangeBoundsRadians(int idx, double *lo, double *hi) { return logiWheelGetOperatingRangeBoundsRadians(idx, lo, hi); }
int    WINAPI shim_logiWheelSetOperatingRangeDegrees(int idx, double d)              { return logiWheelSetOperatingRangeDegrees(idx, d); }
int    WINAPI shim_logiWheelSetOperatingRangeRadians(int idx, double r)              { return logiWheelSetOperatingRangeRadians(idx, r); }

/* ------------------------------------------------------------------ */
/*  LEDs                                                              */
/* ------------------------------------------------------------------ */

int WINAPI shim_logiWheelGetRpmLedCaps(int idx, int *caps)                           { return logiWheelGetRpmLedCaps(idx, caps); }
int WINAPI shim_logiWheelSetRpmLeds(int idx, uint32_t mask)                          { return logiWheelSetRpmLeds(idx, mask); }
int WINAPI shim_logiWheelPlayLeds(int idx, double rpm, double first, double red)     { return logiWheelPlayLeds(idx, rpm, first, red); }

/* ------------------------------------------------------------------ */
/*  Angle / velocity                                                  */
/* ------------------------------------------------------------------ */

double WINAPI shim_logiTrueForceGetAngleDegrees(int idx)            { return logiTrueForceGetAngleDegrees(idx); }
double WINAPI shim_logiTrueForceGetAngleRadians(int idx)            { return logiTrueForceGetAngleRadians(idx); }
double WINAPI shim_logiTrueForceGetAngularVelocityDegrees(int idx)  { return logiTrueForceGetAngularVelocityDegrees(idx); }
double WINAPI shim_logiTrueForceGetAngularVelocityRadians(int idx)  { return logiTrueForceGetAngularVelocityRadians(idx); }

/* ------------------------------------------------------------------ */
/*  Kinetic force                                                     */
/* ------------------------------------------------------------------ */

int    WINAPI shim_logiTrueForceSetTorqueKF(int idx, double t)                         { return logiTrueForceSetTorqueKF(idx, t); }
double WINAPI shim_logiTrueForceGetTorqueKF(int idx)                                   { return logiTrueForceGetTorqueKF(idx); }
int    WINAPI shim_logiTrueForceSetTorqueKFPiecewise(int idx, const double *s, int n)  { return logiTrueForceSetTorqueKFPiecewise(idx, s, n); }
int    WINAPI shim_logiTrueForceClearKF(int idx)                                       { return logiTrueForceClearKF(idx); }
int    WINAPI shim_logiTrueForceSetGainKF(int idx, double g)                           { return logiTrueForceSetGainKF(idx, g); }
double WINAPI shim_logiTrueForceGetGainKF(int idx)                                     { return logiTrueForceGetGainKF(idx); }
double WINAPI shim_logiTrueForceGetMaxContinuousTorqueKF(int idx)                      { return logiTrueForceGetMaxContinuousTorqueKF(idx); }
double WINAPI shim_logiTrueForceGetMaxPeakTorqueKF(int idx)                            { return logiTrueForceGetMaxPeakTorqueKF(idx); }
int    WINAPI shim_logiTrueForceSetReconstructionFilterKF(int idx, int l)              { return logiTrueForceSetReconstructionFilterKF(idx, l); }
int    WINAPI shim_logiTrueForceGetReconstructionFilterKF(int idx)                     { return logiTrueForceGetReconstructionFilterKF(idx); }

/* ------------------------------------------------------------------ */
/*  TF audio stream                                                   */
/* ------------------------------------------------------------------ */

int    WINAPI shim_logiTrueForceSetTorqueTFdouble(int idx, const double *s, int n)   { return logiTrueForceSetTorqueTFdouble(idx, s, n); }
int    WINAPI shim_logiTrueForceSetTorqueTFfloat(int idx, const float *s, int n)     { return logiTrueForceSetTorqueTFfloat(idx, s, n); }
int    WINAPI shim_logiTrueForceSetTorqueTFint16(int idx, const int16_t *s, int n)   { return logiTrueForceSetTorqueTFint16(idx, s, n); }
int    WINAPI shim_logiTrueForceSetTorqueTFint32(int idx, const int32_t *s, int n)   { return logiTrueForceSetTorqueTFint32(idx, s, n); }
int    WINAPI shim_logiTrueForceSetTorqueTFint8(int idx, const int8_t *s, int n)     { return logiTrueForceSetTorqueTFint8(idx, s, n); }
int    WINAPI shim_logiTrueForceSetStreamTF(int idx, const int16_t *s, int n)        { return logiTrueForceSetStreamTF(idx, s, n); }
double WINAPI shim_logiTrueForceGetTorqueTF(int idx)                                 { return logiTrueForceGetTorqueTF(idx); }
int    WINAPI shim_logiTrueForceGetTorqueTFRateBounds(int idx, double *lo, double *hi) { return logiTrueForceGetTorqueTFRateBounds(idx, lo, hi); }
int    WINAPI shim_logiTrueForceClearTF(int idx)                                     { return logiTrueForceClearTF(idx); }
int    WINAPI shim_logiTrueForceSetGainTF(int idx, double g)                         { return logiTrueForceSetGainTF(idx, g); }
double WINAPI shim_logiTrueForceGetGainTF(int idx)                                   { return logiTrueForceGetGainTF(idx); }

/* ------------------------------------------------------------------ */
/*  Damping                                                           */
/* ------------------------------------------------------------------ */

int    WINAPI shim_logiTrueForceSetDamping(int idx, double d)   { return logiTrueForceSetDamping(idx, d); }
double WINAPI shim_logiTrueForceGetDamping(int idx)             { return logiTrueForceGetDamping(idx); }
double WINAPI shim_logiTrueForceGetDampingMax(int idx)          { return logiTrueForceGetDampingMax(idx); }

/* ------------------------------------------------------------------ */
/*  Haptic thread                                                     */
/* ------------------------------------------------------------------ */

double WINAPI shim_logiTrueForceGetHapticRate(int idx)           { return logiTrueForceGetHapticRate(idx); }
int    WINAPI shim_logiTrueForceGetHapticThreadStatus(int idx)   { return logiTrueForceGetHapticThreadStatus(idx); }

/* ------------------------------------------------------------------ */
/*  Pause / resume                                                    */
/* ------------------------------------------------------------------ */

int  WINAPI shim_logiTrueForcePause(int idx)     { return logiTrueForcePause(idx); }
int  WINAPI shim_logiTrueForceResume(int idx)    { return logiTrueForceResume(idx); }
BOOL WINAPI shim_logiTrueForceIsPaused(int idx)  { return logiTrueForceIsPaused(idx); }
int  WINAPI shim_logiTrueForceSync(int idx)      { return logiTrueForceSync(idx); }

/* ------------------------------------------------------------------ */
/*  Advanced                                                          */
/* ------------------------------------------------------------------ */

int WINAPI shim_logiAdvancedGetThreadHandles(int idx, void **handles, int max)
{
	return logiAdvancedGetThreadHandles(idx, handles, max);
}
