/*
 * libtrueforce - native Linux implementation of the Logitech Trueforce
 * SDK (trueforce_sdk_x64.dll, v1.3.11).
 *
 * Covers the RS50 wheel family. Talks to interface 2 via /dev/hidrawN.
 * Kinetic-force (KF) calls route through evdev /dev/input/eventX on the
 * same physical wheel; audio-haptic Trueforce (TF) samples stream
 * directly to the hidraw node at 1 kHz.
 *
 * The API mirrors the Windows SDK surface so the Wine PE shim can
 * forward calls with minimal translation.
 */

#ifndef LIBTRUEFORCE_TRUEFORCE_H
#define LIBTRUEFORCE_TRUEFORCE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LOGITF_MAX_CONTROLLERS 4

/* Return codes. Zero on success, negative errno-like on failure. */
#define LOGITF_OK 0
#define LOGITF_ERR_INVALID_ARG   -1
#define LOGITF_ERR_NOT_FOUND     -2
#define LOGITF_ERR_NOT_SUPPORTED -3
#define LOGITF_ERR_IO            -4
#define LOGITF_ERR_BUSY          -5

/* ---- Module lifecycle (loadable-library init) ---- */

int  dllOpen(void);
int  dllClose(void);

/* COM boilerplate; return success no-op on Linux. */
int  DllRegisterServer(void);
int  DllUnregisterServer(void);

/* ---- Discovery ---- */

bool logiTrueForceAvailable(int index);
bool logiTrueForceSupported(int index);
bool logiTrueForceSupportedByDirectInputA(const void *di_device);
bool logiTrueForceSupportedByDirectInputW(const void *di_device);

bool logiWheelSupportedByDirectInputA(const void *di_device);
bool logiWheelSupportedByDirectInputW(const void *di_device);

/* ---- Session ---- */

int  logiWheelOpenByDirectInputA(const void *di_device);
int  logiWheelOpenByDirectInputW(const void *di_device);
int  logiWheelClose(int index);
bool logiWheelSdkHasControl(int index);

/* ---- Versioning ---- */

int  logiWheelGetCoreLibraryVersion(int *major, int *minor, int *build);
int  logiWheelGetVersion(int index, int *major, int *minor, int *build);

/* ---- Wheel operating range ---- */

int     logiWheelGetForceMode(int index);
int     logiWheelSetForceMode(int index, int mode);
double  logiWheelGetOperatingRangeDegrees(int index);
double  logiWheelGetOperatingRangeRadians(int index);
int     logiWheelGetOperatingRangeBoundsDegrees(int index, double *lo, double *hi);
int     logiWheelGetOperatingRangeBoundsRadians(int index, double *lo, double *hi);
int     logiWheelSetOperatingRangeDegrees(int index, double degrees);
int     logiWheelSetOperatingRangeRadians(int index, double radians);

/* ---- RPM / LED capabilities ---- */

int  logiWheelGetRpmLedCaps(int index, int *caps);
int  logiWheelSetRpmLeds(int index, uint32_t rgb_mask);
int  logiWheelPlayLeds(int index, double current_rpm, double rpm_first_led, double rpm_redline);

/* ---- Angle & angular velocity ---- */

double logiTrueForceGetAngleDegrees(int index);
double logiTrueForceGetAngleRadians(int index);
double logiTrueForceGetAngularVelocityDegrees(int index);
double logiTrueForceGetAngularVelocityRadians(int index);

/* ---- Kinetic-force (classic constant torque) ---- */

int    logiTrueForceSetTorqueKF(int index, double torque_nm);
double logiTrueForceGetTorqueKF(int index);
int    logiTrueForceSetTorqueKFPiecewise(int index, const double *samples, int count);
int    logiTrueForceClearKF(int index);
int    logiTrueForceSetGainKF(int index, double gain);
double logiTrueForceGetGainKF(int index);
double logiTrueForceGetMaxContinuousTorqueKF(int index);
double logiTrueForceGetMaxPeakTorqueKF(int index);
int    logiTrueForceSetReconstructionFilterKF(int index, int level);
int    logiTrueForceGetReconstructionFilterKF(int index);

/* ---- Trueforce audio-haptic stream ----
 *
 * All SetTorqueTF* / SetStreamTF calls feed a 4096-entry internal
 * ring that a dedicated thread drains at the wheel's 1 kHz sample
 * rate. If the ring fills (caller pushing faster than the wheel
 * consumes), these calls block until space is available - roughly
 * up to ~4 s of back-pressure at full saturation. Games driving the
 * stream should treat them as synchronous.
 */

int    logiTrueForceSetTorqueTFdouble(int index, const double  *samples, int count);
int    logiTrueForceSetTorqueTFfloat (int index, const float   *samples, int count);
int    logiTrueForceSetTorqueTFint16 (int index, const int16_t *samples, int count);
int    logiTrueForceSetTorqueTFint32 (int index, const int32_t *samples, int count);
int    logiTrueForceSetTorqueTFint8  (int index, const int8_t  *samples, int count);
int    logiTrueForceSetStreamTF(int index, const int16_t *samples, int count);
double logiTrueForceGetTorqueTF(int index);
int    logiTrueForceGetTorqueTFRateBounds(int index, double *rate_min_hz, double *rate_max_hz);
int    logiTrueForceClearTF(int index);
int    logiTrueForceSetGainTF(int index, double gain);
double logiTrueForceGetGainTF(int index);

/* ---- Damping ---- */

int    logiTrueForceSetDamping(int index, double damping);
double logiTrueForceGetDamping(int index);
double logiTrueForceGetDampingMax(int index);

/* ---- Haptic thread ---- */

double logiTrueForceGetHapticRate(int index);
int    logiTrueForceGetHapticThreadStatus(int index);

/* ---- Pause/resume/sync ---- */

int  logiTrueForcePause(int index);
int  logiTrueForceResume(int index);
bool logiTrueForceIsPaused(int index);
int  logiTrueForceSync(int index);

/* ---- Advanced ---- */

int  logiAdvancedGetThreadHandles(int index, void **handles, int max);

#ifdef __cplusplus
}
#endif

#endif /* LIBTRUEFORCE_TRUEFORCE_H */
