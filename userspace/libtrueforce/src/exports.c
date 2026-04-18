/*
 * libtrueforce - public API surface.
 *
 * Availability/discovery functions are live. All other entry points
 * are intentional stubs at this phase (Rank 22.1) - they return
 * LOGITF_ERR_NOT_SUPPORTED so games get a predictable "no Trueforce
 * yet" response while Rank 22.2+ flesh out session and streaming.
 *
 * The module-init pair (dllOpen/dllClose) triggers a discovery scan
 * so that the index-based availability calls answer correctly without
 * requiring the caller to do anything else first.
 */

#include "internal.h"

#include <stddef.h>

/* ---- Module lifecycle ---- */

int dllOpen(void)
{
	return logitf_discover();
}

int dllClose(void)
{
	struct logitf_device *t = logitf_table();

	for (int i = 0; i < LOGITF_MAX_CONTROLLERS; i++) {
		if (!t[i].in_use)
			continue;
		if (t[i].hidraw_fd >= 0) {
			/* Phase 22.2 will add graceful stop here. */
			t[i].hidraw_fd = -1;
		}
		if (t[i].evdev_fd >= 0)
			t[i].evdev_fd = -1;
		pthread_mutex_destroy(&t[i].lock);
		t[i].in_use = false;
	}
	return LOGITF_OK;
}

int DllRegisterServer(void)
{
	/* COM no-op on Linux. Windows SDK returns S_OK (0) here. */
	return 0;
}

int DllUnregisterServer(void)
{
	return 0;
}

/* ---- Discovery / availability ---- */

bool logiTrueForceAvailable(int index)
{
	struct logitf_device *dev;

	return logitf_find_by_index(index, &dev) == LOGITF_OK;
}

bool logiTrueForceSupported(int index)
{
	return logiTrueForceAvailable(index);
}

bool logiTrueForceSupportedByDirectInputA(const void *di_device)
{
	(void)di_device;
	/* Phase 22.2 will map the DI GUID to a library index. */
	return false;
}

bool logiTrueForceSupportedByDirectInputW(const void *di_device)
{
	(void)di_device;
	return false;
}

bool logiWheelSupportedByDirectInputA(const void *di_device)
{
	(void)di_device;
	return false;
}

bool logiWheelSupportedByDirectInputW(const void *di_device)
{
	(void)di_device;
	return false;
}

/* ---- Session (stubs) ---- */

int logiWheelOpenByDirectInputA(const void *di_device)
{
	(void)di_device;
	return LOGITF_ERR_NOT_SUPPORTED;
}

int logiWheelOpenByDirectInputW(const void *di_device)
{
	(void)di_device;
	return LOGITF_ERR_NOT_SUPPORTED;
}

int logiWheelClose(int index)
{
	(void)index;
	return LOGITF_OK;
}

bool logiWheelSdkHasControl(int index)
{
	(void)index;
	return false;
}

/* ---- Versioning ---- */

int logiWheelGetCoreLibraryVersion(int *major, int *minor, int *build)
{
	if (major) *major = 1;
	if (minor) *minor = 3;
	if (build) *build = 11;
	return LOGITF_OK;
}

int logiWheelGetVersion(int index, int *major, int *minor, int *build)
{
	struct logitf_device *dev;
	int rc = logitf_find_by_index(index, &dev);

	if (rc)
		return rc;
	/* Phase 22.2 queries the device via HID++ for real firmware info. */
	if (major) *major = 0;
	if (minor) *minor = 0;
	if (build) *build = 0;
	return LOGITF_OK;
}

/* ---- Operating range (stubs) ---- */

int    logiWheelGetForceMode(int index) { (void)index; return LOGITF_ERR_NOT_SUPPORTED; }
int    logiWheelSetForceMode(int index, int mode) { (void)index; (void)mode; return LOGITF_ERR_NOT_SUPPORTED; }
double logiWheelGetOperatingRangeDegrees(int index) { (void)index; return 0.0; }
double logiWheelGetOperatingRangeRadians(int index) { (void)index; return 0.0; }
int    logiWheelGetOperatingRangeBoundsDegrees(int index, double *lo, double *hi) { (void)index; if (lo) *lo = 0; if (hi) *hi = 0; return LOGITF_ERR_NOT_SUPPORTED; }
int    logiWheelGetOperatingRangeBoundsRadians(int index, double *lo, double *hi) { (void)index; if (lo) *lo = 0; if (hi) *hi = 0; return LOGITF_ERR_NOT_SUPPORTED; }
int    logiWheelSetOperatingRangeDegrees(int index, double deg) { (void)index; (void)deg; return LOGITF_ERR_NOT_SUPPORTED; }
int    logiWheelSetOperatingRangeRadians(int index, double rad) { (void)index; (void)rad; return LOGITF_ERR_NOT_SUPPORTED; }

/* ---- RPM / LEDs (stubs) ---- */

int logiWheelGetRpmLedCaps(int index, int *caps) { (void)index; if (caps) *caps = 0; return LOGITF_ERR_NOT_SUPPORTED; }
int logiWheelSetRpmLeds(int index, uint32_t rgb_mask) { (void)index; (void)rgb_mask; return LOGITF_ERR_NOT_SUPPORTED; }
int logiWheelPlayLeds(int index, double rpm, double first, double red) { (void)index; (void)rpm; (void)first; (void)red; return LOGITF_ERR_NOT_SUPPORTED; }

/* ---- Angle / velocity (stubs) ---- */

double logiTrueForceGetAngleDegrees(int index) { (void)index; return 0.0; }
double logiTrueForceGetAngleRadians(int index) { (void)index; return 0.0; }
double logiTrueForceGetAngularVelocityDegrees(int index) { (void)index; return 0.0; }
double logiTrueForceGetAngularVelocityRadians(int index) { (void)index; return 0.0; }

/* ---- Kinetic force (stubs) ---- */

int    logiTrueForceSetTorqueKF(int index, double t) { (void)index; (void)t; return LOGITF_ERR_NOT_SUPPORTED; }
double logiTrueForceGetTorqueKF(int index) { (void)index; return 0.0; }
int    logiTrueForceSetTorqueKFPiecewise(int index, const double *s, int n) { (void)index; (void)s; (void)n; return LOGITF_ERR_NOT_SUPPORTED; }
int    logiTrueForceClearKF(int index) { (void)index; return LOGITF_ERR_NOT_SUPPORTED; }
int    logiTrueForceSetGainKF(int index, double g) { (void)index; (void)g; return LOGITF_ERR_NOT_SUPPORTED; }
double logiTrueForceGetGainKF(int index) { (void)index; return 1.0; }
double logiTrueForceGetMaxContinuousTorqueKF(int index) { (void)index; return 0.0; }
double logiTrueForceGetMaxPeakTorqueKF(int index) { (void)index; return 0.0; }
int    logiTrueForceSetReconstructionFilterKF(int index, int level) { (void)index; (void)level; return LOGITF_ERR_NOT_SUPPORTED; }
int    logiTrueForceGetReconstructionFilterKF(int index) { (void)index; return 0; }

/* ---- Trueforce audio stream (stubs) ---- */

int    logiTrueForceSetTorqueTFdouble(int index, const double  *s, int n) { (void)index; (void)s; (void)n; return LOGITF_ERR_NOT_SUPPORTED; }
int    logiTrueForceSetTorqueTFfloat (int index, const float   *s, int n) { (void)index; (void)s; (void)n; return LOGITF_ERR_NOT_SUPPORTED; }
int    logiTrueForceSetTorqueTFint16 (int index, const int16_t *s, int n) { (void)index; (void)s; (void)n; return LOGITF_ERR_NOT_SUPPORTED; }
int    logiTrueForceSetTorqueTFint32 (int index, const int32_t *s, int n) { (void)index; (void)s; (void)n; return LOGITF_ERR_NOT_SUPPORTED; }
int    logiTrueForceSetTorqueTFint8  (int index, const int8_t  *s, int n) { (void)index; (void)s; (void)n; return LOGITF_ERR_NOT_SUPPORTED; }
int    logiTrueForceSetStreamTF(int index, const int16_t *s, int n) { (void)index; (void)s; (void)n; return LOGITF_ERR_NOT_SUPPORTED; }
double logiTrueForceGetTorqueTF(int index) { (void)index; return 0.0; }
int    logiTrueForceGetTorqueTFRateBounds(int index, double *lo, double *hi) { (void)index; if (lo) *lo = 1000.0; if (hi) *hi = 1000.0; return LOGITF_OK; }
int    logiTrueForceClearTF(int index) { (void)index; return LOGITF_ERR_NOT_SUPPORTED; }
int    logiTrueForceSetGainTF(int index, double g) { (void)index; (void)g; return LOGITF_ERR_NOT_SUPPORTED; }
double logiTrueForceGetGainTF(int index) { (void)index; return 1.0; }

/* ---- Damping (stubs) ---- */

int    logiTrueForceSetDamping(int index, double d) { (void)index; (void)d; return LOGITF_ERR_NOT_SUPPORTED; }
double logiTrueForceGetDamping(int index) { (void)index; return 0.0; }
double logiTrueForceGetDampingMax(int index) { (void)index; return 1.0; }

/* ---- Haptic thread (stubs) ---- */

double logiTrueForceGetHapticRate(int index) { (void)index; return 1000.0; }
int    logiTrueForceGetHapticThreadStatus(int index) { (void)index; return 0; }

/* ---- Pause / resume ---- */

int logiTrueForcePause(int index)
{
	struct logitf_device *dev;
	int rc = logitf_find_by_index(index, &dev);

	if (rc)
		return rc;
	dev->tf_paused = true;
	return LOGITF_OK;
}

int logiTrueForceResume(int index)
{
	struct logitf_device *dev;
	int rc = logitf_find_by_index(index, &dev);

	if (rc)
		return rc;
	dev->tf_paused = false;
	return LOGITF_OK;
}

bool logiTrueForceIsPaused(int index)
{
	struct logitf_device *dev;

	if (logitf_find_by_index(index, &dev))
		return false;
	return dev->tf_paused;
}

int logiTrueForceSync(int index)
{
	(void)index;
	return LOGITF_OK;
}

/* ---- Advanced (stub) ---- */

int logiAdvancedGetThreadHandles(int index, void **handles, int max)
{
	(void)index;
	if (handles && max > 0)
		handles[0] = NULL;
	return 0; /* Zero handles exposed. */
}
