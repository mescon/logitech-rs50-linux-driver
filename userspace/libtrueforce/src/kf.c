/*
 * libtrueforce - Kinetic Force (KF) routing via evdev FF_CONSTANT.
 *
 * The TF audio stream on interface 2 is one of two force channels
 * games use; the other is classic constant torque, which the
 * Windows SDK calls "Kinetic Force". Under Windows it reaches the
 * wheel via the DirectInput FFB path. On Linux we route KF through
 * the sibling joystick's evdev FF_CONSTANT effect, so kernel's
 * input_ff takes care of the 64-byte PID FFB packet on the same
 * interface 2 that hidraw doesn't own.
 *
 * We keep a single FF_CONSTANT effect uploaded per device and
 * update its level on every SetTorqueKF call.
 */

#include "internal.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

/*
 * RS50 direct-drive motor peak torque (datasheet / manual page 13:
 * "up to 8 Nm"). We clamp inputs outside +/- MAX so a misbehaving
 * game can't request more than the wheel can physically produce.
 */
#define RS50_MAX_TORQUE_NM 8.0

static int kf_ensure_open(struct logitf_device *dev)
{
	if (dev->evdev_fd >= 0)
		return LOGITF_OK;
	if (dev->evdev_path[0] == '\0')
		return LOGITF_ERR_NOT_FOUND;
	dev->evdev_fd = open(dev->evdev_path, O_RDWR | O_CLOEXEC);
	if (dev->evdev_fd < 0) {
		int e = errno;

		if (e == EACCES || e == EPERM)
			return LOGITF_ERR_BUSY;
		return LOGITF_ERR_IO;
	}
	dev->kf_effect_id = -1;
	return LOGITF_OK;
}

static int kf_upload(struct logitf_device *dev, int16_t level)
{
	struct ff_effect eff;

	memset(&eff, 0, sizeof(eff));
	eff.type = FF_CONSTANT;
	eff.id = dev->kf_effect_id;   /* -1 = allocate new */
	eff.u.constant.level = level;
	eff.direction = 0x4000;       /* East = +X = full right */
	eff.replay.length = 0;        /* infinite until stopped */
	eff.replay.delay = 0;

	if (ioctl(dev->evdev_fd, EVIOCSFF, &eff) < 0)
		return -errno;
	dev->kf_effect_id = eff.id;
	return 0;
}

static int kf_play(struct logitf_device *dev, int start)
{
	struct input_event ev;

	memset(&ev, 0, sizeof(ev));
	ev.type = EV_FF;
	ev.code = (uint16_t)dev->kf_effect_id;
	ev.value = start ? 1 : 0;
	if (write(dev->evdev_fd, &ev, sizeof(ev)) != sizeof(ev))
		return -errno;
	return 0;
}

int logitf_kf_set_torque_nm(struct logitf_device *dev, double torque_nm)
{
	int rc;
	int16_t level;
	double scaled;

	pthread_mutex_lock(&dev->lock);
	rc = kf_ensure_open(dev);
	if (rc) {
		pthread_mutex_unlock(&dev->lock);
		return rc;
	}

	if (torque_nm >  RS50_MAX_TORQUE_NM) torque_nm =  RS50_MAX_TORQUE_NM;
	if (torque_nm < -RS50_MAX_TORQUE_NM) torque_nm = -RS50_MAX_TORQUE_NM;
	scaled = torque_nm * 32767.0 / RS50_MAX_TORQUE_NM;
	level = (int16_t)scaled;

	if (kf_upload(dev, level) < 0) {
		pthread_mutex_unlock(&dev->lock);
		return LOGITF_ERR_IO;
	}
	if (kf_play(dev, 1) < 0) {
		pthread_mutex_unlock(&dev->lock);
		return LOGITF_ERR_IO;
	}
	dev->kf_last_nm = torque_nm;
	dev->kf_playing = true;
	pthread_mutex_unlock(&dev->lock);
	return LOGITF_OK;
}

int logitf_kf_clear(struct logitf_device *dev)
{
	pthread_mutex_lock(&dev->lock);
	if (dev->evdev_fd >= 0 && dev->kf_effect_id >= 0 && dev->kf_playing) {
		kf_play(dev, 0);
		dev->kf_playing = false;
		dev->kf_last_nm = 0.0;
	}
	pthread_mutex_unlock(&dev->lock);
	return LOGITF_OK;
}

int logitf_kf_close(struct logitf_device *dev)
{
	pthread_mutex_lock(&dev->lock);
	if (dev->evdev_fd >= 0) {
		if (dev->kf_effect_id >= 0) {
			ioctl(dev->evdev_fd, EVIOCRMFF, (long)dev->kf_effect_id);
			dev->kf_effect_id = -1;
		}
		close(dev->evdev_fd);
		dev->evdev_fd = -1;
	}
	dev->kf_playing = false;
	dev->kf_last_nm = 0.0;
	pthread_mutex_unlock(&dev->lock);
	return LOGITF_OK;
}

double logitf_kf_get_torque_nm(struct logitf_device *dev)
{
	double v;

	pthread_mutex_lock(&dev->lock);
	v = dev->kf_last_nm;
	pthread_mutex_unlock(&dev->lock);
	return v;
}

double logitf_kf_max_continuous_nm(void) { return 5.0; }
double logitf_kf_max_peak_nm(void)       { return RS50_MAX_TORQUE_NM; }
