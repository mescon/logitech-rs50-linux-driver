/*
 * libtrueforce - wheel angle and angular velocity readback.
 *
 * The RS50's joystick interface (0) already emits the wheel's
 * absolute X axis at 1 kHz via evdev EV_ABS / ABS_X. Rather than
 * decode the ep 0x83 IN status reports ourselves, we piggyback on
 * the kernel's evdev parsing: one reader thread pulls EV_ABS
 * events from the joystick fd, converts ABS_X to degrees using
 * the axis's min/max (which the kernel populates from the HID
 * descriptor), and maintains an exponentially smoothed velocity
 * estimate.
 *
 * The evdev_fd is shared with the KF path - EV_FF writes and
 * EV_ABS reads on the same fd don't conflict.
 */

#include "internal.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

static double ns_now(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (double)ts.tv_sec + ts.tv_nsec / 1e9;
}

static int kf_open_if_needed(struct logitf_device *dev)
{
	if (dev->evdev_fd >= 0)
		return LOGITF_OK;
	if (dev->evdev_path[0] == '\0')
		return LOGITF_ERR_NOT_FOUND;
	dev->evdev_fd = open(dev->evdev_path, O_RDWR | O_CLOEXEC);
	if (dev->evdev_fd < 0)
		return LOGITF_ERR_IO;
	dev->kf_effect_id = -1;
	return LOGITF_OK;
}

/*
 * Find the wheel_range sysfs attribute. Scan /sys/class/hidraw and
 * return the value of the first node that exposes it. Defaults to
 * 1080 (the RS50 OEM default).
 *
 * This is not tied to `dev` because the wheel_range attribute lives
 * on interface 1's HID device (our driver's sysfs home) while `dev`
 * tracks interface 2's hidraw. They belong to the same USB
 * device but have distinct sysfs parents.
 */
static int read_wheel_range_deg(struct logitf_device *dev)
{
	DIR *d;
	struct dirent *ent;
	char path[512];
	int v = 1080;
	bool found = false;

	(void)dev;

	d = opendir("/sys/class/hidraw");
	if (!d)
		return v;
	while ((ent = readdir(d)) && !found) {
		FILE *f;

		if (strncmp(ent->d_name, "hidraw", 6) != 0)
			continue;
		snprintf(path, sizeof(path),
			 "/sys/class/hidraw/%s/device/wheel_range",
			 ent->d_name);
		f = fopen(path, "r");
		if (!f)
			continue;
		if (fscanf(f, "%d", &v) == 1)
			found = true;
		fclose(f);
	}
	closedir(d);
	return v > 0 ? v : 1080;
}

static int read_absinfo(struct logitf_device *dev)
{
	struct input_absinfo info;

	if (ioctl(dev->evdev_fd, EVIOCGABS(ABS_X), &info) < 0)
		return -errno;
	dev->abs_x_min = info.minimum;
	dev->abs_x_max = info.maximum;
	dev->wheel_range_deg = read_wheel_range_deg(dev);
	return 0;
}

/*
 * Convert raw ABS_X to degrees. The kernel driver exposes the wheel
 * position as the full axis range; we map it to [-range_deg/2,
 * +range_deg/2] around centre. Range comes from wheel_range sysfs,
 * but we fall back to 1080 if we can't read it.
 */
static double raw_to_degrees(struct logitf_device *dev, int raw)
{
	int range = dev->wheel_range_deg ? dev->wheel_range_deg : 1080;
	int span = dev->abs_x_max - dev->abs_x_min;
	double norm;

	if (span <= 0)
		return 0.0;
	norm = ((double)raw - dev->abs_x_min) / span * 2.0 - 1.0;  /* -1..+1 */
	return norm * (range / 2.0);
}

static void *status_thread_fn(void *arg)
{
	struct logitf_device *dev = arg;
	struct pollfd pfds[2] = {
		{ .fd = dev->evdev_fd,       .events = POLLIN },
		{ .fd = dev->status_stopfd,  .events = POLLIN },
	};
	struct input_event ev;
	const double vel_tau = 0.02;  /* 20 ms smoothing window */

	for (;;) {
		int pr = poll(pfds, 2, -1);

		if (pr < 0) {
			if (errno == EINTR)
				continue;
			break;
		}
		if (pfds[1].revents & POLLIN)
			break;
		if (!(pfds[0].revents & POLLIN))
			continue;

		ssize_t n = read(dev->evdev_fd, &ev, sizeof(ev));

		if (n != (ssize_t)sizeof(ev))
			continue;
		if (ev.type != EV_ABS || ev.code != ABS_X)
			continue;

		double now = ns_now();
		double deg = raw_to_degrees(dev, ev.value);

		pthread_mutex_lock(&dev->lock);
		if (dev->status_last_time > 0.0) {
			double dt = now - dev->status_last_time;

			if (dt > 0.0) {
				double dv = (deg - dev->wheel_angle_deg) / dt;
				/* Exponential smoothing toward the new delta. */
				double a = dt / (dt + vel_tau);

				dev->wheel_velocity_deg_s =
					dev->wheel_velocity_deg_s * (1.0 - a) + dv * a;
			}
		}
		dev->wheel_angle_deg = deg;
		dev->status_last_time = now;
		pthread_mutex_unlock(&dev->lock);
	}
	return NULL;
}

int logitf_status_start(struct logitf_device *dev)
{
	int rc;

	pthread_mutex_lock(&dev->lock);
	if (dev->status_running) {
		pthread_mutex_unlock(&dev->lock);
		return LOGITF_OK;
	}
	rc = kf_open_if_needed(dev);
	if (rc) {
		pthread_mutex_unlock(&dev->lock);
		return rc;
	}
	if (read_absinfo(dev) < 0) {
		pthread_mutex_unlock(&dev->lock);
		return LOGITF_ERR_IO;
	}
	dev->status_stopfd = eventfd(0, EFD_CLOEXEC);
	if (dev->status_stopfd < 0) {
		pthread_mutex_unlock(&dev->lock);
		return LOGITF_ERR_IO;
	}
	rc = pthread_create(&dev->status_thread, NULL, status_thread_fn, dev);
	if (rc != 0) {
		close(dev->status_stopfd);
		dev->status_stopfd = -1;
		pthread_mutex_unlock(&dev->lock);
		return LOGITF_ERR_IO;
	}
	dev->status_running = true;
	pthread_mutex_unlock(&dev->lock);
	return LOGITF_OK;
}

int logitf_status_stop(struct logitf_device *dev)
{
	uint64_t one = 1;

	pthread_mutex_lock(&dev->lock);
	if (!dev->status_running) {
		pthread_mutex_unlock(&dev->lock);
		return LOGITF_OK;
	}
	pthread_mutex_unlock(&dev->lock);

	if (dev->status_stopfd >= 0)
		write(dev->status_stopfd, &one, sizeof(one));
	pthread_join(dev->status_thread, NULL);

	pthread_mutex_lock(&dev->lock);
	if (dev->status_stopfd >= 0) {
		close(dev->status_stopfd);
		dev->status_stopfd = -1;
	}
	dev->status_running = false;
	pthread_mutex_unlock(&dev->lock);
	return LOGITF_OK;
}

double logitf_status_angle_deg(struct logitf_device *dev)
{
	double v;

	pthread_mutex_lock(&dev->lock);
	v = dev->wheel_angle_deg;
	pthread_mutex_unlock(&dev->lock);
	return v;
}

double logitf_status_velocity_deg_s(struct logitf_device *dev)
{
	double v;

	pthread_mutex_lock(&dev->lock);
	v = dev->wheel_velocity_deg_s;
	pthread_mutex_unlock(&dev->lock);
	return v;
}
