/*
 * libtrueforce - sysfs attribute helpers.
 *
 * The kernel driver (hid-logitech-hidpp) exposes wheel settings as
 * sysfs attributes on the HIDPP interface 1 HID device: wheel_range,
 * wheel_damping, wheel_trueforce, wheel_strength and so on. libtrueforce
 * tracks interface 2's hidraw node (that's where the TF stream goes),
 * so reading or writing a setting means walking /sys/class/hidraw for
 * a sibling entry that sits under the same USB device root and exposes
 * the requested attribute.
 *
 * These helpers are used by the public SDK surface (exports.c) to
 * forward calls like logiWheelSetOperatingRangeDegrees straight into
 * the kernel's sysfs knob instead of maintaining a parallel
 * userspace-only state machine.
 */

#include "internal.h"

#include <dirent.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/*
 * Find the hidraw sibling under dev->usb_root that exposes the given
 * attribute, and return its /sys/class/hidraw/<name>/device/<attr>
 * path in out_path. Returns 0 on success, -1 on failure.
 */
static int find_attr_path(struct logitf_device *dev, const char *attr,
			  char *out_path, size_t out_len)
{
	DIR *d;
	struct dirent *ent;
	char dev_link[256];
	char resolved[PATH_MAX];
	int rc = -1;

	if (!dev || dev->usb_root[0] == '\0' || !attr || !out_path)
		return -1;

	d = opendir("/sys/class/hidraw");
	if (!d)
		return -1;

	while ((ent = readdir(d))) {
		if (strncmp(ent->d_name, "hidraw", 6) != 0)
			continue;

		snprintf(dev_link, sizeof(dev_link),
			 "/sys/class/hidraw/%s/device", ent->d_name);
		if (!realpath(dev_link, resolved))
			continue;
		if (strncmp(resolved, dev->usb_root, strlen(dev->usb_root)) != 0)
			continue;

		snprintf(out_path, out_len,
			 "/sys/class/hidraw/%s/device/%s",
			 ent->d_name, attr);
		if (access(out_path, F_OK) == 0) {
			rc = 0;
			break;
		}
	}
	closedir(d);
	return rc;
}

int logitf_sysfs_read_int(struct logitf_device *dev, const char *attr, int *out)
{
	char path[PATH_MAX];
	FILE *f;
	int v;

	if (!out)
		return -1;
	if (find_attr_path(dev, attr, path, sizeof(path)) < 0)
		return -1;
	f = fopen(path, "r");
	if (!f)
		return -1;
	if (fscanf(f, "%d", &v) != 1) {
		fclose(f);
		return -1;
	}
	fclose(f);
	*out = v;
	return 0;
}

int logitf_sysfs_write_int(struct logitf_device *dev, const char *attr, int val)
{
	char path[PATH_MAX];
	FILE *f;

	if (find_attr_path(dev, attr, path, sizeof(path)) < 0)
		return -1;
	f = fopen(path, "w");
	if (!f)
		return -1;
	if (fprintf(f, "%d", val) < 0) {
		fclose(f);
		return -1;
	}
	fclose(f);
	return 0;
}
