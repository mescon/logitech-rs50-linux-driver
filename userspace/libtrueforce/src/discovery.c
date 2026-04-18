/*
 * libtrueforce - hidraw discovery.
 *
 * Walks /sys/class/hidraw to find RS50-family wheels and records the
 * /dev/hidrawN node attached to interface 2 (the Trueforce audio
 * stream endpoint). Also records the joystick evdev node on interface
 * 0 so KF calls can route through evdev FF_CONSTANT.
 *
 * The by-id path (e.g. /dev/input/by-id/usb-Logitech_RS50_Base_for_
 * PlayStation_PC_<serial>-event-joystick) is used as the stable
 * identity that maps to a Logitech SDK "controller index" (0..3).
 * Indexing is deterministic within a run but may change if the user
 * replugs or reorders devices.
 */

#include "internal.h"

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static struct logitf_device g_table[LOGITF_MAX_CONTROLLERS];
static bool g_discovered;

struct logitf_device *logitf_table(void)
{
	return g_table;
}

/* Read a small sysfs file. Returns 0 on success, -1 on error. Caller-owned buf. */
static int read_sysfs(const char *path, char *buf, size_t bufsize)
{
	int fd = open(path, O_RDONLY | O_CLOEXEC);
	ssize_t n;

	if (fd < 0)
		return -1;
	n = read(fd, buf, bufsize - 1);
	close(fd);
	if (n < 0)
		return -1;
	/* Trim trailing newline and whitespace */
	while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r' ||
			 buf[n - 1] == ' ' || buf[n - 1] == '\t'))
		n--;
	buf[n] = '\0';
	return 0;
}

/* Parse lowercase-hex u16 from a buffer. Returns 0 on success. */
static int parse_hex_u16(const char *s, uint16_t *out)
{
	char *end;
	unsigned long v = strtoul(s, &end, 16);

	if (end == s || v > 0xFFFF)
		return -1;
	*out = (uint16_t)v;
	return 0;
}

/*
 * Given a /sys/class/hidraw/hidrawN path, resolve the parent USB
 * interface and return its bInterfaceNumber, VID, and PID.
 *
 * sysfs layout:
 *   /sys/class/hidraw/hidrawN/device         -> ../../<hid-id>
 *   /sys/class/hidraw/hidrawN/device/../     = USB interface dir
 *                                              (has bInterfaceNumber)
 *   /sys/class/hidraw/hidrawN/device/../../  = USB device dir
 *                                              (has idVendor/idProduct)
 */
static int hidraw_usb_ids(const char *hidraw_name,
			  uint16_t *vid, uint16_t *pid, int *ifnum)
{
	char linkpath[256];
	char resolved[PATH_MAX];
	char buf[32];
	char *slash;

	snprintf(linkpath, sizeof(linkpath),
		 "/sys/class/hidraw/%s/device", hidraw_name);
	if (!realpath(linkpath, resolved))
		return -1;

	/* resolved now points at the HID device dir. Walk up twice:
	 * once to the USB interface, once more to the USB device. */
	slash = strrchr(resolved, '/');
	if (!slash)
		return -1;
	*slash = '\0';
	/* Interface dir: read bInterfaceNumber */
	snprintf(linkpath, sizeof(linkpath),
		 "%s/bInterfaceNumber", resolved);
	if (read_sysfs(linkpath, buf, sizeof(buf)) < 0)
		return -1;
	*ifnum = atoi(buf);

	/* Go up to the USB device dir. */
	slash = strrchr(resolved, '/');
	if (!slash)
		return -1;
	*slash = '\0';

	snprintf(linkpath, sizeof(linkpath), "%s/idVendor", resolved);
	if (read_sysfs(linkpath, buf, sizeof(buf)) < 0)
		return -1;
	if (parse_hex_u16(buf, vid) < 0)
		return -1;

	snprintf(linkpath, sizeof(linkpath), "%s/idProduct", resolved);
	if (read_sysfs(linkpath, buf, sizeof(buf)) < 0)
		return -1;
	if (parse_hex_u16(buf, pid) < 0)
		return -1;

	return 0;
}

/* Is this vid/pid an RS50-family wheel? For now, only RS50 itself. */
static bool is_rs50_family(uint16_t vid, uint16_t pid)
{
	return vid == LOGITF_LOGI_VID && pid == LOGITF_RS50_PID;
}

/*
 * Find the matching /dev/input/eventN (joystick) for the same physical
 * RS50, given the hidraw path that pointed at interface 2. We look
 * under /dev/input/by-id for a -event-joystick symlink whose resolved
 * target shares the same USB device path up through "usb1/1-1".
 *
 * Returns 0 on success. Fills evdev_path with /dev/input/eventN and
 * by_id with the /dev/input/by-id/... path.
 */
static int find_sibling_evdev(const char *hidraw_sysdev,
			      char *evdev_path, size_t evdev_sz,
			      char *by_id, size_t by_id_sz)
{
	DIR *dir;
	struct dirent *ent;
	char usb_root[PATH_MAX];
	char byid_link[PATH_MAX];
	char byid_target[PATH_MAX];
	char *slash;
	int rc = -1;

	/*
	 * hidraw_sysdev points at the HID device directory:
	 *     .../usb1/1-1/1-1:1.2/0003:046D:C276.0003
	 *
	 * Walk up two levels to reach the USB *device* dir (1-1), which
	 * is the identity shared with the joystick interface's evdev
	 * node.
	 */
	snprintf(usb_root, sizeof(usb_root), "%s", hidraw_sysdev);
	for (int i = 0; i < 2; i++) {
		slash = strrchr(usb_root, '/');
		if (!slash)
			return -1;
		*slash = '\0';
	}

	dir = opendir("/dev/input/by-id");
	if (!dir)
		return -1;

	while ((ent = readdir(dir))) {
		if (!strstr(ent->d_name, "-event-joystick"))
			continue;
		if (!strstr(ent->d_name, "Logitech"))
			continue;
		if (!strstr(ent->d_name, "RS50"))
			continue;

		snprintf(byid_link, sizeof(byid_link),
			 "/dev/input/by-id/%s", ent->d_name);
		if (!realpath(byid_link, byid_target))
			continue;

		/*
		 * Walk up the resolved target's sysfs path to find the USB
		 * device. If it matches usb_root, this is the same wheel.
		 */
		char evdev_sysdev[PATH_MAX];
		char evdev_name[64];
		const char *base = strrchr(byid_target, '/');

		if (!base)
			continue;
		snprintf(evdev_name, sizeof(evdev_name), "%s", base + 1);
		snprintf(evdev_sysdev, sizeof(evdev_sysdev),
			 "/sys/class/input/%s/device", evdev_name);
		if (!realpath(evdev_sysdev, byid_target))
			continue;

		for (int i = 0; i < 5; i++) {
			slash = strrchr(byid_target, '/');
			if (!slash)
				break;
			*slash = '\0';
			if (strcmp(byid_target, usb_root) == 0) {
				snprintf(evdev_path, evdev_sz,
					 "/dev/input/%s", evdev_name);
				snprintf(by_id, by_id_sz,
					 "/dev/input/by-id/%s", ent->d_name);
				rc = 0;
				goto out;
			}
		}
	}
out:
	closedir(dir);
	return rc;
}

int logitf_discover(void)
{
	DIR *dir;
	struct dirent *ent;
	int slot = 0;

	/* Idempotent: repeat scans refresh the table. */
	memset(g_table, 0, sizeof(g_table));
	for (int i = 0; i < LOGITF_MAX_CONTROLLERS; i++)
		g_table[i].hidraw_fd = g_table[i].evdev_fd = -1;

	dir = opendir("/sys/class/hidraw");
	if (!dir)
		return LOGITF_ERR_IO;

	while ((ent = readdir(dir)) && slot < LOGITF_MAX_CONTROLLERS) {
		uint16_t vid, pid;
		int ifnum;
		struct logitf_device *dev;
		char hidraw_sysdev[PATH_MAX];
		char linkpath[256];

		if (strncmp(ent->d_name, "hidraw", 6) != 0)
			continue;

		if (hidraw_usb_ids(ent->d_name, &vid, &pid, &ifnum) < 0)
			continue;
		if (!is_rs50_family(vid, pid))
			continue;
		if (ifnum != LOGITF_IFACE_TF)
			continue;

		dev = &g_table[slot];
		dev->vid = vid;
		dev->pid = pid;
		snprintf(dev->hidraw_path, sizeof(dev->hidraw_path),
			 "/dev/%s", ent->d_name);

		/* Resolve sysfs parent for evdev sibling search. */
		snprintf(linkpath, sizeof(linkpath),
			 "/sys/class/hidraw/%s/device", ent->d_name);
		if (realpath(linkpath, hidraw_sysdev))
			find_sibling_evdev(hidraw_sysdev,
					   dev->evdev_path, sizeof(dev->evdev_path),
					   dev->by_id, sizeof(dev->by_id));

		dev->in_use = true;
		dev->stream_timerfd = -1;
		dev->stream_stopfd = -1;
		dev->status_stopfd = -1;
		dev->kf_effect_id = -1;
		pthread_mutex_init(&dev->lock, NULL);
		pthread_mutex_init(&dev->ring_lock, NULL);
		pthread_cond_init(&dev->ring_space, NULL);
		pthread_cond_init(&dev->ring_data, NULL);
		slot++;
	}
	closedir(dir);

	g_discovered = true;
	return LOGITF_OK;
}

int logitf_find_by_index(int index, struct logitf_device **out)
{
	if (index < 0 || index >= LOGITF_MAX_CONTROLLERS)
		return LOGITF_ERR_INVALID_ARG;
	if (!g_discovered)
		logitf_discover();
	if (!g_table[index].in_use)
		return LOGITF_ERR_NOT_FOUND;
	if (out)
		*out = &g_table[index];
	return LOGITF_OK;
}
