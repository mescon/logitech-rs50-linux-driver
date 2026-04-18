/*
 * libtrueforce - private state.
 */

#ifndef LIBTRUEFORCE_INTERNAL_H
#define LIBTRUEFORCE_INTERNAL_H

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>

#include "trueforce.h"

#define LOGITF_LOGI_VID 0x046D
#define LOGITF_RS50_PID 0xC276
#define LOGITF_IFACE_TF 2

struct logitf_device {
	bool in_use;

	/* Identity */
	uint16_t vid;
	uint16_t pid;
	char hidraw_path[64];      /* /dev/hidrawN, interface 2 */
	char evdev_path[128];      /* /dev/input/eventN, joystick */
	char by_id[256];           /* /dev/input/by-id/... for stable identity */

	/* File descriptors (open on first use) */
	int hidraw_fd;             /* TF audio stream */
	int evdev_fd;              /* KF constant force via input_ff */

	/* Session state */
	bool tf_initialized;       /* Init sequence sent since open */
	bool tf_paused;

	pthread_mutex_t lock;      /* Protects mutable state */
};

struct logitf_device *logitf_table(void);

/* discovery.c */
int logitf_discover(void);        /* Scan sysfs, populate the table. Idempotent. */
int logitf_find_by_index(int index, struct logitf_device **out);

#endif /* LIBTRUEFORCE_INTERNAL_H */
