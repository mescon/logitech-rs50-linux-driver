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

#define LOGITF_TF_WINDOW  13          /* samples per packet (rolling window) */
#define LOGITF_TF_NEW     4           /* new samples added per packet */
#define LOGITF_TF_RING    4096        /* sample ring capacity (must be pow2) */
#define LOGITF_TF_PKT_HZ  250         /* packets per second (1000 Hz / 4 new) */

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

	/* KF state */
	int kf_effect_id;
	bool kf_playing;
	double kf_last_nm;

	/* Session state */
	bool tf_initialized;       /* Init sequence sent since open */
	bool tf_paused;
	uint8_t tf_seq;            /* next outgoing packet sequence byte */

	/* Streaming state (managed by stream.c) */
	bool stream_running;
	pthread_t stream_thread;
	int stream_timerfd;
	int stream_stopfd;         /* eventfd; signals the thread to exit */

	uint16_t tf_window[LOGITF_TF_WINDOW]; /* offset-binary, newest at [WINDOW-1] */
	uint16_t tf_last_current;             /* bytes 6-9 of each packet */

	pthread_mutex_t ring_lock;
	pthread_cond_t  ring_space;
	pthread_cond_t  ring_data;
	uint16_t ring[LOGITF_TF_RING];        /* offset-binary samples */
	unsigned ring_head;                    /* producer index (mod RING) */
	unsigned ring_tail;                    /* consumer index (mod RING) */

	pthread_mutex_t lock;      /* Protects mutable non-ring state */
};

struct logitf_device *logitf_table(void);

/* discovery.c */
int logitf_discover(void);        /* Scan sysfs, populate the table. Idempotent. */
int logitf_find_by_index(int index, struct logitf_device **out);

/* session.c */
int logitf_session_ensure(struct logitf_device *dev);
int logitf_session_close(struct logitf_device *dev);

/* stream.c */
int  logitf_stream_start(struct logitf_device *dev);
int  logitf_stream_stop(struct logitf_device *dev);
int  logitf_stream_push_s16(struct logitf_device *dev, const int16_t *samples, int count);
int  logitf_stream_clear(struct logitf_device *dev);

/* kf.c */
int    logitf_kf_set_torque_nm(struct logitf_device *dev, double torque_nm);
int    logitf_kf_clear(struct logitf_device *dev);
int    logitf_kf_close(struct logitf_device *dev);
double logitf_kf_get_torque_nm(struct logitf_device *dev);
double logitf_kf_max_continuous_nm(void);
double logitf_kf_max_peak_nm(void);

/* Helper: convert float [-1.0, 1.0] to offset-binary u16. */
uint16_t logitf_float_to_wire(float sample);
uint16_t logitf_s16_to_wire(int16_t sample);

#endif /* LIBTRUEFORCE_INTERNAL_H */
