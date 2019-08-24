/*
 * Copyright (c) 2015,2018 The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * *    * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define LOG_NIDEBUG 0

#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <stdlib.h>
#include <pthread.h>

#define LOG_TAG "QTI PowerHAL"
#include <utils/Log.h>
#include <hardware/hardware.h>
#include <hardware/power.h>

#include "utils.h"
#include "metadata-defs.h"
#include "hint-data.h"
#include "performance.h"
#include "power-common.h"

#define MIN_VAL(X,Y) ((X>Y)?(Y):(X))

static int saved_interactive_mode = -1;
static int display_hint_sent;
static int video_encode_hint_sent;
pthread_mutex_t camera_hint_mutex = PTHREAD_MUTEX_INITIALIZER;
static int camera_hint_ref_count;
static void process_video_encode_hint(void *metadata);

/* Sustained performance mode */
#define CHECK_HANDLE(x) ((x)>0)
#define NUM_PERF_MODES  3

typedef enum {
    NORMAL_MODE       = 0,
    SUSTAINED_MODE    = 1,
    VR_MODE           = 2,
    VR_SUSTAINED_MODE = (SUSTAINED_MODE|VR_MODE),
    INVALID_MODE      = 0xFF
} perf_mode_type_t;

typedef struct perf_mode {
    perf_mode_type_t type;
    int perf_hint_id;
} perf_mode_t;

perf_mode_t perf_modes[NUM_PERF_MODES] = {
    { SUSTAINED_MODE, SUSTAINED_PERF_HINT },
    { VR_MODE, VR_MODE_HINT },
    { VR_SUSTAINED_MODE, VR_MODE_SUSTAINED_PERF_HINT }
};

static int current_mode = NORMAL_MODE;

static inline  int get_perfd_hint_id(perf_mode_type_t type) {
    int i;
    for (i = 0; i < NUM_PERF_MODES; i++) {
        if (perf_modes[i].type == type) {
            ALOGD("Hint id is 0x%x for mode 0x%x", perf_modes[i].perf_hint_id, type);
            return perf_modes[i].perf_hint_id;
        }
    }
    ALOGD("Couldn't find the hint for mode 0x%x", type);
    return 0;
}

static int switch_mode(perf_mode_type_t mode) {

    int hint_id = 0;
    static int perfd_mode_handle = -1;

    // release existing mode if any
    if (CHECK_HANDLE(perfd_mode_handle)) {
        ALOGD("Releasing handle 0x%x", perfd_mode_handle);
        release_request(perfd_mode_handle);
        perfd_mode_handle = -1;
    }
    // switch to a perf mode
    hint_id = get_perfd_hint_id(mode);
    if (hint_id != 0) {
        perfd_mode_handle = perf_hint_enable(hint_id, 0);
        if (!CHECK_HANDLE(perfd_mode_handle)) {
            ALOGE("Failed perf_hint_interaction for mode: 0x%x", mode);
            return -1;
        }
        ALOGD("Acquired handle 0x%x", perfd_mode_handle);
    }
    return 0;
}

static int process_perf_hint(void *data, perf_mode_type_t mode) {

    // enable
    if (*(int32_t *)data){
        ALOGI("Enable request for mode: 0x%x", mode);
        // check if mode is current mode
        if ( current_mode & mode ) {
            ALOGD("Mode 0x%x already enabled", mode);
            return HINT_HANDLED;
        }
        // enable requested mode
        if ( 0 != switch_mode(current_mode | mode)) {
            ALOGE("Couldn't enable mode 0x%x", mode);
            return HINT_NONE;
        }
        current_mode |= mode;
        ALOGI("Current mode is 0x%x", current_mode);
    // disable
    } else {
        ALOGI("Disable request for mode: 0x%x", mode);
        // check if mode is enabled
        if ( !(current_mode & mode) ) {
            ALOGD("Mode 0x%x already disabled", mode);
            return HINT_HANDLED;
        }
        //disable requested mode
        if ( 0 != switch_mode(current_mode & ~mode)) {
            ALOGE("Couldn't disable mode 0x%x", mode);
            return HINT_NONE;
        }
        current_mode &= ~mode;
        ALOGI("Current mode is 0x%x", current_mode);
    }

    return HINT_HANDLED;
}

/* SDM 439 */
static bool is_target_SDM439() /* Returns value=1 if target is Hathi else value 0 */
{
    int fd;
    bool is_target_SDM439 = false;
    char buf[10] = {0};
    fd = open("/sys/devices/soc0/soc_id", O_RDONLY);
    if (fd >= 0) {
        if (read(fd, buf, sizeof(buf) - 1) == -1) {
            ALOGW("Unable to read soc_id");
            is_target_SDM439 = false;
        } else {
            int soc_id = atoi(buf);
            if (soc_id == 353 || soc_id == 363 || soc_id == 354 || soc_id == 364) {
                is_target_SDM439 = true; /* Above SOCID for SDM439/429 */
            }
        }
    }
    close(fd);
    return is_target_SDM439;
}

int  power_hint_override(struct power_module *module, power_hint_t hint,
        void *data)
{

    int ret_val = HINT_NONE;
    switch(hint) {
        case POWER_HINT_VSYNC:
            break;
        case POWER_HINT_SUSTAINED_PERFORMANCE:
            ret_val = process_perf_hint(data, SUSTAINED_MODE);
            break;
        case POWER_HINT_VR_MODE:
            ret_val = process_perf_hint(data, VR_MODE);
            break;
        case POWER_HINT_VIDEO_ENCODE:
        {
            process_video_encode_hint(data);
            return HINT_HANDLED;
        }
    }
    return ret_val;
}

int  set_interactive_override(struct power_module *module, int on)
{
    char governor[80];
    char tmp_str[NODE_MAX];
    struct video_encode_metadata_t video_encode_metadata;
    int rc;

    ALOGI("Got set_interactive hint");

    if (get_scaling_governor_check_cores(governor, sizeof(governor),CPU0) == -1) {
        if (get_scaling_governor_check_cores(governor, sizeof(governor),CPU1) == -1) {
            if (get_scaling_governor_check_cores(governor, sizeof(governor),CPU2) == -1) {
                if (get_scaling_governor_check_cores(governor, sizeof(governor),CPU3) == -1) {
                    ALOGE("Can't obtain scaling governor.");
                    return HINT_HANDLED;
                }
            }
        }
    }

    if (!on) {
        /* Display off. */
             if ((strncmp(governor, INTERACTIVE_GOVERNOR, strlen(INTERACTIVE_GOVERNOR)) == 0) &&
                (strlen(governor) == strlen(INTERACTIVE_GOVERNOR))) {
               int resource_values[] = {INT_OP_CLUSTER0_TIMER_RATE, BIG_LITTLE_TR_MS_50,
                                        INT_OP_CLUSTER1_TIMER_RATE, BIG_LITTLE_TR_MS_50,
                                        INT_OP_NOTIFY_ON_MIGRATE, 0x00};

               if (!display_hint_sent) {
                   perform_hint_action(DISPLAY_STATE_HINT_ID,
                   resource_values, sizeof(resource_values)/sizeof(resource_values[0]));
                  display_hint_sent = 1;
                }
             } /* Perf time rate set for CORE0,CORE4 8952 target*/

    } else {
        /* Display on. */
          if ((strncmp(governor, INTERACTIVE_GOVERNOR, strlen(INTERACTIVE_GOVERNOR)) == 0) &&
                (strlen(governor) == strlen(INTERACTIVE_GOVERNOR))) {

             undo_hint_action(DISPLAY_STATE_HINT_ID);
             display_hint_sent = 0;
          }
   }
    saved_interactive_mode = !!on;

    return HINT_HANDLED;
}

/* Video Encode Hint */
static void process_video_encode_hint(void *metadata)
{
    char governor[80] = {0};
    int resource_values[20] = {0};
    int num_resources = 0;
    struct video_encode_metadata_t video_encode_metadata;

    ALOGI("Got process_video_encode_hint");

    if (get_scaling_governor_check_cores(governor,
        sizeof(governor),CPU0) == -1) {
            if (get_scaling_governor_check_cores(governor,
                sizeof(governor),CPU1) == -1) {
                    if (get_scaling_governor_check_cores(governor,
                        sizeof(governor),CPU2) == -1) {
                            if (get_scaling_governor_check_cores(governor,
                                sizeof(governor),CPU3) == -1) {
                                    ALOGE("Can't obtain scaling governor.");
                                    return;
                            }
                    }
            }
    }

    /* Initialize encode metadata struct fields. */
    memset(&video_encode_metadata, 0, sizeof(struct video_encode_metadata_t));
    video_encode_metadata.state = -1;
    video_encode_metadata.hint_id = DEFAULT_VIDEO_ENCODE_HINT_ID;

    if (metadata) {
        if (parse_video_encode_metadata((char *)metadata,
            &video_encode_metadata) == -1) {
            ALOGE("Error occurred while parsing metadata.");
            return;
        }
    } else {
        return;
    }

    if (video_encode_metadata.state == 1) {
        if((strncmp(governor, SCHEDUTIL_GOVERNOR,
            strlen(SCHEDUTIL_GOVERNOR)) == 0) &&
            (strlen(governor) == strlen(SCHEDUTIL_GOVERNOR))) {
            if(is_target_SDM439()) {
                /* sample_ms = 10mS
                * SLB for Core0 = -6
                * SLB for Core1 = -6
                * SLB for Core2 = -6
                * SLB for Core3 = -6
                * hispeed load = 95
                * hispeed freq = 998Mhz */
                int res[] = {0x41820000, 0xa,
                             0x40c68100, 0xfffffffa,
                             0x40c68110, 0xfffffffa,
                             0x40c68120, 0xfffffffa,
                             0x40c68130, 0xfffffffa,
                             0x41440100, 0x5f,
                             0x4143c100, 0x3e6,
                             };
                memcpy(resource_values, res, MIN_VAL(sizeof(resource_values), sizeof(res)));
                num_resources = sizeof(res)/sizeof(res[0]);
                pthread_mutex_lock(&camera_hint_mutex);
                camera_hint_ref_count++;
                if (camera_hint_ref_count == 1) {
                    if (!video_encode_hint_sent) {
                        perform_hint_action(video_encode_metadata.hint_id,
                        resource_values, num_resources);
                        video_encode_hint_sent = 1;
                    }
                }
                pthread_mutex_unlock(&camera_hint_mutex);
            }
            else {
                /* sample_ms = 10mS */
                int res[] = {0x41820000, 0xa,
                            };
                memcpy(resource_values, res, MIN_VAL(sizeof(resource_values), sizeof(res)));
                num_resources = sizeof(res)/sizeof(res[0]);
                pthread_mutex_lock(&camera_hint_mutex);
                camera_hint_ref_count++;
                if (camera_hint_ref_count == 1) {
                    if (!video_encode_hint_sent) {
                        perform_hint_action(video_encode_metadata.hint_id,
                        resource_values, num_resources);
                        video_encode_hint_sent = 1;
                    }
                }
                pthread_mutex_unlock(&camera_hint_mutex);
            }
        }
        else if ((strncmp(governor, INTERACTIVE_GOVERNOR,
            strlen(INTERACTIVE_GOVERNOR)) == 0) &&
            (strlen(governor) == strlen(INTERACTIVE_GOVERNOR))) {
           /* Sched_load and migration_notif*/
            int res[] = {INT_OP_CLUSTER0_USE_SCHED_LOAD,
                         0x1,
                         INT_OP_CLUSTER1_USE_SCHED_LOAD,
                         0x1,
                         INT_OP_CLUSTER0_USE_MIGRATION_NOTIF,
                         0x1,
                         INT_OP_CLUSTER1_USE_MIGRATION_NOTIF,
                         0x1,
                         INT_OP_CLUSTER0_TIMER_RATE,
                         BIG_LITTLE_TR_MS_40,
                         INT_OP_CLUSTER1_TIMER_RATE,
                         BIG_LITTLE_TR_MS_40
                         };
            memcpy(resource_values, res, MIN_VAL(sizeof(resource_values), sizeof(res)));
            num_resources = sizeof(res)/sizeof(res[0]);
            pthread_mutex_lock(&camera_hint_mutex);
            camera_hint_ref_count++;
            if (!video_encode_hint_sent) {
                perform_hint_action(video_encode_metadata.hint_id,
                resource_values,num_resources);
                video_encode_hint_sent = 1;
            }
            pthread_mutex_unlock(&camera_hint_mutex);
        }
    } else if (video_encode_metadata.state == 0) {
        if (((strncmp(governor, INTERACTIVE_GOVERNOR,
            strlen(INTERACTIVE_GOVERNOR)) == 0) &&
            (strlen(governor) == strlen(INTERACTIVE_GOVERNOR))) ||
            ((strncmp(governor, SCHEDUTIL_GOVERNOR,
            strlen(SCHEDUTIL_GOVERNOR)) == 0) &&
            (strlen(governor) == strlen(SCHEDUTIL_GOVERNOR)))) {
            pthread_mutex_lock(&camera_hint_mutex);
            camera_hint_ref_count--;
            if (!camera_hint_ref_count) {
                undo_hint_action(video_encode_metadata.hint_id);
                video_encode_hint_sent = 0;
            }
            pthread_mutex_unlock(&camera_hint_mutex);
            return ;
        }
    }
    return;
}

