#ifndef _DIR_DETECTOR_H_
#define _DIR_DETECTOR_H_

#include "esp_timer.h"

typedef struct {
        int last_pos;
        int direction;          // +1 = forward, -1 = reverse, 0 = unknown
        int hysteresis;         // number of steps needed to confirm a change
        int pending_dir;        // direction currently being tested
        int pending_count;      // how many consecutive steps seen in that direction
        int start_pos;          // position at start of confirmed direction
        int first_pending_pos;
        int64_t start_pos_time;
} DirDetector;

void dir_detector_init(DirDetector *d, int hysteresis);

int dir_detector_update(DirDetector *d, int current_pos);

int get_start_pos(DirDetector *d);

#endif /*_DIR_DETECTOR_H_*/
