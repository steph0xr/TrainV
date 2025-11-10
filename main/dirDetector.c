#include <stdio.h>
#include <stdlib.h>
#include "dirDetector.h"
#include "esp_timer.h"


void dir_detector_init(DirDetector *d, int hysteresis)
{
        d->last_pos = 0;
        d->direction = 0;
        d->hysteresis = hysteresis;
        d->pending_dir = 0;
        d->pending_count = 0;
        d->start_pos = 0;
        d->first_pending_pos = 0;
}

int get_start_pos(DirDetector *d)
{
        return d->start_pos;
}

int dir_detector_update(DirDetector *d, int current_pos)
{
        /* printf("dir_detector_update"); */
        int delta = current_pos - d->last_pos;
        d->last_pos = current_pos;

        int new_dir = (delta > 0) ? +1 : (delta < 0) ? -1 : 0;

        if (new_dir == 0)
                return d->direction;  // no movement

        if (new_dir == d->direction) {
                // stable same direction
                d->pending_count = 0;
                d->pending_dir = 0;
        } else {
                // possible direction change
                if (new_dir == d->pending_dir) {
                        d->pending_count++;
                        /* printf("d->pending_count: %d\n", d->pending_count); */
                        if (d->pending_count >= d->hysteresis) {
                                /* printf("d->pending_count >= d->hysteresis %d\n",  */
                                                /* d->hysteresis); */
                                d->direction = new_dir;  // confirmed change
                                d->pending_count = 0;
                                d->pending_dir = 0;
                                d->start_pos = d->first_pending_pos;  // store start of this new direction
                        }
                } else {
                        d->pending_dir = new_dir;
                        d->pending_count = 1;
                        d->first_pending_pos = current_pos;  // first value of this potential group
                        d->start_pos_time = esp_timer_get_time();
                }
        }

        return d->direction;
}
