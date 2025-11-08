/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#include <math.h>
#include "esp_timer.h"

static const char *TAG = "dev";

#define EXAMPLE_PCNT_HIGH_LIMIT 30000
#define EXAMPLE_PCNT_LOW_LIMIT  -30000

#define EXAMPLE_EC11_GPIO_A 25
#define EXAMPLE_EC11_GPIO_B 26

#define MAX_REPS 50

static pcnt_unit_handle_t pcnt_unit = NULL;

struct dev_config {
        int dt;
        int count_mt_coeff;
        int reset_timeout_seconds;
        int movement_tolerance_pulses; 
        float last_rep_speed_threshold;
};

struct dev_state {
        int pulse_count;
        int last_pulse_count;
        int event_count;
        int dp; 
        int last_dp; 
        int stability_count;
        float abs_s; 
        float rep_start_s; 
        float v; 
        float max_v_pos; 
        float max_v_neg; 
        float ds; 
        float rep_rom; 
        float max_neg_s;
        int reps_n;
        bool series_ongoing;
        bool rep_ongoing;
        bool direction_switched;
        float reps_speed[MAX_REPS];
        int64_t ms_start;
};

static const struct dev_config cfg = {
        .dt = 100,                      //ms
        .count_mt_coeff = 333*10,       // 200 count are 10 cm
        .reset_timeout_seconds = 4,
        .movement_tolerance_pulses = 3, 
        .last_rep_speed_threshold = 0.025,         //m/s
};

static struct dev_state state = {
        .pulse_count = 0,
        .last_pulse_count = 0,
        .event_count = 0,
        .dp = 0, 
        .last_dp = 0, 
        .stability_count = 0,
        .abs_s = 0, 
        .rep_start_s = 0, 
        .v = 0, 
        .max_v_pos = 0, 
        .max_v_neg = 0, 
        .ds = 0, 
        .rep_rom = 0, 
        .max_neg_s = 0,
        .series_ongoing = false,
        .rep_ongoing = false,
        .reps_n = 0,
        .direction_switched = false,
};

static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
        BaseType_t high_task_wakeup;
        QueueHandle_t queue = (QueueHandle_t)user_ctx;
        // send event data to queue, from this interrupt callback
        xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
        return (high_task_wakeup == pdTRUE);
}

QueueHandle_t init_encoder()
{
        ESP_LOGI(TAG, "install pcnt unit");
        pcnt_unit_config_t unit_config = {
                .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
                .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
        };
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

        ESP_LOGI(TAG, "set glitch filter");
        pcnt_glitch_filter_config_t filter_config = {
                .max_glitch_ns = 1000,
        };
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

        ESP_LOGI(TAG, "install pcnt channels");
        pcnt_chan_config_t chan_a_config = {
                .edge_gpio_num = EXAMPLE_EC11_GPIO_A,
                .level_gpio_num = EXAMPLE_EC11_GPIO_B,
        };
        pcnt_channel_handle_t pcnt_chan_a = NULL;
        ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
        pcnt_chan_config_t chan_b_config = {
                .edge_gpio_num = EXAMPLE_EC11_GPIO_B,
                .level_gpio_num = EXAMPLE_EC11_GPIO_A,
        };
        pcnt_channel_handle_t pcnt_chan_b = NULL;
        ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

        ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

        ESP_LOGI(TAG, "add watch points and register callbacks");
        int watch_points[] = {EXAMPLE_PCNT_LOW_LIMIT, -1000, 0, 1000, EXAMPLE_PCNT_HIGH_LIMIT};
        for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
                ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
        }
        pcnt_event_callbacks_t cbs = {
                .on_reach = example_pcnt_on_reach,
        };
        QueueHandle_t queue = xQueueCreate(10, sizeof(int));
        ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));

        ESP_LOGI(TAG, "enable pcnt unit");
        ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
        ESP_LOGI(TAG, "clear pcnt unit");
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
        ESP_LOGI(TAG, "start pcnt unit");
        ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

#if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
        // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
        ESP_ERROR_CHECK(gpio_wakeup_enable(EXAMPLE_EC11_GPIO_A, GPIO_INTR_LOW_LEVEL));
        ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
        ESP_ERROR_CHECK(esp_light_sleep_start());
#endif
        return queue;
}

bool is_in_movement()
{
        bool mov;

        mov = state.last_pulse_count >= 
               state.pulse_count + cfg.movement_tolerance_pulses ||
               state.last_pulse_count <= 
               state.pulse_count - cfg.movement_tolerance_pulses; 

        return mov;
}

bool is_stable()
{
        return state.series_ongoing && state.stability_count > (cfg.reset_timeout_seconds * 1000/cfg.dt);
}

void reset_state()
{
        ESP_LOGI(TAG, "Speed and ROM reset");
        state.stability_count = 0;
        state.max_v_neg = 0;
        state.max_v_pos = 0;
        state.max_neg_s = 0;
        state.reps_n = 0;
}

void print_results()
{
        printf("\n");
        ESP_LOGI(TAG, "Rep closed");
        ESP_LOGI(TAG, "start point %.3f m", state.rep_start_s);
        ESP_LOGI(TAG, "end point %.3f m", state.max_neg_s);
        ESP_LOGI(TAG, "ROM %.3f m", state.rep_rom);
        ESP_LOGI(TAG, "Max Speed Negative = %.3f m/s", state.max_v_neg);
        ESP_LOGI(TAG, "Max Speed Positive = %.3f m/s", state.max_v_pos);
        printf("\n");
        printf("\n");
        ESP_LOGI(TAG, "Stable condition.");
}

void evaluate_speed_decrement()
{
        float last_speed_decrement,
              decrement_from_start;

        if (state.reps_n < 2)
                return;  // not enought elements to evaluate decrement 

        int idx = state.reps_n - 1;

        last_speed_decrement = ( 100 * state.reps_speed[idx] / state.reps_speed[idx - 1] ) - 100;

        decrement_from_start = ( 100 * state.reps_speed[idx] / state.reps_speed[0] ) - 100;

        ESP_LOGI(TAG, "speed decrement -> last: %d %%, from start: %d %%", 
                 (int)last_speed_decrement, (int)decrement_from_start);
        
        if (state.reps_speed[state.reps_n] <= cfg.last_rep_speed_threshold)
                ESP_LOGW(TAG, 
                         "LOW SPEED LIMIT REACHED (%.3f (limit:%.3f) m/s)", 
                         state.reps_speed[state.reps_n], 
                         cfg.last_rep_speed_threshold);
}

void print_rep_speed_array()
{
        printf("[ ");
        for (int i = 0; i < state.reps_n; i++) { 
                printf("%.3f ", state.reps_speed[i]); 
        }
        printf("]\n");
}

void store_rep_speed(int rep, float v) 
{
        state.reps_speed[rep - 1] = v;
        print_rep_speed_array();
        evaluate_speed_decrement();
}

void check_if_new_max_speed_reached()
{
        ESP_LOGD(TAG, "ds: %.3f, dp: %d, pulse: %d, last pulse: %d", 
                 state.ds, state.dp, state.pulse_count, state.last_pulse_count);

        if (state.dp < 0 &&
            abs(state.dp) > cfg.movement_tolerance_pulses) {
                if (state.direction_switched) {
                        /* ESP_LOGD(TAG, "new max speed negative %.3f", state.v); */
                        state.reps_n++;
                        state.ms_start = esp_timer_get_time() / 1000;
                }

                state.rep_ongoing = true;

                if (state.v < state.max_v_neg) {
                        state.max_v_neg = state.v;
                }
        }

        if (state.dp > 0 && 
            abs(state.dp) > cfg.movement_tolerance_pulses) {
                if (state.direction_switched) {
                        ESP_LOGI(TAG, "rep n.%d speed: %.3f", state.reps_n, state.v);
                        state.rep_rom = state.max_neg_s - state.rep_start_s;
                        int64_t ms_end = esp_timer_get_time() / 1000;
                        int64_t delta_t =  ms_end - state.ms_start;
                        float v = state.rep_rom / delta_t;
                        store_rep_speed(state.reps_n, v);
                }

                state.rep_ongoing = false;

                if (state.v > state.max_v_pos) {
                        /* ESP_LOGD(TAG, "new max speed positive %.3f", v); */
                        state.max_v_pos = state.v;
                }
        }
}

void evaluate_direction_switched()
{
        if ((state.last_dp <= 0 && state.dp > 0) || 
            (state.last_dp >= 0 && state.dp < 0)) {
                state.direction_switched = true;
                ESP_LOGD(TAG, "direction switched. last dp: %d, dp: %d",
                         state.last_dp, state.dp);
        } else 
                state.direction_switched = false;
}

void speed_evaluation()
{
        state.last_dp = state.dp;
        state.dp = state.pulse_count - state.last_pulse_count;
        state.ds = (float) state.dp / (float) cfg.count_mt_coeff;

        evaluate_direction_switched();

        state.v = (float) state.ds / ((float) cfg.dt / 1000); 

        check_if_new_max_speed_reached();
}


void periodic_pulse_count_evaluation()
{
        state.last_pulse_count = state.pulse_count;
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &state.pulse_count));

        if (is_in_movement()) {
                state.series_ongoing = true;
                state.stability_count = 0;

                state.abs_s = (float) state.pulse_count / (float) cfg.count_mt_coeff;

                ESP_LOGD(TAG, "ROM = %.3f mt   [pulse count %d]", state.abs_s, state.pulse_count);

                if (state.abs_s < state.max_neg_s)
                        state.max_neg_s = state.abs_s;

        } else {
                state.stability_count++;

                if (is_stable()) {               // is stable for enougth time
                        state.rep_rom = state.max_neg_s - state.rep_start_s;
                        print_results();
                        reset_state();
                        state.rep_start_s = state.abs_s;
                        ESP_LOGI(TAG, "new starting point %.3f m", state.rep_start_s);
                        state.series_ongoing = false;
                }

        }

        speed_evaluation();
}



void app_main(void)
{
        QueueHandle_t queue = init_encoder();

        /* esp_log_level_set(TAG, ESP_LOG_INFO); */
        esp_log_level_set(TAG, ESP_LOG_DEBUG);

        ESP_LOGI(TAG, "\n\nReady to Rock!\n");

        while (1) {
                if (xQueueReceive(queue, &state.event_count, pdMS_TO_TICKS(cfg.dt))) {
                        /* ESP_LOGD(TAG, "Watch point event, count: %d", event_count); */
                } else {
                        periodic_pulse_count_evaluation();
                }
        }
}
