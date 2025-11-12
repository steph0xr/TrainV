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
#include "dirDetector.h"

static const char *TAG = "dev";

#define EXAMPLE_PCNT_HIGH_LIMIT 30000
#define EXAMPLE_PCNT_LOW_LIMIT  -30000

#define EXAMPLE_EC11_GPIO_A 25
#define EXAMPLE_EC11_GPIO_B 26

#define MAX_REPS 50

static pcnt_unit_handle_t pcnt_unit = NULL;
        
static QueueHandle_t xQueue = NULL;

typedef struct {
        int pulse;
        int64_t time;
} point_t;

struct dev_config {
        int dt;
        int count_mt_coeff;
        int reset_timeout_seconds;
        int movement_tolerance_pulses; 
        int direction_hyst; 
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
        int64_t rep_start_time;
        int64_t rep_end_time;
        int64_t t_last;
        DirDetector d;
        int last_dir;
};

static const struct dev_config cfg = {
        .dt = 3,                      //ms
        .count_mt_coeff = 333*10,       // 200 count are 10 cm
        .reset_timeout_seconds = 4,
        .movement_tolerance_pulses = 4, 
        .last_rep_speed_threshold = 0.250,         //m/s
        .direction_hyst = 20, 
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
        .last_dir = 0,
};

static bool pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
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
                .on_reach = pcnt_on_reach,
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

void print_rep_speed_array()
{
        printf("[ ");
        for (int i = 0; i < state.reps_n; i++) { 
                printf("%.3f ", state.reps_speed[i]); 
        }
        printf("]\n");
}

void eval_elapsed_time()
{
        int64_t t_now = esp_timer_get_time();
        int64_t elapsed = t_now - state.t_last;
        state.t_last = t_now;
        if (elapsed > cfg.dt*1000+20)
                ESP_LOGE(TAG, "out of max elapse time: %lld us", elapsed);

        ESP_LOGV(TAG, "Periodic timer called, elapsed time from prev: %lld us", elapsed);
}

static void simple_periodic_timer_callback(void* arg)
{
        point_t p;

        p.time = esp_timer_get_time();
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &p.pulse));

        if (xQueueSend(xQueue, &p, pdMS_TO_TICKS(100)) == pdPASS) {
                /* printf("sending %lld - %d\n", p.time, p.pulse); */
        } else {
                ESP_LOGE(TAG, "Queue full, failed to send");
        }
}

void timer_init() 
{
        const esp_timer_create_args_t periodic_timer_args = {
                .callback = &simple_periodic_timer_callback,
                .name = "periodic encoder poll"
        };

        esp_timer_handle_t periodic_timer;
        ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
        ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, cfg.dt*1000));
}

void vConsumerTask(void *pvParameters)
{
        point_t p;
        for (;;) {
                /* Wait indefinitely for data to arrive */
                if (xQueueReceive(xQueue, &p, portMAX_DELAY) == pdPASS) {
                        printf("%lld us | %d cnt\n", 
                                        p.time - state.t_last, p.pulse);
                        state.t_last = p.time;
                }
        }
}

void app_main(void)
{
        esp_log_level_set(TAG, ESP_LOG_INFO);
        /* esp_log_level_set(TAG, ESP_LOG_DEBUG); */
        /* esp_log_level_set(TAG, ESP_LOG_VERBOSE); */

        xQueue = xQueueCreate(10, sizeof(point_t));

        timer_init();

        ESP_LOGI(TAG, "\n\nReady to Rock!\n");

        xTaskCreatePinnedToCore(vConsumerTask, "ConsumerTask",
                        4096, NULL, 2, NULL, 1); // run on core 1
}
