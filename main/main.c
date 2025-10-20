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

static const char *TAG = "Train";

#define EXAMPLE_PCNT_HIGH_LIMIT 10000
#define EXAMPLE_PCNT_LOW_LIMIT  -10000

#define EXAMPLE_EC11_GPIO_A 25
#define EXAMPLE_EC11_GPIO_B 26

static pcnt_unit_handle_t pcnt_unit = NULL;

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



void app_main(void)
{
        QueueHandle_t queue = init_encoder();

        /* esp_log_level_set(TAG, ESP_LOG_DEBUG); */

        // Report counter value
        int pulse_count = 0;
        int last_pulse_count = 0;
        int event_count = 0;
        int dt = 100; //ms
        int count_mt_coeff = 230*10;               // 200 count sono 10 cm
        int dp, stability_count = 0;
        float abs_s, rep_start_s, v, max_v_pos = 0, max_v_neg = 0, ds, rep_rom, max_neg_s = 0;

        int reset_timeout_seconds = 4;
        int movement_tolerance_pulses = 5; 
        bool rep_ongoing = false;


        while (1) {
                if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(dt))) {
                        /* ESP_LOGD(TAG, "Watch point event, count: %d", event_count); */
                } else {
                        last_pulse_count = pulse_count;
                        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));

                        // print ROM
                        if (last_pulse_count >= pulse_count + movement_tolerance_pulses ||                // isMovement
                            last_pulse_count <= pulse_count - movement_tolerance_pulses ) {
                                rep_ongoing = true;
                                abs_s = (float) pulse_count / (float) count_mt_coeff;
                                ESP_LOGD(TAG, "ROM = %.3f mt   [pulse count %d]", abs_s, pulse_count);

                                if (abs_s < max_neg_s)
                                        max_neg_s = abs_s;

                                stability_count = 0;
                        } else {
                                stability_count++;
                                if (rep_ongoing && stability_count > (reset_timeout_seconds * 1000/dt)) {               // isStable for enougth time
                                        rep_rom = max_neg_s - rep_start_s;
                                        printf("\n");
                                        ESP_LOGI(TAG, "Rep closed");
                                        ESP_LOGI(TAG, "start point %.3f m", rep_start_s);
                                        ESP_LOGI(TAG, "end point %.3f m", max_neg_s);
                                        ESP_LOGI(TAG, "ROM %.3f m", rep_rom);
                                        ESP_LOGI(TAG, "Max Speed Negative = %.3f m/s", max_v_neg);
                                        ESP_LOGI(TAG, "Max Speed Positive = %.3f m/s", max_v_pos);
                                        printf("\n");
                                        printf("\n");
                                        ESP_LOGI(TAG, "Stable condition.");
                                        ESP_LOGI(TAG, "Speed and ROM reset");
                                        stability_count = 0;
                                        max_v_neg = 0;
                                        max_v_pos = 0;
                                        max_neg_s = 0;
                                        rep_start_s = abs_s;
                                        ESP_LOGI(TAG, "new starting point %.3f m", rep_start_s);
                                        rep_ongoing = false;
                                }
                                
                        }

                        // speed eval
                        dp = pulse_count - last_pulse_count;
                        ds = (float) dp / (float) count_mt_coeff;
                        v = (float) ds / ((float) dt / 1000); 
                        if (ds < 0)
                                if (v < max_v_neg) {
                                        /* ESP_LOGD(TAG, "new max speed negative %.3f", v); */
                                        max_v_neg = v;
                                }

                        if (ds > 0)
                                if (v > max_v_pos) {
                                        /* ESP_LOGD(TAG, "new max speed positive %.3f", v); */
                                        max_v_pos = v;
                                }
                }
        }
}
