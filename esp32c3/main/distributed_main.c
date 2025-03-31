/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"

#include "pins.h"
#include "sensor.h"
#include "motor.h"
#include "pid.h"

SemaphoreHandle_t setpoint_mutex;

static PID_VAL_TYPE setpoint;
static PID_VAL_TYPE setpoint_min;
static PID_VAL_TYPE setpoint_max;

typedef union {
    uint8_t bytes[sizeof(PID_VAL_TYPE)];
    PID_VAL_TYPE new_setpoint;
} SetpointFloat;

/*
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{

}

static void espnow_recv_cb(const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int data_len)
{
    SetpointFloat setpoint_recv;
    memcpy(setpoint_recv.bytes, data, sizeof(PID_VAL_TYPE));
}
*/

static void setpoint_update_task(void * arg)
{
    PID_VAL_TYPE setpoint_recv;
    while(1)
    {
        if (xQueueReceive(setpoint_mutex, &setpoint_recv, portMAX_DELAY))
        {
            if (xSemaphoreTake(setpoint_mutex, portMAX_DELAY))
            {
                setpoint = setpoint_recv < setpoint_min ? setpoint_min : (setpoint_recv > setpoint_max ? setpoint_max : setpoint_recv);
                xSemaphoreGive(setpoint_mutex);
            }
        }
    }
}

static void app_task(void * arg)
{
    TickType_t prev_wake_time = xTaskGetTickCount();
    const TickType_t task_freq = pdMS_TO_TICKS(10); // TODO tune this, probably based on sensor sampling speed
    while(1)
    {
        if (xSemaphoreTake(setpoint_mutex, portMAX_DELAY))
        {
            motor_set_command(calc_pid(setpoint, get_sensor_val()));
        }
        xTaskDelayUntil(&prev_wake_time, task_freq);
    }
}

void gpio_setup()
{
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM); // TODO figure out IRAM stuff

    sensor_gpio_setup();
    motor_gpio_setup();
}

void task_setup()
{
    // espnow_init(espnow_send_cb, espnow_recv_cb);
    sensor_task_setup();
    motor_task_setup();

    // TODO tune task parameters
    xTaskCreate(app_task, "App Task", 512, NULL, 10, NULL);
}

void app_main(void)
{
    gpio_setup();
    task_setup();
}
