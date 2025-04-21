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
#include "esp_task_wdt.h"

#include "pins.h"
#include "sensor.h"
#include "motor.h"
#include "pid.h"

bool limit_hit;

QueueHandle_t setpoint_q;
// for now, set setpoint between -100 to 100 percent
static const PID_VAL_TYPE setpoint_min = -100.0f;
static PID_VAL_TYPE setpoint_max = 100.0f;

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

#ifdef CONFIG_I2C_SENSOR
#define BLINKING_PIN PWM_INPUT
#elif CONFIG_PWM_SENSOR
#define BLINKING_PIN SCL_IO_PIN
#endif

static void test_task(void * arg)
{
    PID_VAL_TYPE cmd = 0.0f;
    PID_VAL_TYPE inc = 50.0f / 5.0f / 300.0f;
    TickType_t prev_wake_time = xTaskGetTickCount();
    const TickType_t task_freq = 1;
    while(1)
    {
        if (cmd > 50.0f) cmd = -50.0f;
        // motor_set_command(cmd);
        // cmd += inc;
        xQueueOverwrite(setpoint_q, &cmd);
        xTaskDelayUntil(&prev_wake_time, task_freq);
    }
}

static void test_task2(void * arg)
{
    uint8_t curr_level = 0;
    while(1)
    {
        gpio_set_level(BLINKING_PIN, curr_level);
        curr_level = curr_level ^ 1;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// static void setpoint_update_task(void * arg)
// {
//     PID_VAL_TYPE setpoint_recv;
//     while(1)
//     {
//         if (xQueueReceive(setpoint_q, &setpoint_recv, portMAX_DELAY))
//         {
//             if (xSemaphoreTake(setpoint_mutex, portMAX_DELAY))
//             {
//                 setpoint = setpoint_recv < setpoint_min ? setpoint_min : (setpoint_recv > setpoint_max ? setpoint_max : setpoint_recv);
//                 xSemaphoreGive(setpoint_mutex);
//             }
//         }
//     }
// }

static void limit_switch_task(void * arg)
{
    esp_task_wdt_delete(NULL); // ensure the watchdog doesn't trigger from long timeouts
    uint32_t limit_notif;
    limit_notif_t limit_switch_event;
    BaseType_t res;
    // TODO wait for 2 seconds for limit switch to be released - is this reasonable?
    TickType_t notif_check_period = pdMS_TO_TICKS(2000);
    while(1)
    {
        res = xTaskNotifyWait(pdFALSE, ULONG_MAX, &limit_notif, notif_check_period);
        if (res == pdFALSE)
        {
            if (limit_hit) {
                // TODO error handling when there is a timeout while waiting - indicates switch is still pressed for some reason
                start_limit_protocol(); // TODO assume the issue is with the motor module
            }
            continue;
        }

        limit_switch_event = (limit_notif_t)limit_notif;
        switch (limit_switch_event)
        {
        // interrupt driven
        case UPPER_PRESSED:
            limit_hit = true;
            set_upper_limit();
            start_limit_protocol();
            break;
        case LOWER_PRESSED:
            limit_hit = true;
            set_lower_limit();
            start_limit_protocol();
            break;
        // notifications after protocol
        case RELEASED:
            limit_hit = false;
            break;
        default:
        // TODO error handling
            break;
        }
    }
}

static void app_task(void * arg)
{
    PID_VAL_TYPE setpoint_recv;
    TickType_t prev_wake_time = xTaskGetTickCount();
    const TickType_t task_freq = 3; // TODO tune this, probably based on sensor sampling speed
    while(1)
    {
        if (!limit_hit)
        {
            if (xQueueReceive(setpoint_q, &setpoint_recv, 2))
            {
                setpoint_recv = setpoint_recv < setpoint_min ? setpoint_min : (setpoint_recv > setpoint_max ? setpoint_max : setpoint_recv);
            }
            // TODO for now, receiving mock PID outputs on setpoint_q, not setpoints
            motor_set_command(setpoint_recv);
            // TODO full motor command step - commented for now
            // motor_set_command(calc_pid(setpoint_recv, get_sensor_val()));
        }
        xTaskDelayUntil(&prev_wake_time, task_freq);
    }
}

void gpio_setup()
{
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM); // TODO figure out IRAM stuff

    sensor_gpio_setup();
    motor_gpio_setup();

    // TEST
    gpio_config_t handshake_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(BLINKING_PIN)
    };

    gpio_config(&handshake_conf);
}

void task_setup()
{
    // espnow_init(espnow_send_cb, espnow_recv_cb);
    sensor_task_setup();
    motor_task_setup();

    // TODO tune task parameters
    TaskHandle_t limit_t;
    setpoint_q = xQueueCreate(1, sizeof(PID_VAL_TYPE));

    xTaskCreate(app_task, "App Task", 512, NULL, 10, NULL);
    xTaskCreate(limit_switch_task, "Limit Switch Task", 512, NULL, 12, &limit_t);

    attach_limit_switch_listener(limit_t);

    xTaskCreate(test_task, "Test Task", 512, NULL, 11, NULL);
    xTaskCreate(test_task2, "Test Task 2", 512, NULL, configMAX_PRIORITIES - 5, NULL);
}

void app_main(void)
{
    gpio_setup();
    task_setup();
}
