#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"

#include "sensor.h"
#include "pins.h"

typedef int32_t encoder_count_t;

static QueueHandle_t q;
// TODO switch to uint32_t for atomicity? or use mutex
static uint16_t pwm_distance;
static uint16_t pwm_distance_offset; // is set during homing sequence to get zero position
static volatile encoder_count_t encoder_count; // TODO should this ever be negative
static const encoder_count_t count_to_dist_scale;

static void IRAM_ATTR encoder_isr_handler(void* arg)
{
    // TODO do I need error handling?
    encoder_count += 1 | -(gpio_get_level(ENCODER_B_PIN)); // TODO test if optimization works
}

void sensor_gpio_setup()
{
    // set both encoder pins to be outputs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENCODER_A_PIN) | (1ULL << ENCODER_B_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // change encoder A to have rising edge interrupt
    gpio_set_intr_type(ENCODER_A_PIN, GPIO_INTR_POSEDGE);

    // TODO see if doing this here is a good idea
    gpio_isr_handler_add(ENCODER_A_PIN, encoder_isr_handler, NULL);
}

static bool rmt_on_recv_callback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup = pdFALSE;
    xQueueSendFromISR(q, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

static bool check_distance_pulse(uint16_t high_time_us)
{
    if (high_time_us < 1000)
    {
        return false; // TODO invalid reading (too close)
    }
    else if (high_time_us > 1650)
    {
        return false; // TODO no object detected
    }
    pwm_distance = ((high_time_us - 1000) << 1) - pwm_distance_offset; // TODO current failure mode is to use previous value?
    return true;
}

static void rmt_task(void * arg)
{
    // channel configuration
    rmt_rx_channel_config_t rmt_rx_channel_config = {
        .gpio_num = PWM_INPUT,
        .clk_src = RMT_CLK_SRC_DEFAULT, 
        .resolution_hz = 1000000, // TODO, current usec period
        .mem_block_symbols = 48, // TODO
        .flags.invert_in = 0, // TODO
        .intr_priority = 15, // TODO
        .flags.allow_pd = 0, // TODO
        .flags.io_loop_back = 0, // TODO
    };
    rmt_channel_handle_t rx_chan = NULL;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rmt_rx_channel_config, &rx_chan));

    // callback configuration
    rmt_rx_event_callbacks_t rmt_callbacks = {
        .on_recv_done = rmt_on_recv_callback
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &rmt_callbacks, NULL));
    ESP_ERROR_CHECK(rmt_enable(rx_chan));

    // message configuration
    rmt_receive_config_t rx_recv_config = {
        .signal_range_min_ns = 900000, // 900 usec for now (smallest pulse is around 1 ms)
        .signal_range_max_ns = 6000000, // 6 ms to capture most of low period
    };

    rmt_symbol_word_t raw_symbols[16]; // TODO tune size
    rmt_rx_done_event_data_t rx_data;

    // application variables
    uint8_t no_receive_count = 0;
    bool high_pulse_found = false;
    while(1)
    {
        if (xQueueReceive(q, &rx_data, pdMS_TO_TICKS(2)) == pdPASS)
        {
            // TODO loop through all symbols?
            for (size_t i = 0; i < rx_data.num_symbols; i++)
            {
                rmt_symbol_word_t curr_symbol = rx_data.received_symbols[i];
                // checks if the distance pulse was valid
                if (!check_distance_pulse(curr_symbol.level0 ? curr_symbol.duration0 : curr_symbol.duration1))
                    continue;
                
                // pulse corresponding to valid distance was found
                high_pulse_found = true;
            }

            if (high_pulse_found)
            {
                no_receive_count = 0;
            }
            else
            {
                no_receive_count++;
            }
            high_pulse_found = false;
            // ESP_ERROR_CHECK(rmt_receive(rx_chan, raw_symbols, sizeof(raw_symbols), &rx_recv_config));
        }
        else
        {
            no_receive_count++; // TODO setup failure handling
        }
        // TODO call on every loop iteration? handles errors at least
        ESP_ERROR_CHECK(rmt_receive(rx_chan, raw_symbols, sizeof(raw_symbols), &rx_recv_config));
    }
}

void sensor_task_setup()
{
    q = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(q);
    xTaskCreate(rmt_task, "read_pwm_task", 1, NULL, 8, NULL); // TODO configure properly
}

static void distance_fusion()
{
    float encoder_distance = encoder_count * count_to_dist_scale;
}

void homing_sequence()
{
    // TODO move all the way to min limit switch
    // define zero position
    uint8_t zero_angle[2];
    pwm_distance_offset = pwm_distance; // TODO add delay to stabilize value?

    // TODO move all the way to max limit switch
    // TODO define max position?
    uint8_t max_angle[2];
    
}

PID_VAL_TYPE get_sensor_val()
{
    return (PID_VAL_TYPE) pwm_distance; // TODO use fusion?
}