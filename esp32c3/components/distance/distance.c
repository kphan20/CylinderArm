#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"

#include "esp_timer.h"

#include "sensor.h"
#include "pins.h"
#include "spline.h"

#ifdef CONFIG_DEBUG
#include "esp_log.h"
#endif

typedef int32_t encoder_count_t;

static QueueHandle_t q; // used for when RMT is done reading a value
static uint16_t pwm_distance; // used to store value from RMT processing - TODO maybe make this local
static uint16_t pwm_distance_offset; // is set during homing sequence to get "zero" position
static volatile encoder_count_t encoder_count; // tracking encoder ticks since boot
static const PID_VAL_TYPE COUNT_TO_DIST_SCALE; // conversion from each encoder tick to distance moved

static const int64_t PREDICTION_TIMEOUT = 100; // ns, prevents kalman prediction from being run in quick succession
static PID_VAL_TYPE fused_distance; // caches latest state estimate from kalman filter
static PID_VAL_TYPE p; // estimate covariance
static const PID_VAL_TYPE R = 10.0f; // sensor noise, TODO used fixed for now, but may make this varying with measured distance

static void IRAM_ATTR encoder_isr_handler(void* arg)
{
    encoder_count += 1 | -(gpio_get_level(ENCODER_B_PIN)); // TODO test if optimization works
}

static void kalman_prediction()
{
    static int64_t prev_time = 0;
    static encoder_count_t prev_encoder_count = 0;

    // TODO protection against calling prediction twice in a row - see if this is a valid concern
    int64_t curr_time = esp_timer_get_time();
    if (curr_time - prev_time < PREDICTION_TIMEOUT) return;
    prev_time = curr_time;

    PID_VAL_TYPE encoder_distance = (encoder_count - prev_encoder_count) * COUNT_TO_DIST_SCALE;
    prev_encoder_count = encoder_count;
    fused_distance += encoder_distance; // TODO add noise?
    // TODO add a flat noise? Scale based on time since last prediction?
    p += 4 * COUNT_TO_DIST_SCALE;
}

static void kalman_update()
{
    PID_VAL_TYPE dh = eval_dh_spline(pwm_distance);
    PID_VAL_TYPE y = pwm_distance - eval_h_spline(pwm_distance);
    PID_VAL_TYPE s = dh * dh * p + R;
    PID_VAL_TYPE k = p * dh / s;
    fused_distance = fused_distance + k * y;
    p *= 1 - k * dh;
}

void sensor_gpio_setup()
{
    // set both encoder pins to be outputs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENCODER_A_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, // there will be external pullup
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE // with inverter, rising edges are actually falling
    };
    gpio_config(&io_conf);

    // configure encoder b separately for not since it won't have interrupts for now
    io_conf.pin_bit_mask = (1ULL << ENCODER_B_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE,
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = 1ULL << PWM_INPUT;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE, // TODO check if this is required
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // TODO see if esp can handle both pins
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
    if (high_time_us < 2000)
    {
        #ifdef CONFIG_DEBUG
        ESP_LOGW("PWM_SENSOR", "TOO SHORT");
        #endif
        return false; // invalid reading (too close)
    }
    else if (high_time_us > 3300)
    {
        #ifdef CONFIG_DEBUG
        ESP_LOGW("PWM_SENSOR", "TOO LONG");
        #endif
        return false; // no object detected
    }
    pwm_distance = (high_time_us - 2000) - pwm_distance_offset; // TODO current failure mode is to use previous value?
    kalman_prediction();
    kalman_update();
    return true;
}

static void rmt_task(void * arg)
{
    // channel configuration
    rmt_rx_channel_config_t rmt_rx_channel_config = {
        .gpio_num = PWM_INPUT,
        .clk_src = RMT_CLK_SRC_DEFAULT, 
        .resolution_hz = 2000000, // TODO, current 0.5 usec period
        .mem_block_symbols = 64, // TODO
//        .flags.invert_in = 0, // TODO
//        .intr_priority = configMAX_PRIORITIES - 5, // TODO
//        .flags.allow_pd = 0, // TODO
//        .flags.io_loop_back = 0, // TODO
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
        .signal_range_min_ns = 1250,//900000, // 900 usec for now (smallest pulse is around 1 ms)
        .signal_range_max_ns = 6000000, // 6 ms to capture most of low period
    };

    rmt_symbol_word_t raw_symbols[64]; // TODO tune size
    rmt_rx_done_event_data_t rx_data;

    // application variables
    uint8_t no_receive_count = 0;
    bool high_pulse_found = false;
    ESP_ERROR_CHECK(rmt_receive(rx_chan, raw_symbols, sizeof(raw_symbols), &rx_recv_config));
    while(1)
    {
        if (xQueueReceive(q, &rx_data, 2) == pdPASS)
        {
            // TODO loop through all symbols? currently breaks early
            for (size_t i = 0; i < rx_data.num_symbols; i++)
            {
                rmt_symbol_word_t curr_symbol = rx_data.received_symbols[i];
                // checks if the distance pulse was valid
                if (!check_distance_pulse(curr_symbol.level0 ? curr_symbol.duration0 : curr_symbol.duration1))
                    continue;
                
                // pulse corresponding to valid distance was found
                high_pulse_found = true;
                #ifdef CONFIG_DEBUG
                ESP_LOGI("PWM_SENSOR", "Distance: %u", pwm_distance);
                #endif
                break;
            }

            if (high_pulse_found)
            {
                no_receive_count = 0;
            }
            else
            {
                no_receive_count++;
                #ifdef CONFIG_DEBUG
                ESP_LOGW("PWM_SENSOR", "INVALID READ, failed receive count: %u", no_receive_count);
                #endif
            }
            high_pulse_found = false;

            // if something was sent in queue, then pulse was received and can call receive again
            ESP_ERROR_CHECK(rmt_receive(rx_chan, raw_symbols, sizeof(raw_symbols), &rx_recv_config));
        }
        else
        {
            no_receive_count++; // TODO setup failure handling - probably just call rmt_receive after number of failures
            #ifdef CONFIG_DEBUG
            ESP_LOGW("PWM_SENSOR", "INVALID READ (timeout), failed receive count: %u", no_receive_count);
            #endif
        }
    }
}

void sensor_task_setup()
{
    q = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(q);
    xTaskCreate(rmt_task, "read_pwm_task", 4096, NULL, 8, NULL); // TODO configure properly
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
    kalman_prediction();
    return fused_distance;
}
