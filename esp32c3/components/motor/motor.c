#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#include "pins.h"
#include "motor.h"

static QueueHandle_t motor_cmd_q;

static const ledc_mode_t LEDC_MODE = LEDC_LOW_SPEED_MODE;
static const ledc_channel_t LEDC_CHANNEL = LEDC_CHANNEL_0;

static volatile gpio_num_t * hit_switch;

static const ledc_timer_bit_t DUTY_RESOLUTION = LEDC_TIMER_10_BIT;

// defines [min, max] of duty cycle values
static const uint32_t DUTY_MIN = 0;
static const uint32_t DUTY_MAX = (1 << DUTY_RESOLUTION) - 1;


void stop_motor()
{
    ledc_set_duty_and_update(LEDC_MODE, LEDC_CHANNEL, 0, 0);
}

static void IRAM_ATTR limit_isr_handler(void* arg)
{
    stop_motor();
    *hit_switch = (gpio_num_t) arg;
}

void motor_gpio_setup()
{
    // set both limit switch pins to have pullups and be inputs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << UPPER_LIM_SWITCH) | (1ULL << LOWER_LIM_SWITCH),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // configure motor output pins (TODO see if this is necessary)
    io_conf.pin_bit_mask = (1ULL << MOTOR_DIR) | (1ULL << MOTOR_PWM);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // configure motor PWM peripheral
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .freq_hz = 25000, // TODO
        .duty_resolution = DUTY_RESOLUTION, // TODO
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_0, //TODO
        .intr_type = LEDC_INTR_DISABLE, //TODO
        .gpio_num = MOTOR_PWM,
        .duty = 0,
        .hpoint = 0, // TODO
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ESP_ERROR_CHECK(gpio_set_intr_type(LOWER_LIM_SWITCH, GPIO_INTR_NEGEDGE)); // TODO figure out if it should be falling edge
    ESP_ERROR_CHECK(gpio_set_intr_type(UPPER_LIM_SWITCH, GPIO_INTR_NEGEDGE));

    gpio_isr_handler_add(LOWER_LIM_SWITCH, limit_isr_handler, (void*) LOWER_LIM_SWITCH);
    gpio_isr_handler_add(UPPER_LIM_SWITCH, limit_isr_handler, (void*) UPPER_LIM_SWITCH);
}

static void command_motor_task(void * arg)
{
    motor_cmd_t duty;
    TickType_t prev_wake_time = xTaskGetTickCount();
    const TickType_t task_freq = pdMS_TO_TICKS(5);
    while(1)
    {
        if (hit_switch != NULL)
        {
            if(ledc_get_duty(LEDC_MODE, LEDC_CHANNEL))
                stop_motor();
            
            // TODO limit switch error handling
            if (*hit_switch == LOWER_LIM_SWITCH)
            {

            }
            else
            {

            }
        }
        // use delay to stay responsive
        else if (xQueueReceive(motor_cmd_q, &duty, task_freq) == pdPASS)
        {
            // TODO check for limit switch here?
            // TODO based on the sign of the duty cycle set direction
            gpio_set_level(MOTOR_DIR, (uint32_t)((duty >> 31) & 1));
            
            // set duty cycle to absolute value
            // TODO should hpoint just be zero?
            uint32_t duty_conv = (uint32_t)((duty ^ (duty >> 31)) - (duty >> 31));
            // clamp value
            duty_conv = duty_conv < DUTY_MIN ? DUTY_MIN : (duty_conv > DUTY_MAX ? DUTY_MAX : duty_conv);
            ledc_set_duty_and_update(LEDC_MODE, LEDC_CHANNEL, duty_conv, 0);
        }
        xTaskDelayUntil(&prev_wake_time, task_freq);
    }
}

void motor_task_setup()
{
    motor_cmd_q = xQueueCreate(4, sizeof(motor_cmd_t));
    // TODO configure this correctly
    xTaskCreate(command_motor_task, "Motor Task", 1, NULL, 7, NULL);
}

void motor_set_command(PID_VAL_TYPE command)
{
    // TODO command should never not fit into an int32_t
    motor_cmd_t cmd_converted = round(command);
    xQueueSend(motor_cmd_q, &cmd_converted, portMAX_DELAY);
}