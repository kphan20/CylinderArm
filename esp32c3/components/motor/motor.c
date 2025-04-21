#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
// #include "driver/gptimer.h"

#include "pins.h"
#include "motor.h"
// #include "sensor.h"

#ifdef CONFIG_DEBUG
#include "esp_log.h"
#endif

typedef struct
{
    uint32_t dir;
    uint32_t duty_cycle;
} motor_cmd_t;

static QueueHandle_t motor_cmd_q;

static const ledc_mode_t LEDC_MODE = LEDC_LOW_SPEED_MODE;
static const ledc_channel_t LEDC_CHANNEL = LEDC_CHANNEL_0;
static const ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
static const uint32_t PWM_FREQ = 25000; // Hz

static volatile gpio_num_t * hit_switch;

static const ledc_timer_bit_t DUTY_RESOLUTION = LEDC_TIMER_10_BIT; // TODO determine good resolution

// defines [min, max] of duty cycle values
static const uint32_t DUTY_MIN = 0;
static const uint32_t DUTY_MAX = (1 << DUTY_RESOLUTION) - 1;
static const uint32_t LIMIT_PROTOCOL_DUTY = DUTY_MAX >> 2; // TODO start with quarter of max speed

static volatile TaskHandle_t task_listener = NULL; // used to notify the sensor task that a limit switch has been hit
static bool limit_protocol_started; // start moving to release the limit switch

static inline void update_motor_duty(uint32_t duty)
{
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void stop_motor()
{
    update_motor_duty(0);
}

static void IRAM_ATTR limit_isr_handler(void* arg)
{
    stop_motor();
    *hit_switch = (gpio_num_t) arg;
    BaseType_t higher_priority_woken = pdFAIL;
    if (task_listener != NULL)
    {
        xTaskNotifyFromISR(task_listener, *hit_switch == LOWER_LIM_SWITCH, eSetValueWithOverwrite, &higher_priority_woken);
        portYIELD_FROM_ISR(higher_priority_woken); // TODO see if this context switch is necessary
    }
}

void motor_gpio_setup()
{
    // set both limit switch pins to be inputs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << UPPER_LIM_SWITCH) | (1ULL << LOWER_LIM_SWITCH),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, // external debounce and pullup
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // configure motor output pins
    io_conf.pin_bit_mask = (1ULL << MOTOR_DIR) | (1ULL << MOTOR_PWM);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // configure motor PWM peripheral
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .freq_hz = PWM_FREQ,
        .duty_resolution = DUTY_RESOLUTION,
        .clk_cfg = LEDC_AUTO_CLK,
        .timer_num = LEDC_TIMER
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .timer_sel = LEDC_TIMER,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_PWM,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    gpio_isr_handler_add(LOWER_LIM_SWITCH, limit_isr_handler, (void*) LOWER_LIM_SWITCH);
    gpio_isr_handler_add(UPPER_LIM_SWITCH, limit_isr_handler, (void*) UPPER_LIM_SWITCH);
}

static void command_motor_task(void * arg)
{
    motor_cmd_t cmd;
    TickType_t prev_wake_time = xTaskGetTickCount();
    const TickType_t task_freq = pdMS_TO_TICKS(5);
    while(1)
    {
        if (hit_switch != NULL)
        {
            if((!limit_protocol_started) & ledc_get_duty(LEDC_MODE, LEDC_CHANNEL))
                stop_motor();
            else if (limit_protocol_started)
            {
                // TODO high level indicates switch is released
                if (gpio_get_level(*hit_switch))
                {
                    stop_motor(); // stop controlling motor for now
                    hit_switch = NULL;
                    limit_protocol_started = false;
                    xTaskNotify(task_listener, RELEASED, eSetValueWithOverwrite); // TODO add a little bit of delay?
                }
                // continue with release protocol since switch is pressed
                else
                {
                    gpio_set_level(MOTOR_DIR, *hit_switch == LOWER_LIM_SWITCH); // TODO change based on direction
                    update_motor_duty(LIMIT_PROTOCOL_DUTY);
                }
            }
        }
        // use delay to stay responsive
        else if (xQueueReceive(motor_cmd_q, &cmd, task_freq) == pdPASS)
       {
            // TODO check for limit switch here?
            // TODO based on the sign of the duty cycle set direction
            gpio_set_level(MOTOR_DIR, cmd.dir);
            
            // set duty cycle to absolute value
            update_motor_duty(cmd.duty_cycle);
       }
        xTaskDelayUntil(&prev_wake_time, task_freq);
        #ifdef CONFIG_DEBUG
        ESP_LOGI("MOTOR_TASK", "STACK WATER MARK: %d", uxTaskGetStackHighWaterMark(NULL));
        #endif
    }
}

void motor_task_setup()
{
    motor_cmd_q = xQueueCreate(1, sizeof(motor_cmd_t));
    // TODO configure this correctly
#ifdef CONFIG_DEBUG
    xTaskCreate(command_motor_task, "Motor Task", 2048, NULL, 7, NULL);
#else
    xTaskCreate(command_motor_task, "Motor Task", 512, NULL, 7, NULL);
#endif
    // TODO use a timer instead for faster PID/control rates
    // gptimer_handle_t gptimer = NULL;
    // gptimer_config_t timer_config = {
    //     .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    //     .direction = GPTIMER_COUNT_UP,
    //     .resolution_hz = 1000, // start with 1khz
    // };

    // ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    // gptimer_event_callbacks_t timer_cbs = {
    //     .on_alarm = NULL,
    // };
    // gptimer_register_event_callbacks(gptimer, &timer_cbs, NULL);
    // gptimer_enable(gptimer);
    // gptimer_alarm_config_t alarm_cfg = {
    //     .alarm_count = 1
    // };
    // gptimer_set_alarm_action(gptimer, &alarm_cfg);
}

// Takes in a value between -100 percent and 100 percent
void motor_set_command(PID_VAL_TYPE command)
{
    // TODO command should never not fit into an int32_t
    uint32_t duty = (uint32_t)((float)DUTY_MAX * (fabsf(command) / 100.0f));
    // TODO see if clamp is required
    duty = duty < DUTY_MIN ? DUTY_MIN : (duty > DUTY_MAX ? DUTY_MAX : duty);
    motor_cmd_t cmd = {
        .dir = (uint32_t)(command < 0.0f),
        .duty_cycle = duty
    };

    // TODO use overwriting logic for now
    xQueueOverwrite(motor_cmd_q, &cmd);
    // xQueueSend(motor_cmd_q, &cmd, portMAX_DELAY);
}

void attach_limit_switch_listener(TaskHandle_t t)
{
    task_listener = t;
    // TODO notify tasks if limit switch is already hit while attaching
    if (hit_switch != NULL)
    {
        xTaskNotify(task_listener, *hit_switch == LOWER_LIM_SWITCH, eSetValueWithoutOverwrite);
    }
}

void start_limit_protocol()
{
    // if other task is telling motor to release limit switch, but it isn't currently pressed,
    // just send notification back that it has been released
    if (hit_switch == NULL)
    {
        xTaskNotify(task_listener, RELEASED, eSetValueWithOverwrite);
    }
    else
    {
        limit_protocol_started = true;
    }
}