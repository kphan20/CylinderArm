#include <string.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/spi_slave.h"
#include "driver/gpio.h"

#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"

#include "pins.h"

#define NUM_DISTRIBUTED_CONTROLLERS 1
#define BYTES_PER_CONTROLLER 2
#define SPI_TRANSACTION_SIZE (NUM_DISTRIBUTED_CONTROLLERS * BYTES_PER_CONTROLLER)

// SPI response related variables
static uint8_t sensor_data[SPI_TRANSACTION_SIZE]; // sent over SPI to main compute
static SemaphoreHandle_t sensor_data_mutex; // mutex for the send buffer
QueueHandle_t sensor_data_update_q; // notifies that there should be an update to the send buffer based on distributed controller update

typedef struct {
    size_t sensor_data_idx;
    uint8_t data[BYTES_PER_CONTROLLER];
} sensor_data_update;

// Distributed controller related variables
QueueHandle_t setpoint_update_q; // queue of setpoint updates for distributed controllers

void IRAM_ATTR spi_post_setup_cb(spi_slave_transaction_t *trans)
{
    // tell SPI controller that transaction is ready
    gpio_set_level(HANDSHAKE_PIN, 1);
}

void IRAM_ATTR spi_post_trans_cb(spi_slave_transaction_t *trans)
{
    // TODO see if this works
    ESP_LOGI("TESTING", "Recieved message!");
    // set handshake low
    gpio_set_level(HANDSHAKE_PIN, 0);
    // send update to setpoint update queue for sending to distributed controllers
    // xQueueSendFromISR(setpoint_update_q, (uint8_t*) trans->rx_buffer, NULL);
}

// task that will copy whatever is in the send buffer and send it over SPI
void spi_task(void * arg)
{
    // since we are using DMA, make sure the tx and rx are word aligned
    WORD_ALIGNED_ATTR uint8_t recv_buf[SPI_TRANSACTION_SIZE];
    WORD_ALIGNED_ATTR uint8_t local_sensor_data[SPI_TRANSACTION_SIZE];

    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    // used to set 100Hz rate of sending sensor data and receiving setpoints
    TickType_t prev_wake_time = xTaskGetTickCount();
    const TickType_t task_freq = pdMS_TO_TICKS(10);
    while(1)
    {
        // TODO see how fast I should obtain mutex and send messages
        // tries to access the send buffer
        if (xSemaphoreTake(sensor_data_mutex, 1) == pdPASS)
        {
            // copies contents of send buffer to local buffer
            memcpy(local_sensor_data, sensor_data, SPI_TRANSACTION_SIZE);
            xSemaphoreGive(sensor_data_mutex);
        }

        // prepare transaction struct
        t.length = SPI_TRANSACTION_SIZE * 8;
        t.tx_buffer = local_sensor_data;
        t.rx_buffer = recv_buf;

        // block until SPI controller is ready
        spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
        // TODO add logging to see if delay actually occurs, if not then probably starvation on other tasks
        xTaskDelayUntil(&prev_wake_time, task_freq); // TODO add explicit delay to ensure no other tasks starve?
        ESP_LOGI("STACK WATER MARK", "SPI TASK: %d", uxTaskGetStackHighWaterMark(NULL));
    }
}

// task that will take updates from ESPNOW and put them in the send buffer
void update_sensor_data_task(void * arg)
{
    sensor_data_update update;
    while (1)
    {
        // block until an update is received
        if (xQueueReceive(sensor_data_update_q, &update, portMAX_DELAY))
        {
            // block until semaphore is free
            if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY))
            {
                memcpy(&sensor_data[update.sensor_data_idx * BYTES_PER_CONTROLLER], update.data, BYTES_PER_CONTROLLER);
                xSemaphoreGive(sensor_data_mutex);
            }
        }
    }
}

void app_main(void)
{
    sensor_data_mutex = xSemaphoreCreateMutex();
    // TODO tune this size
    sensor_data_update_q = xQueueCreate(16, sizeof(sensor_data_update));

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = MOSI_PIN,
        .miso_io_num = MISO_PIN,
        .sclk_io_num = SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        // .intr_flags
    };

    spi_slave_interface_config_t slv_cfg = {
        .mode = 0,
        .spics_io_num = CS_PIN,
        .queue_size = 3, // TODO tune this
        .flags = 0,
        .post_setup_cb = spi_post_setup_cb,
        .post_trans_cb = spi_post_trans_cb
    };

    gpio_config_t handshake_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(HANDSHAKE_PIN)
    };

    gpio_config(&handshake_conf);

    //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(MOSI_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SCLK_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(CS_PIN, GPIO_PULLUP_ONLY);
    
    ESP_ERROR_CHECK(spi_slave_initialize(SPI2_HOST, &bus_cfg, &slv_cfg, SPI_DMA_DISABLED));

    // TODO give two KB to spi task for now
    // make SPI high priority
    xTaskCreate(spi_task, "SPI Task", 512, NULL, configMAX_PRIORITIES-2, NULL);
    // TODO currently 1kb
    xTaskCreate(update_sensor_data_task, "Update Send Buffer Task", 256, NULL, configMAX_PRIORITIES-3, NULL);
}