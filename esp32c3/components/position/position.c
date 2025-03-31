#include <inttypes.h>
#include "driver/i2c_master.h"

#include "sensor.h"
#include "pins.h"

const uint32_t I2C_FREQUENCY = 100000; // TODO figure this out, less than 400kHz
const i2c_port_num_t PORT_NUMBER = -1; // this is automatic selection, maybe explicitly decide it?
const uint16_t AS5600_I2C_ADDRESS = 0x36;

const uint8_t AS5600_ZPOS_REG = 0x01; // start position register
const uint8_t AS5600_MPOS_REG = 0x03; // stop position register
const uint8_t AS5600_RAW_REG = 0x0C;
const uint8_t AS5600_ANG_REG = 0x0E; // is this just based on what zero is compared to the raw?

static const int I2C_INTERRUPT_PRIORITY = 10; // TODO look into configMAX_SYSCALL_INTERRUPT_PRIORITY boundary

static i2c_master_bus_config_t i2c_bus_config;
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t as5600_handle;

// sets up the I2C bus for communication with AS5600
void init_AS5600()
{
    // set up device as i2c master
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = PORT_NUMBER,
        .sda_io_num = SDA_IO_PIN,
        .scl_io_num = SCL_IO_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7, // TODO figure this one out
        .intr_priority = I2C_INTERRUPT_PRIORITY
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
    i2c_device_config_t as5600_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = I2C_FREQUENCY,
        .device_address = AS5600_I2C_ADDRESS
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &as5600_conf, &as5600_handle));
}

// currently writes to zero position register
void write_AS5600(uint8_t write_reg, uint8_t * write_buffer)
{
    if (sizeof(write_buffer) < 2) return; // assumes write buffer should be at least 2 bytes
    uint8_t buffer[] = {write_reg, write_buffer[0], write_buffer[1]};
    i2c_master_transmit(as5600_handle, buffer, 3, -1); // TODO currently blocks
}
// currently reads the raw angle register
void read_AS5600(uint8_t read_reg, uint8_t * read_buffer)
{
    uint8_t register_address[1] = {read_reg};
    i2c_master_transmit_receive(as5600_handle, register_address, 1, read_buffer, sizeof(read_buffer), -1); // TODO figure out timeout
}

void sensor_gpio_setup()
{

}

void sensor_task_setup()
{

}

void homing_sequence()
{
    // TODO move all the way to min limit switch
    // define zero angle
    uint8_t zero_angle[2]; // TODO check datasheet to see which register to read
    read_AS5600(AS5600_RAW_REG, zero_angle);
    write_AS5600(AS5600_ZPOS_REG, zero_angle);

    // TODO move all the way to max limit switch
    // define max angle
    uint8_t max_angle[2];
    read_AS5600(AS5600_RAW_REG, max_angle);
    write_AS5600(AS5600_MPOS_REG, max_angle);
}

PID_VAL_TYPE get_sensor_val()
{
    return 0; // TODO
}

void cleanup_i2c()
{
    i2c_master_bus_rm_device(as5600_handle);
}