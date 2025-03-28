#ifndef _SEED_BOARD_CONFIG
#define _SEED_BOARD_CONFIG

// DISTRIBUTED CONTROLLER VARIABLES
#define PWM_INPUT GPIO_NUM_5

#define ENCODER_A_PIN GPIO_NUM_20
#define ENCODER_B_PIN GPIO_NUM_21

#define SDA_IO_PIN GPIO_NUM_6
#define SCL_IO_PIN GPIO_NUM_7

#define UPPER_LIM_SWITCH GPIO_NUM_3
#define LOWER_LIM_SWITCH GPIO_NUM_4

#define MOTOR_PWM GPIO_NUM_10
#define MOTOR_DIR GPIO_NUM_8


// MASTER CONTROLLER VARIABLES
#define MISO_PIN GPIO_NUM_6 // TODO see if I can just use the strapping pin
#define MOSI_PIN GPIO_NUM_10
#define SCLK_PIN GPIO_NUM_8
#define CS_PIN GPIO_NUM_7
#define HANDSHAKE_PIN GPIO_NUM_3

#endif