#ifndef PINOUT_H
#define PINOUT_H

#include <esp_system.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <math.h>

#include "nvs_flash.h"
#include "nvs.h"

#include "data_structures.h"

#define UART_PORT_NUM 1
#define UART_TX_PIN GPIO_NUM_1
#define UART_RX_PIN GPIO_NUM_3

#define ACCESORY_PORT_1 GPIO_NUM_22
#define ACCESORY_PORT_2 GPIO_NUM_23

#define LEFT_ENCODER_A GPIO_NUM_4
#define LEFT_ENCODER_B GPIO_NUM_34

#define RIGHT_ENCODER_A GPIO_NUM_19
#define RIGHT_ENCODER_B GPIO_NUM_21

#define ENABLE_HALF_STEPS true  // Set to true to enable tracking of rotary encoder at half step resolution
#define RESET_AT          0     // Set to a positive non-zero number to reset the position if this value is exceeded
#define FLIP_DIRECTION    true  // Set to true to reverse the clockwise/counterclockwise sense

#define ENCODER_STEPS 2048.0
#define RATIO 1.7778            //9.7800 for new robot, 1.7778 for old one

#define LEFT_MOTOR_DIR GPIO_NUM_32
#define LEFT_MOTOR_PWM GPIO_NUM_27

#define RIGHT_MOTOR_DIR GPIO_NUM_33
#define RIGHT_MOTOR_PWM GPIO_NUM_13

#define PID_LIMIT 1024
#define PID_ERROR_LIMIT 1024.0

#define ALTERNATIVE_CONTROL_SCHEME

#define ERROR_QUEUE_SIZE 1024

void pinInit();
void dataInit();
void readConstantsFromFlash();
void saveConstantsToFlash();
void resetFlash();
void checkError(esp_err_t state);

#endif