#ifndef PINOUT_H
#define PINOUT_H

#include <esp_system.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <math.h>

#include "nvs_flash.h"
#include "nvs.h"

#include "data_structures.h"
#include "config.h"

#define UART_PORT_NUM 1
#define UART_TX_PIN GPIO_NUM_1
#define UART_RX_PIN GPIO_NUM_3

#define ACCESORY_PORT_1 GPIO_NUM_22
#define ACCESORY_PORT_2 GPIO_NUM_23

#define LEFT_ENCODER_A GPIO_NUM_4
#define LEFT_ENCODER_B GPIO_NUM_34

#define RIGHT_ENCODER_A GPIO_NUM_19
#define RIGHT_ENCODER_B GPIO_NUM_21

#define LEFT_MOTOR_DIR GPIO_NUM_32
#define LEFT_MOTOR_PWM GPIO_NUM_27

#define RIGHT_MOTOR_DIR GPIO_NUM_33
#define RIGHT_MOTOR_PWM GPIO_NUM_13

void pinInit();
void dataInit();
void readConstantsFromFlash();
void saveConstantsToFlash();
void resetFlash();
void checkError(esp_err_t state);

#endif