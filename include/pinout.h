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

#if ROBOT_CLASS == 1
//--------------------------------- Encoders ---------------------------------
    #define LEFT_ENCODER_A GPIO_NUM_4
    #define LEFT_ENCODER_B GPIO_NUM_34

    #define RIGHT_ENCODER_A GPIO_NUM_19
    #define RIGHT_ENCODER_B GPIO_NUM_21

//--------------------------------- Motors ---------------------------------
    #define LEFT_MOTOR_DIR GPIO_NUM_32
    #define LEFT_MOTOR_PWM GPIO_NUM_27

    #define RIGHT_MOTOR_DIR GPIO_NUM_33
    #define RIGHT_MOTOR_PWM GPIO_NUM_13

#elif ROBOT_CLASS == 2

    #define FL_ENCODER_A GPIO_NUM_34
    #define FL_ENCODER_B GPIO_NUM_35

    #define FR_ENCODER_A GPIO_NUM_25
    #define FR_ENCODER_B GPIO_NUM_26

    #define RL_ENCODER_A GPIO_NUM_19
    #define RL_ENCODER_B GPIO_NUM_18

    #define RR_ENCODER_A GPIO_NUM_16
    #define RR_ENCODER_B GPIO_NUM_4


//--------------------------------- Motors ---------------------------------
    #define FL_MOTOR_DIR GPIO_NUM_32
    #define FL_MOTOR_PWM GPIO_NUM_33

    #define FR_MOTOR_DIR GPIO_NUM_27
    #define FR_MOTOR_PWM GPIO_NUM_14

    #define RL_MOTOR_DIR GPIO_NUM_5
    #define RL_MOTOR_PWM GPIO_NUM_17
    
    #define RR_MOTOR_DIR GPIO_NUM_0
    #define RR_MOTOR_PWM GPIO_NUM_2

    #define STEERING_MOTOR_DIR GPIO_NUM_14
    #define STEERING_MOTOR_PWM GPIO_NUM_12
#else 
    #error Unrecognized robot class.
#endif

/*
* @brief Function responsible for initialization of uC pins.
*/
void pinInit();

/*
* @brief Function responsible for initialization of the volatile variables.
*/
void dataInit();

/*
* @brief Functon used for reading the persistant variables from uC flash memory.
*/
void readConstantsFromFlash();

/*
* @brief Function used for overwriting persistent variables in the uC flash memory.
*/
void saveConstantsToFlash();

/*
* @brief Function used for reseting all persistant variables to known hard-coded values. 
*/
void resetFlash();

/*
* @brief Function used to pass any esp_err_t codes to the communication task.
*
* The function checks if the passed argument "state" is different than ESP_OK.
* If yes then the content of said argument is added to a queue to be sent to the master device.
*
* @param state Error code returned by a different function.
*/
void checkError(esp_err_t state);

#endif