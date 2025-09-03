/**
 * @file comm_task.h
 * @brief File containing the definition of the communication task.
 */

#ifndef COMM_TASK_H
#define COMM_TASK_H

#include <esp_system.h>
#include <driver/gpio.h>
#include <driver/uart.h>

#include <string.h>

#include "pinout.h"

#define UART_BAUD_RATE 115200
#define BUFF_SIZE 128

/*
* @brief Task responsible for communication between the uC and controlling computer.
*/
void comm_task();

#endif