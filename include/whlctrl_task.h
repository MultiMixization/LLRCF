/**
 * @file whlctrl_task.h
 * @brief Header file containing definition of the wheel control task.
 */

#ifndef WHLCTRL_TASK_H
#define WHLCTRL_TASK_H

#include <esp_system.h>
#include <driver/gpio.h>
#include <driver/ledc.h>

#include "pinout.h"

/*
* @brief Task responsible for controling the movement system of the robot.
*/
void whlctrl_task();

#endif