/**
 * @file odom_task.h
 * @brief File containing the definition of the odometry calculation task.
 */

#ifndef ODOM_TASK_H
#define ODOM_TASK_H

#include <esp_system.h>
#include <driver/gpio.h>
#include <math.h>

#include "pinout.h"
#include "rotary_encoder.h"
#include "AS5040.h"

/*
* @brief Task responsible for calculation of robots odometry.
*
* The task grabs the robot constants values from the constantsDataStruct structure at startup.
* They are then saved localy and used to calculate the size of each wheel.
* After the setup the task operates at 
*/
void odom_task();

#endif