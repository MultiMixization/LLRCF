#ifndef ODOM_TASK_H
#define ODOM_TASK_H

#include <esp_system.h>
#include <driver/gpio.h>
#include <math.h>

#include "pinout.h"
#include "rotary_encoder.h"

/*
* @brief Task responsible for calculation of robots odometry.
*/
void odom_task();

#endif