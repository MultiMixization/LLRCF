#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <esp_system.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include "Quaternion.h"

typedef struct RobotData
{
    float left_wheel_position;              //Measured in m
    float left_wheel_speed;                 //Measured in m/tick
    float left_wheel_target_speed;          //Measured in m/tick
    float left_wheel_acceleration;          //Measured in m/tick^2

    float right_wheel_position;             //Measured in m
    float right_wheel_speed;                //Measured in m/tick
    float right_wheel_target_speed;         //Measured in m/tick
    float right_wheel_acceleration;         //Measured in m/tick^2

    float pos_x;                            //Measured in m
    float pos_y;                            //Measured in m
    float pos_z;                            //Measured in m

    Quaternion orientation;
}RobotData;

typedef struct RobotConstants
{
    float wheel_size;                       //Diameter of the wheel measured in m
    float wheel_separation;                 //Distance between the wheels or the length of the axle measured in m

    float KP;
    float KI;
}RobotConstants;

typedef struct OpSys
{
    TaskHandle_t comm_task_handler;
    TaskHandle_t odom_task_handler;
    TaskHandle_t whlctrl_task_handler;

    TickType_t comm_last_wakeup;
    TickType_t odom_last_wakeup;
    TickType_t whlctrl_last_wakeup;

    SemaphoreHandle_t RobotDataAccess;
    SemaphoreHandle_t ConstantDataAccess;
    SemaphoreHandle_t OpSysAccess;

    QueueHandle_t errorQueue;

}OpSyS;


#endif