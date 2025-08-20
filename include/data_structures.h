#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <esp_system.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include "Quaternion.h"
#include "config.h"

/*
* @brief Structure holding volatile variables used by the robot.
*/
typedef struct RobotData
{
    float pos_x;                            //Measured in m
    float pos_y;                            //Measured in m
    float pos_z;                            //Measured in m

    Quaternion orientation;

    #if ROBOT_CLASS == 1
        float left_wheel_position;              //Measured in m
        float left_wheel_speed;                 //Measured in m/tick
        float left_wheel_target_speed;          //Measured in m/tick
        float left_wheel_acceleration;          //Measured in m/tick^2

        float right_wheel_position;             //Measured in m
        float right_wheel_speed;                //Measured in m/tick
        float right_wheel_target_speed;         //Measured in m/tick
        float right_wheel_acceleration;         //Measured in m/tick^2
        
    #elif ROBOT_CLASS == 2

        float FL_position;                      //Measured in m
        float FL_speed;                         //Measured in m/tick
        float FL_acceleration;                  //Measured in m/tick^2
        float FL_target_speed;                  //Measured in m/tick

        float FR_position;                      //Measured in m
        float FR_speed;                         //Measured in m/tick
        float FR_acceleration;                  //Measured in m/tick^2
        float FR_target_speed;                  //Measured in m/tick

        float RL_position;                      //Measured in m
        float RL_speed;                         //Measured in m/tick
        float RL_acceleration;                  //Measured in m/tick^2
        float RL_target_speed;                  //Measured in m/tick

        float RR_position;                      //Measured in m
        float RR_speed;                         //Measured in m/tick
        float RR_acceleration;                  //Measured in m/tick^2
        float RR_target_speed;                  //Measured in m/tick

        float steering_position;                //Measured in degrees
        float steering_speed;                   //Measured in degrees/s
        float steering_acceleration;            //Measured in degrees/s^2
        float steering_target_position;         //Measured in degrees
    #else
        #error Unrecognized robot class.
    #endif
}RobotData;

/*
* @brief Structure holding persistant values used by the robot.
*/
typedef struct RobotConstants
{
    #if ROBOT_CLASS == 1
        float wheel_size;                       //Diameter of the wheel measured in m
        float wheel_separation;                 //Distance between the wheels or the length of the axle measured in m

        float KP;
        float KI;

    #elif ROBOT_CLASS == 2
        float front_wheel_size;
        float rear_wheel_size;
        float front_wheel_separation;
        float rear_wheel_separation;
        float front_wheel_longitudinal_distance;
        float rear_wheel_longitudinal_distance;

        float velocity_KP;
        float velocity_KI;
        float velocity_KD;

        float angle_KP;
        float angle_KI;
        float angle_KD;

    #else
        #error Unrecognized robot class
    #endif
}RobotConstants;

/*
* @brief Syructure holding variables used by the RTOS.
*/
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
}OpSys;


#endif