/**
 * @file data_structures.h
 * @brief Header file containing definitions of data structures used in the robot.
 */

#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <esp_system.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include "Quaternion.h"
#include "config.h"

/**
 * @struct RobotData
 * @brief Structure holding important volatile variables used by the robot.
 */
typedef struct
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

    #ifdef SAFETY_SWITCH
        bool safety_switch_state;
    #endif
}RobotData;

typedef struct
{
    #if ACCESORY_1_TYPE == 1
        bool accesory_1_state;
    #elif ACCESORY_1_TYPE == 2
        bool accesory_1_value;
    #endif

    #if ACCESORY_2_TYPE == 1
        bool accesory_2_state;
    #elif ACCESORY_2_TYPE == 2
        bool accesory_2_value;
    #endif
}LowImportantData;


/**
 * @struct RobotConstants
 * @brief Structure holding non volatile robot constants.
 */
typedef struct 
{
    uint32_t odometry_frequency;
    uint32_t wheel_ctrl_frequency;
    #if ROBOT_CLASS == 1
        float wheel_size;                       /** Diameter of the wheel measured in m */
        float wheel_separation;                 /** Distance between the wheels or the length of the axle measured in m */

        float KP;                               /** Value of the proportional term multiplier. */
        float KI;                               /** Value of the integral term multiplier. */

    #elif ROBOT_CLASS == 2
        float front_wheel_size;                 /** Size of the front wheels. */
        float rear_wheel_size;                  /** Size of the rear wheels. */
        float front_wheel_separation;           /** Distance between the front wheels of the robot. */
        float rear_wheel_separation;            /** Distance between the rear wheels of the robot. */
        float front_wheel_longitudinal_distance;/** Longitudinal distance between the front axle and rear axle of the robot. */

        float velocity_KP;                      /** Value of the proportional term multiplier for the velocity controling PID controllers. */
        float velocity_KI;                      /** Value of the integral term multiplier for the velocity controling PID controllers. */
        float velocity_KD;                      /** Value of the differential term multiplier for the velocity controling PID controllers. */

        float angle_KP;                         /** Value of the proportional term multiplier for the angle controling PID controller. */
        float angle_KI;                         /** Value of the integral term multiplier for the angle controling PID controller. */
        float angle_KD;                         /** Value of the differential term multiplier for the angle controling PID controller. */

        float angle_offset;                     /** Offset allowing for adjusting the neutral position for steering. */
        float angle_max;                        /** Maximum steering angle. */
        float angle_min;                        /** Minimum steering angle. */
    #else
        #error Unrecognized robot class
    #endif
}RobotConstants;

/**
* @struct OpSys
* @brief Syructure holding variables used by the RTOS.
*/
typedef struct
{
    TaskHandle_t comm_task_handler;             /** Handle for communication task. */
    TaskHandle_t odom_task_handler;             /** Handle for the odometry calculation task. */
    TaskHandle_t whlctrl_task_handler;          /** Handle for the wheel control task. */
    TaskHandle_t accesory_task_handler;         /** Handle for the accesory control task. */

    TickType_t comm_last_wakeup;                /** Amount of ticks when communication task last ran. */
    TickType_t odom_last_wakeup;                /** Amount of ticks when odometry task last ran. */
    TickType_t whlctrl_last_wakeup;             /** Amount of ticks when wheel control task last ran. */
    TickType_t accesory_last_wakeup;            /** Amount of ticks when accesory task last ran. */

    SemaphoreHandle_t RobotDataAccess;          /** Semaphore for accesing the RobotData structure. */
    SemaphoreHandle_t ConstantDataAccess;       /** Semaphore for accesing the ConstantData structure. */
    SemaphoreHandle_t OpSysAccess;              /** Semaphore for accesing the OpSys structure. */
    SemaphoreHandle_t LowImpDataAccess;         /** Semaphore for accesing the LoWImportanceData structure. */

    QueueHandle_t errorQueue;
}OpSys;

#endif