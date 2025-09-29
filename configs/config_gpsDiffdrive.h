/**
 * @file config.h
 * @brief File containing configuration variables for the project.
 */

#ifndef CONFIG_H
#define CONFIG_H

    // ----- RTOS SETTINGS ----- //

    #define COMM_TASK_FREQ 10.0         /** Frequency at which the main loop of the communication task runs. */
    #define ODOM_TASK_FREQ 100.0        /** Frequency at which the main loop of the odometry calculation task runs. */
    #define WHEEL_CTRL_TASK_FREQ 10.0   /** Frequency at which the main loop of the wheel controling task runs. */
    #define ACCESORY_TASK_FREQ 5.0      /** Frequency at which the main loop of the accesory controling task runs. */

    // ----- MOTOR CONTROL HARDWARE ----- //

    /*
    *   Declare what class of motion system the robot has.
    *
    *   Possible options:
    *   1   -   Class (2, 0)
    *   2   -   Class (1, 1)
    */
    #define ROBOT_CLASS 1

    /*
    *   You need to declare what kind of motor drivers the robot uses.
    *
    *   Possible options:
    *   1   -   PWM and DIR pins line a Cytron MDD20A dual channel motor driver 
    *   2   -   two PWM pins like a BTS7960 IBT2 dual channel motor driver
    */
    #define CONTROL_SCHEME 1

    #if ROBOT_CLASS == 1
        #define FLIP_LEFT_MOTOR_DIRECTION true
        #define FLIP_RIGHT_MOTOR_DIRECTION false
    #elif ROBOT_CLASS == 2
        #define FLIP_FL_MOTOR_DIRECTION false
        #define FLIP_FR_MOTOR_DIRECTION true
        #define FLIP_RL_MOTOR_DIRECTION false
        #define FLIP_RR_MOTOR_DIRECTION true
    #endif

    // ----- ENCODER SETTINGS ----- //

    #define ENABLE_HALF_STEPS true      // Set to true to enable tracking of rotary encoder at half step resolution.
    #define RESET_AT          0         // Set to a positive non-zero number to reset the position if this value is exceeded.
    #if ROBOT_CLASS == 1
        #define FLIP_DIRECTION true         // Set to true to reverse the clockwise/counterclockwise sense.
        #define ENCODER_STEPS 1024.0
    #elif ROBOT_CLASS == 2
        #define FL_FLIP_DIRECTION true      // Set to true to reverse the clockwise/counterclockwise sense for front left wheel.
        #define FR_FLIP_DIRECTION false     // Set to true to reverse the clockwise/counterclockwise sense for front right wheel.
        #define RL_FLIP_DIRECTION true      // Set to true to reverse the clockwise/counterclockwise sense for rear left wheel.
        #define RR_FLIP_DIRECTION false     // Set to true to reverse the clockwise/counterclockwise sense for rear right wheel.
        #define STEERING_FLIP_DIRECTION true

        #define FL_ENCODER_STEPS 4096.0
        #define FR_ENCODER_STEPS 1024.0
        #define RL_ENCODER_STEPS 4096.0
        #define RR_ENCODER_STEPS 1024.0
    #endif

    #define RATIO 1.7778            //Ratio between motor and wheel (9.7800 for new robot, 1.7778 for old one).

    // ----- PID SETTINGS ----- //

    #define PID_LIMIT 1024          // Maximumm value of a PID regulator
    #define PID_ERROR_LIMIT 1024.0  // Maximum value of an error integral for the PID regulator.

    // ----- MISCELLANEOUS SETTINGS ----- //

    #define ROBOT_ID "RD"           //Two letter ID of the robot
    #define ERROR_QUEUE_SIZE 1024   //Maximu size of the error queue.

    /**
     * Declare what type of accessories are connected.
     * 
     * Possible options:
     * 0    -   None        - no accesory is connected.
     * 1    -   Binary      - the accesory is handled by a digital pin.
     * 2    -   Analogue    - the accesory is handled by a DAC capable pin and uses analogue signal.
     */
    #define ACCESORY_1_TYPE 0

    /**
     * Declare what type of accessories are connected.
     * 
     * Possible options:
     * 0    -   None        - no accesory is connected.
     * 1    -   Binary      - the accesory is handled by a digital pin.
     * 2    -   Analogue    - the accesory is handled by a DAC capable pin and uses analogue signal.
     */
    #define ACCESORY_2_TYPE 0

    #define ACCESORY_1_BASE_STATE 1 //Starting state of the pin controlling the accesory 1
    //#define ACCESORY_2_BASE_STATE 1 //Starting state of the pin controlling the accesory 2

    //#define SAFETY_SWITCH

    #define DEBUG_MSSGS             //Send human readable messages that do not conform to the formal standard. Comment this option for deployment.

#endif