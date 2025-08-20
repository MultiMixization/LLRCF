#ifndef CONFIG_H
#define CONFIG_H

    // ----- ENCODER SETTINGS ----- //

    #define ENABLE_HALF_STEPS true  // Set to true to enable tracking of rotary encoder at half step resolution.
    #define RESET_AT          0     // Set to a positive non-zero number to reset the position if this value is exceeded.
    #define FLIP_DIRECTION    true  // Set to true to reverse the clockwise/counterclockwise sense.

    #define ENCODER_STEPS 2048.0
    #define RATIO 1.7778            //Ratio between motor and wheel (9.7800 for new robot, 1.7778 for old one).

    // ----- PID SETTINGS ----- //

    #define PID_LIMIT 1024          // Maximumm value of a PID regulator
    #define PID_ERROR_LIMIT 1024.0  // Maximum value of an error integral for the PID regulator.

    // ----- MOTOR CONTROL HARDWARE ----- //

    /*
    *   Declare what class of motion system the robot has.
    *
    *   Possible options:
    *   1   -   Class (2, 0)
    *   2   -   Class (1, 1)
    */
    #define ROBOT_CLASS 2

    /*
    *   You need to declare what kind of motor drivers the robot uses.
    *
    *   Possible options:
    *   1   -   Cytron MDD20A dual channel motor driver 
    *   2   -   BTS7960 IBT2 dual channel motor driver
    */
    #define CONTROL_SCHEME 1

    // ----- MISCELLANEOUS SETTINGS ----- //

    #define ROBOT_ID "RD"           //Two letter ID of the robot
    #define ERROR_QUEUE_SIZE 1024   //Maximu size of the error queue.

    #define ACCESORY_1_BASE_STATE 1 //State of the pin controlling the accesory 1
    #define ACCESORY_2_BASE_STATE 1 //State of the pin controlling the accesory 2

    #define DEBUG_MSSGS             //Send human readable messages that do not conform to the formal standard. Comment this option for deployment.

#endif