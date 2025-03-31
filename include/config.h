#ifndef CONFIG_H
#define CONFIG_H

    // ----- ENCODER SETTINGS ----- //

    #define ENABLE_HALF_STEPS true  // Set to true to enable tracking of rotary encoder at half step resolution.
    #define RESET_AT          0     // Set to a positive non-zero number to reset the position if this value is exceeded.
    #define FLIP_DIRECTION    true  // Set to true to reverse the clockwise/counterclockwise sense.

    #define ENCODER_STEPS 2048.0
    #define RATIO 1.7778            //Ratio between motor and wheel (9.7800 for new robot, 1.7778 for old one).

    // ----- PID SETTINGS ----- //

    #define PID_LIMIT 1024
    #define PID_ERROR_LIMIT 1024.0

    // ----- MOTOR CONTROL HARDWARE ----- //

    /*
    *   If the robot is a dual drive robot of class (2, 0) then you need to declare what motor drivers it uses.
    *
    *   Possible options:
    *   1   -   Cytron MDD20A dual channel motor driver  
    *   2   -   BTS7960 IBT2 dual channel motor driver
    */

    #define CONTROL_SCHEME 2

    // ----- MISCELLANEOUS SETTINGS ----- //

    #define ERROR_QUEUE_SIZE 1024   //Maximu size of the error queue.

#endif