# LLRCF - Low Level Robot Control Framework

Low Level Robot Control Framework is an ESP32 based framework that allows for control over a robot using ROS and ROS2 frameworks via serial port connection

# Configuration

To configure the framework to fit your implementation please change values in include/config.h

Premade configuration files are availeable in the config folder. To use them simply replace the config.h file in the include folder with a selected configuration file.

# Communication frame

The software uses a simple ASCII frames to communicate with other devices.
The output frame is structured like this:

Position    Type        Description
1-2         char        Robot ID
3           float       Position x
4           float       Position y
5           float       Position z
6           float       Rotation x
7           float       Rotation y
8           float       Rotation z
9           float       Rotation w 

The input frame used to control the robot is structured like this:

Position    Type        Desccription
1-2         char        Robot ID
3-6         char        Command ID
7           int/float   Optional value

If the ESP reports an error it will send a frame like this:

The input frame used to control the robot is structured like this:

Position    Type        Desccription
1-2         char        Robot ID
5-6         char        Constant words "ERR"
8-?         char        Error identifier

# Building

This software was compiled and tested using Visual Studio Code with PlatformIO extension and Espressif 32 embedded platform version 6.10.