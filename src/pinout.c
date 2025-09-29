/**
 * @file pinout.c
 * @brief File containing implementation of the functions from the pinout.h file.
 */

#include "pinout.h"

extern RobotData mainDataStruct;
extern RobotConstants constantsDataStruct;
extern OpSys SystemHandles;

void pinInit()
{
    #if ROBOT_CLASS == 1
        esp_rom_gpio_pad_select_gpio(LEFT_ENCODER_A);
        gpio_set_direction(LEFT_ENCODER_A, GPIO_MODE_INPUT);

        esp_rom_gpio_pad_select_gpio(LEFT_ENCODER_B);
        gpio_set_direction(LEFT_ENCODER_B, GPIO_MODE_INPUT);

        esp_rom_gpio_pad_select_gpio(RIGHT_ENCODER_A);
        gpio_set_direction(RIGHT_ENCODER_A, GPIO_MODE_INPUT);

        esp_rom_gpio_pad_select_gpio(RIGHT_ENCODER_B);
        gpio_set_direction(RIGHT_ENCODER_B, GPIO_MODE_INPUT);

        esp_rom_gpio_pad_select_gpio(LEFT_MOTOR_DIR);
        gpio_set_direction(LEFT_MOTOR_DIR, GPIO_MODE_OUTPUT);

        esp_rom_gpio_pad_select_gpio(RIGHT_MOTOR_DIR);
        gpio_set_direction(RIGHT_MOTOR_DIR, GPIO_MODE_OUTPUT);
    #elif ROBOT_CLASS == 2
        //----------------------- ENCODERS -----------------------
        esp_rom_gpio_pad_select_gpio(FL_ENCODER_A);
        gpio_set_direction(FL_ENCODER_A, GPIO_MODE_INPUT);

        esp_rom_gpio_pad_select_gpio(FL_ENCODER_B);
        gpio_set_direction(FL_ENCODER_B, GPIO_MODE_INPUT);

        esp_rom_gpio_pad_select_gpio(FR_ENCODER_A);
        gpio_set_direction(FR_ENCODER_A, GPIO_MODE_INPUT);

        esp_rom_gpio_pad_select_gpio(FR_ENCODER_B);
        gpio_set_direction(FR_ENCODER_B, GPIO_MODE_INPUT);

        esp_rom_gpio_pad_select_gpio(RL_ENCODER_A);
        gpio_set_direction(RL_ENCODER_A, GPIO_MODE_INPUT);

        esp_rom_gpio_pad_select_gpio(RL_ENCODER_B);
        gpio_set_direction(RL_ENCODER_B, GPIO_MODE_INPUT);

        esp_rom_gpio_pad_select_gpio(RR_ENCODER_A);
        gpio_set_direction(RR_ENCODER_A, GPIO_MODE_INPUT);

        esp_rom_gpio_pad_select_gpio(RR_ENCODER_B);
        gpio_set_direction(RR_ENCODER_B, GPIO_MODE_INPUT);

        //----------------------- MOTORS -----------------------

        esp_rom_gpio_pad_select_gpio(FL_MOTOR_DIR);
        gpio_set_direction(FL_MOTOR_DIR, GPIO_MODE_OUTPUT);

        esp_rom_gpio_pad_select_gpio(FR_MOTOR_DIR);
        gpio_set_direction(FR_MOTOR_DIR, GPIO_MODE_OUTPUT);

        esp_rom_gpio_pad_select_gpio(RL_MOTOR_DIR);
        gpio_set_direction(RL_MOTOR_DIR, GPIO_MODE_OUTPUT);

        esp_rom_gpio_pad_select_gpio(RR_MOTOR_DIR);
        gpio_set_direction(RR_MOTOR_DIR, GPIO_MODE_OUTPUT);

        esp_rom_gpio_pad_select_gpio(STEERING_MOTOR_DIR);
        gpio_set_direction(STEERING_MOTOR_DIR, GPIO_MODE_OUTPUT);
    #else
         #error Unrecognized robot class
    #endif

    #if ACCESORY_1_TYPE != 0
        //esp_rom_gpio_pad_select_gpio(ACCESORY_PORT_1);
        gpio_set_direction(ACCESORY_PORT_1, GPIO_MODE_OUTPUT_OD);
        gpio_set_level(ACCESORY_PORT_1, ACCESORY_1_BASE_STATE);
    #endif
    #if ACCESORY_2_TYPE != 0
        esp_rom_gpio_pad_select_gpio(ACCESORY_PORT_2);
        gpio_set_direction(ACCESORY_PORT_2, GPIO_MODE_OUTPUT_OD);
        gpio_set_level(ACCESORY_PORT_2, ACCESORY_2_BASE_STATE);
    #endif

    #ifdef SAFETY_SWITCH
        esp_rom_gpio_pad_select_gpio(SAFETY_SWITCH_PIN);
        gpio_set_pull_mode(SAFETY_SWITCH_PIN, GPIO_PULLDOWN_ONLY);
        gpio_set_direction(SAFETY_SWITCH_PIN, GPIO_MODE_INPUT);
    #endif

    gpio_install_isr_service(0);
}

void dataInit()
{
    xSemaphoreTake(SystemHandles.RobotDataAccess, portMAX_DELAY);
    #if ROBOT_CLASS == 1
        mainDataStruct.left_wheel_position = 0.0;
        mainDataStruct.left_wheel_speed = 0.0;
        mainDataStruct.left_wheel_target_speed = 0.0;
        mainDataStruct.left_wheel_acceleration = 0.0;

        mainDataStruct.right_wheel_position = 0.0;
        mainDataStruct.right_wheel_speed = 0.0;
        mainDataStruct.right_wheel_target_speed = 0.0;
        mainDataStruct.right_wheel_acceleration = 0.0;

    #elif ROBOT_CLASS == 2
        mainDataStruct.FL_acceleration = 0.0;
        mainDataStruct.FL_position = 0.0;
        mainDataStruct.FL_speed = 0.0;
        mainDataStruct.FL_target_speed = 0.0;

        mainDataStruct.FR_acceleration = 0.0;
        mainDataStruct.FR_position = 0.0;
        mainDataStruct.FR_speed = 0.0;
        mainDataStruct.FR_target_speed = 0.0;

        mainDataStruct.RL_acceleration = 0.0;
        mainDataStruct.RL_position = 0.0;
        mainDataStruct.RL_speed = 0.0;
        mainDataStruct.RL_target_speed = 0.0;

        mainDataStruct.RR_acceleration = 0.0;
        mainDataStruct.RR_position = 0.0;
        mainDataStruct.RR_speed = 0.0;
        mainDataStruct.RR_target_speed = 0.0;

        mainDataStruct.steering_acceleration = 0.0;
        mainDataStruct.steering_position = 0.0;
        mainDataStruct.steering_speed = 0.0;
        mainDataStruct.steering_target_position = 0.0;
    #else
         #error Unrecognized robot class
    #endif

    mainDataStruct.pos_x = 0.0;
    mainDataStruct.pos_y = 0.0;
    mainDataStruct.pos_z = 0.0;

    Quaternion_setIdentity(&mainDataStruct.orientation);

    #ifdef SAFETY_SWITCH
        mainDataStruct.safety_switch_state = false;
    #endif

    xSemaphoreGive(SystemHandles.RobotDataAccess);
    SystemHandles.errorQueue = xQueueCreate(ERROR_QUEUE_SIZE, sizeof(esp_err_t));
}

void readConstantsFromFlash()
{
    nvs_handle_t flashHandle;
    size_t dataSize;

    nvs_open("storage", NVS_READONLY, &flashHandle);

    nvs_get_blob(flashHandle, "robotConstants", &constantsDataStruct, &dataSize);
    nvs_close(flashHandle);   
}

void saveConstantsToFlash()
{
    nvs_handle_t flashHandle;
    
    nvs_open("storage", NVS_READWRITE, &flashHandle);
    xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
    nvs_set_blob(flashHandle, "robotConstants", &constantsDataStruct, sizeof(RobotConstants));
    xSemaphoreGive(SystemHandles.ConstantDataAccess);
    nvs_commit(flashHandle);
    nvs_close(flashHandle);
}

void resetFlash()
{
    xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
    #if ROBOT_CLASS == 1
        constantsDataStruct.wheel_separation = 1.0;
        constantsDataStruct.wheel_size = 0.1;
        constantsDataStruct.KP = 100.0;
        constantsDataStruct.KI = 10.0;
    #elif ROBOT_CLASS == 2
        constantsDataStruct.front_wheel_size = 0.1;
        constantsDataStruct.rear_wheel_size = 0.1;
        constantsDataStruct.front_wheel_separation = 1.0;
        constantsDataStruct.rear_wheel_separation = 1.0;
        constantsDataStruct.front_wheel_longitudinal_distance = 1.0;

        constantsDataStruct.velocity_KP = 100.0;
        constantsDataStruct.velocity_KI = 10.0;
        constantsDataStruct.velocity_KD = 1.0;

        constantsDataStruct.angle_KP = 100.0;
        constantsDataStruct.angle_KI = 10.0;
        constantsDataStruct.angle_KD = 1.0;

        constantsDataStruct.angle_offset = 0.0;
        constantsDataStruct.angle_max = 30.0;
        constantsDataStruct.angle_min = -30.0;
    #else
         #error Unrecognized robot class
    #endif
    xSemaphoreGive(SystemHandles.ConstantDataAccess);
    saveConstantsToFlash();
}

void checkError(esp_err_t state)
{
    if(state != ESP_OK)
    {
        xSemaphoreTake(SystemHandles.OpSysAccess, portMAX_DELAY);
        xQueueSend(SystemHandles.errorQueue, &state, portMAX_DELAY);
        xSemaphoreGive(SystemHandles.OpSysAccess);
    }
}