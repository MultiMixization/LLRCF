#include "pinout.h"

extern RobotData mainDataStruct;
extern RobotConstants constantsDataStruct;
extern OpSyS SystemHandles;

void pinInit()
{
    gpio_set_direction(ACCESORY_PORT_1, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(ACCESORY_PORT_1, 1);

    //gpio_set_direction(ACCESORY_PORT_2, GPIO_MODE_OUTPUT_OD);

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

    //esp_rom_gpio_pad_select_gpio(LEFT_MOTOR_PWM);
    //gpio_set_direction(LEFT_MOTOR_PWM, GPIO_MODE_OUTPUT);

    esp_rom_gpio_pad_select_gpio(RIGHT_MOTOR_DIR);
    gpio_set_direction(RIGHT_MOTOR_DIR, GPIO_MODE_OUTPUT);

    //esp_rom_gpio_pad_select_gpio(RIGHT_MOTOR_PWM);
    //gpio_set_direction(RIGHT_MOTOR_PWM, GPIO_MODE_OUTPUT);

    gpio_install_isr_service(0);
}

void dataInit()
{
    xSemaphoreTake(SystemHandles.RobotDataAccess, portMAX_DELAY);
    mainDataStruct.left_wheel_position = 0.0;
    mainDataStruct.left_wheel_speed = 0.0;
    mainDataStruct.left_wheel_target_speed = 0.0;
    mainDataStruct.left_wheel_acceleration = 0.0;

    mainDataStruct.right_wheel_position = 0.0;
    mainDataStruct.right_wheel_speed = 0.0;
    mainDataStruct.right_wheel_target_speed = 0.0;
    mainDataStruct.right_wheel_acceleration = 0.0;

    mainDataStruct.pos_x = 0.0;
    mainDataStruct.pos_y = 0.0;
    mainDataStruct.pos_z = 0.0;

    Quaternion_setIdentity(&mainDataStruct.orientation);
    xSemaphoreGive(SystemHandles.RobotDataAccess);
    SystemHandles.errorQueue = xQueueCreate(ERROR_QUEUE_SIZE, sizeof(esp_err_t));
}

void readConstantsFromFlash()                           //Reads constant values from flash
{
    nvs_handle_t flashHandle;
    size_t dataSize;

    nvs_open("storage", NVS_READONLY, &flashHandle);

    nvs_get_blob(flashHandle, "robotConstants", &constantsDataStruct, &dataSize);
    nvs_close(flashHandle);   
}

void saveConstantsToFlash()                             //Saves current Constant values to flash
{
    nvs_handle_t flashHandle;
    
    nvs_open("storage", NVS_READWRITE, &flashHandle);
    xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
    nvs_set_blob(flashHandle, "robotConstants", &constantsDataStruct, sizeof(RobotConstants));
    xSemaphoreGive(SystemHandles.ConstantDataAccess);
    nvs_commit(flashHandle);
    nvs_close(flashHandle);
}

void resetFlash()                                       //Sets constants to known values and saves them to flash
{
    xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
    constantsDataStruct.wheel_separation = 1.0;
    constantsDataStruct.wheel_size = 0.1;
    constantsDataStruct.KP = 100.0;
    constantsDataStruct.KI = 10.0;
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