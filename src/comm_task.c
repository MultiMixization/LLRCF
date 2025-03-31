#include "comm_task.h"

extern RobotData mainDataStruct;
extern RobotConstants constantsDataStruct;
extern OpSyS SystemHandles;

void comm_task()
{
    char outputFrame[BUFF_SIZE];
    char inputData[BUFF_SIZE];
    char temp1[6], temp2[6], temp3;
    char cmd[3];
    int frameLength;
    esp_err_t temp = ESP_OK;

    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    int intr_alloc_flags = 0;

    #if CONFIG_UART_ISR_IN_IRAM
        intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    #endif

    checkError(uart_driver_install(UART_PORT_NUM, BUFF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    checkError(uart_param_config(UART_PORT_NUM, &uart_config));
    checkError(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    SystemHandles.comm_last_wakeup = xTaskGetTickCount();

    while(true)
    {
        frameLength = uart_read_bytes(UART_PORT_NUM, inputData, BUFF_SIZE-1, 20);
        if(frameLength > 0)
        {
            if(strncmp(inputData, "RD;", 3) == 0)
            {
                strncpy(cmd, inputData + 3, 3);
                if(strcmp(cmd, "RST") == 0)
                {
                    sprintf(outputFrame, "Reseting ESP32...\n");
                    uart_write_bytes(UART_PORT_NUM, outputFrame, strlen(outputFrame));
                    vTaskDelay(100);
                    esp_restart();
                }
                else if (strcmp(cmd, "WHD") == 0)                                    //Change wheel dimension
                {
                    strncpy(temp1, inputData + 7, 6);
                    xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                    constantsDataStruct.wheel_size = strtof(temp1, NULL);
                    sprintf(outputFrame, "Changed wheel size to %6.2f\n", constantsDataStruct.wheel_size);
                    xSemaphoreGive(SystemHandles.ConstantDataAccess);
                    saveConstantsToFlash();
                }
                else if (strcmp(cmd, "WHS") == 0)                                    //Change wheel separation distance
                {
                    strncpy(temp1, inputData + 8, 6);
                    xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                    constantsDataStruct.wheel_separation = strtof(temp1, NULL);
                    sprintf(outputFrame, "Changed wheel separation distance to %6.2f\n", constantsDataStruct.wheel_separation);
                    xSemaphoreGive(SystemHandles.ConstantDataAccess);
                    saveConstantsToFlash();
                }
                else if (strcmp(cmd, "DSC") == 0)                                    //Change wheel separation distance
                {
                    xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                    sprintf(outputFrame, "Current constants\nWheel size: %7.3f\nWheel spacing: %7.3f\nKP: %7.3f\nKI: %7.3f\n", constantsDataStruct.wheel_size, constantsDataStruct.wheel_separation, constantsDataStruct.KP, constantsDataStruct.KI);
                    xSemaphoreGive(SystemHandles.ConstantDataAccess);
                }
                else if (strcmp(cmd, "FRS") == 0)                                    //Flash memory reset
                {
                    sprintf(outputFrame, "Reseting flash memory to known values...\n");
                    resetFlash();
                }
                else if (strcmp(cmd, "RSO") == 0)                                    //Robot data reset
                {
                    sprintf(outputFrame, "Reseting robot odometry...\n");
                    dataInit();
                }
                else if (strcmp(cmd, "CKP") == 0)                                    //Robot data reset
                {
                    strncpy(temp1, inputData + 7, 7);
                    xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                    constantsDataStruct.KP = strtof(temp1, NULL);
                    sprintf(outputFrame, "Changed KP to %7.3f\n", constantsDataStruct.KP);
                    xSemaphoreGive(SystemHandles.ConstantDataAccess);
                    saveConstantsToFlash();
                }
                else if (strcmp(cmd, "CKI") == 0)                                    //Robot data reset
                {
                    strncpy(temp1, inputData + 7, 7);
                    xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                    constantsDataStruct.KI = strtof(temp1, NULL);
                    sprintf(outputFrame, "Changed KI to %7.3f\n", constantsDataStruct.KI);
                    xSemaphoreGive(SystemHandles.ConstantDataAccess);
                    saveConstantsToFlash();
                }
                else if (strcmp(cmd, "SSL") == 0)                                    //Set left wheel speed
                {
                    strncpy(temp1, inputData + 7, 7);
                    xSemaphoreTake(SystemHandles.RobotDataAccess, portMAX_DELAY);
                    mainDataStruct.left_wheel_target_speed = strtof(temp1, NULL);
                    sprintf(outputFrame, "Setting left wheel speed to %7.3f\n", mainDataStruct.left_wheel_target_speed * 100.0);
                    xSemaphoreGive(SystemHandles.RobotDataAccess);
                }
                else if (strcmp(cmd, "SSR") == 0)                                    //Set right wheel speed
                {
                    strncpy(temp1, inputData + 7, 7);
                    xSemaphoreTake(SystemHandles.RobotDataAccess, portMAX_DELAY);
                    mainDataStruct.right_wheel_target_speed = strtof(temp1, NULL);
                    sprintf(outputFrame, "Setting right wheel speed to %7.3f\n", mainDataStruct.right_wheel_target_speed * 100.0);
                    xSemaphoreGive(SystemHandles.RobotDataAccess);
                }
                else if (strcmp(cmd, "CTR") == 0)                                    //Main control frame
                {
                    strncpy(temp1, inputData + 7, 6);
                    strncpy(temp2, inputData + 14, 6);
                    strncpy(&temp3, inputData + 21, 1);
                    gpio_set_level(ACCESORY_PORT_1, !((bool)strtol(&temp3, NULL, 10)));
                    xSemaphoreTake(SystemHandles.RobotDataAccess, portMAX_DELAY);
                    xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                    mainDataStruct.right_wheel_target_speed = (strtof(temp1, NULL) + strtof(temp2, NULL) * constantsDataStruct.wheel_separation / 2.0);
                    mainDataStruct.left_wheel_target_speed = (strtof(temp1, NULL) - strtof(temp2, NULL) * constantsDataStruct.wheel_separation / 2.0);
                    xSemaphoreGive(SystemHandles.ConstantDataAccess);
                    sprintf(outputFrame, "RD;%6.2f;%6.2f;%6.2f;%6.3f;%6.3f;%6.3f;%6.3f\n", mainDataStruct.pos_x, mainDataStruct.pos_y, mainDataStruct.pos_z, mainDataStruct.orientation.v[0], mainDataStruct.orientation.v[1], mainDataStruct.orientation.v[2], mainDataStruct.orientation.w);
                    xSemaphoreGive(SystemHandles.RobotDataAccess);
                }
                else if (strcmp(cmd, "ACC") == 0)
                {
                    strncpy(temp1, inputData + 7, 1);
                    strncpy(temp2, inputData + 9, 1);
                    if(strtol(temp1, NULL, 10) == 1)
                    {
                        gpio_set_level(ACCESORY_PORT_1, !((bool)strtol(temp2, NULL, 10)));
                    }
                    /*else if(strtol(temp1, NULL, 10) == 2)
                    {
                        gpio_set_level(ACCESORY_PORT_2, strtol(temp2, NULL, 10));
                    }*/
                   sprintf(outputFrame, "Switching accesory %d state to %d ...\n", strtol(temp1, NULL, 10) == 1, strtol(temp2, NULL, 10) == 1);
                }
            }
        }
        else
        {
            xSemaphoreTake(SystemHandles.RobotDataAccess, portMAX_DELAY);
            sprintf(outputFrame, "RD;%6.2f;%6.2f;%6.2f;%6.3f;%6.3f;%6.3f;%6.3f\n", mainDataStruct.pos_x, mainDataStruct.pos_y, mainDataStruct.pos_z, mainDataStruct.orientation.v[0], mainDataStruct.orientation.v[1], mainDataStruct.orientation.v[2], mainDataStruct.orientation.w);
            xSemaphoreGive(SystemHandles.RobotDataAccess);
        }
        uart_write_bytes(UART_PORT_NUM, outputFrame, strlen(outputFrame));

        xSemaphoreTake(SystemHandles.OpSysAccess, portMAX_DELAY);
        while(uxQueueMessagesWaiting(SystemHandles.errorQueue)>0)
        {
            xQueueReceive(SystemHandles.errorQueue, &temp, portMAX_DELAY);
            sprintf(outputFrame, "ER;%s\n", esp_err_to_name(temp));
            uart_write_bytes(UART_PORT_NUM, outputFrame, strlen(outputFrame));
        }
        xSemaphoreGive(SystemHandles.OpSysAccess);
        
        xTaskDelayUntil(&SystemHandles.comm_last_wakeup, 100);
    }
}