/**
 * @file comm_task.c
 * @brief File containing implementation of the communication task.
 */

#include "comm_task.h"

extern RobotData mainDataStruct;
extern RobotConstants constantsDataStruct;
extern OpSys SystemHandles;

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
            if(strncmp(inputData, ROBOT_ID, 2) == 0)
            {
                strncpy(cmd, inputData + 3, 3);
                if(strcmp(cmd, "RST") == 0)
                {
                    #ifdef DEBUG_MSSGS
                        sprintf(outputFrame, "Reseting ESP32...\n");
                    #endif
                    uart_write_bytes(UART_PORT_NUM, outputFrame, strlen(outputFrame));
                    vTaskDelay(100);
                    esp_restart();
                }
                else if (strcmp(cmd, "RSO") == 0)                   //Robot data reset
                {
                    #ifdef DEBUG_MSSGS
                        sprintf(outputFrame, "Reseting robot odometry...\n");
                    #endif
                    dataInit();
                }
                else if (strcmp(cmd, "FRS") == 0)                   //Flash memory reset
                {
                    #ifdef DEBUG_MSSGS
                        sprintf(outputFrame, "Reseting flash memory to known values...\n");
                    #endif
                        resetFlash();
                }
                else if (strcmp(cmd, "ACC") == 0)                   //Accesory control
                {
                    strncpy(temp1, inputData + 7, 1);
                    strncpy(temp2, inputData + 9, 1);
                    switch (strtol(temp1, NULL, 10))
                    {
                    case 1:
                        #if ACCESORY_1_TYPE != 0
                        
                            #ifdef DEBUG_MSSGS
                                sprintf(outputFrame, "Switching accesory 1 state to %d ...\n", !((bool)strtol(temp2, NULL, 10)));
                            #endif
                        #endif
                        break;
                    case 2:
                        #if ACCESORY_2_TYPE != 0
                            
                            #ifdef DEBUG_MSSGS
                                sprintf(outputFrame, "Switching accesory 2 state to %d ...\n", !((bool)strtol(temp2, NULL, 10)));
                            #endif
                        #endif
                        break;
                    default:
                        #ifdef DEBUG_MSSGS
                            sprintf(outputFrame, "Unexisting accesory...\n");
                        #endif
                        break;
                    }
                }
                #if ROBOT_CLASS == 1    
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
                else if (strcmp(cmd, "WHD") == 0)                                    //Change wheel dimension
                {
                    strncpy(temp1, inputData + 7, 6);
                    xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                    constantsDataStruct.wheel_size = strtof(temp1, NULL);
                    #ifdef DEBUG_MSSGS
                        sprintf(outputFrame, "Changed wheel size to %6.2f\n", constantsDataStruct.wheel_size);
                    #endif
                    xSemaphoreGive(SystemHandles.ConstantDataAccess);
                    saveConstantsToFlash();
                }
                else if (strcmp(cmd, "WHS") == 0)                                    //Change wheel separation distance
                {
                    strncpy(temp1, inputData + 8, 6);
                    xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                    constantsDataStruct.wheel_separation = strtof(temp1, NULL);
                    #ifdef DEBUG_MSSGS 
                        sprintf(outputFrame, "Changed wheel separation distance to %6.2f\n", constantsDataStruct.wheel_separation);
                    #endif
                    xSemaphoreGive(SystemHandles.ConstantDataAccess);
                    saveConstantsToFlash();
                }
                else if (strcmp(cmd, "DSC") == 0)                                    //Display constants
                {
                    #ifdef DEBUG_MSSGS
                        xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                        sprintf(outputFrame, "Current constants\nWheel size: %7.3f\nWheel spacing: %7.3f\nKP: %7.3f\nKI: %7.3f\n", constantsDataStruct.wheel_size, constantsDataStruct.wheel_separation, constantsDataStruct.KP, constantsDataStruct.KI);
                        xSemaphoreGive(SystemHandles.ConstantDataAccess);
                    #endif
                }
                else if (strcmp(cmd, "CKP") == 0)                                    //Change KP
                {
                    strncpy(temp1, inputData + 7, 7);
                    xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                    constantsDataStruct.KP = strtof(temp1, NULL);
                    #ifdef DEBUG_MSSGS
                        sprintf(outputFrame, "Changed KP to %7.3f\n", constantsDataStruct.KP);
                    #endif
                    xSemaphoreGive(SystemHandles.ConstantDataAccess);
                    saveConstantsToFlash();
                }
                else if (strcmp(cmd, "CKI") == 0)                                    //Change KI
                {
                    strncpy(temp1, inputData + 7, 7);
                    xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                    constantsDataStruct.KI = strtof(temp1, NULL);
                    #ifdef DEBUG_MSSGS
                        sprintf(outputFrame, "Changed KI to %7.3f\n", constantsDataStruct.KI);
                    #endif
                    xSemaphoreGive(SystemHandles.ConstantDataAccess);
                    saveConstantsToFlash();
                }
                else if (strcmp(cmd, "SSL") == 0)                                    //Set left wheel speed
                {
                    strncpy(temp1, inputData + 7, 7);
                    xSemaphoreTake(SystemHandles.RobotDataAccess, portMAX_DELAY);
                    mainDataStruct.left_wheel_target_speed = strtof(temp1, NULL);
                    #ifdef DEBUG_MSSGS
                        sprintf(outputFrame, "Setting left wheel speed to %7.3f\n", mainDataStruct.left_wheel_target_speed * 100.0);
                    #endif
                    xSemaphoreGive(SystemHandles.RobotDataAccess);
                }
                else if (strcmp(cmd, "SSR") == 0)                                    //Set right wheel speed
                {
                    strncpy(temp1, inputData + 7, 7);
                    xSemaphoreTake(SystemHandles.RobotDataAccess, portMAX_DELAY);
                    mainDataStruct.right_wheel_target_speed = strtof(temp1, NULL);
                    #ifdef DEBUG_MSSGS
                        sprintf(outputFrame, "Setting right wheel speed to %7.3f\n", mainDataStruct.right_wheel_target_speed * 100.0);
                    #endif
                    xSemaphoreGive(SystemHandles.RobotDataAccess);
                }
                #elif ROBOT_CLASS == 2
                else if (strcmp(cmd, "CKP") == 0)                   /** Change value of P coefficient for angle or speed controling PI controller. */
                {
                    strncpy(temp1, inputData + 7, 1);
                    strncpy(temp2, inputData + 9, 7);
                    if (strtol(temp1, NULL, 10) == 1)
                    {
                        xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                        constantsDataStruct.velocity_KP = strtof(temp2, NULL);
                        #ifdef DEBUG_MSSGS
                            sprintf(outputFrame, "Changed velocity KP to %7.3f\n", constantsDataStruct.velocity_KP);
                        #endif
                        xSemaphoreGive(SystemHandles.ConstantDataAccess);
                        saveConstantsToFlash();
                    }
                    else if (strtol(temp1, NULL, 10) == 2)
                    {
                        xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                        constantsDataStruct.angle_KP = strtof(temp2, NULL);
                        #ifdef DEBUG_MSSGS
                            sprintf(outputFrame, "Changed angle KP to %7.3f\n", constantsDataStruct.angle_KP);
                        #endif
                        xSemaphoreGive(SystemHandles.ConstantDataAccess);
                        saveConstantsToFlash();
                    }
                }
                else if (strcmp(cmd, "CKI") == 0)                   /** Change value of I coefficient for angle or speed controling PI controller. */
                {
                    strncpy(temp1, inputData + 7, 1);
                    strncpy(temp2, inputData + 9, 7);
                    if (strtol(temp1, NULL, 10) == 1)
                    {
                        xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                        constantsDataStruct.velocity_KI = strtof(temp2, NULL);
                        #ifdef DEBUG_MSSGS
                            sprintf(outputFrame, "Changed velocity KI to %7.3f\n", constantsDataStruct.velocity_KI);
                        #endif
                        xSemaphoreGive(SystemHandles.ConstantDataAccess);
                        saveConstantsToFlash();
                    }
                    else if (strtol(temp1, NULL, 10) == 2)
                    {
                        xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                        constantsDataStruct.angle_KI = strtof(temp2, NULL);
                        #ifdef DEBUG_MSSGS
                            sprintf(outputFrame, "Changed angle KI to %7.3f\n", constantsDataStruct.angle_KI);
                        #endif
                        xSemaphoreGive(SystemHandles.ConstantDataAccess);
                        saveConstantsToFlash();
                    }
                }
                else if (strcmp(cmd, "WHD") == 0)                   /** Change sizes of front or back wheels. */
                {
                    strncpy(temp1, inputData + 7, 1);
                    strncpy(temp2, inputData + 9, 7);
                    switch (strtol(temp1, NULL, 10))
                    {
                    case 1:
                        xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                        constantsDataStruct.front_wheel_size = strtof(temp2, NULL);
                        #ifdef DEBUG_MSSGS
                            sprintf(outputFrame, "Changed front wheels sizes to %7.4f\n", constantsDataStruct.front_wheel_size);
                        #endif
                        xSemaphoreGive(SystemHandles.ConstantDataAccess);
                        saveConstantsToFlash();
                        break;
                    case 2:
                        xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                        constantsDataStruct.rear_wheel_size = strtof(temp2, NULL);
                        #ifdef DEBUG_MSSGS
                            sprintf(outputFrame, "Changed rear wheels sizes to %7.4f\n", constantsDataStruct.rear_wheel_size);
                        #endif
                        xSemaphoreGive(SystemHandles.ConstantDataAccess);
                        saveConstantsToFlash();
                        break;
                    default:
                        break;
                    }
                }
                else if (strcmp(cmd, "WHS") == 0)                   /** Change separation of front or back wheels. */
                {
                    strncpy(temp1, inputData + 7, 1);
                    strncpy(temp2, inputData + 9, 7);
                    switch (strtol(temp1, NULL, 10))
                    {
                    case 1:
                        xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                        constantsDataStruct.front_wheel_separation = strtof(temp2, NULL);
                        #ifdef DEBUG_MSSGS
                            sprintf(outputFrame, "Changed front wheels separation to %7.4f\n", constantsDataStruct.front_wheel_separation);
                        #endif
                        xSemaphoreGive(SystemHandles.ConstantDataAccess);
                        saveConstantsToFlash();
                        break;
                    case 2:
                        xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                        constantsDataStruct.rear_wheel_separation = strtof(temp2, NULL);
                        #ifdef DEBUG_MSSGS
                            sprintf(outputFrame, "Changed rear wheels separation to %7.4f\n", constantsDataStruct.rear_wheel_separation);
                        #endif
                        xSemaphoreGive(SystemHandles.ConstantDataAccess);
                        saveConstantsToFlash();
                        break;
                    default:
                        break;
                    }
                }
                else if (strcmp(cmd, "WJL") == 0)                   /** Change distance between axles. */
                {
                    strncpy(temp1, inputData + 8, 6);
                    xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                    constantsDataStruct.front_wheel_longitudinal_distance = strtof(temp1, NULL);
                    #ifdef DEBUG_MSSGS
                        sprintf(outputFrame, "Changed distance between axles to %7.4f\n", constantsDataStruct.front_wheel_longitudinal_distance);
                    #endif
                    xSemaphoreGive(SystemHandles.ConstantDataAccess);
                    saveConstantsToFlash();
                }
                else if (strcmp(cmd, "CAO") == 0)                   /** Change steering angle offset. */
                {
                    strncpy(temp1, inputData + 8, 6);
                    xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                    constantsDataStruct.angle_offset = strtof(temp1, NULL);
                    #ifdef DEBUG_MSSGS
                        sprintf(outputFrame, "Changed steering angle offset to %7.4f\n", constantsDataStruct.angle_offset);
                    #endif
                    xSemaphoreGive(SystemHandles.ConstantDataAccess);
                    saveConstantsToFlash();
                }
                else if (strcmp(cmd, "DSC") == 0)                   /** Display current robot constants. */
                {
                    #ifdef DEBUG_MSSGS
                        xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                        sprintf(outputFrame, "Current constants\nFront wheel size: %7.3f\nFront wheel spacing: %7.3f\nRear wheel_size: %7.3f\nRear_wheel_separation: %7.3f\nVelocity KP: %7.3f\nVelocity KI: %7.3f\n", constantsDataStruct.front_wheel_size, constantsDataStruct.front_wheel_separation, constantsDataStruct.rear_wheel_size, constantsDataStruct.rear_wheel_separation, constantsDataStruct.velocity_KP, constantsDataStruct.velocity_KI);
                        xSemaphoreGive(SystemHandles.ConstantDataAccess);
                    #endif
                }
                else if (strcmp(cmd, "CTR") == 0)                   /** Main control frame. */
                {
                    strncpy(temp1, inputData + 7, 6);
                    strncpy(temp2, inputData + 14, 6);
                    strncpy(&temp3, inputData + 21, 1);
                    #if ACCESORY_1_TYPE != 0
                    gpio_set_level(ACCESORY_PORT_1, !((bool)strtol(&temp3, NULL, 10)));
                    #endif
                    xSemaphoreTake(SystemHandles.RobotDataAccess, portMAX_DELAY);
                    xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
                    mainDataStruct.FL_target_speed = strtof(temp1, NULL);
                    mainDataStruct.FR_target_speed = strtof(temp1, NULL);
                    mainDataStruct.RL_target_speed = strtof(temp1, NULL);
                    mainDataStruct.RR_target_speed = strtof(temp1, NULL);   //Dorzucić kiedyś różne prędkości kół przy skrętach (elektroniczny dyferencjał)
                    mainDataStruct.steering_target_position = strtof(temp2, NULL);
                    xSemaphoreGive(SystemHandles.ConstantDataAccess);
                    sprintf(outputFrame, "RD;%6.2f;%6.2f;%6.2f;%6.3f;%6.3f;%6.3f;%6.3f\n", mainDataStruct.pos_x, mainDataStruct.pos_y, mainDataStruct.pos_z, mainDataStruct.orientation.v[0], mainDataStruct.orientation.v[1], mainDataStruct.orientation.v[2], mainDataStruct.orientation.w);
                    xSemaphoreGive(SystemHandles.RobotDataAccess);
                }
                #else
                    #error Unrecognized robot class.
                #endif
            }
        }
        else                                                        /** Default - no command, only send data. */
        {
            xSemaphoreTake(SystemHandles.RobotDataAccess, portMAX_DELAY);
            sprintf(outputFrame, "%s;%6.2f;%6.2f;%6.2f;%6.3f;%6.3f;%6.3f;%6.3f\n", ROBOT_ID, mainDataStruct.RL_speed, mainDataStruct.RR_speed, mainDataStruct.pos_z, mainDataStruct.orientation.v[0], mainDataStruct.orientation.v[1], mainDataStruct.orientation.v[2], mainDataStruct.orientation.w);
            xSemaphoreGive(SystemHandles.RobotDataAccess);
        }
        uart_write_bytes(UART_PORT_NUM, outputFrame, strlen(outputFrame));

        xSemaphoreTake(SystemHandles.OpSysAccess, portMAX_DELAY);
        while(uxQueueMessagesWaiting(SystemHandles.errorQueue) > 0)
        {
            xQueueReceive(SystemHandles.errorQueue, &temp, portMAX_DELAY);
            sprintf(outputFrame, "%s;ERR;%s\n", ROBOT_ID, esp_err_to_name(temp));
            uart_write_bytes(UART_PORT_NUM, outputFrame, strlen(outputFrame));
        }
        xSemaphoreGive(SystemHandles.OpSysAccess);
        
        xTaskDelayUntil(&SystemHandles.comm_last_wakeup, pdMS_TO_TICKS(1000 / COMM_TASK_FREQ));
    }
}