#include <esp_system.h>
#include <driver/gpio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "pinout.h"
#include "comm_task.h"
#include "odom_task.h"
#include "whlctrl_task.h"
#include "data_structures.h"

RobotData mainDataStruct;
RobotConstants constantsDataStruct;
OpSyS SystemHandles;

void app_main()
{
    pinInit();
    nvs_flash_init();

    SystemHandles.RobotDataAccess = xSemaphoreCreateMutex();
    SystemHandles.ConstantDataAccess = xSemaphoreCreateMutex();
    SystemHandles.OpSysAccess = xSemaphoreCreateMutex();

    dataInit(&mainDataStruct);
    readConstantsFromFlash(&constantsDataStruct);
    
    xTaskCreatePinnedToCore(comm_task, "comm_task", 4096, NULL, 1, &SystemHandles.comm_task_handler, 1);
    xTaskCreatePinnedToCore(odom_task, "odom_task", 4096, NULL, 2, &SystemHandles.odom_task_handler, 1);
    xTaskCreatePinnedToCore(whlctrl_task, "whlctrl_task", 4096, NULL, 2, &SystemHandles.whlctrl_task_handler, 0);
}