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
#include "accesory_task.h"

RobotData mainDataStruct;
RobotConstants constantsDataStruct;
LowImportantData nonImportantDataStruct;
OpSys SystemHandles;

void app_main()
{
    pinInit();
    nvs_flash_init();

    SystemHandles.RobotDataAccess = xSemaphoreCreateMutex();
    SystemHandles.ConstantDataAccess = xSemaphoreCreateMutex();
    SystemHandles.OpSysAccess = xSemaphoreCreateMutex();
    SystemHandles.LowImpDataAccess = xSemaphoreCreateMutex();

    dataInit();
    readConstantsFromFlash();
    
    xTaskCreatePinnedToCore(comm_task, "comm_task", 4096, NULL, 2, &SystemHandles.comm_task_handler, 1);
    xTaskCreatePinnedToCore(odom_task, "odom_task", 4096, NULL, 1, &SystemHandles.odom_task_handler, 1);
    xTaskCreatePinnedToCore(whlctrl_task, "whlctrl_task", 4096, NULL, 1, &SystemHandles.whlctrl_task_handler, 0);
    xTaskCreatePinnedToCore(accesory_task, "accesory_task", 4096, NULL, 3, &SystemHandles.accesory_task_handler, 0);
}