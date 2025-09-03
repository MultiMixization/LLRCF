/**
 * @file accesory_task.c
 * @brief File containing implementation of the accesory control task.
 */

 #include <accesory_task.h>

 extern OpSys SystemHandles;
 extern LowImportantData nonImportantDataStruct;

 void accesory_task()
 {
    SystemHandles.accesory_last_wakeup = xTaskGetTickCount();
    while(true)
    {
        xSemaphoreTake(SystemHandles.LowImpDataAccess, portMAX_DELAY);
        #if ACCESORY_1_TYPE == 1
        gpio_set_level(ACCESORY_PORT_1, nonImportantDataStruct.accesory_1_state);
        #elif ACCESORY_1_TYPE == 2
            #warning Not implemented!
        #endif 
        #if ACCESORY_2_TYPE == 1
        gpio_set_level(ACCESORY_PORT_2, nonImportantDataStruct.accesory_2_state);
        #elif ACCESORY_2_TYPE == 2
            #warning Not implemented!
        #endif
        xSemaphoreGive(SystemHandles.LowImpDataAccess);
        xTaskDelayUntil(&SystemHandles.accesory_last_wakeup, pdMS_TO_TICKS(1000 / ACCESORY_TASK_FREQ));
    }
 }