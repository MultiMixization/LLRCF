#include "whlctrl_task.h"

extern RobotData mainDataStruct;
extern RobotConstants constantsDataStruct;
extern OpSyS SystemHandles;

void whlctrl_task()
{
    int16_t left_ctrl_value = 0, right_ctrl_value = 0;

    float left_error_integral = 0.0;
    float right_error_integral = 0.0;

    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 20000,  // Set output frequency at 20 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    checkError(ledc_timer_config(&timer));

    #ifndef ALTERNATIVE_CONTROL_SCHEME
        ledc_channel_config_t left_motor = {
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = LEDC_CHANNEL_0,
            .timer_sel      = LEDC_TIMER_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = LEFT_MOTOR_PWM,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
        };
        checkError(ledc_channel_config(&left_motor));

        ledc_channel_config_t right_motor = {
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = LEDC_CHANNEL_1,
            .timer_sel      = LEDC_TIMER_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = RIGHT_MOTOR_PWM,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
        };
        checkError(ledc_channel_config(&right_motor));
    #endif

    #ifdef ALTERNATIVE_CONTROL_SCHEME
        ledc_channel_config_t left_motor_forward = {
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = LEDC_CHANNEL_0,
            .timer_sel      = LEDC_TIMER_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = LEFT_MOTOR_PWM,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
        };
        checkError(ledc_channel_config(&left_motor_forward));

        ledc_channel_config_t left_motor_backwards = {
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = LEDC_CHANNEL_1,
            .timer_sel      = LEDC_TIMER_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = LEFT_MOTOR_DIR,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
        };
        checkError(ledc_channel_config(&left_motor_backwards));

        ledc_channel_config_t right_motor_forward = {
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = LEDC_CHANNEL_2,
            .timer_sel      = LEDC_TIMER_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = RIGHT_MOTOR_PWM,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
        };
        checkError(ledc_channel_config(&right_motor_forward));

        ledc_channel_config_t right_motor_backwards = {
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = LEDC_CHANNEL_3,
            .timer_sel      = LEDC_TIMER_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = RIGHT_MOTOR_DIR,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
        };
        checkError(ledc_channel_config(&right_motor_backwards));

    #endif

    SystemHandles.whlctrl_last_wakeup = xTaskGetTickCount();

    while(true)
    {
        xSemaphoreTake(SystemHandles.RobotDataAccess, portMAX_DELAY);
        left_error_integral += (mainDataStruct.left_wheel_target_speed - mainDataStruct.left_wheel_speed);
        right_error_integral += (mainDataStruct.right_wheel_target_speed - mainDataStruct.right_wheel_speed);

        if(abs(mainDataStruct.left_wheel_speed < 0.001) && left_ctrl_value == 0) left_error_integral = 0.0;
        if(abs(mainDataStruct.right_wheel_speed < 0.001 ) && right_ctrl_value == 0) right_error_integral = 0.0;

        xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
        left_ctrl_value = constantsDataStruct.KP * (mainDataStruct.left_wheel_target_speed - mainDataStruct.left_wheel_speed) + (constantsDataStruct.KI * left_error_integral);
        right_ctrl_value = constantsDataStruct.KP * (mainDataStruct.right_wheel_target_speed - mainDataStruct.right_wheel_speed) + (constantsDataStruct.KI * right_error_integral);
        xSemaphoreGive(SystemHandles.ConstantDataAccess); 
        xSemaphoreGive(SystemHandles.RobotDataAccess);

        if(left_ctrl_value >= PID_LIMIT) left_ctrl_value = PID_LIMIT;
        if(left_ctrl_value <= -PID_LIMIT) left_ctrl_value = -PID_LIMIT;

        if(right_ctrl_value >= PID_LIMIT) right_ctrl_value = PID_LIMIT;
        if(right_ctrl_value <= -PID_LIMIT) right_ctrl_value = -PID_LIMIT;

        if(left_error_integral >= PID_ERROR_LIMIT) left_ctrl_value = PID_ERROR_LIMIT;
        if(left_error_integral <= -PID_ERROR_LIMIT) left_ctrl_value = -PID_ERROR_LIMIT;

        if(right_error_integral >= PID_ERROR_LIMIT) right_error_integral = PID_ERROR_LIMIT;
        if(right_error_integral <= -PID_ERROR_LIMIT) right_error_integral = -PID_ERROR_LIMIT;

        #ifndef ALTERNATIVE_CONTROL_SCHEME
            if(left_ctrl_value > 0)
            {
                gpio_set_level(LEFT_MOTOR_DIR, 0);
            }
            else
            {
                gpio_set_level(LEFT_MOTOR_DIR, 1);
            }

            if (right_ctrl_value > 0)
            {
                gpio_set_level(RIGHT_MOTOR_DIR, 1);
            }
            else
            {
                gpio_set_level(RIGHT_MOTOR_DIR, 0);
            }
        checkError(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs(left_ctrl_value)));
        checkError(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, abs(right_ctrl_value)));
        checkError(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
        checkError(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
        #endif

        #ifdef ALTERNATIVE_CONTROL_SCHEME
            if(left_ctrl_value > 0)
            {
                checkError(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0));
                checkError(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1)); 
                checkError(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs(left_ctrl_value)));
                checkError(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
            }
            else
            {
                checkError(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0));
                checkError(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
                checkError(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, abs(left_ctrl_value)));
                checkError(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1)); 
            }

            if (right_ctrl_value > 0)
            {
                checkError(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0));
                checkError(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3)); 
                checkError(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, abs(right_ctrl_value)));
                checkError(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2));
            }
            else
            {
                checkError(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0));
                checkError(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2));
                checkError(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, abs(right_ctrl_value)));
                checkError(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3)); 
            }
        #endif

        xTaskDelayUntil(&SystemHandles.whlctrl_last_wakeup, 10);
    }
}