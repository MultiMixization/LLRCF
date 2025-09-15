/**
 * @file whlctrl_task.cpp
 * @brief File holding the implementation of the wheel control task.
 * @
 */

#include "whlctrl_task.h"

extern RobotData mainDataStruct;
extern RobotConstants constantsDataStruct;
extern OpSys SystemHandles;

#if ROBOT_CLASS == 1
    void whlctrl_task()
    {
        int16_t left_ctrl_value = 0, right_ctrl_value = 0;
        float left_error_integral = 0.0, right_error_integral = 0.0;

        ledc_timer_config_t timer = {
            .speed_mode       = LEDC_LOW_SPEED_MODE,
            .duty_resolution  = LEDC_TIMER_10_BIT,
            .timer_num        = LEDC_TIMER_0,
            .freq_hz          = 20000,  // Set output frequency at 20 kHz
            .clk_cfg          = LEDC_AUTO_CLK
        };
        checkError(ledc_timer_config(&timer));

        #if CONTROL_SCHEME == 1
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

        #elif CONTROL_SCHEME == 2
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

        #else
            #error Unrecognized control scheme.
        #endif

        SystemHandles.whlctrl_last_wakeup = xTaskGetTickCount();

        while(true)
        {
            xSemaphoreTake(SystemHandles.RobotDataAccess, portMAX_DELAY);
            left_error_integral += (mainDataStruct.left_wheel_target_speed - mainDataStruct.left_wheel_speed);
            right_error_integral += (mainDataStruct.right_wheel_target_speed - mainDataStruct.right_wheel_speed);

            if(fabs(mainDataStruct.left_wheel_speed) < 0.01 && fabs(mainDataStruct.left_wheel_target_speed) < 0.01) left_error_integral = 0.0;
            if(fabs(mainDataStruct.right_wheel_speed) < 0.01 && fabs(mainDataStruct.right_wheel_target_speed) < 0.01) right_error_integral = 0.0;

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

            #if CONTROL_SCHEME == 1
                if(left_ctrl_value > 0)
                {
                    gpio_set_level(LEFT_MOTOR_DIR, !FLIP_LEFT_MOTOR_DIRECTION);
                }
                else
                {
                    gpio_set_level(LEFT_MOTOR_DIR, FLIP_LEFT_MOTOR_DIRECTION);
                }

                if (right_ctrl_value > 0)
                {
                    gpio_set_level(RIGHT_MOTOR_DIR, !FLIP_RIGHT_MOTOR_DIRECTION);
                }
                else
                {
                    gpio_set_level(RIGHT_MOTOR_DIR, FLIP_RIGHT_MOTOR_DIRECTION);
                }
                checkError(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs(left_ctrl_value)));
                checkError(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, abs(right_ctrl_value)));
                checkError(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
                checkError(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));

            #elif CONTROL_SCHEME == 2
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

#elif ROBOT_CLASS == 2
    void whlctrl_task()
    {
        int16_t FL_control_value = 0, FR_control_value = 0, RL_control_value = 0, RR_control_value = 0, steering_control_value = 0;
        float FL_error_integral = 0.0, FR_error_integral = 0.0, RL_error_integral = 0.0, RR_error_integral = 0.0, steering_error_integral = 0;

        ledc_timer_config_t timer = {
            .speed_mode       = LEDC_LOW_SPEED_MODE,
            .duty_resolution  = LEDC_TIMER_10_BIT,
            .timer_num        = LEDC_TIMER_0,
            .freq_hz          = 20000,  // Set output frequency at 20 kHz
            .clk_cfg          = LEDC_AUTO_CLK
        };
        checkError(ledc_timer_config(&timer));

        #if CONTROL_SCHEME == 1
            ledc_channel_config_t FL_motor = {
                .speed_mode     = LEDC_LOW_SPEED_MODE,
                .channel        = LEDC_CHANNEL_0,
                .timer_sel      = LEDC_TIMER_0,
                .intr_type      = LEDC_INTR_DISABLE,
                .gpio_num       = FL_MOTOR_PWM,
                .duty           = 0, // Set duty to 0%
                .hpoint         = 0
            };
            checkError(ledc_channel_config(&FL_motor));

            ledc_channel_config_t FR_motor = {
                .speed_mode     = LEDC_LOW_SPEED_MODE,
                .channel        = LEDC_CHANNEL_1,
                .timer_sel      = LEDC_TIMER_0,
                .intr_type      = LEDC_INTR_DISABLE,
                .gpio_num       = FR_MOTOR_PWM,
                .duty           = 0, // Set duty to 0%
                .hpoint         = 0
            };
            checkError(ledc_channel_config(&FR_motor));

            ledc_channel_config_t RL_motor = {
                .speed_mode     = LEDC_LOW_SPEED_MODE,
                .channel        = LEDC_CHANNEL_2,
                .timer_sel      = LEDC_TIMER_0,
                .intr_type      = LEDC_INTR_DISABLE,
                .gpio_num       = RL_MOTOR_PWM,
                .duty           = 0, // Set duty to 0%
                .hpoint         = 0
            };
            checkError(ledc_channel_config(&RL_motor));

            ledc_channel_config_t RR_motor = {
                .speed_mode     = LEDC_LOW_SPEED_MODE,
                .channel        = LEDC_CHANNEL_3,
                .timer_sel      = LEDC_TIMER_0,
                .intr_type      = LEDC_INTR_DISABLE,
                .gpio_num       = RR_MOTOR_PWM,
                .duty           = 0, // Set duty to 0%
                .hpoint         = 0
            };
            checkError(ledc_channel_config(&RR_motor));

            ledc_channel_config_t steering_actuator = {
                .speed_mode     = LEDC_LOW_SPEED_MODE,
                .channel        = LEDC_CHANNEL_4,
                .timer_sel      = LEDC_TIMER_0,
                .intr_type      = LEDC_INTR_DISABLE,
                .gpio_num       = STEERING_MOTOR_PWM,
                .duty           = 0, // Set duty to 0%
                .hpoint         = 0
            };
            checkError(ledc_channel_config(&steering_actuator));

        #elif CONTROL_SCHEME == 2
            #error Not enough PWM channels on this uC.
        #else
            #error Unrecognized control scheme.
        #endif

        SystemHandles.whlctrl_last_wakeup = xTaskGetTickCount();

        while(true)
        {
            xSemaphoreTake(SystemHandles.RobotDataAccess, portMAX_DELAY);
            xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
            FL_error_integral += (mainDataStruct.FL_target_speed - mainDataStruct.FL_speed);
            FR_error_integral += (mainDataStruct.FR_target_speed - mainDataStruct.FR_speed);
            RL_error_integral += (mainDataStruct.RL_target_speed - mainDataStruct.RL_speed);
            RR_error_integral += (mainDataStruct.RR_target_speed - mainDataStruct.RR_speed);
            steering_error_integral += (mainDataStruct.steering_position - mainDataStruct.steering_target_position);

            FL_control_value = constantsDataStruct.velocity_KP * (mainDataStruct.FL_target_speed - mainDataStruct.FL_speed) + constantsDataStruct.velocity_KI * FL_error_integral + constantsDataStruct.velocity_KD * (0);
            FR_control_value = constantsDataStruct.velocity_KP * (mainDataStruct.FR_target_speed - mainDataStruct.FR_speed) + constantsDataStruct.velocity_KI * FR_error_integral + constantsDataStruct.velocity_KD * (0);
            RL_control_value = constantsDataStruct.velocity_KP * (mainDataStruct.RL_target_speed - mainDataStruct.RL_speed) + constantsDataStruct.velocity_KI * RL_error_integral + constantsDataStruct.velocity_KD * (0);
            RR_control_value = constantsDataStruct.velocity_KP * (mainDataStruct.RR_target_speed - mainDataStruct.RR_speed) + constantsDataStruct.velocity_KI * RR_error_integral + constantsDataStruct.velocity_KD * (0);
            steering_control_value = constantsDataStruct.angle_KP * (mainDataStruct.steering_position - mainDataStruct.steering_target_position) + constantsDataStruct.angle_KI * steering_error_integral;

            xSemaphoreGive(SystemHandles.RobotDataAccess);
            xSemaphoreGive(SystemHandles.ConstantDataAccess); 

            if(FL_control_value >= PID_LIMIT) FL_control_value = PID_LIMIT;
            if(FL_control_value <= -PID_LIMIT) FL_control_value = -PID_LIMIT;

            if(FR_control_value >= PID_LIMIT) FR_control_value = PID_LIMIT;
            if(FR_control_value <= -PID_LIMIT) FR_control_value = -PID_LIMIT;

            if(RL_control_value >= PID_LIMIT) RL_control_value = PID_LIMIT;
            if(RL_control_value <= -PID_LIMIT) RL_control_value = -PID_LIMIT;

            if(RR_control_value >= PID_LIMIT) RR_control_value = PID_LIMIT;
            if(RR_control_value <= -PID_LIMIT) RR_control_value = -PID_LIMIT;

            if(steering_control_value >= PID_LIMIT) steering_control_value = PID_LIMIT;
            if(steering_control_value <= -PID_LIMIT) steering_control_value = -PID_LIMIT;

            if(FL_error_integral >= PID_ERROR_LIMIT) FL_error_integral = PID_ERROR_LIMIT;
            if(FL_error_integral <= -PID_ERROR_LIMIT) FL_error_integral = -PID_ERROR_LIMIT;

            if(FR_error_integral >= PID_ERROR_LIMIT) FR_error_integral = PID_ERROR_LIMIT;
            if(FR_error_integral <= -PID_ERROR_LIMIT) FR_error_integral = -PID_ERROR_LIMIT;

            if(RL_error_integral >= PID_ERROR_LIMIT) RL_error_integral = PID_ERROR_LIMIT;
            if(RL_error_integral <= -PID_ERROR_LIMIT) RL_error_integral = -PID_ERROR_LIMIT;

            if(RR_error_integral >= PID_ERROR_LIMIT) RR_error_integral = PID_ERROR_LIMIT;
            if(RR_error_integral <= -PID_ERROR_LIMIT) RR_error_integral = -PID_ERROR_LIMIT;

            if(steering_error_integral >= PID_ERROR_LIMIT) steering_error_integral = PID_ERROR_LIMIT;
            if(steering_error_integral <= -PID_ERROR_LIMIT) steering_error_integral = -PID_ERROR_LIMIT;

            #ifdef SAFETY_SWITCH
                xSemaphoreTake(SystemHandles.RobotDataAccess, portMAX_DELAY);
                mainDataStruct.safety_switch_state = gpio_get_level(SAFETY_SWITCH_PIN);
                if(mainDataStruct.safety_switch_state)
                {
                    FL_control_value = 0.0;
                    FL_error_integral = 0.0;
                    FR_control_value = 0.0;
                    FR_error_integral = 0.0;
                    RL_control_value = 0.0;
                    RL_error_integral = 0.0;
                    RR_control_value = 0.0;
                    RR_error_integral = 0.0;
                }
                xSemaphoreGive(SystemHandles.RobotDataAccess);
            #endif  

            #if CONTROL_SCHEME == 1
                FL_control_value >= 0 ? gpio_set_level(FL_MOTOR_DIR, !FLIP_FL_MOTOR_DIRECTION) : gpio_set_level(FL_MOTOR_DIR, FLIP_FL_MOTOR_DIRECTION);
                FR_control_value >= 0 ? gpio_set_level(FR_MOTOR_DIR, !FLIP_FR_MOTOR_DIRECTION) : gpio_set_level(FR_MOTOR_DIR, FLIP_FR_MOTOR_DIRECTION);
                RL_control_value >= 0 ? gpio_set_level(RL_MOTOR_DIR, !FLIP_RL_MOTOR_DIRECTION) : gpio_set_level(RL_MOTOR_DIR, FLIP_RL_MOTOR_DIRECTION);
                RR_control_value >= 0 ? gpio_set_level(RR_MOTOR_DIR, !FLIP_RR_MOTOR_DIRECTION) : gpio_set_level(RR_MOTOR_DIR, FLIP_RR_MOTOR_DIRECTION);
                steering_control_value >= 0 ? gpio_set_level(STEERING_MOTOR_DIR, 0) : gpio_set_level(STEERING_MOTOR_DIR, 1);

                checkError(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs(FL_control_value)));
                checkError(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, abs(FR_control_value)));
                checkError(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, abs(RL_control_value)));
                checkError(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, abs(RR_control_value)));
                checkError(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, abs(steering_control_value)));
                checkError(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
                checkError(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
                checkError(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2));
                checkError(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3));
                checkError(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4));
            #elif CONTROL_SCHEME == 2
                #error Not enough PWM channels on this uC.
            #else 
                #error Unrecognized control scheme.
            #endif
            xTaskDelayUntil(&SystemHandles.whlctrl_last_wakeup, pdMS_TO_TICKS(1000 / WHEEL_CTRL_TASK_FREQ));
        }
    }
#else
    #error Unrecognized robot class.
#endif