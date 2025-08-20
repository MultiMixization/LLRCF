#include "odom_task.h"

extern RobotData mainDataStruct;
extern RobotConstants constantsDataStruct;
extern OpSys SystemHandles;

#if ROBOT_CLASS == 1
    void odom_task()
    {
        float wheel_size, wheel_separation, rot_diff, left_wheel_prev_position = 0.0, right_wheel_prev_position = 0.0, left_wheel_prev_speed = 0.0, right_wheel_prev_speed = 0.0, curr_eul[3];
        Quaternion rot_diff_quat;

        Quaternion_setIdentity(&rot_diff_quat);

        rotary_encoder_info_t left = { 0 };
        rotary_encoder_info_t right = { 0 };
        rotary_encoder_state_t state_left = { 0 };
        rotary_encoder_state_t state_right = { 0 };
        QueueHandle_t left_event_queue = rotary_encoder_create_queue();
        QueueHandle_t right_event_queue = rotary_encoder_create_queue();

        checkError(rotary_encoder_init(&left, LEFT_ENCODER_A, LEFT_ENCODER_B));
        checkError(rotary_encoder_init(&right, RIGHT_ENCODER_A, RIGHT_ENCODER_B));
        checkError(rotary_encoder_enable_half_steps(&left, ENABLE_HALF_STEPS));
        checkError(rotary_encoder_enable_half_steps(&right, ENABLE_HALF_STEPS));
        checkError(rotary_encoder_flip_direction(&left));
        checkError(rotary_encoder_flip_direction(&right));

        checkError(rotary_encoder_set_queue(&left, left_event_queue));
        checkError(rotary_encoder_set_queue(&right, right_event_queue));

        xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
        wheel_size = 2.0 * M_PI * constantsDataStruct.wheel_size;
        wheel_separation = constantsDataStruct.wheel_separation;
        xSemaphoreGive(SystemHandles.ConstantDataAccess);

        SystemHandles.odom_last_wakeup = xTaskGetTickCount();

        while(true)
        {
            checkError(rotary_encoder_get_state(&left, &state_left));
            checkError(rotary_encoder_get_state(&right, &state_right));
            xSemaphoreTake(SystemHandles.RobotDataAccess, portMAX_DELAY);

            mainDataStruct.left_wheel_position = (float)state_left.position / (ENCODER_STEPS * RATIO) * wheel_size;
            mainDataStruct.right_wheel_position = (float)state_right.position / (ENCODER_STEPS * RATIO) * wheel_size;

            mainDataStruct.left_wheel_speed = (mainDataStruct.left_wheel_position - left_wheel_prev_position) * 100.0;
            mainDataStruct.right_wheel_speed = (mainDataStruct.right_wheel_position - right_wheel_prev_position) * 100.0;

            mainDataStruct.left_wheel_acceleration = (mainDataStruct.left_wheel_speed - left_wheel_prev_speed) * 100.0;
            mainDataStruct.right_wheel_acceleration = (mainDataStruct.right_wheel_speed - right_wheel_prev_speed) * 100.0;

            left_wheel_prev_position = mainDataStruct.left_wheel_position;
            right_wheel_prev_position = mainDataStruct.right_wheel_position;

            left_wheel_prev_speed = mainDataStruct.left_wheel_speed;
            right_wheel_prev_speed = mainDataStruct.right_wheel_speed;

            rot_diff = (mainDataStruct.right_wheel_speed - mainDataStruct.left_wheel_speed) / wheel_separation * 0.01;
            Quaternion_toEulerZYX(&mainDataStruct.orientation, curr_eul);
            Quaternion_setIdentity(&rot_diff_quat);
            Quaternion_fromZRotation(rot_diff, &rot_diff_quat);
            mainDataStruct.pos_x += (mainDataStruct.left_wheel_speed + mainDataStruct.right_wheel_speed) / 2.0 * cos(curr_eul[2] + rot_diff / 2.0) * 0.01;
            mainDataStruct.pos_y += (mainDataStruct.left_wheel_speed + mainDataStruct.right_wheel_speed) / 2.0 * sin(curr_eul[2] + rot_diff / 2.0) * 0.01;

            Quaternion_multiply(&rot_diff_quat, &mainDataStruct.orientation, &mainDataStruct.orientation);

            xSemaphoreGive(SystemHandles.RobotDataAccess);
            xTaskDelayUntil(&SystemHandles.odom_last_wakeup, 10);
        }
    }
#elif ROBOT_CLASS == 2
    void odom_task()
    {
        
    }

#else
    #error Unrecognized robot class.
#endif