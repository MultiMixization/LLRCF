/**
 * @file odom_task.c
 * @brief File containing the implementation of the odometry calculation task.
 */

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

            mainDataStruct.left_wheel_speed = (mainDataStruct.left_wheel_position - left_wheel_prev_position) * ODOM_TASK_FREQ;
            mainDataStruct.right_wheel_speed = (mainDataStruct.right_wheel_position - right_wheel_prev_position) * ODOM_TASK_FREQ;

            mainDataStruct.left_wheel_acceleration = (mainDataStruct.left_wheel_speed - left_wheel_prev_speed) * ODOM_TASK_FREQ;
            mainDataStruct.right_wheel_acceleration = (mainDataStruct.right_wheel_speed - right_wheel_prev_speed) * ODOM_TASK_FREQ;

            left_wheel_prev_position = mainDataStruct.left_wheel_position;
            right_wheel_prev_position = mainDataStruct.right_wheel_position;

            left_wheel_prev_speed = mainDataStruct.left_wheel_speed;
            right_wheel_prev_speed = mainDataStruct.right_wheel_speed;

            rot_diff = (mainDataStruct.right_wheel_speed - mainDataStruct.left_wheel_speed) / (wheel_separation * ODOM_TASK_FREQ);
            Quaternion_toEulerZYX(&mainDataStruct.orientation, curr_eul);
            Quaternion_setIdentity(&rot_diff_quat);
            Quaternion_fromZRotation(rot_diff, &rot_diff_quat);
            mainDataStruct.pos_x += (mainDataStruct.left_wheel_speed + mainDataStruct.right_wheel_speed) / 2.0 * cos(curr_eul[2] + rot_diff / 2.0) / ODOM_TASK_FREQ;
            mainDataStruct.pos_y += (mainDataStruct.left_wheel_speed + mainDataStruct.right_wheel_speed) / 2.0 * sin(curr_eul[2] + rot_diff / 2.0) / ODOM_TASK_FREQ;

            Quaternion_multiply(&rot_diff_quat, &mainDataStruct.orientation, &mainDataStruct.orientation);

            xSemaphoreGive(SystemHandles.RobotDataAccess);
            xTaskDelayUntil(&SystemHandles.odom_last_wakeup, 10);
        }
    }
#elif ROBOT_CLASS == 2
    void odom_task()
    {
        float front_wheel_size, rear_wheel_size, front_wheel_longitudinal_distance;
        float FL_prev_position = 0.0, FR_prev_position = 0.0, RL_prev_position = 0.0, RR_prev_position = 0.0, steering_prev_position = 0.0;
        float FL_prev_speed = 0.0, FR_prev_speed = 0.0, RL_prev_speed = 0.0, RR_prev_speed = 0.0;
        float rot_diff = 0.0, avg_speed = 0.0, curr_eul[3];
        Quaternion rot_diff_quat;

        Quaternion_setIdentity(&rot_diff_quat);

        rotary_encoder_info_t FL = { 0 };
        rotary_encoder_info_t FR = { 0 };
        rotary_encoder_info_t RL = { 0 };
        rotary_encoder_info_t RR = { 0 };
        rotary_encoder_state_t state_FL = { 0 };
        rotary_encoder_state_t state_FR = { 0 };
        rotary_encoder_state_t state_RL = { 0 };
        rotary_encoder_state_t state_RR = { 0 };
        QueueHandle_t FL_event_queue = rotary_encoder_create_queue();
        QueueHandle_t FR_event_queue = rotary_encoder_create_queue();
        QueueHandle_t RL_event_queue = rotary_encoder_create_queue();
        QueueHandle_t RR_event_queue = rotary_encoder_create_queue();

        checkError(rotary_encoder_init(&FL, FL_ENCODER_A, FL_ENCODER_B));
        checkError(rotary_encoder_init(&FR, FR_ENCODER_A, FR_ENCODER_B));
        checkError(rotary_encoder_init(&RL, RL_ENCODER_A, RL_ENCODER_B));
        checkError(rotary_encoder_init(&RR, RR_ENCODER_A, RR_ENCODER_B));

        checkError(rotary_encoder_enable_half_steps(&FL, ENABLE_HALF_STEPS));
        checkError(rotary_encoder_enable_half_steps(&FR, ENABLE_HALF_STEPS));
        checkError(rotary_encoder_enable_half_steps(&RL, ENABLE_HALF_STEPS));
        checkError(rotary_encoder_enable_half_steps(&RR, ENABLE_HALF_STEPS));

        #if FL_FLIP_DIRECTION == true
            checkError(rotary_encoder_flip_direction(&FL));
        #endif
        #if FR_FLIP_DIRECTION == true
            checkError(rotary_encoder_flip_direction(&FR));
        #endif
        #if RL_FLIP_DIRECTION == true
            checkError(rotary_encoder_flip_direction(&RL));
        #endif
        #if RR_FLIP_DIRECTION == true
            checkError(rotary_encoder_flip_direction(&RR));
        #endif

        checkError(rotary_encoder_set_queue(&FL, FL_event_queue));
        checkError(rotary_encoder_set_queue(&FR, FR_event_queue));
        checkError(rotary_encoder_set_queue(&RL, RL_event_queue));
        checkError(rotary_encoder_set_queue(&RR, RR_event_queue));

        xSemaphoreTake(SystemHandles.ConstantDataAccess, portMAX_DELAY);
        front_wheel_size = 2.0 * M_PI * constantsDataStruct.front_wheel_size;
        rear_wheel_size = 2.0 * M_PI * constantsDataStruct.rear_wheel_size;
        front_wheel_longitudinal_distance = constantsDataStruct.front_wheel_longitudinal_distance;

        AS5040_info angle_sensor = {
            .pin_clk = ANGLE_SENSOR_CLK,
            .pin_cs = ANGLE_SENSOR_CS,
            .pin_data = ANGLE_SENSOR_DATA,
            .offset = constantsDataStruct.angle_offset
        };
        xSemaphoreGive(SystemHandles.ConstantDataAccess);

        AS5040_init(&angle_sensor);

        SystemHandles.odom_last_wakeup = xTaskGetTickCount();

        while (true)
        {
            checkError(rotary_encoder_get_state(&FL, &state_FL));
            checkError(rotary_encoder_get_state(&FR, &state_FR));
            checkError(rotary_encoder_get_state(&RL, &state_RL));
            checkError(rotary_encoder_get_state(&RR, &state_RR));

            xSemaphoreTake(SystemHandles.RobotDataAccess, portMAX_DELAY);
            mainDataStruct.FL_position = (float)state_FL.position / (FL_ENCODER_STEPS * RATIO) * front_wheel_size;
            mainDataStruct.FR_position = (float)state_FR.position / (FR_ENCODER_STEPS * RATIO) * front_wheel_size;
            mainDataStruct.RL_position = (float)state_RL.position / (RL_ENCODER_STEPS * RATIO) * rear_wheel_size;
            mainDataStruct.RR_position = (float)state_RR.position / (RR_ENCODER_STEPS * RATIO) * rear_wheel_size;

            mainDataStruct.FL_speed = (mainDataStruct.FL_position - FL_prev_position) * ODOM_TASK_FREQ;
            mainDataStruct.FR_speed = (mainDataStruct.FR_position - FR_prev_position) * ODOM_TASK_FREQ;
            mainDataStruct.RL_speed = (mainDataStruct.RL_position - RL_prev_position) * ODOM_TASK_FREQ;
            mainDataStruct.RR_speed = (mainDataStruct.RR_position - RR_prev_position) * ODOM_TASK_FREQ;

            mainDataStruct.FL_acceleration = (mainDataStruct.FL_speed - FL_prev_speed) * ODOM_TASK_FREQ;
            mainDataStruct.FR_acceleration = (mainDataStruct.FR_speed - FR_prev_speed) * ODOM_TASK_FREQ;
            mainDataStruct.RL_acceleration = (mainDataStruct.RL_speed - RL_prev_speed) * ODOM_TASK_FREQ;
            mainDataStruct.RR_acceleration = (mainDataStruct.RR_speed - RR_prev_speed) * ODOM_TASK_FREQ;

            AS5040_read(&angle_sensor, &mainDataStruct.steering_position);
            mainDataStruct.steering_speed = (mainDataStruct.steering_position - steering_prev_position) * ODOM_TASK_FREQ;

            FL_prev_position = mainDataStruct.FL_position;
            FR_prev_position = mainDataStruct.FR_position;
            RL_prev_position = mainDataStruct.RL_position;
            RR_prev_position = mainDataStruct.RR_position;
            steering_prev_position = mainDataStruct.steering_position;

            FL_prev_speed = mainDataStruct.FL_speed;
            FR_prev_speed = mainDataStruct.FR_speed;
            RL_prev_speed = mainDataStruct.RL_speed;
            RR_prev_speed = mainDataStruct.RR_speed;

            avg_speed = (mainDataStruct.FL_speed + mainDataStruct.FR_speed + mainDataStruct.RL_speed + mainDataStruct.RR_speed) / 4.0;

            rot_diff = avg_speed * tan(mainDataStruct.steering_position) / (front_wheel_longitudinal_distance * ODOM_TASK_FREQ);

            Quaternion_toEulerZYX(&mainDataStruct.orientation, curr_eul);
            Quaternion_setIdentity(&rot_diff_quat);
            Quaternion_fromZRotation(rot_diff, &rot_diff_quat);

            mainDataStruct.pos_x = avg_speed * cos(curr_eul[2] + rot_diff) / ODOM_TASK_FREQ;
            mainDataStruct.pos_y = avg_speed * sin(curr_eul[2] + rot_diff) / ODOM_TASK_FREQ; // Taken from Springers Handbook of Robotics 2nd edition page 1239

            Quaternion_multiply(&rot_diff_quat, &mainDataStruct.orientation, &mainDataStruct.orientation);
            xSemaphoreGive(SystemHandles.RobotDataAccess);

            xTaskDelayUntil(&SystemHandles.odom_last_wakeup, pdMS_TO_TICKS(1000 / ODOM_TASK_FREQ));
        }
    }

#else
    #error Unrecognized robot class.
#endif