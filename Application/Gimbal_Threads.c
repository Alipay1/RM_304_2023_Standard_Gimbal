#include "Gimbal_Threads.h"

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern uint8_t u1_rx_buf[256];
extern uint8_t u6_rx_buf[256];

void MotorCheckTask(void *pvParameters);
void BuzzerTask(void *pvParameters);
void CAN_Transmition_Thread(void *arg)
{
    TickType_t xStartupTick = xTaskGetTickCount();
    PID *pxPITCH_V = pid_get_struct_pointer(0, PITCH_MOTOR);
    PID *pxPITCH_A = pid_get_struct_pointer(1, PITCH_MOTOR);

    PID *pxYAW_V = pid_get_struct_pointer(0, YAW_MOTOR);
    PID *pxYAW_P = pid_get_struct_pointer(1, YAW_MOTOR);

    PID *FlywheelL = pid_get_struct_pointer(0, NORMAL_MOTOR);
    PID *FlywheelR = pid_get_struct_pointer(1, NORMAL_MOTOR);

    PID *Feeder_Wheel = pid_get_struct_pointer(2, NORMAL_MOTOR);

    ammo_booster_data *booster_info = get_ammo_booster_info_ptr();

    // TaskHandle_t YawMotorCheckTaskHandle;
    // TaskHandle_t PitMotorCheckTaskHandle;
    // TaskHandle_t FeederMotorCheckTaskHandle;

    const RC_ctrl_t *rc = get_remote_control_point();
    chassis_transported_controller_data *rc_chas = get_rc_data_from_chassis_pointer();
    motor_measure_t *Spin = get_measure_pointer(5);
    motor_measure_t *T = get_measure_pointer(YAW_MOTOR_NUM);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    // #ifdef PITCH_IN_RESPONSE_TEST_MODE
    // int counter = 0;

    // float Given_Value = 0;

    // #endif // PITCH_IN_RESPONSE_TEST_MODE

    PID_Setup();

    // xTaskCreate(MotorCheckTask, "MTC", 128, pxYAW_V, 0, &YawMotorCheckTaskHandle);
    // xTaskCreate(MotorCheckTask, "MTC1", 128, pxPITCH_V, 0, &PitMotorCheckTaskHandle);
    // xTaskCreate(MotorCheckTask, "MTC1", 128, Feeder_Wheel, 0, &FeederMotorCheckTaskHandle);
    buzzer_off();
    while (1)
    {
#ifdef PITCH_IN_RESPONSE_TEST_MODE
        counter++;
        if (counter == 500)
        {
            Given_Value = 5000;
        }
        else if (counter == 1000)
        {
            Given_Value = -5000;
            counter = 0;
        }
        FlywheelL->ideal = Given_Value;
        FlywheelR->ideal = -Given_Value;
        PID_Gimbal_Calculate();
        PID_YAW_Union_Calculate(pxYAW_V, pxYAW_P, Given_Value, false);
        // VisualScope_Output(T->speed_rpm, Given_Value, 0, 0);
        USART_Print("$%d %d %d;", (int)FlywheelL->actual, -(int)FlywheelR->actual, (int)FlywheelL->ideal, (int)FlywheelL->actual + (int)FlywheelR->actual);
#else
        // counter++;
        // if (counter == 500)
        // {
        //     Given_Value = 10.0f;
        // }
        // else if (counter == 1000)
        // {
        //     Given_Value = -10.0f;
        //     counter = 0;
        // }

        if ((rc_chas->mouse_integeral.y * 0.01f) > 15.0f)
        {
            (rc_chas->mouse_integeral.y) = 1500.0f;
        }
        else if ((rc_chas->mouse_integeral.y * 0.01f) < -23.0f)
        {
            (rc_chas->mouse_integeral.y) = -2300.0f;
        }

        if ((rc_chas->mouse.press_l == 1 || rc->rc.s[1] == 2))
        {
            Feeder_Wheel->ideal = SHOOTING_RATE;
            buzzer_on(500, 1500);
        }
        else if (rc_chas->mouse.press_r == 1)
        {
            Feeder_Wheel->ideal = -SHOOTING_RATE;
        }
        else
        {
            Feeder_Wheel->ideal = 0;
            buzzer_off();
        }

        PID_YAW_Union_Calculate(pxYAW_V, pxYAW_P, -(rc_chas->mouse_integeral.x) * 0.01f, true);
        PID_PITCH_Union_Calculate(pxPITCH_V, pxPITCH_A, (-rc_chas->mouse_integeral.y * 0.01f));

        // PID_PITCH_Union_Calculate(pxPITCH_V, pxPITCH_A,Given_Value);
        // PID_PITCH_Union_Calculate(pxPITCH_V, pxPITCH_A, Given_Value);
        PID_Gimbal_Calculate();
#endif // PITCH_IN_RESPONSE_TEST_MODE
        if ((rc_chas->key.v & KEY_PRESSED_OFFSET_F) == 0)
        {
            TIM_Set_PWM(&htim1, TIM_CHANNEL_1, 1570);
        }
        else
        {
            TIM_Set_PWM(&htim1, TIM_CHANNEL_1, 2150);
        }

        // USART_Print("$%d %d;", (int)(pxPITCH_A->ideal * 10), (int)(pxPITCH_A->actual * 10));
        USART_Print("$%d %d;", (int)(pc_aimbot_data.yaw * 1000), (int)(pc_aimbot_data.pit * 1000));

        CAN_SendMessage(CAN_CHANNEL_1, MOTOR_5678, pxPITCH_V->output, pxYAW_V->output, 0, 0);
        CAN_SendMessage(CAN_CHANNEL_1, MOTOR_1234, FlywheelL->output, FlywheelR->output, Feeder_Wheel->output, 0);

        vTaskDelayUntil(&xStartupTick, pdMS_TO_TICKS(2));
    }

    // Task Never Reach Here

    while (1)
    {
        ;
    }
}

void MotorCheckTask(void *pvParameters)
{
    PID *pxV = (PID *)pvParameters;
    int32_t timeout_counter = 0;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(10)); // 每隔10ms检测一次

        if (abs_i(pxV->actual) <= 10 && abs_i(pxV->output) >= 0.8f * pxV->Limit_Out) // 检测到电机堵转
        {
            timeout_counter++;
        }
        else
        {
            timeout_counter = 0;
        }

        if (timeout_counter >= 300) // 如果连续100次检测到电机堵转，则发出报警
        {
            // printf("Motor stall detected!\n");
            // 发出报警后，可以根据具体情况执行一些操作，比如停止电机运行等等

            if (abs_i(pxV->output) >= 0.8f * pxV->Limit_Out)
            {
                buzzer_on(22, 1500);
                pxV->Error |= MOTOR_STALL;
                pxV->output = 0;
                // pxV->ideal = 0;
            }
            while (1)
            {
                // vTaskDelay(pdMS_TO_TICKS(10));
                if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0)
                {
                    buzzer_off();
                    pxV->Error = 0;
                    // pxV->ideal = 0;
                    pxV->integral = 0;
                    timeout_counter = 0;
                    break;
                }
            }
        }
    }
}

void YawEcdReportTask(void *pvParameters)
{
    TickType_t xStartupTick = xTaskGetTickCount();
    motor_measure_t *Yaw = get_measure_pointer(5);
    uint32_t prc = 0;
    uint8_t ifyaw, ifpit;
    while (1)
    {
        CAN_SendMessage(CAN_CHANNEL_2, ECD_REPORT, (Yaw->ecd), (Yaw->ecd >> 8), 0, 0);
        prc++;
        if (prc == 3)
        {
            prc = 0;
            PC_data_encode();
        }
        vTaskDelayUntil(&xStartupTick, pdMS_TO_TICKS(5));
    }
}

void BuzzerTask(void *pvParameters)
{
    while (1)
    {
        /* code */
    }
}
