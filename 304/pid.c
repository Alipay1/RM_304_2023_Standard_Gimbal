#include "pid.h"

float abs_f(float value)
{
    if (value >= 0)
    {
        return value;
    }
    else
    {
        return 0 - value;
    }
}

static PID pid_speed_struct[PID_SPEED_STRUCT_NUM] = {0};

int Angle_Position_Calc(int MotorNum);

int PID_Setup(void)
{
    uint8_t i = 0;
    (pid_speed_struct + i)->motor_number = 0;
    (pid_speed_struct + i)->active = true;
    (pid_speed_struct + i)->ideal = -6000;
    (pid_speed_struct + i)->Kp = 5.0f; /*1*/
    (pid_speed_struct + i)->Ki = 0.02f;
    (pid_speed_struct + i)->Kd = 0.1f;
    (pid_speed_struct + i)->Limit_Iout = 30000;
    (pid_speed_struct + i)->Limit_Out = 30000;
    (pid_speed_struct + i)->Error = 0;
    i++;
    (pid_speed_struct + i)->motor_number = 1;
    (pid_speed_struct + i)->active = true;
    (pid_speed_struct + i)->ideal = 6000;
    (pid_speed_struct + i)->Kp = 5.0f; /*2*/
    (pid_speed_struct + i)->Ki = 0.02f;
    (pid_speed_struct + i)->Kd = 0.1f;
    (pid_speed_struct + i)->Limit_Iout = 30000;
    (pid_speed_struct + i)->Limit_Out = 30000;
    (pid_speed_struct + i)->Error = 0;
    i++;
    (pid_speed_struct + i)->motor_number = 2;
    (pid_speed_struct + i)->active = true;
    (pid_speed_struct + i)->ideal = -4000;
    (pid_speed_struct + i)->Kp = 6.5f; /*3*/
    (pid_speed_struct + i)->Ki = 0.5f;
    (pid_speed_struct + i)->Kd = 0.0f;
    (pid_speed_struct + i)->Limit_Iout = 16384;
    (pid_speed_struct + i)->Limit_Out = 16384;
    (pid_speed_struct + i)->Error = 0;
    i++;
    (pid_speed_struct + i)->motor_number = 3;
    (pid_speed_struct + i)->active = false;
    (pid_speed_struct + i)->ideal = 0;
    (pid_speed_struct + i)->Kp = 0; /*4*/
    (pid_speed_struct + i)->Ki = 0;
    (pid_speed_struct + i)->Kd = 0;
    (pid_speed_struct + i)->Limit_Iout = 30000;
    (pid_speed_struct + i)->Limit_Out = 30000;
    (pid_speed_struct + i)->Error = 0;
    i++;
    (pid_speed_struct + i)->motor_number = 4;
    (pid_speed_struct + i)->active = false;
    (pid_speed_struct + i)->ideal = 0;
    (pid_speed_struct + i)->Kp = 0; /*5*/
    (pid_speed_struct + i)->Ki = 0;
    (pid_speed_struct + i)->Kd = 0;
    (pid_speed_struct + i)->Limit_Iout = 30000;
    (pid_speed_struct + i)->Limit_Out = 30000;
    (pid_speed_struct + i)->Error = 0;
    i++;
    (pid_speed_struct + i)->motor_number = 5;
    (pid_speed_struct + i)->active = false;
    (pid_speed_struct + i)->ideal = 0;
    (pid_speed_struct + i)->Kp = 0; /*6*/
    (pid_speed_struct + i)->Ki = 0;
    (pid_speed_struct + i)->Kd = 0;
    (pid_speed_struct + i)->Limit_Iout = 30000;
    (pid_speed_struct + i)->Limit_Out = 30000;
    (pid_speed_struct + i)->Error = 0;
    i++;
    (pid_speed_struct + i)->motor_number = 6;
    (pid_speed_struct + i)->active = false;
    (pid_speed_struct + i)->ideal = 0;
    (pid_speed_struct + i)->Kp = 0; /*7*/
    (pid_speed_struct + i)->Ki = 0;
    (pid_speed_struct + i)->Kd = 0;
    (pid_speed_struct + i)->Limit_Iout = 30000;
    (pid_speed_struct + i)->Limit_Out = 30000;
    (pid_speed_struct + i)->Error = 0;
    i++;
    (pid_speed_struct + i)->motor_number = 7;
    (pid_speed_struct + i)->active = false;
    (pid_speed_struct + i)->ideal = 0;
    (pid_speed_struct + i)->Kp = 0; /*8*/
    (pid_speed_struct + i)->Ki = 0;
    (pid_speed_struct + i)->Kd = 0;
    (pid_speed_struct + i)->Limit_Iout = 30000;
    (pid_speed_struct + i)->Limit_Out = 30000;
    (pid_speed_struct + i)->Error = 0;
    return 0;
}

PID PITCH_V = {
    .motor_number = 4,
    .active = true,
    .ideal = 0,
    .actual = 0,
    .output = 0,
    .integral = 0,
    .err = 0,
    .err_last = 0,
    .err_last_last = 0,
    .Kp = 5550.0f,
    .Ki = 195.0f,
    .Kd = 0.0f,
    .Limit_Iout = 25000,
    .Limit_Out = 30000,
    .Error = 0};
PID PITCH_A = {
    .motor_number = 4,
    .active = true,
    .ideal = 0,
    .actual = 0,
    .output = 0,
    .integral = 0,
    .err = 0,
    .err_last = 0,
    .err_last_last = 0,
    .Kp = 0.13f,
    // .Ki = 0.8000,
    .Ki = 0.0000f,
    .Kib = 0.000f,
    .Kd = 0.0f,
    .Limit_Iout = 250,
    .Limit_Out = 300,
    .Error = 0};

PID YAW_V = {
    .motor_number = 5,
    .ideal = 0,
    .actual = 0,
    .output = 0,
    .integral = 0,
    .err = 0,
    .err_last = 0,
    .err_last_last = 0,
    .Kp = 6000.0f, // 13000 10000
    .Ki = 115.0f,  // 550
    .Kd = 0.0f,
    .Limit_Iout = 28000,
    .Limit_Out = 30000,
    .Error = 0};

PID YAW_P = {
    .motor_number = 5,
    .ideal = 0,
    .actual = 0,
    .output = 0,
    .integral = 0,
    .err = 0,
    .err_last = 0,
    .err_last_last = 0,
    .Kp = 0.14f,
    //    .Kp = 0.0051f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .Limit_Iout = 20,
    .Limit_Out = 100,
    .Error = 0};
static int32_t YAW_Motor_Position = 0;

PID IMU_T = {
    .ideal = 40.0f,
    .actual = 0,
    .output = 0,
    .integral = 0,
    .err = 0,
    .err_last = 0,
    .err_last_last = 0,
    .Kp = 1600.0f,
    .Ki = 0.2f,
    .Kd = 0.00f,
    .Limit_Iout = 30000,
    .Limit_Out = 30000,
    .Error = 0};

void PID_Out_Limit(PID *pxStruct)
{
    // 限制积分值的范围
    pxStruct->integral = (pxStruct->integral > pxStruct->Limit_Iout) ? pxStruct->Limit_Iout : pxStruct->integral;
    pxStruct->integral = (pxStruct->integral < -pxStruct->Limit_Iout) ? -pxStruct->Limit_Iout : pxStruct->integral;

    // 限制输出值的范围
    pxStruct->output = (pxStruct->output > pxStruct->Limit_Out) ? pxStruct->Limit_Out : pxStruct->output;
    pxStruct->output = (pxStruct->output < -pxStruct->Limit_Out) ? -pxStruct->Limit_Out : pxStruct->output;
}

/**
 * @brief PITCH轴串级PID计算
 *
 * @param pxStructV 角速度环PID结构体 PI
 * @param pxStructA 角度环PID结构体 P
 * @param absolute_angle 目标角度 1 = (360/8192)°
 * @return fp32 电机电流
 */
fp32 PID_PITCH_Union_Calculate(PID *pxStructV, PID *pxStructA, float absolute_angle)
{
    motor_measure_t *motinfo = (motor_measure_t *)get_measure_pointer(PITCH_MOTOR_NUM);

    if (pxStructV->Error != 0)
    {
        // buzzer_on(22, 7000);
        pxStructV->output = 0;
        return 0;
    }
    /*角度环*/
    pxStructA->ideal = absolute_angle;
    // pxStructA->actual = motinfo->ecd;
    pxStructA->actual = INS.Pitch;

    pxStructA->err = pxStructA->ideal - pxStructA->actual;
    pxStructA->integral += pxStructA->err;

    pxStructA->Pout = pxStructA->Kp * pxStructA->err;
    if (pxStructA->err < 0.5f)
    {
        pxStructA->Iout = pxStructA->Kib * pxStructA->integral;
    }
    else
    {
        pxStructA->Iout = pxStructA->Ki * pxStructA->integral;
    }
    pxStructA->Dout = pxStructA->Kd * (pxStructA->err - 2.0f * pxStructA->err_last + pxStructA->err_last_last);

    pxStructA->output = pxStructA->Pout + pxStructA->Iout + pxStructA->Dout;

    pxStructA->err_last_last = pxStructA->err_last;
    pxStructA->err_last = pxStructA->err;

    PID_Out_Limit(pxStructA);

    /*角速度环*/
    // pxStructV->ideal = absolute_angle;

    pxStructV->ideal = pxStructA->output;

    // pxStructV->ideal = 150;
    // pxStructV->actual = (motinfo->speed_rpm);
    pxStructV->actual = INS.Gyro[0];
    pxStructV->err = pxStructV->ideal - pxStructV->actual;
    pxStructV->integral += pxStructV->err;

    pxStructV->Pout = pxStructV->Kp * pxStructV->err;
    pxStructV->Iout = pxStructV->Ki * pxStructV->integral;
    pxStructV->Dout = pxStructV->Kd * (pxStructV->err - 2.0f * pxStructV->err_last + pxStructV->err_last_last);

    pxStructV->output = pxStructV->Pout + pxStructV->Iout + pxStructV->Dout;
    pxStructV->err_last_last = pxStructV->err_last;
    pxStructV->err_last = pxStructV->err;

    PID_Out_Limit(pxStructV);

    return pxStructV->output;
}

// fp32 PID_PITCH_Union_Calculate(PID *pxStructV, PID *pxStructA, uint32_t absolute_angle)
// {
//     motor_measure_t *motinfo = (motor_measure_t *)get_measure_pointer(PITCH_MOTOR_NUM);

//     if (pxStructV->Error != 0)
//     {
//         // buzzer_on(22, 7000);
//         pxStructV->output = 0;
//         return 0;
//     }
//     /*角度环*/
//     pxStructA->ideal = absolute_angle;
//     pxStructA->actual = motinfo->ecd;

//     pxStructA->err = pxStructA->ideal - pxStructA->actual;
//     pxStructA->integral += pxStructA->err;

//     pxStructA->Pout = pxStructA->Kp * pxStructA->err;
//     pxStructA->Iout = pxStructA->Ki * pxStructA->integral;
//     pxStructA->Dout = pxStructA->Kd * (pxStructA->err - 2.0f * pxStructA->err_last + pxStructA->err_last_last);

//     pxStructA->output = pxStructA->Pout + pxStructA->Iout + pxStructA->Dout;

//     pxStructA->err_last_last = pxStructA->err_last;
//     pxStructA->err_last = pxStructA->err;

//     PID_Out_Limit(pxStructA);

//     /*角速度环*/
//     pxStructV->ideal = pxStructA->output;
//     // pxStructV->ideal = 150;
//     pxStructV->actual = (motinfo->speed_rpm);
//     pxStructV->err = pxStructV->ideal - pxStructV->actual;
//     pxStructV->integral += pxStructV->err;

//     pxStructV->Pout = pxStructV->Kp * pxStructV->err;
//     pxStructV->Iout = pxStructV->Ki * pxStructV->integral;
//     pxStructV->Dout = pxStructV->Kd * (pxStructV->err - 2.0f * pxStructV->err_last + pxStructV->err_last_last);

//     pxStructV->output = pxStructV->Pout + pxStructV->Iout + pxStructV->Dout;
//     pxStructV->err_last_last = pxStructV->err_last;
//     pxStructV->err_last = pxStructV->err;

//     PID_Out_Limit(pxStructV);

//     return pxStructV->output;
// }
// fp32 PID_PITCH_Union_Calculate(PID *pxStructV, PID *pxStructA, uint32_t absolute_angle)
// {
//     motor_measure_t *motinfo = (motor_measure_t *)get_measure_pointer(PITCH_MOTOR_NUM);
//     /*角度环*/
//     pxStructA->ideal = absolute_angle;
//     pxStructA->actual = motinfo->ecd;

//     pxStructA->err = pxStructA->ideal - pxStructA->actual;
//     pxStructA->integral += pxStructA->err;
//     pxStructA->output = pxStructA->Kp * pxStructA->err + pxStructA->Ki * pxStructA->integral +
//                         pxStructA->Kd * (pxStructA->err - 2.0f * pxStructA->err_last + pxStructA->err_last_last);
//     pxStructA->err_last_last = pxStructA->err_last;
//     pxStructA->err_last = pxStructA->err;

//     PID_Out_Limit(pxStructA);
//     /*角速度环*/
//     pxStructV->ideal = pxStructA->output;
//     pxStructV->actual = (motinfo->speed_rpm);
//     pxStructV->err = pxStructV->ideal - pxStructV->actual;

//     pxStructV->integral += pxStructV->err;
//     pxStructV->output = pxStructV->Kp * pxStructV->err + pxStructV->Ki * pxStructV->integral +
//                         pxStructV->Kd * (pxStructV->err - 2.0f * pxStructV->err_last + pxStructV->err_last_last);
//     pxStructV->err_last_last = pxStructV->err_last;
//     pxStructV->err_last = pxStructV->err;

//     PID_Out_Limit(pxStructV);

//     return pxStructV->output;
// }
/**
 * @brief 电机位置计算
 *
 * @param MotorNum 电机在CAN总线中的序号
 * @return int 返回电机转动的角度值(1 = (360/8192)°)
 */
int Angle_Position_Calc(int MotorNum)
{
    motor_measure_t *motinfo = (motor_measure_t *)get_measure_pointer(MotorNum);

    if (abs_f(motinfo->ecd - motinfo->last_ecd) < 8000)
    {
        return (motinfo->ecd - motinfo->last_ecd);
    }
    else
    {
        if (motinfo->ecd - motinfo->last_ecd > 0)
        {
            return motinfo->ecd - motinfo->last_ecd - 8191;
        }
        else if (motinfo->ecd - motinfo->last_ecd < 0)
        {
            return motinfo->ecd - motinfo->last_ecd + 8191;
        }
    }

    return 0;
}
// fp32 PID_YAW_Union_Calculate(PID *pxStructV, PID *pxStructP, int32_t position, bool mode)
//{
//     motor_measure_t *motinfo = (motor_measure_t *)get_measure_pointer(YAW_MOTOR_NUM);
//     if (mode == false)
//     {
//         pxStructV->ideal = position;
//         goto YAW_SPEED_MODE;
//     }
//     /*角度环*/
//     pxStructP->ideal = position;
//     YAW_Motor_Position += Angle_Position_Calc(YAW_MOTOR_NUM);
//     pxStructP->actual = YAW_Motor_Position;
//     if (pxStructV->Error != 0)
//     {
//         // buzzer_on(22, 7000);
//         pxStructV->output = 0;
//         return 0;
//     }
//     pxStructP->err = pxStructP->ideal - pxStructP->actual;
//     pxStructP->integral += pxStructP->err;

//    pxStructP->Pout = pxStructP->Kp * pxStructP->err;
//    pxStructP->Iout = pxStructP->Ki * pxStructP->integral;
//    pxStructP->Dout = pxStructP->Kd * (pxStructP->err - 2.0f * pxStructP->err_last + pxStructP->err_last_last);

//    pxStructP->output = pxStructP->Pout + pxStructP->Iout + pxStructP->Dout;
//    pxStructP->err_last_last = pxStructP->err_last;
//    pxStructP->err_last = pxStructP->err;
//    PID_Out_Limit(pxStructP);

//    // if (pxStructP->err < 100)
//    // {
//    //     pxStructP->output *= 0.02;
//    // }

//    // pxStructV->ideal = position;

//    //		 if (abs_f(pxStructP->err) < 1 )
//    //     {
//    //         pxStructP->output = 0;
//    //     }
//    pxStructV->ideal = pxStructP->output;
// YAW_SPEED_MODE:
//    pxStructV->actual = (motinfo->speed_rpm);
//    pxStructV->err = pxStructV->ideal - pxStructV->actual;

//    pxStructV->integral += pxStructV->err;

//    pxStructV->Pout = pxStructV->Kp * pxStructV->err;
//    pxStructV->Iout = pxStructV->Ki * pxStructV->integral;
//    pxStructV->Dout = pxStructV->Kd * (pxStructV->err - 2.0f * pxStructV->err_last + pxStructV->err_last_last);

//    pxStructV->output = pxStructV->Pout + pxStructV->Iout + pxStructV->Dout;
//    pxStructV->err_last_last = pxStructV->err_last;
//    pxStructV->err_last = pxStructV->err;

//    PID_Out_Limit(pxStructV);

//    return pxStructV->output;
//}
fp32 PID_YAW_Union_Calculate(PID *pxStructV, PID *pxStructP, float position, bool mode)
{
    motor_measure_t *motinfo = (motor_measure_t *)get_measure_pointer(YAW_MOTOR_NUM);
    if (mode == false)
    {
        pxStructV->ideal = position;
        goto YAW_SPEED_MODE;
    }
    // /*角度环*/
    // pxStructP->ideal = position * 22.75277777f;
    // // YAW_Motor_Position += Angle_Position_Calc(YAW_MOTOR_NUM);
    // pxStructP->actual = INS.YawTotalAngle * 22.75277777f;

    pxStructP->ideal = position;
    // YAW_Motor_Position += Angle_Position_Calc(YAW_MOTOR_NUM);
    pxStructP->actual = INS.YawTotalAngle;
    if (pxStructV->Error != 0)
    {
        // buzzer_on(22, 7000);
        pxStructV->output = 0;
        return 0;
    }
    pxStructP->err = pxStructP->ideal - pxStructP->actual;
    pxStructP->integral += pxStructP->err;

    pxStructP->Pout = pxStructP->Kp * pxStructP->err;
    pxStructP->Iout = pxStructP->Ki * pxStructP->integral;
    pxStructP->Dout = pxStructP->Kd * (pxStructP->err - 2.0f * pxStructP->err_last + pxStructP->err_last_last);

    pxStructP->output = pxStructP->Pout + pxStructP->Iout + pxStructP->Dout;
    pxStructP->err_last_last = pxStructP->err_last;
    pxStructP->err_last = pxStructP->err;
    PID_Out_Limit(pxStructP);

    // if (pxStructP->err < 100)
    // {
    //     pxStructP->output *= 0.02;
    // }

    // pxStructV->ideal = position;

    //		 if (abs_f(pxStructP->err) < 1 )
    //     {
    //         pxStructP->output = 0;
    //     }
    pxStructV->ideal = pxStructP->output;
YAW_SPEED_MODE:
    pxStructV->actual = INS.Gyro[2];
    pxStructV->err = pxStructV->ideal - pxStructV->actual;

    pxStructV->integral += pxStructV->err;

    pxStructV->Pout = pxStructV->Kp * pxStructV->err;
    pxStructV->Iout = pxStructV->Ki * pxStructV->integral;
    pxStructV->Dout = pxStructV->Kd * (pxStructV->err - 2.0f * pxStructV->err_last + pxStructV->err_last_last);

    pxStructV->output = pxStructV->Pout + pxStructV->Iout + pxStructV->Dout;
    pxStructV->err_last_last = pxStructV->err_last;
    pxStructV->err_last = pxStructV->err;

    PID_Out_Limit(pxStructV);

    return pxStructV->output;
}
int PID_Gimbal_Calculate(void)
{
    motor_measure_t *motinfo = (motor_measure_t *)get_measure_pointer(0);
    chassis_transported_controller_data *rc_chsas = get_rc_data_from_chassis_pointer();
    // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
    for (int i = 0; i < PID_SPEED_STRUCT_NUM;)
    {
        if ((pid_speed_struct + i)->active == true)
        {
            if ((pid_speed_struct + i)->Error == 0)
            {
                (pid_speed_struct + i)->actual = motinfo->speed_rpm;
                (pid_speed_struct + i)->err = (pid_speed_struct + i)->ideal - (pid_speed_struct + i)->actual;
                (pid_speed_struct + i)->integral += (pid_speed_struct + i)->err;

                (pid_speed_struct + i)->Pout = (pid_speed_struct + i)->Kp * (pid_speed_struct + i)->err;
                (pid_speed_struct + i)->Iout = (pid_speed_struct + i)->Ki * (pid_speed_struct + i)->integral;
                (pid_speed_struct + i)->Dout = (pid_speed_struct + i)->Kd * ((pid_speed_struct + i)->err - 2.0f * (pid_speed_struct + i)->err_last + (pid_speed_struct + i)->err_last_last);

                (pid_speed_struct + i)->output = (pid_speed_struct + i)->Pout + (pid_speed_struct + i)->Iout + (pid_speed_struct + i)->Dout;
                (pid_speed_struct + i)->err_last = (pid_speed_struct + i)->err;
                PID_Out_Limit(pid_speed_struct + i);
            }
        }
        i++;
        motinfo++;
    }

    CAN_SendMessage(CAN_CHANNEL_1, MOTOR_1234, pid_speed_struct[0].output, pid_speed_struct[1].output, pid_speed_struct[2].output, pid_speed_struct[3].output);
    //    CAN_SendMessage(CAN_CHANNEL_1, MOTOR_5678, PITCH.output, pid_speed_struct[5].output, pid_speed_struct[6].output, pid_speed_struct[7].output);
    // CAN_SendMessage(CAN_CHANNEL_1, MOTOR_5678, 0, pid_speed_struct[5].output, pid_speed_struct[6].output, pid_speed_struct[7].output);

    // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
    return 0;
}

PID *pid_get_struct_pointer(uint32_t num, uint32_t in_array)
{
    switch (in_array)
    {
    case NORMAL_MOTOR:
        return pid_speed_struct + num;
        break;
    case PITCH_MOTOR:
        switch (num)
        {
        case 0:
            return &PITCH_V;
            break;
        case 1:
            return &PITCH_A;
            break;
        default:
            return NULL;
            break;
        }
    case YAW_MOTOR:
        switch (num)
        {
        case 0:
            return &YAW_V;
            break;
        case 1:
            return &YAW_P;
            break;
        default:
            return NULL;
            break;
        }
        break;

    default:
        return NULL;
        break;
    }
}

int32_t get_yaw_motor_position(void)
{
    return YAW_Motor_Position;
}
