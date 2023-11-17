#include "can_driver.h"

static CAN_TxHeaderTypeDef hcan1_tx_header;
static CAN_TxHeaderTypeDef hcan2_tx_header;

static uint8_t hcan1_message[8];
static uint8_t hcan2_message[8];

motor_measure_t MotorInfo[8];

chassis_transported_controller_data rc_ctcd = {0};

ammo_booster_data ammo_booster_info = {0};

motor_measure_t *get_measure_pointer(uint32_t i)
{
    return MotorInfo + i;
}

chassis_transported_controller_data *get_rc_data_from_chassis_pointer(void)
{
    return &rc_ctcd;
};

void CAN_FilterSetup(void)
{
    CAN_FilterTypeDef setup_handle;

    setup_handle.FilterActivation = ENABLE;
    setup_handle.FilterMode = CAN_FILTERMODE_IDMASK;
    setup_handle.FilterScale = CAN_FILTERSCALE_32BIT;
    setup_handle.FilterIdHigh = 0x0000;
    setup_handle.FilterIdLow = 0x0000;
    setup_handle.FilterMaskIdHigh = 0x0000;
    setup_handle.FilterMaskIdLow = 0x0000;
    setup_handle.FilterBank = 0;
    setup_handle.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &setup_handle);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    setup_handle.SlaveStartFilterBank = 14;
    setup_handle.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &setup_handle);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

#define get_motor_measure(ptr, data)                                   \
    {                                                                  \
        (ptr)->last_last_ecd = (ptr)->last_ecd;                        \
        (ptr)->last_ecd = (ptr)->ecd;                                  \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temperate = (data)[6];                                  \
    }

#define get_rc_from_chassis(ptr, data)                          \
    {                                                           \
        (ptr)->mouse.x = (int16_t)((data)[1] << 8 | (data)[0]); \
        (ptr)->mouse.y = (int16_t)((data)[3] << 8 | (data)[2]); \
        (ptr)->mouse.press_l = (data)[4];                       \
        (ptr)->mouse.press_r = (data)[5];                       \
        (ptr)->key.v = (uint16_t)((data)[7] << 8 | (data)[6]);  \
                                                                \
        (ptr)->mouse_integeral.x += (float)(ptr)->mouse.x;      \
        (ptr)->mouse_integeral.y += (float)(ptr)->mouse.y;      \
        (ptr)->mouse_integeral.z += (float)(ptr)->mouse.z;      \
    }

void get_ammo_booster_info(ammo_booster_data *ptr, uint8_t *data)
{
    (ptr)->ID = data[0];
    (ptr)->shooting_rate = data[1];
    (ptr)->bullet_speed = (data[3] << 8) | data[2];
    (ptr)->muzzle_heat = data[4];
    (ptr)->muzzle_heat_lim = data[5];
    (ptr)->muzzle_cooling_rate = data[6];
    (ptr)->muzzle_speed_lim = data[7];
}

uint8_t rx_data[8];
uint16_t dat = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;

    RC_ctrl_t *rc = (RC_ctrl_t *)get_remote_control_point();
    PID *booster = pid_get_struct_pointer(2, NORMAL_MOTOR);

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan->Instance == CAN1)
    {
        switch (rx_header.StdId)
        {

        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:

        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
        {
            get_motor_measure(MotorInfo + rx_header.StdId - CAN_3508_M1_ID, rx_data);
            break;
        }

        default:
        {
            break;
        }
        }
    }

    if (hcan->Instance == CAN2)
    {
        switch (rx_header.StdId)
        {
        case CHASSIS_CONTROLLER:
        {
            get_rc_from_chassis(&rc_ctcd, rx_data);
            break;
        }
        case AMMO_BOOSTER:
        {
            get_ammo_booster_info(&ammo_booster_info, rx_data);
            break;
        }
        case RC_SWITCH:
        {
            rc->rc.s[0] = rx_data[0];
            rc->rc.s[1] = rx_data[1];
            break;
        }
        default:
            break;
        }
    }
}

uint32_t CAN_SendMessage(can_channel_id CAN_Channel, motor_id_range MOTOR_ID_RANGE, int16_t Motor1, int16_t Motor2, int16_t Motor3, int16_t Motor4)
{
    uint32_t TxMailBox = 0;
    CAN_HandleTypeDef *pxhcan = NULL;
    CAN_TxHeaderTypeDef *pxHeader = NULL;
    uint8_t *pxMessage = NULL;

    switch (CAN_Channel)
    {
    case CAN_CHANNEL_1:
        pxhcan = &hcan1;
        pxHeader = &hcan1_tx_header;
        pxMessage = (uint8_t *)&hcan1_message;
        break;
    case CAN_CHANNEL_2:
        pxhcan = &hcan2;
        pxHeader = &hcan2_tx_header;
        pxMessage = (uint8_t *)&hcan2_message;
        break;
    default:
        return 0;
        break;
    }

    switch (MOTOR_ID_RANGE)
    {
    case MOTOR_1234:

        pxHeader->StdId = CAN_CHASSIS_ALL_ID;
        break;
    case MOTOR_5678:
        pxHeader->StdId = CAN_GIMBAL_ALL_ID;
        break;
    case ECD_REPORT:
        pxHeader->StdId = ECD_REPORT_ID;
        break;

    default:
        break;
    }
    pxHeader->IDE = CAN_ID_STD;
    pxHeader->RTR = CAN_RTR_DATA;
    pxHeader->DLC = 0x08;

    pxMessage[0] = (Motor1 >> 8);
    pxMessage[1] = Motor1;
    pxMessage[2] = (Motor2 >> 8);
    pxMessage[3] = Motor2;
    pxMessage[4] = (Motor3 >> 8);
    pxMessage[5] = Motor3;
    pxMessage[6] = (Motor4 >> 8);
    pxMessage[7] = Motor4;

    HAL_CAN_AddTxMessage(pxhcan, pxHeader, pxMessage, &TxMailBox);
    return TxMailBox;
}

chassis_transported_controller_data *get_rc_data_from_chassis(void)
{
    return &rc_ctcd;
}

ammo_booster_data *get_ammo_booster_info_ptr(void)
{
    return &ammo_booster_info;
}