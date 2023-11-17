#include "PC_Communication.h"
#include "string.h"

pc_data_rx_t pc_aimbot_data = {0};

void PC_data_decode(pc_data_rx_t *data, uint8_t *raw)
{
    float tempyaw, temppit;
    // if (raw[7] != '1' || raw[7] != '0')
    // {
    //     return;
    // }
    // else if (raw[13] != '1' || raw[13] != '0')
    // {
    //     return;
    // }
    for (uint8_t i = 0; i < 14; i++)
    {
        if (raw[i] >= '0' && raw[i] <= '9')
        {
            continue;
        }
        else
        {
            return;
        }
    }

    tempyaw = (raw[0] - '0') * 10000 +
              (raw[1] - '0') * 1000 +
              (raw[2] - '0') * 100 +
              (raw[3] - '0') * 10 +
              (raw[4] - '0') +
              (raw[5] - '0') * 0.1f +
              (raw[6] - '0') * 0.01f;

    if (tempyaw > INS.YawTotalAngle + 30.0f)
    {
        tempyaw = INS.YawTotalAngle;
    }

    if (raw[7] == '1')
    {
        tempyaw *= -1.0f;
    }

    data->yaw = tempyaw;

    temppit = (raw[8] - '0') * 100 +
              (raw[9] - '0') * 10 +
              (raw[10] - '0') +
              (raw[11] - '0') * 0.1f +
              (raw[12] - '0') * 0.01f;

    if (temppit > INS.Pitch + 15.0f)
    {
        temppit = INS.Pitch;
    }

    if (raw[13] == '1')
    {
        temppit *= -1.0f;
    }

    data->pit = temppit;

    data->valid = true;
}

void PC_data_encode()
{
    uint8_t DATA[15] = {0};
    int tempyaw = abs_f(INS.YawTotalAngle * 100);
    int temppit = abs_f(INS.Pitch * 100);

    DATA[0] = (int)tempyaw / 1000000 % 10 + '0';
    DATA[1] = (int)tempyaw / 100000 % 10 + '0';
    DATA[2] = (int)tempyaw / 10000 % 10 + '0';
    DATA[3] = (int)tempyaw / 1000 % 10 + '0';
    DATA[4] = (int)tempyaw / 100 % 10 + '0';
    DATA[5] = (int)tempyaw / 10 % 10 + '0';
    DATA[6] = (int)tempyaw % 10 + '0';

    if (INS.YawTotalAngle > 0)
    {
        DATA[7] = '0';
    }
    else
    {
        DATA[7] = '1';
    }

    DATA[8] = (int)temppit / 10000 % 10 + '0';
    DATA[9] = (int)temppit / 1000 % 10 + '0';
    DATA[10] = (int)temppit / 100 % 10 + '0';
    DATA[11] = (int)temppit / 10 % 10 + '0';
    DATA[12] = (int)temppit % 10 + '0';

    if (INS.Pitch > 0)
    {
        DATA[13] = '0';
    }
    else
    {
        DATA[13] = '1';
    }

    DATA[14] = '\r';

    //! 更改发送长度
    HAL_UART_Transmit(&huart1, DATA, 14, 1000);

    // todo 在空闲c板测试此程序
}

// pc_data_rx_t *get_aimbot_data_pointer(void)
// {
//     return &pc_aimbot_data;
// }
