#pragma once

#include "main.h"
#include "stdbool.h"

/**
 * @brief 帧格式：
 * @brief 0~5 yaw
 * @brief 6 yaw sign bit
 * @brief 6-10 pit
 * @brief 11 pit sign bit
 */
typedef struct
{
    float pit;
    float yaw;

    float pit_l;
    float yaw_l;
    bool valid;
} pc_data_rx_t;

extern pc_data_rx_t pc_aimbot_data;

void PC_data_decode(pc_data_rx_t *data, uint8_t *raw);
void PC_data_encode();