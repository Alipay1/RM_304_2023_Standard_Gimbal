#include "isr.h"

uint8_t u1_rx_buf[256] = {0};
uint8_t u6_rx_buf[256] = {0};

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;

extern IWDG_HandleTypeDef hiwdg;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    UBaseType_t uxSavedInterruptStatus;
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    if (huart->Instance == USART1)
    {
        //        memcpy(u1_rx_buf, 0, 256);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, u1_rx_buf, 256);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
        //        if (Size != 14)
        //        {
        //            goto EXIT_USART_ISR;
        //        }
        PC_data_decode(&pc_aimbot_data, u1_rx_buf);
        // HAL_UART_Transmit(&huart1, u1_rx_buf, 50, 1000);
    }

    if (huart->Instance == USART6)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, u6_rx_buf, 256);
        __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
        HAL_UART_Transmit(&huart6, u6_rx_buf, 50, 1000);
    }

EXIT_USART_ISR:
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if (huart->Instance == USART1)
//     {

//         UBaseType_t uxSavedInterruptStatus;
//         uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

//         HAL_UART_Transmit_IT(&huart1, u1_rx_buf, 14);
//         PC_data_decode(&pc_aimbot_data, u1_rx_buf);
//         // HAL_UART_Receive_IT(&huart1, u1_rx_buf, 14);
//         taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
//     }
// }
