#include "Initializations.h"

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern uint8_t u1_rx_buf[256];
extern uint8_t u6_rx_buf[256];
extern SPI_HandleTypeDef hspi1;

void Init_Before_OS(void)
{
    
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

    buzzer_on(44, 4500);
    /*内部初始化*/
    CAN_FilterSetup();
    // remote_control_init();

    /*外部芯片初始化*/
#ifdef USE_ST7735_MONITOR
    Lcd_Init();
    LCD_Fill(0xF800);
    LCD_Upgrade_Gram();
#endif // USE_ST7735_MONITOR
    DWT_Init(168);
    while (BMI088_init(&hspi1, 1) != BMI088_NO_ERROR)
        ;
    INS_Init();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, u1_rx_buf, 256);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    /*全局变量初始化*/
}
