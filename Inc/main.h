/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "struct_typedef.h"

#include "ins_task.h"

//#include "ST7735.h"

#include "Initializations.h"
#include "can_driver.h"
#include "dbus_driver.h"
#include "BMI088driver.h"

//#include "ST7735.h"
#include "visualscope.h"

#include "pid.h"

#include "PC_Communication.h"

#include "bsp_dwt.h"

#include "bsp_imu_pwm.h"

#include "Gimbal_Threads.h"
#include "user_lib.h"

#include "bsp_buzzer.h"
#include "bsp_PWM.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;  
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim1;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HALL_SENSOR_PIN2_Pin GPIO_PIN_7
#define HALL_SENSOR_PIN2_GPIO_Port GPIOI
#define HALL_SENSOR_PIN2_EXTI_IRQn EXTI9_5_IRQn
#define HALL_SENSOR_PIN1_Pin GPIO_PIN_6
#define HALL_SENSOR_PIN1_GPIO_Port GPIOI
#define HALL_SENSOR_PIN1_EXTI_IRQn EXTI9_5_IRQn
#define RSTN_IST8310_Pin GPIO_PIN_6
#define RSTN_IST8310_GPIO_Port GPIOG
#define IMU_HEAT_Pin GPIO_PIN_6
#define IMU_HEAT_GPIO_Port GPIOF
#define DRDY_IST8310_Pin GPIO_PIN_3
#define DRDY_IST8310_GPIO_Port GPIOG
#define DRDY_IST8310_EXTI_IRQn EXTI3_IRQn
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
//#define USE_ST7735_MONITOR 1U

void USART_Print(char *fmt, ...);
// void USART_Print1(char *fmt, ...);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
