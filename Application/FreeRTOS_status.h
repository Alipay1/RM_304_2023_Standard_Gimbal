#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "string.h"

/*
    configureTimerForRunTimeStats()与getRunTimeCounterValue()是cubeMX生成的，需要在getRunTimeCounterValue()添加1个计数器供RTOS使用
*/
// __weak void configureTimerForRunTimeStats(void)
// {
// }

// __weak unsigned long getRunTimeCounterValue(void)
// {
//     //@ 添加此变量供RTOS系统调用
//     static uint64_t rtosStatusCount = 0;
//     return rtosStatusCount++;
// }


void rtosDebugTask(void *argument);
