#include "FreeRTOS_status.h"

void rtosDebugTask(void *argument)
{
    static uint8_t CPU_RunInfo[800]; // 保存任务运行时间信息

    while (1)
    {
        /*
//        X 运行状态
//        R Redy就绪状态
//        B Block阻塞状态
//        S Suspend挂起状态
//        D Deletedr任务已经被删除
//        */
//        memset(CPU_RunInfo, 0, 800); /* 信息缓冲区清零 */

//        vTaskList((char *)&CPU_RunInfo); // 获取任务运行时间信息

//        USART_Print("----------------------------------------------------\r\n");
//        USART_Print("task_name task_status priority stack_remain task_id\r\n");
//        USART_Print("%s", CPU_RunInfo);
//        USART_Print("----------------------------------------------------\r\n");

//        memset(CPU_RunInfo, 0, 800); /* 信息缓冲区清零 */

//        vTaskGetRunTimeStats((char *)&CPU_RunInfo);

//        USART_Print("task_name       run_cnt         usage_rate   \r\n");
//        USART_Print("%s", CPU_RunInfo);
//        USART_Print("----------------------------------------------------\r\n");

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
