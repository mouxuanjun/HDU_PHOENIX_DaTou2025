#include "GimbalTask.h"

extern Car_Mode_t Car_Mode;

void GimbalTask(void const * argument)
{
    portTickType currentTime;
    while(1)
    { 
        currentTime = xTaskGetTickCount();//当前系统时间
        switch (Car_Mode.State)
        {
        case Car_Remote:
            Gimbal_Remote_Control();
            Gimbal_Move();
            break;
        case Car_Stop:
            Gimbal_Stop();
            break;
                
        default:
            break;
        }
        vTaskDelayUntil(&currentTime,1);//绝对延时
    }
}
