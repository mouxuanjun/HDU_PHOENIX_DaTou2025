#include "ChassisTask.h"

extern Car_Mode_t Car_Mode;

void ChassisTask(void const * argument)
{
    portTickType currentTime;
    while (1)   
    { 
        currentTime = xTaskGetTickCount();//当前系统时间
        Control_Mode_Choose();//选择模式
        switch (Car_Mode.State)
        {
        case Car_Remote:
			Chassis_Remote_Control();
            Chassis_Move();
            break;
        case Car_Keyboard:
            Keyboard_mode_Choose();
            CHAS_Key_Ctrl();
            Chassis_Move();
            break;
        case Car_Stop:
            Chassis_Stop();
            break;
                
        default:
            break;
        }
        vTaskDelayUntil(&currentTime,2);//绝对延时
    }

}

