#ifndef __SHOOT_H__
#define __SHOOT_H__

#include "main.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "Remote.h"
#include "M3508.h"
#include "M2006.h"

void Shoot_Reload_Choose(void);
void Shoot_Remote_Control(void);
void Shoot_Move(void);
void Shoot_PID_Init_ALL(void);
void Shoot_PID_Calc(void);
void Shoot_PID_Clean_ALL(void);
void Shoot_Stop(void);

#endif


