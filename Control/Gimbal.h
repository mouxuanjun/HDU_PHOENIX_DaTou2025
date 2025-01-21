#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "main.h"
#include "Chassis.h"
#include "Remote.h"
#include "GM6020.h"
#include "Car_Mode.h"
#include "Computer.h"

#define Gimbal_Pitch_MAX 48.0f
#define Gimbal_Pitch_MIN -26.5f
#define Gimbal_Pithch_Set 2000
#define Gimbal_Yaw_Set 2000  //Last:50

typedef struct
{
    float Yaw;
    float Pitch;
}Gimbal_Add_t;

void Gimbal_Remote_Control(void);
void Gimbal_Init(void);
void Gimbal_Move(void);
void Gimbal_Stop(void);
void Gimbal_PID_Calc(void);
void Gimbal_PID_Init_All(void);
void Gimbal_PID_Clean_All(void);

#endif
