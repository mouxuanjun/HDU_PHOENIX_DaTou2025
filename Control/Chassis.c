#include "Chassis.h"

extern Car_Mode_t Car_Mode;
extern RC_t RC;
extern Chassis_Speed_t Chassis_Speed;
extern Chassis_Speed_t Temp_Chassis_Speed;
extern Moto_GM6020_t GM6020_Yaw;
extern Moto_M3508_t M3508_Chassis[4];
extern PID_struct_t Follow_PID;
float Angle;
float err;

/********************解算部分********************/
void Chassis_Solution(void);
void Chassis_Motor_Solution(void);
float Find_Angle(void);

/********************输入控制部分********************/
void Chassis_Remote_Control(void);

/********************PID部分********************/
void Chassis_PID_Init_All(void);
void Chassis_PID_Clean_All(void);
void Chassis_PID_Calc(void);

/********************输出控制部分********************/
void Chassis_Move(void);
void Chassis_Stop(void);


/**
 * @file Chassis.c
 * @brief 把云台坐标系解算到底盘的坐标系
 * @author HWX
 * @date 2024/10/20
 */
void Chassis_Solution(void)
{
    Angle = Find_Angle();
    Chassis_Speed.vx = Temp_Chassis_Speed.vx * cos(Angle) + Temp_Chassis_Speed.vy * sin(Angle);
    Chassis_Speed.vy = -Temp_Chassis_Speed.vx * sin(Angle) + Temp_Chassis_Speed.vy * cos(Angle);
    Chassis_Speed.vw = Temp_Chassis_Speed.vw;
}

/**
 * @file Chassis.c
 * @brief 把底盘运动解算到电机运动
 * @author HWX
 * @date 2024/10/20
 */
void Chassis_Motor_Solution(void)
{
    float Wheel_Rpm_Ratio = 60.0f/(WHEEL_PERIMETER*3.14f) * CHASSIS_DECELE_RATIO * 10000;
    M3508_Chassis[0].Set_Speed = Chassis_Speed.vx * 0.7071068f * Speed_Set + Chassis_Speed.vy * 0.7071068f * Speed_Set + Chassis_Speed.vw * Gimbal_length * Wheel_Rpm_Ratio;
    M3508_Chassis[1].Set_Speed = -Chassis_Speed.vx * 0.7071068f * Speed_Set + Chassis_Speed.vy * 0.7071068f * Speed_Set + Chassis_Speed.vw * Gimbal_length * Wheel_Rpm_Ratio;
    M3508_Chassis[2].Set_Speed = -Chassis_Speed.vx * 0.7071068f * Speed_Set - Chassis_Speed.vy * 0.7071068f * Speed_Set + Chassis_Speed.vw * Gimbal_length * Wheel_Rpm_Ratio;
    M3508_Chassis[3].Set_Speed = Chassis_Speed.vx * 0.7071068f * Speed_Set - Chassis_Speed.vy * 0.7071068f * Speed_Set + Chassis_Speed.vw * Gimbal_length * Wheel_Rpm_Ratio;
}


/**
 * @file Chassis.c
 * @brief 底盘开始运动
 * @author HWX
 * @date 2024/10/20
 */
void Chassis_Move(void)
{

    Chassis_Solution();

    Chassis_Motor_Solution();

	Chassis_PID_Calc();

    Set_M3508_Chassis_Voltage(&hcan1,M3508_Chassis);
}

/**
 * @file Chassis.c
 * @brief 通过清空PID使输出为0
 * @author HWX
 * @date 2024/10/20
 */
void Chassis_Stop(void)
{
    Chassis_PID_Calc();
    Set_M3508_Chassis_Voltage(&hcan1,M3508_Chassis);
}

/**
 * @file Chassis.c
 * @brief 找出底盘和云台+x轴的角度差
 * @retval 角度差（-PI~PI）
 * @author HWX
 * @date 2024/10/20
 */
float Find_Angle(void)
{
	float Angle,Zero;
	Angle = GM6020_Yaw.rotor_angle;
	Zero = Gimbal_Yaw_ZERO;
	
	if(Angle - Zero > 4096)
	{
		Zero+=8190;
	}
	else if(Angle - Zero < -4096)
	{
		Zero-=8190;
	}
    err = Angle - Zero;
	float temp1 = err * 2 * 3.1415926f / 8192;
    if(temp1 > 3.141593f)
        temp1 = 3.141593f;
    else if(temp1 < -3.141593f)
        temp1 = -3.141593f;
    return temp1;
}

/**
 * @file Chassis.c
 * @brief 遥控器控制底盘
 * @author HWX
 * @date 2024/10/20
 */
void Chassis_Remote_Control(void)
{
    switch (Car_Mode.Action)
    {
    case GYROSCOPE:
        Temp_Chassis_Speed.vx = (float)RC.ch3/300;//数值得大于287！！！不然超范围了
        Temp_Chassis_Speed.vy = (float)RC.ch2/300;
        Temp_Chassis_Speed.vw = 0.0046;
        break;
    case NORMAL:
        Temp_Chassis_Speed.vx = (float)RC.ch3/250;
        Temp_Chassis_Speed.vy = (float)RC.ch2/250;
        Temp_Chassis_Speed.vw = 0;
        break;
    case FOLLOW:
        Temp_Chassis_Speed.vx = (float)RC.ch3/250;
        Temp_Chassis_Speed.vy = (float)RC.ch2/250;
				PID_Calc_Angle(&Follow_PID,0.0f,err);
        Temp_Chassis_Speed.vw = Follow_PID.output/1000;
    default:
        break;
    }
}

/**
 * @file Chassis.c
 * @brief 计算电机PID
 * @author HWX
 * @date 2024/10/20
 */
void Chassis_PID_Calc(void)
{
    PID_Calc_Speed(&M3508_Chassis[0].PID,M3508_Chassis[0].Set_Speed,M3508_Chassis[0].rotor_speed);
    PID_Calc_Speed(&M3508_Chassis[1].PID,M3508_Chassis[1].Set_Speed,M3508_Chassis[1].rotor_speed);
    PID_Calc_Speed(&M3508_Chassis[2].PID,M3508_Chassis[2].Set_Speed,M3508_Chassis[2].rotor_speed);
    PID_Calc_Speed(&M3508_Chassis[3].PID,M3508_Chassis[3].Set_Speed,M3508_Chassis[3].rotor_speed);    
}

/**
 * @file Chassis.c
 * @brief 底盘电机PID初始化
 * @author HWX
 * @date 2024/10/20
 */
void Chassis_PID_Init_All(void)
{
    PID_init(&Follow_PID,0.007,0,0.02,16308,16308);
    PID_init(&(M3508_Chassis[0].PID),10,1,10,16308,16308);
    PID_init(&(M3508_Chassis[1].PID),10,1,10,16308,16308);
    PID_init(&(M3508_Chassis[2].PID),10,1,10,16308,16308);
    PID_init(&(M3508_Chassis[3].PID),10,1,10,16308,16308);
}

/**
 * @file Chassis.c
 * @brief 底盘电机PID清空
 * @author HWX
 * @date 2024/10/20
 */
void Chassis_PID_Clean_All(void)
{
    PID_init(&Follow_PID,0,0,0,0,0);
    PID_init(&(M3508_Chassis[0].PID),0,0,0,0,0);
    PID_init(&(M3508_Chassis[1].PID),0,0,0,0,0);
    PID_init(&(M3508_Chassis[2].PID),0,0,0,0,0);
    PID_init(&(M3508_Chassis[3].PID),0,0,0,0,0);
}
