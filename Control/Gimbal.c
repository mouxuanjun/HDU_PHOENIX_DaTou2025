#include "Gimbal.h"

extern Car_Action Last_Action;

extern float IMU_angle[3];
extern Moto_GM6020_t GM6020_Yaw,GM6020_Pitch;
extern Car_Mode_t Car_Mode;
extern RC_t RC;
extern Gimbal_Add_t Gimbal_Add;
extern float Set_Yaw;
extern Computer_Rx_Message_t Computer_Rx_Message;
//自瞄过滤器
static float Yaw_ZiMiao_Filter[2]={0.005,0.995};
static float Pitch_ZiMiao_Filter[2]={0.1,0.9};
//大符过滤器
//static float Yaw_Fu_Filter[2]={0.01,0.99};
//static float Pitch_Fu_Filter[2]={0.01,0.99};

/********************输入控制部分********************/
void Gimbal_Remote_Control(void);

/********************解算部分********************/
void Gimbal_Calculate(void);
void Gimbal_Init(void);

/********************输出控制部分********************/
void Gimbal_Move(void);
void Gimbal_Stop(void);

/********************PID控制部分********************/
void Gimbal_PID_Init_All(void);
void Gimbal_PID_Clean_All(void);
void Gimbal_PID_Calc(void);

/**
 * @file Gimbal.c
 * @brief 云台初始化
 * @author HWX
 * @date 2024/10/20
 */
void Gimbal_Init(void)
{
    GM6020_Pitch.Set_Angle = Gimbal_Pitch_ZERO;
    Set_Yaw = IMU_angle[0];
}

/**
 * @file Gimbal.c
 * @brief 计算目标值
 * @author HWX
 * @date 2024/10/20
 */
void Gimbal_Calculate(void)
{
    //Yaw
    switch(Car_Mode.Shoot)
    {
		case Shoot_Sustain:
            break;
		case Shoot_Single:
		{
            Set_Yaw -= Gimbal_Add.Yaw;
            GM6020_Pitch.Set_Angle += Gimbal_Add.Pitch;
            break;
		}
		case Shoot_Plugins:
		{
			if(Computer_Rx_Message.find_bool == '1')
			{
				Set_Yaw=Set_Yaw*Yaw_ZiMiao_Filter[0]+Computer_Rx_Message.yaw*Yaw_ZiMiao_Filter[1]-Gimbal_Add.Yaw/0.5f;
				GM6020_Pitch.Set_Angle = GM6020_Pitch.Set_Angle*Pitch_ZiMiao_Filter[0]+Computer_Rx_Message.pitch*Yaw_ZiMiao_Filter[1]+Gimbal_Add.Pitch/0.5f;
			}else
			{
				Set_Yaw -= Gimbal_Add.Yaw;
				GM6020_Pitch.Set_Angle += Gimbal_Add.Pitch;
			}
				break;
		}
	}
    while(Set_Yaw > 360)
    {
        Set_Yaw -= 360;
    }
    while(Set_Yaw < 0)
    {
        Set_Yaw += 360;
    }
    GM6020_Pitch.Set_Angle = Limit_Min_Max(GM6020_Pitch.Set_Angle,Gimbal_Pitch_MIN,Gimbal_Pitch_MAX);
}
    
/**
 * @file Gimbal.c
 * @brief 云台开始运动
 * @author HWX
 * @date 2024/10/20
 */
void Gimbal_Move(void)
{
    Gimbal_Calculate();

    Gimbal_PID_Calc();

    Set_GM6020_Gimbal_Voltage(&hcan1,GM6020_Yaw,GM6020_Pitch);
}

/**
 * @file Gimbal.c
 * @brief 云台断电
 * @author HWX
 * @date 2024/10/20
 */
void Gimbal_Stop(void)
{
    Gimbal_PID_Calc();
    Set_GM6020_Gimbal_Voltage(&hcan1,GM6020_Yaw,GM6020_Pitch);
}

/**
 * @file Gimbal.c
 * @brief 遥控器控制云台
 * @author HWX
 * @date 2024/10/20
 */
void Gimbal_Remote_Control(void)
{
	Gimbal_Add.Pitch = (float)RC.ch1/Gimbal_Pithch_Set;
	Gimbal_Add.Yaw = (float)RC.ch0/Gimbal_Yaw_Set;
}

/**
 * @file Gimbal.c
 * @brief 云台PID计算
 * @author HWX
 * @date 2024/10/20
 */
void Gimbal_PID_Calc(void)
{
    //Yaw
    PID_Calc_Ink(&GM6020_Yaw.Angle_PID,Set_Yaw,IMU_angle[0]);
    GM6020_Yaw.Set_Speed = GM6020_Yaw.Angle_PID.output;
    PID_Calc_Speed(&(GM6020_Yaw.Speed_PID),GM6020_Yaw.Set_Speed,GM6020_Yaw.rotor_speed);
    //Pitch
    PID_Calc_Angle(&(GM6020_Pitch.Angle_PID),GM6020_Pitch.Set_Angle,GM6020_Pitch.rotor_angle);
    GM6020_Pitch.Set_Speed = GM6020_Pitch.Angle_PID.output;
    PID_Calc_Speed(&(GM6020_Pitch.Speed_PID),GM6020_Pitch.Set_Speed,GM6020_Pitch.rotor_speed);
}

/**
 * @file Gimbal.c
 * @brief 云台电机PID初始化
 * @author HWX
 * @date 2024/10/20
 */
void Gimbal_PID_Init_All(void)
{
	Gimbal_Init();

	PID_init(&GM6020_Yaw.Angle_PID,110,0.045,800,25000,25000);
	//110,0.05,830
  PID_init(&(GM6020_Yaw.Speed_PID),80,0.45,150,25000,25000);
	//80,0.45,150

    PID_init(&(GM6020_Pitch.Angle_PID),80,0.4,60,25000,25000);//80,0.4,60
	//2.5，0.01，0//2,0,0//2,0,0//60,0,60
    PID_init(&(GM6020_Pitch.Speed_PID),5,0,0,25000,25000);//5,0,0
	//50，0.3，0//35,0.25,0//50,0.3,50//3,0.4,0
}

/**
 * @file Gimbal.c
 * @brief 云台电机PID清空
 * @author HWX
 * @date 2024/10/20
 */
void Gimbal_PID_Clean_All(void)
{
    PID_init(&(GM6020_Pitch.Angle_PID),0,0,0,0,0);
    PID_init(&(GM6020_Pitch.Speed_PID),0,0,0,0,0);
    PID_init(&GM6020_Yaw.Angle_PID,0,0,0,0,0);
    PID_init(&(GM6020_Yaw.Speed_PID),0,0,0,0,0);
}
