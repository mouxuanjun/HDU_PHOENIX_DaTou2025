#include "Gimbal.h"

extern Car_Action Last_Action;

extern float IMU_angle[3];
extern Moto_GM6020_t GM6020_Yaw,GM6020_Pitch;
extern Car_Mode_t Car_Mode;
extern RC_t RC;
extern Gimbal_Add_t Gimbal_Add;
extern float Set_Yaw,Set_Pitch;
extern Computer_Rx_Message_t Computer_Rx_Message;
//自喵过滤器
float Yaw_ZiMiao_Filter[2]={0.01,0.99};
float Pitch_ZiMiao_Filter[2]={0.01,0.99};
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
    Set_Pitch = 0.0f;
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
        Set_Yaw -= Gimbal_Add.Yaw;
        Set_Pitch += Gimbal_Add.Pitch;
				break;
    case Shoot_Single: 
        Set_Yaw -= Gimbal_Add.Yaw;
        Set_Pitch += Gimbal_Add.Pitch;
        break;
    case Shoot_Plugins:

/***********************测试代码**************************/

        Set_Pitch=Set_Pitch*Pitch_ZiMiao_Filter[1]+Computer_Rx_Message.pitch*Pitch_ZiMiao_Filter[0];
        Set_Yaw=Set_Yaw*Yaw_ZiMiao_Filter[1]+Computer_Rx_Message.yaw*Yaw_ZiMiao_Filter[0];

/**************************测试完******************************/
		if(Computer_Rx_Message.find_bool == '1')
		{
            Set_Yaw=Computer_Rx_Message.yaw*Yaw_ZiMiao_Filter[0]+Set_Yaw*Yaw_ZiMiao_Filter[1];
            GM6020_Pitch.Set_Angle=Computer_Rx_Message.pitch*Pitch_ZiMiao_Filter[0]+GM6020_Pitch.Set_Angle*Pitch_ZiMiao_Filter[1];
		}else
		{
            Set_Yaw -= Gimbal_Add.Yaw;
            Set_Pitch += Gimbal_Add.Pitch;
		}
        break;

    }
    while(Set_Yaw > 180)
    {
        Set_Yaw -= 360;
    }
    while(Set_Yaw < -180)
    {
        Set_Yaw += 360;
    }
    while(Set_Pitch > Gimbal_Pitch_MAX)
    {
        Set_Pitch = Gimbal_Pitch_MAX;
    }
    while(Set_Pitch < Gimbal_Pitch_MIN)
    {
        Set_Pitch = Gimbal_Pitch_MIN;
    }    
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
    PID_Calc_Ink(&GM6020_Pitch.Angle_PID,Set_Pitch,IMU_angle[2]);
    GM6020_Pitch.Set_Speed = -GM6020_Pitch.Angle_PID.output;
    PID_Calc_Speed(&GM6020_Pitch.Speed_PID,GM6020_Pitch.Set_Speed,GM6020_Pitch.rotor_speed);
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

	PID_init(&GM6020_Yaw.Angle_PID,50,0,2000,25000,25000);//60,0,2500//50,0,2000
	//110,0.05,830//110,0.045,800
    PID_init(&(GM6020_Yaw.Speed_PID),195,1.3,0,25000,25000);//200,1.3.0//195,1.3,0
	//80,0.45,150//80,0.45,150
	
    PID_init(&(GM6020_Pitch.Angle_PID),35,0,1000,25000,25000);
	//编码器；//200,0,75//200,0,200//80,0.4,60//60,0,90//2.5，0.01，0//2,0,0//2,0,0//60,0,60//200,0,400//30,0,600
    PID_init(&(GM6020_Pitch.Speed_PID),150,2,0,25000,25000);//120,1,0
	//编码器：//2.75,0,0//1,0,0//5,0,0//3,0,0//50，0.3，0//35,0.25,0//50,0.3,50//3,0.4,0//1.2,0,0//120,1,0
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
