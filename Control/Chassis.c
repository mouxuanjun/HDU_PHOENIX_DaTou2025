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
int16_t   timeXFron,timeXBack,timeYLeft,timeYRigh;//键盘    w   s   a   d
//键盘模式下全向移动计算,斜坡量
float Slope_Chassis_Move_Fron, Slope_Chassis_Move_Back;
float Slope_Chassis_Move_Left, Slope_Chassis_Move_Righ;
float Chassis_Standard_Move_Max;//调整速度限幅,水平移动

/********************解算部分********************/
void Chassis_Solution(void);
void Chassis_Motor_Solution(void);
float Find_Angle(void);

/********************输入控制部分********************/
void Chassis_Remote_Control(void);
void Chassis_Mode_Choose(void);
float Chassis_Key_MoveRamp(uint8_t status,int16_t *time,int16_t inc,int16_t dec);

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

/**
  * @brief  键盘选择底盘模式
  * @author HWX
  * @attention 模式选择,进入小陀螺模式后退出到普通模式
  * @date 2025/1/26
  */
void Chassis_Mode_Choose(void)
{
//////////////////F键选择底盘跟随云台/////////////
    if(IF_KEY_PRESSED_F)
    {
        Car_Mode.Action=FOLLOW;
    }else
/////////////////Shift键选择小陀螺/////////////////////
    if(IF_KEY_PRESSED_SHIFT)
    {
        Car_Mode.Action = GYROSCOPE;
    }else
    if(IF_KEY_PRESSED_R)
    {
		Car_Mode.Action=NORMAL;
    }else
    if(!IF_KEY_PRESSED_SHIFT)
    {
        if(Car_Mode.Action==GYROSCOPE)
            Car_Mode.Action= NORMAL;
    }
}

/**
  * @brief  键盘模式下底盘运动计算，键盘解算
  * @param  前面是最大速度(最大293)
  * @param  后两个是斜坡量增加速度
  * @attention  键盘控制前后左右平移,平移无机械和陀螺仪模式之分.需要获取时间来进行斜坡函数计算
  */
void Chassis_Keyboard_Control( int16_t sMoveMax, int16_t sMoveRamp_inc, int16_t sMoveRamp_dec )
{
    static portTickType  ulCurrentTime = 0;

    static uint32_t  ulDelay = 0;
	static uint16_t res = 3;
	//消抖参数
    static uint16_t w_cnt = 0;
    static bool W = 0;
    static uint16_t s_cnt = 0;
    static bool S = 0;
    static uint16_t a_cnt = 0;
    static bool A = 0;
    static uint16_t d_cnt = 0;
    static bool D = 0;
    Chassis_Standard_Move_Max = sMoveMax;//调整速度限幅,水平移动
    ulCurrentTime = xTaskGetTickCount();//当前系统时间
    if (ulCurrentTime >= ulDelay)//每10ms变化一次斜坡量
    {
        ulDelay = ulCurrentTime + 14;
		//下面是键盘消抖
        if (IF_KEY_PRESSED_W)
        {
            W = 1;
            timeXFron = 0;//按下前进则后退斜坡归零,方便下次计算后退斜坡
        }
        if(!IF_KEY_PRESSED_W)//放开按键w_cnt++归零
        {
            w_cnt++;
        }
        if(w_cnt > res)
        {
            w_cnt = 0;
            W = 0;
        }
        if (IF_KEY_PRESSED_S)
        {
            S = 1;
            timeXBack = 0;//同理
        }
        if(!IF_KEY_PRESSED_S)//类似于消抖的操作
        {
            s_cnt++;
        }
        if(s_cnt > res)
        {
            s_cnt = 0;
            S = 0;
        }

        if (IF_KEY_PRESSED_D)
        {
            D=1;
            timeYRigh = 0;
        }
        if(!IF_KEY_PRESSED_D)
        {
            d_cnt++;
        }
        if(d_cnt > res)
        {
            d_cnt = 0;
            D = 0;
        }

        if (IF_KEY_PRESSED_A)
        {
            A=1;
            timeYLeft = 0;
        }
        if(!IF_KEY_PRESSED_A)
        {
            a_cnt++;
        }
        if(a_cnt > res)
        {
            a_cnt = 0;
            A = 0;
        }

		//消抖后斜坡计算，算出整体底盘速度，w按下进入加速，放开进入减速
        Slope_Chassis_Move_Fron = (int16_t)(Chassis_Standard_Move_Max * Chassis_Key_MoveRamp( W, &timeXFron, sMoveRamp_inc, sMoveRamp_dec ) );//乘一个0到1的比例，用斜坡,W为1时触发
        Slope_Chassis_Move_Back = (int16_t)(Chassis_Standard_Move_Max * Chassis_Key_MoveRamp( S, &timeXBack, sMoveRamp_inc, sMoveRamp_dec ) );
        Slope_Chassis_Move_Left = (int16_t)(Chassis_Standard_Move_Max * Chassis_Key_MoveRamp( A, &timeYRigh, sMoveRamp_inc, sMoveRamp_dec ) );
        Slope_Chassis_Move_Righ = (int16_t)(Chassis_Standard_Move_Max * Chassis_Key_MoveRamp( D, &timeYLeft, sMoveRamp_inc, sMoveRamp_dec ) );
    }
}

/**
  * @brief  底盘键盘斜坡函数
  * @param status：表示当前的状态，1 表示加速，0 表示减速。
  * @param time 指向一个整数变量，表示时间累加值。
  * @param inc 加速时的时间增量。
  * @param dec：减速时的时间减量。
  * @retval 斜坡比例系数
  * @attention  0~1
  */
float Chassis_Key_MoveRamp(uint8_t status,int16_t *time,int16_t inc,int16_t dec)
{
    float  factor = 0;
    factor = 0.15 * sqrt( 0.15 * (*time) );  //计算速度斜坡,time累加到296.3斜坡就完成
    if (status == 1) {
        if (factor < 1)//防止time太大
        {
            *time += inc;
        }
    }
    else {
        if (factor > 0) {
            *time -= dec;
            if (*time < 0) {
                *time = 0;
            }
        }
    }
//    LimtValue_f(&factor, 1, 0 );//注意一定是float类型限幅
    return factor;  //注意方向
}

/**
  * @brief  线性斜坡函数
  * @param final：表示最终的目标值。
  * @param now：表示当前的值。
  * @param ramp：表示斜坡值，即每次调整的最大增量或减量。(越大越快)
  * @retval 当前输出
  */
float see_now=0;
float RAMP_float( float final, float now, float ramp )
{
	see_now=now;
	float buffer = 0;
	buffer = final - see_now;
	if (buffer > 0){
			if (buffer > ramp){  
					see_now += ramp;
			}   
			else{
					see_now += buffer;
			}
	}
	else{
			if (buffer < -ramp){
					see_now += -ramp;
			}
			else{
					see_now += buffer;
			}
	}
	return see_now;
}

/**
  * @brief  键盘控制底盘模式
  * @param  void
  * @retval void
  * @attention
  */
int SZUPUP=2000;
int speed_fly_start=3000;
int temp_power_limit = 0;
float temp_vw_gyroscope = 0.0f;
float temp_vm = 0.0f;
void CHAS_Key_Ctrl(void) 
{
	int max_speed=0;//按键速度上限
	int max_speed_gyroscope=0;//小陀螺速度上限
//	int power_limit=JUDGE_usGetPowerLimit();
//    temp_power_limit = power_limit;
	float vw_gyroscope=0;
    if(temp_power_limit==40){//比赛刚刚开始未设置模式
			max_speed=3500;
			max_speed_gyroscope=2000;
			vw_gyroscope=0.0035;
		}else if(temp_power_limit==45){//血量优先功率设置
			max_speed=4000;
			max_speed_gyroscope=2250;
			vw_gyroscope=0.0040;
		}else if(temp_power_limit==50){//血量优先下保证续航
			max_speed=4500;
			max_speed_gyroscope=2500;
			vw_gyroscope=0.0045;
		}else if(temp_power_limit==55){
			max_speed=5000;
			max_speed_gyroscope=2750;
			vw_gyroscope=0.0050;
		}else if(temp_power_limit==60){
			max_speed=5500;
			max_speed_gyroscope=3000;
			vw_gyroscope=0.0055;
		}else if(temp_power_limit==65){
			max_speed=6000;
			max_speed_gyroscope=3250;
			vw_gyroscope=0.060;
        }else if(temp_power_limit==70){
			max_speed=6500;
			max_speed_gyroscope=3500;
			vw_gyroscope=0.065;
        }else if(temp_power_limit==75){
			max_speed=7000;
			max_speed_gyroscope=3750;
			vw_gyroscope=0.0070;
        }else if(temp_power_limit==80){
			max_speed=7500;
			max_speed_gyroscope=4000;
			vw_gyroscope=0.0070;
		}else if(temp_power_limit==85){
			max_speed=8000;
			max_speed_gyroscope=4250;
			vw_gyroscope=0.0070;
        }else if(temp_power_limit==90){
			max_speed=8000;
			max_speed_gyroscope=4500;
			vw_gyroscope=0.0070;
        }else if(temp_power_limit==95){
			max_speed=8000;
			max_speed_gyroscope=5000;
			vw_gyroscope=0.00700;
        }else if(temp_power_limit==100){
			max_speed=8000;
			max_speed_gyroscope=5000;
			vw_gyroscope=0.0070;
		}
        else if(temp_power_limit==120){
			max_speed=8000;
			max_speed_gyroscope=5000;
			vw_gyroscope=0.00700;
        }else if(temp_power_limit==200){//自检三分钟
			max_speed=10000;
			max_speed_gyroscope=5000;
			vw_gyroscope=0.0070;
		}else{
			max_speed=8000;
			max_speed_gyroscope=5000;
			vw_gyroscope=0.0070;
		}
//	if(IF_KEY_PRESSED_B){
//		max_speed=max_speed*0.2;
//	}
//    if(IF_KEY_PRESSED_Z)
//    {
//        max_speed=2000;
//        max_speed_gyroscope=2000;
//    }
    switch (Car_Mode.Action) {//模式选择
    case NORMAL://正常的模式，底盘不跟随云台行走
    { 
        Chassis_Keyboard_Control(max_speed, 1,40);//3,25
		Chassis_Speed.vw=0;
        break;
	}
    case FOLLOW://跟随云台走
    {   
        Chassis_Keyboard_Control(max_speed, 1,40);
        PID_Calc_Angle(&Follow_PID,0.0f,err);
        Chassis_Speed.vw = Follow_PID.output/1000;
    }
    break;
    case GYROSCOPE://小陀螺模式
    {   
        Chassis_Keyboard_Control(max_speed_gyroscope, 1,20);
        Chassis_Speed.vw = RAMP_float(vw_gyroscope, Chassis_Speed.vw, 0.00006);
    }
    break;
    default:
        break;
    }
}

