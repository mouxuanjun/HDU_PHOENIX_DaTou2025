#include "Computer.h"

uint8_t Rx_data[32],Tx_data[32];
uint32_t len;

extern Computer_Rx_Message_t Computer_Rx_Message;
extern Computer_Tx_Message_t Computer_Tx_Message;
extern float IMU_angle[3];
extern Moto_GM6020_t GM6020_Pitch;

/**
 * @file Computer.c
 * @brief 小电脑通信初始化
 * @author HWX
 * @date 2024/11/2
 */
void Computer_Init(void)
{
    Computer_Tx_Message.start = 's';
    Computer_Tx_Message.type = 0xB0;//调试用
    Computer_Tx_Message.enemy_team_color = 0;//调试用
    Computer_Tx_Message.mode = 0;//调试用
    Computer_Tx_Message.rune_flag = 0;//调试用
    Computer_Tx_Message.end = 'e';
}

/**
 * @file Computer.c
 * @brief 小电脑通信接收处理
 * @author HWX
 * @date 2024/11/2
 */
void Computer_Rx(void)
{
  uint32_t data;
	USBD_Interface_fops_FS.Receive(Rx_data,&len);

	if(Rx_data[0] == 's'&&Rx_data[31] == 'e')
	{
		Computer_Rx_Message.start = Rx_data[0];

		Computer_Rx_Message.type = Rx_data[1];
    Computer_Rx_Message.find_bool = Rx_data[2];

		data = ((Rx_data[3])|(Rx_data[4]<<8)|(Rx_data[5]<<16)|(Rx_data[6]<<24));
		Computer_Rx_Message.yaw = *(float*)&data;
		data = ((Rx_data[7])|(Rx_data[8]<<8)|(Rx_data[9]<<16)|(Rx_data[10]<<24));
		Computer_Rx_Message.pitch = *(float*)&data;

		Computer_Rx_Message.end = Rx_data[31];
		
		Computer_Rx_Message.yaw *= 57.32484f;
    Computer_Rx_Message.pitch = Gimbal_Pitch_ZERO+Computer_Rx_Message.pitch*1304.4586f;//把弧度值转化成编码器的值
	}
}

/**
 * @file Computer.c
 * @brief 小电脑通信初发送处理
 * @author HWX
 * @date 2024/11/2
 */
void Computer_Tx(void)
{
		Computer_Tx_Message.yaw = IMU_angle[0]*0.0174444f;
    Computer_Tx_Message.pitch = (GM6020_Pitch.rotor_angle-Gimbal_Pitch_ZERO)*0.0007660156f;//把编码器的值转换成Pitch角度值
		
    Tx_data[0] = *(char*)&Computer_Tx_Message.start;

    Tx_data[1] = *(char*)&Computer_Tx_Message.type;

    Tx_data[2] = (*(int32_t*)&Computer_Tx_Message.yaw)&0xff;
    Tx_data[3] = (*(int32_t*)&Computer_Tx_Message.yaw>>8)&0xff;
    Tx_data[4] = (*(int32_t*)&Computer_Tx_Message.yaw>>16)&0xff;
    Tx_data[5] = (*(int32_t*)&Computer_Tx_Message.yaw>>24)&0xff;

    Tx_data[6] = (*(int32_t*)&Computer_Tx_Message.pitch)&0xff;
    Tx_data[7] = (*(int32_t*)&Computer_Tx_Message.pitch>>8)&0xff;
    Tx_data[8] = (*(int32_t*)&Computer_Tx_Message.pitch>>16)&0xff;
    Tx_data[9] = (*(int32_t*)&Computer_Tx_Message.pitch>>24)&0xff;

    Tx_data[10] = Computer_Tx_Message.enemy_team_color;
    Tx_data[11] = Computer_Tx_Message.mode;
    Tx_data[12] = Computer_Tx_Message.rune_flag;


    Tx_data[31] = *(char*)&Computer_Tx_Message.end;
    CDC_Transmit_FS(Tx_data,32);
}
