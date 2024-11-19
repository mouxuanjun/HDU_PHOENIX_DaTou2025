#ifndef __COMPUTER_H__
#define __COMPUTER_H__

#include "main.h"
#include "usb_device.h"
#include "GM6020.h"
#include "usbd_cdc_if.h"
#include "cmsis_os.h"
#include "Gimbal.h"

#pragma pack(1)

typedef struct
{
char start;               //0 帧头 's’
char type;                //1 消息类型 0xA0
float yaw;                //2-5 偏航角
float pitch;              //6-9 俯仰角
char find_bool;           //10 是否找到 '0' or '1'
char shoot_bool;          //11 是否开火 '0' or '1'
                          //......  12-30 预留空位
char end;                 //31 帧尾 'e’
}Computer_Rx_Message_t;

typedef struct
{
char start;               //0 帧头 's’
char type;                //1 消息类型 0xB0
char enemy_team_color;    //2 敌方颜色 蓝'b' 红'r'
char mode;                //3 自瞄模式 装甲板'a' 符'r'
char rune_flag;           //4 符模式 不可激活'0' 小符'1' 大符'2'
float yaw;                //5-8 偏航角
float pitch;              //9-12 俯仰角
                          //......  13-30 预留空位
char end;                 //31 帧尾 'e’
}Computer_Tx_Message_t;

#pragma pack()

void Computer_Init(void);
void Computer_Rx(void);
void Computer_Tx(void);

#endif
