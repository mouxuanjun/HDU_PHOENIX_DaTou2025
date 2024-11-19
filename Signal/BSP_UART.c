#include "BSP_UART.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		Computer_Rx_Message();
	}else if(huart->Instance == USART3)
	{
			RC_Processing_Data();
	}
}
