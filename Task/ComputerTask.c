#include "ComputerTask.h"



void ComputerTask(void const * argument)
{
  Computer_Init();
  while(1)
  { 
    Computer_Rx();
    Computer_Tx();
		osDelay(1);
  }
}
