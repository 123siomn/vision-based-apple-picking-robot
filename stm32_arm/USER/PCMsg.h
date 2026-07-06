#ifndef _PC_MSG_H_
#define _PC_MSG_H_
#include "include.h"

void InitUart1(void);
void Uart1SendData(BYTE dat);
void TaskPCMsgHandle(void);

#endif
