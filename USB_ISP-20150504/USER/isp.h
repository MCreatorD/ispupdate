#ifndef __USART2_H
#define __USART2_H

#include "stdio.h"	
#include "sys.h" 

#define USART2_REC_LEN     			5  	//定义最大接收字节数 5
#define USART2_SEND_LEN  			200  	//定义最大发送字节数 200
#define EN_USART2_RX 			1		//使能（1）/禁止（0）串口1接收

enum EP_ISP_STEP
{
    ISP_STOP=0,
    ISP_CONNECT,
    ISP_ERASE_START,
    ISP_ERASE_ALL,
    ISP_ERASEWHITE_WAIT,
    ISP_PROGRAM_START,
    ISP_PROGRAM_ADD,
    ISP_PROGRAM_DATA,
    ISP_PROGRAM_END,
    ISP_GO,
    ISP_GO_ADD,
};


#define ISP_RESET_PORT PBout(13)	// PA0
#define ISP_BOOT0_PORT PBout(14)	// PA1

extern volatile u8  USART2_RX_BUF[USART2_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u8  USART2_TX_BUF[USART2_SEND_LEN]; //接收缓冲,最大USART_SEND_LEN个字节.末字节为换行符 
extern u8 USART2_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void ISP_init(void);
void IspProcess(void);
u8 mf_GetIspStep(void);
u8 SearchRecentFile(unsigned char *filecnt);
#endif


