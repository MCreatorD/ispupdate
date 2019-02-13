#include "isp.h"

#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "tftlcd.h"
#include "key.h"
#include "malloc.h"  
#include "MMC_SD.h" 
#include "flash.h"	
#include "mass_mal.h"
#include "usb_lib.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "memory.h"	    
#include "usb_bot.h"  

#include "usmart.h" 
#include "ff.h"  
#include "exfuns.h"


#include "string.h"



volatile u8  USART2_RX_BUF[USART2_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
u8  USART2_TX_BUF[USART2_SEND_LEN]; //���ջ���,���USART_SEND_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
u8  USART2_RX_STA;         		//����״̬���	

u8 ucISP_Step = 0;          //ISP���裬��ϸ�����IspProcess()
u8 bISP_ConnectSuccessful = 0;//�������ӳɹ���־ 0=δ���ӳɹ� 1=���ӳɹ�
u8 bProgramFault = 0;       //���ʧ�ܱ�־ 1=ʧ�� 0=����
u8 ucConnectNum = 0;        //���ִ���

u32 ulAddressBase=0x08000000;   //����̵Ļ���ַ
u32 ulAddressOffset = 0;        //ÿ��д��code��Ļ���ַ
u16 uiISP_SendPacket = 0;          //ÿ��дcode�����ݰ��ĸ���,��������200�ֽ�һ�����ݰ���ĩβ��С��200
u8 ucISP_SendEndPacByte = 0;
u8 ucISP_SendByteFill = 0;      //ĩβ�ֽ������������0xFF��������㹻4�ı���
//u16 uiISP_SendPacketCount = 0;

//������ִ���3��
#define MAX_CONNECT_NUM 3   

#define ACK 0x79
#define NACK 0x1f


 //�����ļ�
 //path:·��
 //����ֵ:ִ�н��
char cfilename[120];
u8 cfilenamelen;

void USART3_IRQHandler(void)                	//����1�жϷ������
{
	u8 Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART3);//(USART1->DR);	//��ȡ���յ�������
		USART2_RX_BUF[0] = Res;
		USART2_RX_STA  = 1;
		 		 
    } 
}


void ISP_init(void)
{
//    uart2_init(230400); //���ڳ�ʼ��Ϊ230400 	
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//ʹ��USART1��GPIObʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//ʹ��USART1��GPIObʱ��
 	USART_DeInit(USART3);  //��λ����1
	 //USART3_TX   Pb.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //Pb1
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOB, &GPIO_InitStructure); //��ʼ��Pb10
   
    //USART3_RX	  Pb.11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);  //��ʼ��Pb11

   //Usart1 NVIC ����

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = 460800;//һ������Ϊ;
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_Even;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

    USART_Init(USART3, &USART_InitStructure); //��ʼ������
    printf("UART3_CR1:0X%X%X\r\n",USART3->CR1>>8,USART3->CR1);
    
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�����ж�
    USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ��� 
    USART_ClearFlag(USART3, USART_FLAG_TC);
    
    
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_13;				 //LED0-->PB.0,1 �˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.0 1
    GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14);						 //Pb.13 �����
//    GPIO_ResetBits(GPIOB,GPIO_Pin_14);						 //Pb.14 �����

}

int UART2_SendOneByte(int ch)
{      
     
    while(USART_GetFlagStatus(USART3, USART_FLAG_TXE)==RESET)
    {
    }
    USART_SendData(USART3,(u8)ch); //
//	while((USART3->SR&0X40)==0);//ѭ������,ֱ���������   
//    USART3->DR = (u8) ch;      
	return ch;
}

int UART2_SendAddressPacket(u8 * buf,u8 len)   //�Զ���packetβ������xorУ����
{
    u8 ucXor = 0;
    u8 i;
    for(i=0;i<len;i++)
    {
        UART2_SendOneByte(buf[i]);
        ucXor = ucXor ^ buf[i];
    }
    UART2_SendOneByte(ucXor);
    return ucXor;
}

int UART2_SendDataPacket(u8 * buf,u8 len)   //�Զ���packetβ������xorУ����
{
    u8 ucXor = 0;
    u8 i;
    UART2_SendOneByte(len-1);
    ucXor = len-1;
    for(i=0;i<len;i++)
    {
        UART2_SendOneByte(buf[i]);
        ucXor = ucXor ^ buf[i];
    }
    UART2_SendOneByte(ucXor);
//    printf("Send Data Packet %d",ulAddressOffset/len);
    return ucXor;
}

u8 SearchRecentFile(unsigned char *filecnt)
{
	FRESULT res;	
    u16 i = 0;
    WORD fdate,ftime;
    char *FileName;   /* This function is assuming non-Unicode cfg. */
    u8 linelcd = 28;
		char fnum = '0';
		unsigned char tmp = 0;
	
	if(filecnt == NULL) return FR_INT_ERR;
		*filecnt = 0;	
	
	
		fileinfo.lfsize = _MAX_LFN * 2 + 1;
		fileinfo.lfname = (TCHAR*)mymalloc(fileinfo.lfsize);
		fdate = 0;
		ftime = 0;	


		res = f_opendir(&dir,"0:"); //��һ��Ŀ¼
		if (res == FR_OK) 
		{
				while(1)
				{
							res = f_readdir(&dir, &fileinfo);                   //��ȡĿ¼�µ�һ���ļ�
							if (res != FR_OK || fileinfo.fname[0] == 0) break;  //������/��ĩβ��,�˳�
							
							if(fdate < fileinfo.fdate)
							{
									FileName = *fileinfo.lfname ? fileinfo.lfname : fileinfo.fname;
							}
							else if((fdate == fileinfo.fdate)&&(ftime < fileinfo.ftime))
							{
									FileName = *fileinfo.lfname ? fileinfo.lfname : fileinfo.fname;
							}
							printf("FileName:%s\r\n",  FileName);//��ӡ�ļ���
							
							(*filecnt)++;						
							if(*filecnt == 1) continue;

							LCD_write_EN(0,linelcd,fnum,RED,WHITE);fnum++;
							LCD_write_EN(7,linelcd,':',RED,WHITE);
							LCD_write_EN_string(14,linelcd,FileName,BLUE,WHITE);
							linelcd += 14;
							tmp  = ((*filecnt)-2)*30;
							for(i=0;i<30;i++)
							{
									cfilename[i+tmp]=FileName[i];
									if(cfilename[i] == '\0') break;
							}			  
				} 
				(*filecnt)--;//������Ϻ��ȥ��һ�ε�����  System Volume Infomation  
		}
    
//    printf("FileName:%s\r\n",  cfilename);//��ӡ�ļ���	
	myfree(fileinfo.lfname); 
    return  res;   
}


//���ʧ�ܴ���
void ProgramFault(void)
{
    printf("ISP STEP %d ERROR\r\n",ucISP_Step);
    ucISP_Step = ISP_STOP;
    bProgramFault = 1;
    bISP_ConnectSuccessful = 0;
    ucConnectNum = 0;
    ISP_BOOT0_PORT = 0;
    ISP_RESET_PORT = 1;
    LED0 = 0;
    LED1 = 1;
    ulAddressOffset = 0;
    f_close(file);
    
}
u8 mf_GetIspStep(void)
{
    return ucISP_Step;
}
void IspProcess(void)
{
    static u16 delay = 0;
    static u8 tmp;
    u8 res,t=0;
    u16 i,tlen=0;
    u8 adr[4];
    u32 Address;
	
		unsigned char filenum;
		unsigned char Chosefilenum;
	
    if(bISP_ConnectSuccessful == 1)
    {
        if(tmp++>200)
        {
            tmp = 0;
            LED1 = ~LED1;
        }
        
    }
    switch(ucISP_Step)
    {
        case ISP_STOP:
        {
            //�ȴ���������
            t=KEY_Scan(0);
            if(t!=0) printf("Key id:%d\r\n",t);
            if((KEY1_PRES == t)||(ucConnectNum > 0))
            {
                ucConnectNum ++;
                ISP_BOOT0_PORT = 1;
                ISP_RESET_PORT = 0;
                delay_ms(200);
                ISP_RESET_PORT = 1;
                delay_ms(500);
                
                UART2_SendOneByte(0x7f);
//                UART2_SendOneByte(0xff);UART2_SendOneByte(0x55);
                bProgramFault = 0;
                LED0 = 1;
                LED1 = 0;
                ucISP_Step = ISP_CONNECT;
                delay = 100;
                printf("ucISP_Step-0:%d\r\n",ucISP_Step);
                
                //�����������ļ�

							if(SearchRecentFile(&filenum) != FR_OK) return ;
			
							//������ѡ��
							LCD_write_EN_string(0,28+filenum*14,"Chose bin: 0",BLUE,WHITE);
							printf("Chose bin: 0\r\n");
							//�ȴ��������� �����л�����
							do
							{
									delay_us(50);	
									t=KEY_Scan(0);
									if(t != 0) printf("chose  Key id:%d\r\n",t);
									if(t == 1)
									{
										Chosefilenum++;
										Chosefilenum = Chosefilenum%filenum;
										LCD_write_EN(77,28+filenum*14,0x30+Chosefilenum,BLUE,WHITE);
										//printf("File Name is:%s\r\n",cfilename+Chosefilenum*30);
									}							
								}while(t!=2);//����key2 һֱ�ȴ�


							//����ѡ����Ŵ��ļ�
							printf("File Name is:%s\r\n",cfilename+Chosefilenum*30);
						
                //
						  	//LCD_write_EN_string(0,28,cfilename,RED,WHITE);
								//LCD_write_EN_string(0,14,"USB Connecting..",RED,WHITE);
								
								
								
								
								
                //���ļ�
                res = f_open(file,(const TCHAR*)(cfilename+Chosefilenum*30000),1);
                if(res == FR_OK)
                {
									  //LCD_write_EN_string(0,42,"OK",RED,WHITE);
                    printf("File Name is:%s     OK\r\n",cfilename+Chosefilenum*30);
                    printf("File Size is:%ld\r\n",file->fsize);
                }
                else 
                {
                    printf("File Name is:%s                       Error\r\n",cfilename);
                    ProgramFault();
                    return;
                }
                //����ְ��������ֽ����������β����С���ȵ�    fsize
                uiISP_SendPacket = 1+file->fsize/USART2_SEND_LEN;
                ucISP_SendEndPacByte = file->fsize%USART2_SEND_LEN;
                ucISP_SendByteFill = 4 - ucISP_SendEndPacByte%4;
                printf("Packet:%d  . EndPacByte:%d  \r\n",uiISP_SendPacket,ucISP_SendEndPacByte);
                
            }
            
        }break;
        case ISP_CONNECT:
        {
            if(bISP_ConnectSuccessful == 1)
            {
                ProgramFault();
            }

            {
                if(delay--)
                {
                    if(USART2_RX_STA == 1)
                    {
                        USART2_RX_STA = 0;
                        if((USART2_RX_BUF[0] == ACK)||(USART2_RX_BUF[0] == NACK))
                        {
                            bISP_ConnectSuccessful = 1;
                            ucConnectNum = 0;
                            ucISP_Step = ISP_ERASE_START;
                            printf("ucISP_Step:%d\r\n",ucISP_Step);
                        }
                    }
                }
                else
                {
                    if(ucConnectNum > 3)    ProgramFault();
                    else    ucISP_Step = ISP_STOP;
                }
            }
        }break;
        
        case ISP_ERASE_START:
        {
            if(bISP_ConnectSuccessful != 1)
            {
                ProgramFault();
            }
            ucConnectNum = 0;
            UART2_SendOneByte(0x43);UART2_SendOneByte(0xBC);
            delay = 100;
            ucISP_Step = ISP_ERASE_ALL;
            printf("ucISP_Step:%d\r\n",ucISP_Step);
        }break;
        case ISP_ERASE_ALL:
        {
            if(bISP_ConnectSuccessful != 1)
            {
                ProgramFault();
            }
            if(delay--)
            {
                if(USART2_RX_STA == 1)
                {
                    USART2_RX_STA = 0;
                    if(USART2_RX_BUF[0] == ACK)
                    {
                        UART2_SendOneByte(0xFF);UART2_SendOneByte(0x00);
                        ucISP_Step = ISP_ERASEWHITE_WAIT;
                        delay = 2000;
                        printf("ucISP_Step:%d\r\n",ucISP_Step);
                    }
                    else
                    {
                        ProgramFault();
                    }
                }
            }
            else
            {
                ProgramFault();
            }
            
        }break;
        case ISP_ERASEWHITE_WAIT:
        {
            if(bISP_ConnectSuccessful != 1)
            {
                ProgramFault();
            }
            if(delay--)
            {
                if(USART2_RX_STA == 1)
                {
                    USART2_RX_STA = 0;
                    if(USART2_RX_BUF[0] == ACK)
                    {
                        ucISP_Step = ISP_PROGRAM_START;
                        printf("ucISP_Step:%d\r\n",ucISP_Step);
//                        delay = 100;
                    }
                    else
                    {
                        printf("This is uart3 rec error.%x\r\n",USART2_RX_BUF[0]);
                        ProgramFault();
                    }
                }
            }
            else
            {
                ProgramFault();
            }
        }break;
        case ISP_PROGRAM_START://5
        {
            
            //��һ������
            res=f_read(file,fatbuf,USART2_SEND_LEN%512,&br);
            if(res)	//�����ݳ�����
            {
                ProgramFault();  
            }else
            {
                tlen+=br;
                for(i=0;i<br;i++)USART2_TX_BUF[i]=fatbuf[i];    //br=�������ֽ������������������٣����Ͷ��٣�����β�����ô���200��
            }
            UART2_SendOneByte(0x31);UART2_SendOneByte(0xce);
            delay = 100;
            ucISP_Step = ISP_PROGRAM_ADD;
            printf("ucISP_Step:%d\r\n",ucISP_Step);
            
        }break;
        case ISP_PROGRAM_ADD://6
        {
            if(bISP_ConnectSuccessful != 1)
            {
                ProgramFault();
            }
            if(delay--)
            {
                if(USART2_RX_STA == 1)
                {
                    USART2_RX_STA = 0;
                    if(USART2_RX_BUF[0] == ACK)
                    {
                        Address = ulAddressBase+ulAddressOffset;								
		    			adr[3] = (u8)Address;
                        for(i = 1; i < 4; i++)
                        {
                            Address >>= 8;
                            adr[3-i]=(u8)Address;
                        }
                        UART2_SendAddressPacket(&adr[0],4);
                        if(uiISP_SendPacket > (1+ulAddressOffset/USART2_SEND_LEN))
                        {
                            ucISP_Step = ISP_PROGRAM_DATA;
                        }
                        else if(uiISP_SendPacket == (1+ulAddressOffset/USART2_SEND_LEN))
                        {
                            ucISP_Step = ISP_PROGRAM_END;
                        }
                        else
                        {
                            ProgramFault();
                        }
                        delay = 300;
                        printf("ucISP_Step:%d\r\n",ucISP_Step);
//                        printf("This white address:0x%x\r\n",ulAddressBase+ulAddressOffset);
                    }
                    else
                    {
                        ProgramFault();
                    }
                }
            }
            else
            {
                ProgramFault();
            }
        }break;
        case ISP_PROGRAM_DATA: //7
        {
            if(bISP_ConnectSuccessful != 1)
            {
                ProgramFault();
            }
            if(delay--)
            {
                if(USART2_RX_STA == 1)
                {
                    USART2_RX_STA = 0;
                    if(USART2_RX_BUF[0] == ACK)
                    {
                        if(uiISP_SendPacket > (1+ulAddressOffset/USART2_SEND_LEN))
                        {
                            UART2_SendDataPacket(&USART2_TX_BUF[0],USART2_SEND_LEN);
                            ulAddressOffset += USART2_SEND_LEN;
                            ucISP_Step = ISP_ERASEWHITE_WAIT;
                            delay = 50000;
                            printf("ucISP_Step:%d\r\n",ucISP_Step);
                            
                        }
                    }
                    else
                    {
                        ProgramFault();
                    }
                }
            }
            else
            {
                ProgramFault();
            }
        }break;
        case ISP_PROGRAM_END:
        {
            if(uiISP_SendPacket != (1+ulAddressOffset/USART2_SEND_LEN))
            {
                ProgramFault();
            }
            if(bISP_ConnectSuccessful != 1)
            {
                ProgramFault();
            }
            if(delay--)
            {
                if(USART2_RX_STA == 1)
                {
                    USART2_RX_STA = 0;
                    if(USART2_RX_BUF[0] == ACK)
                    {
                        if(uiISP_SendPacket == (1+ulAddressOffset/USART2_SEND_LEN))
                        {
                            UART2_SendDataPacket(&USART2_TX_BUF[0],ucISP_SendEndPacByte+ucISP_SendByteFill);

                            ucISP_Step = ISP_GO;
                            delay = 20000;
                            printf("ucISP_Step:%d\r\n",ucISP_Step);
                        }
                    }
                    else
                    {
                        ProgramFault();
                    }
                }
            }
            else
            {
                ProgramFault();
            }
        }break;
        case ISP_GO:
        {
            if(bISP_ConnectSuccessful != 1)
            {
                ProgramFault();
            }
            if(delay--)
            {
                if(USART2_RX_STA == 1)
                {
                    USART2_RX_STA = 0;
                    if(USART2_RX_BUF[0] == ACK)
                    {
                        UART2_SendOneByte(0x21);UART2_SendOneByte(0xde);
                        ucISP_Step = ISP_GO_ADD;
                        delay = 100;
                        printf("ucISP_Step:%d\r\n",ucISP_Step);
                    }
                    else
                    {
                        ProgramFault();
                    }
                }
            }
            else
            {
                ProgramFault();
            }
        }break;
        case ISP_GO_ADD:
        {
            if(bISP_ConnectSuccessful != 1)
            {
                ProgramFault();
            }
            if(delay--)
            {
                if(USART2_RX_STA == 1)
                {
                    USART2_RX_STA = 0;
                    if(USART2_RX_BUF[0] == ACK)
                    {
                        Address = ulAddressBase;								
		    			adr[3] = (u8)Address;
                        for(i = 1; i < 4; i++)
                        {
                            Address >>= 8;
                            adr[3-i]=(u8)Address;
                        }
                        UART2_SendAddressPacket(&adr[0],4);

                        ucISP_Step = ISP_STOP;

                        bISP_ConnectSuccessful = 0;
                        ucConnectNum = 0;
                        ISP_BOOT0_PORT = 0;
                        ISP_RESET_PORT = 1;
                        LED0 = 1;
                        LED1 = 0;
                        ulAddressOffset = 0;
                        f_close(file);
                    }
                    else
                    {
                        ProgramFault();
                    }
                }
            }
            else
            {
                ProgramFault();
            }
        }break;
        default:
        {
            ucISP_Step = 0;
        }break;
    }
}


 

