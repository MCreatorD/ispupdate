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

#include "isp.h"
//ALIENTEK Mini STM32�����巶������34
//USB������ʵ��  
//����֧�֣�www.openedv.com
//������������ӿƼ����޹�˾ 

//����USB ����/����
//enable:0,�Ͽ�
//       1,��������	   
void usb_port_set(u8 enable)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	  	   	 
	if(enable)_SetCNTR(_GetCNTR()&(~(1<<1)));//�˳��ϵ�ģʽ
	else
	{	  
		_SetCNTR(_GetCNTR()|(1<<1));  // �ϵ�ģʽ
		GPIOA->CRH&=0XFFF00FFF;
		GPIOA->CRH|=0X00033000;
		PAout(12)=0;	    		  
	}
} 
 
 int main(void)
 {
	NVIC_Configuration(); 
	delay_init();	    	 //��ʱ������ʼ�� 
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ9600 	 
    
	LED_Init();         //LED��ʼ��	 		
    KEY_Init();         //LED��ʼ��	
	 InitLcd();
	 LCD_write_EN_string(0,0,"    update app       ",RED,WHITE);
	usmart_dev.init(72);	
 	mem_init();			//��ʼ���ڴ��		  	 
	exfuns_init();		//Ϊfatfs��ر��������ڴ�	

 	SPI_Flash_Init();
    
    ISP_init();

	if(SPI_FLASH_TYPE!=W25Q64)
    {
        printf("W25Q64 Error!");	//���SD������
    }
	else //SPI FLASH ����
	{   
		f_mount(fs[0],"0:",1); 					//����FLASH.
 	   	Mass_Memory_Size[0]=2048*1024*4;//2M�ֽ�
	    Mass_Block_Size[0] =4096;//��Ϊ������Init����������SD���Ĳ����ֽ�Ϊ512��,��������һ����512���ֽ�.
	    Mass_Block_Count[0]=Mass_Memory_Size[0]/Mass_Block_Size[0];
		printf("SPI FLASH Size:2048KB");	 
	}
 	
	delay_ms(800);
 	usb_port_set(0); 	//USB�ȶϿ�
	delay_ms(300);
   	usb_port_set(1);	//USB�ٴ�����	   
 	printf("USB Connecting...");//��ʾSD���Ѿ�׼����	
	LCD_write_EN_string(0,14,"USB Connecting..",RED,WHITE);
	Data_Buffer=mymalloc(BULK_MAX_PACKET_SIZE*2*4);	//ΪUSB���ݻ����������ڴ�
	Bulk_Data_Buff=mymalloc(BULK_MAX_PACKET_SIZE);	//�����ڴ�
 	//USB����
 	USB_Interrupts_Config();    
 	Set_USBClock();   
 	USB_Init();	    
	delay_ms(500);	 
    LED0 = 1;
    LED1 = 0;
    
	while(1)
	{	
		delay_us(50);	
        
		IspProcess();
        
	};  										    			    
}








