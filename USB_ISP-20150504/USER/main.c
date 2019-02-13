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
//ALIENTEK Mini STM32开发板范例代码34
//USB读卡器实验  
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司 

//设置USB 连接/断线
//enable:0,断开
//       1,允许连接	   
void usb_port_set(u8 enable)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	  	   	 
	if(enable)_SetCNTR(_GetCNTR()&(~(1<<1)));//退出断电模式
	else
	{	  
		_SetCNTR(_GetCNTR()|(1<<1));  // 断电模式
		GPIOA->CRH&=0XFFF00FFF;
		GPIOA->CRH|=0X00033000;
		PAout(12)=0;	    		  
	}
} 
 
 int main(void)
 {
	NVIC_Configuration(); 
	delay_init();	    	 //延时函数初始化 
	uart_init(115200);	 	//串口初始化为9600 	 
    
	LED_Init();         //LED初始化	 		
    KEY_Init();         //LED初始化	
	 InitLcd();
	 LCD_write_EN_string(0,0,"    update app       ",RED,WHITE);
	usmart_dev.init(72);	
 	mem_init();			//初始化内存池		  	 
	exfuns_init();		//为fatfs相关变量申请内存	

 	SPI_Flash_Init();
    
    ISP_init();

	if(SPI_FLASH_TYPE!=W25Q64)
    {
        printf("W25Q64 Error!");	//检测SD卡错误
    }
	else //SPI FLASH 正常
	{   
		f_mount(fs[0],"0:",1); 					//挂载FLASH.
 	   	Mass_Memory_Size[0]=2048*1024*4;//2M字节
	    Mass_Block_Size[0] =4096;//因为我们在Init里面设置了SD卡的操作字节为512个,所以这里一定是512个字节.
	    Mass_Block_Count[0]=Mass_Memory_Size[0]/Mass_Block_Size[0];
		printf("SPI FLASH Size:2048KB");	 
	}
 	
	delay_ms(800);
 	usb_port_set(0); 	//USB先断开
	delay_ms(300);
   	usb_port_set(1);	//USB再次连接	   
 	printf("USB Connecting...");//提示SD卡已经准备了	
	LCD_write_EN_string(0,14,"USB Connecting..",RED,WHITE);
	Data_Buffer=mymalloc(BULK_MAX_PACKET_SIZE*2*4);	//为USB数据缓存区申请内存
	Bulk_Data_Buff=mymalloc(BULK_MAX_PACKET_SIZE);	//申请内存
 	//USB配置
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








