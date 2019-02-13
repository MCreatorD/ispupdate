#include "tftlcd.h"
#include "stm32f10x.h"
#include "delay.h"
#include "font.h"
//1. lcd��spi io�ڳ�ʼ��
void IO_INTI(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE); 
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	//LCD_SCL LCD_SDA LCD_RST  LCD_DC  //PIO_Pin_9|GPIO_Pin_8|GPIO_Pin_7|GPIO_Pin_6
	GPIO_InitStructure.GPIO_Pin = LCD_SCL|LCD_SDA|LCD_RST|LCD_DC;	   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure); //GPIOC	

	GPIO_SetBits(GPIOC,GPIO_Pin_9|GPIO_Pin_8|GPIO_Pin_7|GPIO_Pin_6);
}

//��SPI���ߴ���һ��8λ����
//ʱ�������LCD_SCL_SET����������Ǹ� CPOL = 1;
//ǰ����� ����׼������               CPHA = 1;
//�����Ч��SCK�½���,�½��زɼ� ���������
//SCK �����ص�MOSI��Ч
void  SendByte(unsigned char Data)
{
  unsigned char i=0;
  for(i=8;i>0;i--)
  {
    if(Data&0x80)	
      LCD_SDA_SET;//�������
    else 
      LCD_SDA_CLR;

    LCD_SCL_CLR;	

    LCD_SCL_SET;
      Data<<=1; 
  }
}

//��SPI���ߴ���һ��8λ����
void  SPI_WriteData(unsigned char Data)
{
  unsigned char i=0;
  
  for(i=8;i>0;i--)
  {
    if(Data&0x80)	
      LCD_SDA_SET;//�������
    else 
      LCD_SDA_CLR;
    
    LCD_SCL_CLR;       
    LCD_SCL_SET;
    Data<<=1; 
  }
}

//��Һ����дһ��8λָ��
void Lcd_WriteIndex(unsigned char Index)
{ 
  //LCD_CS_CLR;  //ALD del  SPIд����ʱ��ʼ
  LCD_DC_CLR;
  SPI_WriteData(Index);
  //LCD_CS_SET;  //ALD del
}
//��Һ����дһ��8λ����
void Lcd_WriteData(unsigned char Data)
{
  //LCD_CS_CLR;  //ALD del
  LCD_DC_SET;
  SPI_WriteData(Data);
  //LCD_CS_SET;  //ALD del 
}
//��Һ����дһ��16λ����
void LCD_WriteData_16Bit(unsigned short Data)
{
  //LCD_CS_CLR;  //ALD del
  LCD_DC_SET;
  SPI_WriteData(Data>>8); //д���8λ����
  SPI_WriteData(Data); 	  //д���8λ����
  //LCD_CS_SET;  //ALD del 
}
/****************************************************************************
* ��    ��: LCD_write_byte()
* ��    ��: ���ݵ�LCD 
* ��ڲ���: data    ��д�������
*           command ��д����/����ѡ��  
* ���ڲ���: �� 
****************************************************************************/
static void LCD_write_byte(unsigned char cd_data, unsigned char command)
{
  ////LCD_CS_CLR ;// ʹ��LCD
  if (command == 0)
    Lcd_WriteIndex(cd_data); // ��������
  else
    Lcd_WriteData(cd_data) ; // ��������
  //SendByte(data);
  //LCD_CS_SET; // �ر�LCD
  //LCD_DC_SET;	
}
// ��λlcd
void Lcd_Reset(void)  //rst 0 ... 1 ...
{
  LCD_RST_CLR;
  delay_ms(150);
  LCD_RST_SET;
  delay_ms(50);
}

/****************************************************************************
* ��    ��: LCD_clear()
* ��    ��: ��ָ����ɫ����
* ��ڲ���: color ��������ɫ
* ���ڲ���: ��
****************************************************************************/
void LCD_clear(unsigned int color)
{
  register unsigned int  i=0,j=0;
  register unsigned char ch;
  register unsigned char cl;
  
  LCD_set_window(0,0,X_MAX_PIXEL,Y_MAX_PIXEL);
  ch = color>>8;
  cl = color&0xFF;
  LCD_write_byte(0x2C,LCD_COMMAND);
  //LCD_CS_CLR;
  LCD_DC_SET;
  for (i=0;i<Y_MAX_PIXEL;i++)
    for (j=0;j<2;j++)
    {
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
    }
  LCD_DC_SET;
  //LCD_CS_SET;
}

/****************************************************************************
* ��    ��: LCD_set_window()
* ��    ��: ������ʾ����
* ��ڲ���: X - ��ʾ�������ϽǺ�����      Y ����ʾ�������Ͻ�������
*           Width ����ʾ���ڿ��          Height ����ʾ���ڸ߶�
* ���ڲ���: ��
****************************************************************************/
void LCD_set_window(unsigned int X    , unsigned int Y, 
                           unsigned int Width, unsigned int Height)
{
  LCD_write_byte(0x2A,LCD_COMMAND);         //�е�ַ��������
  LCD_write_byte(X>>8,LCD_DATA) ;           //������ʾ�������ϽǺ������λ
  LCD_write_byte((X+2)&0xFF,LCD_DATA);      //������ʾ�������ϽǺ������λ 	
  LCD_write_byte((X+Width+1)>>8,LCD_DATA);  //���ÿ�ȸ�λ 	
  LCD_write_byte((X+Width+1)&0xFF,LCD_DATA);// ���ÿ�ȵ�λ 	
  
  LCD_write_byte(0x2B,LCD_COMMAND);         //�е�ַ��������
  LCD_write_byte(Y>>8,LCD_DATA) ;           //������ʾ�������Ͻ��������λ
  LCD_write_byte((Y+3)&0xFF,LCD_DATA);      //������ʾ�������Ͻ��������λ 
  LCD_write_byte((Y+Height+2)>>8,LCD_DATA); // ������ʾ���ڸ߶ȸ�λ 	
  LCD_write_byte((Y+Height+2)&0xFF,LCD_DATA);// ������ʾ���ڸ߶ȵ�λ 	  
  LCD_write_byte(0x2C,LCD_COMMAND) ;
}

/****************************************************************************
* ��    ��: DrawPixel()
* ��    ��: ��ָ��λ����ָ����ɫ��ʾ����
****************************************************************************/
void DrawPixel(unsigned int x, unsigned int y, unsigned int color)
{
  //LCD_set_XY(x,y);  //�趨��ʾλ�� andy
  LCD_write_byte(color>>8,LCD_DATA);  //������ʾ��ɫ���ݸ�λ�ֽ�
  LCD_write_byte(color&0xFF,LCD_DATA);//������ʾ��ɫ���ݵ�λ�ֽ�
}

/****************************************************************************
* ��    ��: DrawVerticalLine()
* ��    ��: ��ָ��λ����ָ����ɫ����ֱ��
* ��ڲ���: X - ��ֱ����ʼλ�ú�����      Y ����ֱ����ʼλ��������
*           Length ����ֱ�߳���           Color ����ֱ����ɫ
* ���ڲ���: ��
****************************************************************************/
void DrawVerticalLine(unsigned int x, unsigned int y, 
                      unsigned int Length,unsigned int Color)               
{
  register int  index = 0;
  register char ch,cl;
  
  ch = (unsigned char)(Color>>8);
  cl = (unsigned char)(Color);
  LCD_set_window(x,y,1,Length);
  for(index=0;index<Length;index++)
  {
  LCD_write_byte(ch,LCD_DATA) ;
  LCD_write_byte(cl,LCD_DATA) ;
  }
}

/****************************************************************************
* ��    ��: DrawHorizonLine()
* ��    ��: ��ָ��λ����ָ����ɫ��ˮƽ��
* ��ڲ���: X - ˮƽ����ʼλ�ú�����      Y ��ˮƽ����ʼλ��������
*           Length ��ˮƽ�߳���           Color ��ˮƽ����ɫ
* ���ڲ���: ��
****************************************************************************/
void DrawHorizonLine(unsigned int x, unsigned int y, 
                     unsigned int Length,unsigned int Color)               
{
  register int  index = 0;
  register char ch,cl;
  
  ch = (unsigned char)(Color>>8);
  cl = (unsigned char)(Color);
  LCD_set_window(x,y,Length,1);
  for(index=0;index<Length;index++)
  {
    LCD_write_byte(ch,LCD_DATA);
    LCD_write_byte(cl,LCD_DATA);
  }
}

/****************************************************************************
* ��    ��: DrawRect()
* ��    ��: ��ָ��λ����ָ����ɫ������
* ��ڲ���: X - ������ʼλ�ú�����      Y ��������ʼλ��������
*           Width  �����ο��           Height �����θ߶�                   
*           Color  ��������ɫ             
* ���ڲ���: ��
****************************************************************************/
void DrawRect(unsigned int Xpos, unsigned int Ypos, unsigned int Width, 
              unsigned int Height,unsigned Color)
{
  DrawHorizonLine(Xpos, Ypos, Width, Color) ;
  DrawHorizonLine(Xpos, (Ypos + Height), Width, Color) ;
  DrawVerticalLine(Xpos, Ypos, Height, Color) ;
  DrawVerticalLine((Xpos+Width-1) ,Ypos, Height, Color);
}

/****************************************************************************
* ��    ��: DrawRectFill()
* ��    ��: ��ָ��λ����ָ����ɫ������
* ��ڲ���: X - ������ʼλ�ú�����      Y ��������ʼλ��������
*           Width  �����ο��           Height �����θ߶�                   
*           Color  �����������ɫ             
* ���ڲ���: ��
****************************************************************************/
void DrawRectFill(unsigned int Xpos  , unsigned int Ypos, unsigned int Width, 
                  unsigned int Height, unsigned Color                        )
{
  register unsigned int i;
  register unsigned int index = Width*Height/16+1;
  register char ch,cl;
  
  ch = (unsigned char)(Color>>8);
  cl = (unsigned char)(Color);
  LCD_set_window(Xpos,Ypos,Width,Height);
  LCD_write_byte(0x2C,LCD_COMMAND);
  //LCD_CS_CLR ; andy
  LCD_DC_SET ;
  for(i=0;i<index;i++)
  {
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl); 
      SendByte(ch);
      SendByte(cl);      
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
      SendByte(ch);
      SendByte(cl);
  }    
}

void InitLcd(void)
{
	IO_INTI();

  Lcd_Reset();
   
  LCD_write_byte(0x11,LCD_COMMAND); //�˳�˯��ģʽ
  delay_ms(1); 

  ///ST7735R Frame Rate
  LCD_write_byte(0xB1,LCD_COMMAND); 
  LCD_write_byte(0x01,LCD_DATA); 
  LCD_write_byte(0x2C,LCD_DATA); 
  LCD_write_byte(0x2D,LCD_DATA); 
  LCD_write_byte(0xB2,LCD_COMMAND); 
  LCD_write_byte(0x01,LCD_DATA); 
  LCD_write_byte(0x2C,LCD_DATA); 
  LCD_write_byte(0x2D,LCD_DATA); 
  LCD_write_byte(0xB3,LCD_COMMAND); 
  LCD_write_byte(0x01,LCD_DATA); 
  LCD_write_byte(0x2C,LCD_DATA); 
  LCD_write_byte(0x2D,LCD_DATA); 
  LCD_write_byte(0x01,LCD_DATA); 
  LCD_write_byte(0x2C,LCD_DATA); 
  LCD_write_byte(0x2D,LCD_DATA); 

  LCD_write_byte(0xB4,LCD_COMMAND);//�жԵ�
  LCD_write_byte(0x07,LCD_DATA); 
  //ST7735R�ӵ�����
  LCD_write_byte(0xC0,LCD_COMMAND); 
  LCD_write_byte(0xA2,LCD_DATA); 
  LCD_write_byte(0x02,LCD_DATA); 
  LCD_write_byte(0x84,LCD_DATA); 
  LCD_write_byte(0xC1,LCD_COMMAND); 
  LCD_write_byte(0xC5,LCD_DATA); 
  LCD_write_byte(0xC2,LCD_COMMAND); 
  LCD_write_byte(0x0A,LCD_DATA); 
  LCD_write_byte(0x00,LCD_DATA); 
  LCD_write_byte(0xC3,LCD_COMMAND); 
  LCD_write_byte(0x8A,LCD_DATA); 
  LCD_write_byte(0x2A,LCD_DATA); 
  LCD_write_byte(0xC4,LCD_COMMAND); 
  LCD_write_byte(0x8A,LCD_DATA); 
  LCD_write_byte(0xEE,LCD_DATA); 

  LCD_write_byte(0xC5,LCD_COMMAND); //��VCOM 
  LCD_write_byte(0x0E,LCD_DATA); 

  LCD_write_byte(0x36,LCD_COMMAND); //RGBģʽ 
  LCD_write_byte(0xC8,LCD_DATA); 
  
  // ST7735R Gamma У������
  LCD_write_byte(0xe0,LCD_COMMAND); 
  LCD_write_byte(0x0f,LCD_DATA); 
  LCD_write_byte(0x1a,LCD_DATA); 
  LCD_write_byte(0x0f,LCD_DATA); 
  LCD_write_byte(0x18,LCD_DATA); 
  LCD_write_byte(0x2f,LCD_DATA); 
  LCD_write_byte(0x28,LCD_DATA); 
  LCD_write_byte(0x20,LCD_DATA); 
  LCD_write_byte(0x22,LCD_DATA); 
  LCD_write_byte(0x1f,LCD_DATA); 
  LCD_write_byte(0x1b,LCD_DATA); 
  LCD_write_byte(0x23,LCD_DATA); 
  LCD_write_byte(0x37,LCD_DATA); 
  LCD_write_byte(0x00,LCD_DATA); 

  LCD_write_byte(0x07,LCD_DATA); 
  LCD_write_byte(0x02,LCD_DATA); 
  LCD_write_byte(0x10,LCD_DATA); 
  LCD_write_byte(0xe1,LCD_COMMAND); 
  LCD_write_byte(0x0f,LCD_DATA); 
  LCD_write_byte(0x1b,LCD_DATA); 
  LCD_write_byte(0x0f,LCD_DATA); 
  LCD_write_byte(0x17,LCD_DATA); 
  LCD_write_byte(0x33,LCD_DATA); 
  LCD_write_byte(0x2c,LCD_DATA); 
  LCD_write_byte(0x29,LCD_DATA); 
  LCD_write_byte(0x2e,LCD_DATA); 
  LCD_write_byte(0x30,LCD_DATA); 
  LCD_write_byte(0x30,LCD_DATA); 
  LCD_write_byte(0x39,LCD_DATA); 
  LCD_write_byte(0x3f,LCD_DATA); 
  LCD_write_byte(0x00,LCD_DATA); 
  LCD_write_byte(0x07,LCD_DATA); 
  LCD_write_byte(0x03,LCD_DATA); 
  LCD_write_byte(0x10,LCD_DATA);  

  LCD_write_byte(0x2a,LCD_COMMAND);
  LCD_write_byte(0x00,LCD_DATA);
  LCD_write_byte(0x00,LCD_DATA);
  LCD_write_byte(0x00,LCD_DATA);
  LCD_write_byte(0x7f,LCD_DATA);
  LCD_write_byte(0x2b,LCD_COMMAND);
  LCD_write_byte(0x00,LCD_DATA);
  LCD_write_byte(0x00,LCD_DATA);
  LCD_write_byte(0x00,LCD_DATA);
  LCD_write_byte(0x9f,LCD_DATA);

  LCD_write_byte(0xF0,LCD_COMMAND); //ʹ�ܲ�������  
  LCD_write_byte(0x01,LCD_DATA); 
  LCD_write_byte(0xF6,LCD_COMMAND); //�ر�ramʡ��ģʽ 
  LCD_write_byte(0x00,LCD_DATA); 

  LCD_write_byte(0x3A,LCD_COMMAND); //ɫ��16bit 
  LCD_write_byte(0x05,LCD_DATA); 
  
  LCD_write_byte(0x29,LCD_COMMAND); //����ʾ  
  LCD_clear(WINDOW_BK_COLOR);       //�Ա���ɫ���� 
}

/****************************************************************************
* ��    ��: LCD_write_EN()
* ��    ��: ��ָ��λ����ʾ 7 ��14 ����ASCII�ַ� 
* ��ڲ���: X - ��ʾλ�����ϽǺ�����      Y ����ʾλ�����Ͻ�������
*           c ����ʾ�ַ�                  
* ���ڲ���: ��
****************************************************************************/
void LCD_write_EN(unsigned int X, unsigned int Y, unsigned char c,unsigned int Color,unsigned int Color_BK)
{
  unsigned char i,j,temp;
//          Color    = BLACK; 
//        Color_BK = WHITE;  
 LCD_set_window(X,Y,7,14);
  //LCD_CS_CLR ;
  LCD_DC_SET ;
  for(i=0;i<14;i++)
  {
    temp = ASCII7x14[c-32][i];   //ʮ���ƴ��� �ո� ������һ��
    for(j=0;j<7;j++)
    {
      if(temp&0x80)
      {
        SendByte(Color>>8);  
        SendByte(Color) ;
      }
      else
      {
        SendByte(Color_BK>>8) ;  
        SendByte(Color_BK);
      }            
      temp <<= 1 ;
    }
  }
  //LCD_CS_SET ; andy
  LCD_DC_SET ;
}

/****************************************************************************
* ��    ��: LCD_write_EN_string()
* ��    ��: ��ָ��λ����ʾ 7 ��14 ����ASCII�ַ��� 
* ��ڲ���: X - ��ʾλ�����ϽǺ�����      Y ����ʾλ�����Ͻ�������
*           s ���ַ���ָ��                  
* ���ڲ���: ��
****************************************************************************/
void LCD_write_EN_string(unsigned char X,unsigned char Y,uint8 *s,unsigned int Color,unsigned int Color_BK)
{
  unsigned char i=0;
  
  while (*s)
  {
    LCD_write_EN(X+i*7,Y,*s, Color, Color_BK);
    s++;
    i++;
    if(i>=HAL_LCD_MAX_CHARS)return;
  }
  return;
}

/****************************************************************************
* ��    ��: LCD_write_CN()
* ��    ��: ��ָ��λ����ʾ 14 ��14 �������ַ�  
* ��ڲ���: X - ��ʾλ�����ϽǺ�����      Y ����ʾλ�����Ͻ�������
*           c ����ʾ�ַ�ָ��                  
* ���ڲ���: ��
****************************************************************************/
static void LCD_write_CN(unsigned int X, unsigned int Y, unsigned char c,unsigned int Color,unsigned int Color_BK)
{
  unsigned char i,j,temp;
  
  LCD_set_window(X,Y,14,14);  //���ַ������С������ʾ����
  //LCD_CS_CLR;               //Һ��SPIʹ��andy
  LCD_DC_SET;                 //���������ź�ʹ��
  for(i=0;i<28;i++)           // 14x14�ַ��������ݴ�СΪ2x14=28�ֽ�
  {
    temp = CN14x14[c][i] ;    //����ģ����
    for(j=0;j<8;j++)          // ��λ��ʾ
    {
      if(temp&0x80)           //���λֵΪ1����ʾ�ַ�ɫ
      {
        SendByte(Color>>8);  
        SendByte(Color) ;
      }
      else                    //���λֵΪ0����ʾ����ɫ
      {
        SendByte(Color_BK>>8) ;  
        SendByte(Color_BK);
      }            
      temp <<= 1 ;
    }
    i++ ;                     //ָ����һ�ֽ�
    temp = CN14x14[c][i];
    for(j=0;j<6;j++)          // ��λ��ʾ���������Ϊ14���أ�
    {                         // ����ÿ�е�2�ֽ����2λ����
      if(temp&0x80)           // ���λֵΪ1����ʾ�ַ�ɫ
      {
        SendByte(Color>>8);  
        SendByte(Color) ;
      }
      else                    // ���λֵΪ0����ʾ����ɫ
      {
        SendByte(Color_BK>>8) ;  
        SendByte(Color_BK);
      }            
      temp <<= 1 ;
    }
  }
  //LCD_CS_SET;               //Һ��SPI��ֹ
  LCD_DC_SET;                 //���ݿ����źŽ�ֹ
}

/****************************************************************************
* ��    ��: LCD_write_CN_string()
* ��    ��: ��ָ��λ����ʾ 14 ��14 ���������ַ���   
* ��ڲ���: X - ��ʾλ�����ϽǺ�����      Y ����ʾλ�����Ͻ�������
*           s ���ַ���ָ��                  
* ���ڲ���: ��
****************************************************************************/
void LCD_write_CN_string(unsigned char X,unsigned char Y,uint8 *s,unsigned int Color,unsigned int Color_BK)
{
  unsigned char i=0;
  static unsigned char j=0;
  while (*s)
  {
    for(j=0;j<ALL_CNS;j++)                  //����������ģ
    { 
      if(  *s    ==CN14x14_Index[j*2]            
         &&*(s+1)==CN14x14_Index[j*2+1])    //���������������ģ����������
      {
        LCD_write_CN(X+i*14,Y,j, Color, Color_BK) ;          //��ʾ����
        break; 
      }
    }
    if(j>=ALL_CNS)                          //�޴��ֵĵ�����ģ
    {
      DrawRectFill(X+i*14,Y,14,14,Color_BK);//��ʾ�ո�
    }
    s++; s++ ;                              //�ַ�ָ����λ
    i++;
    if(i>=HAL_LCD_MAX_CHARS/2)  
      return;                               //���������ʾ�ַ���������
  }
  return ;
}

/****************************************************************************
* ��    ��: ShowImage()
* ��    ��: ��ʾͼƬ   
* ��ڲ���: p - ͼƬ��������  ȡģ��ʽ ˮƽɨ�� ������ ��λ��ǰ           
* ���ڲ���: ��
****************************************************************************/
void ShowImage(const unsigned char *p) 
{
  int i; 
  unsigned char picH,picL;
  //LCD_clear(White); //����
  
  LCD_set_window(24,5,80,40); //��������
  for(i=0;i<80*40;i++)
  {	
    picL=*(p+i*2);	//���ݵ�λ��ǰ
    picH=*(p+i*2+1);				
    LCD_WriteData_16Bit(picH<<8|picL);  						
  }	
}