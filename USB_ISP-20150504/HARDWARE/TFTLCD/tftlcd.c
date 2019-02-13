#include "tftlcd.h"
#include "stm32f10x.h"
#include "delay.h"
#include "font.h"
//1. lcd的spi io口初始化
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

//向SPI总线传输一个8位数据
//时钟线最后LCD_SCL_SET，代表空闲是高 CPOL = 1;
//前沿输出 后延准备数据               CPHA = 1;
//输出有效到SCK下降沿,下降沿采集 上升沿输出
//SCK 上升沿到MOSI有效
void  SendByte(unsigned char Data)
{
  unsigned char i=0;
  for(i=8;i>0;i--)
  {
    if(Data&0x80)	
      LCD_SDA_SET;//输出数据
    else 
      LCD_SDA_CLR;

    LCD_SCL_CLR;	

    LCD_SCL_SET;
      Data<<=1; 
  }
}

//向SPI总线传输一个8位数据
void  SPI_WriteData(unsigned char Data)
{
  unsigned char i=0;
  
  for(i=8;i>0;i--)
  {
    if(Data&0x80)	
      LCD_SDA_SET;//输出数据
    else 
      LCD_SDA_CLR;
    
    LCD_SCL_CLR;       
    LCD_SCL_SET;
    Data<<=1; 
  }
}

//向液晶屏写一个8位指令
void Lcd_WriteIndex(unsigned char Index)
{ 
  //LCD_CS_CLR;  //ALD del  SPI写命令时序开始
  LCD_DC_CLR;
  SPI_WriteData(Index);
  //LCD_CS_SET;  //ALD del
}
//向液晶屏写一个8位数据
void Lcd_WriteData(unsigned char Data)
{
  //LCD_CS_CLR;  //ALD del
  LCD_DC_SET;
  SPI_WriteData(Data);
  //LCD_CS_SET;  //ALD del 
}
//向液晶屏写一个16位数据
void LCD_WriteData_16Bit(unsigned short Data)
{
  //LCD_CS_CLR;  //ALD del
  LCD_DC_SET;
  SPI_WriteData(Data>>8); //写入高8位数据
  SPI_WriteData(Data); 	  //写入低8位数据
  //LCD_CS_SET;  //ALD del 
}
/****************************************************************************
* 名    称: LCD_write_byte()
* 功    能: 数据到LCD 
* 入口参数: data    —写入的数据
*           command —写数据/命令选择  
* 出口参数: 无 
****************************************************************************/
static void LCD_write_byte(unsigned char cd_data, unsigned char command)
{
  ////LCD_CS_CLR ;// 使能LCD
  if (command == 0)
    Lcd_WriteIndex(cd_data); // 传送命令
  else
    Lcd_WriteData(cd_data) ; // 传送数据
  //SendByte(data);
  //LCD_CS_SET; // 关闭LCD
  //LCD_DC_SET;	
}
// 复位lcd
void Lcd_Reset(void)  //rst 0 ... 1 ...
{
  LCD_RST_CLR;
  delay_ms(150);
  LCD_RST_SET;
  delay_ms(50);
}

/****************************************************************************
* 名    称: LCD_clear()
* 功    能: 以指定颜色清屏
* 入口参数: color —清屏颜色
* 出口参数: 无
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
* 名    称: LCD_set_window()
* 功    能: 设置显示窗口
* 入口参数: X - 显示窗口左上角横坐标      Y —显示窗口左上角纵坐标
*           Width —显示窗口宽度          Height —显示窗口高度
* 出口参数: 无
****************************************************************************/
void LCD_set_window(unsigned int X    , unsigned int Y, 
                           unsigned int Width, unsigned int Height)
{
  LCD_write_byte(0x2A,LCD_COMMAND);         //列地址设置命令
  LCD_write_byte(X>>8,LCD_DATA) ;           //设置显示窗口左上角横坐标高位
  LCD_write_byte((X+2)&0xFF,LCD_DATA);      //设置显示窗口左上角横坐标低位 	
  LCD_write_byte((X+Width+1)>>8,LCD_DATA);  //设置宽度高位 	
  LCD_write_byte((X+Width+1)&0xFF,LCD_DATA);// 设置宽度低位 	
  
  LCD_write_byte(0x2B,LCD_COMMAND);         //行地址设置命令
  LCD_write_byte(Y>>8,LCD_DATA) ;           //设置显示窗口左上角纵坐标高位
  LCD_write_byte((Y+3)&0xFF,LCD_DATA);      //设置显示窗口左上角纵坐标低位 
  LCD_write_byte((Y+Height+2)>>8,LCD_DATA); // 设置显示窗口高度高位 	
  LCD_write_byte((Y+Height+2)&0xFF,LCD_DATA);// 设置显示窗口高度低位 	  
  LCD_write_byte(0x2C,LCD_COMMAND) ;
}

/****************************************************************************
* 名    称: DrawPixel()
* 功    能: 在指定位置以指定颜色显示像素
****************************************************************************/
void DrawPixel(unsigned int x, unsigned int y, unsigned int color)
{
  //LCD_set_XY(x,y);  //设定显示位置 andy
  LCD_write_byte(color>>8,LCD_DATA);  //发送显示颜色数据高位字节
  LCD_write_byte(color&0xFF,LCD_DATA);//发送显示颜色数据低位字节
}

/****************************************************************************
* 名    称: DrawVerticalLine()
* 功    能: 在指定位置以指定颜色画垂直线
* 入口参数: X - 垂直线起始位置横坐标      Y —垂直线起始位置纵坐标
*           Length —垂直线长度           Color —垂直线颜色
* 出口参数: 无
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
* 名    称: DrawHorizonLine()
* 功    能: 在指定位置以指定颜色画水平线
* 入口参数: X - 水平线起始位置横坐标      Y —水平线起始位置纵坐标
*           Length —水平线长度           Color —水平线颜色
* 出口参数: 无
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
* 名    称: DrawRect()
* 功    能: 在指定位置以指定颜色画矩形
* 入口参数: X - 矩形起始位置横坐标      Y —矩形起始位置纵坐标
*           Width  —矩形宽度           Height —矩形高度                   
*           Color  —矩形颜色             
* 出口参数: 无
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
* 名    称: DrawRectFill()
* 功    能: 在指定位置以指定颜色画矩形
* 入口参数: X - 矩形起始位置横坐标      Y —矩形起始位置纵坐标
*           Width  —矩形宽度           Height —矩形高度                   
*           Color  —矩形填充颜色             
* 出口参数: 无
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
   
  LCD_write_byte(0x11,LCD_COMMAND); //退出睡眠模式
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

  LCD_write_byte(0xB4,LCD_COMMAND);//列对调
  LCD_write_byte(0x07,LCD_DATA); 
  //ST7735R加电序列
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

  LCD_write_byte(0xC5,LCD_COMMAND); //加VCOM 
  LCD_write_byte(0x0E,LCD_DATA); 

  LCD_write_byte(0x36,LCD_COMMAND); //RGB模式 
  LCD_write_byte(0xC8,LCD_DATA); 
  
  // ST7735R Gamma 校正序列
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

  LCD_write_byte(0xF0,LCD_COMMAND); //使能测试命令  
  LCD_write_byte(0x01,LCD_DATA); 
  LCD_write_byte(0xF6,LCD_COMMAND); //关闭ram省电模式 
  LCD_write_byte(0x00,LCD_DATA); 

  LCD_write_byte(0x3A,LCD_COMMAND); //色深16bit 
  LCD_write_byte(0x05,LCD_DATA); 
  
  LCD_write_byte(0x29,LCD_COMMAND); //开显示  
  LCD_clear(WINDOW_BK_COLOR);       //以背景色清屏 
}

/****************************************************************************
* 名    称: LCD_write_EN()
* 功    能: 在指定位置显示 7 ×14 点阵ASCII字符 
* 入口参数: X - 显示位置左上角横坐标      Y —显示位置左上角纵坐标
*           c —显示字符                  
* 出口参数: 无
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
    temp = ASCII7x14[c-32][i];   //十进制代表 空格 字码表第一个
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
* 名    称: LCD_write_EN_string()
* 功    能: 在指定位置显示 7 ×14 点阵ASCII字符串 
* 入口参数: X - 显示位置左上角横坐标      Y —显示位置左上角纵坐标
*           s —字符串指针                  
* 出口参数: 无
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
* 名    称: LCD_write_CN()
* 功    能: 在指定位置显示 14 ×14 点阵汉字字符  
* 入口参数: X - 显示位置左上角横坐标      Y —显示位置左上角纵坐标
*           c —显示字符指针                  
* 出口参数: 无
****************************************************************************/
static void LCD_write_CN(unsigned int X, unsigned int Y, unsigned char c,unsigned int Color,unsigned int Color_BK)
{
  unsigned char i,j,temp;
  
  LCD_set_window(X,Y,14,14);  //按字符点阵大小设置显示窗口
  //LCD_CS_CLR;               //液晶SPI使能andy
  LCD_DC_SET;                 //发送数据信号使能
  for(i=0;i<28;i++)           // 14x14字符点阵数据大小为2x14=28字节
  {
    temp = CN14x14[c][i] ;    //读字模数组
    for(j=0;j<8;j++)          // 按位显示
    {
      if(temp&0x80)           //如果位值为1，显示字符色
      {
        SendByte(Color>>8);  
        SendByte(Color) ;
      }
      else                    //如果位值为0，显示背景色
      {
        SendByte(Color_BK>>8) ;  
        SendByte(Color_BK);
      }            
      temp <<= 1 ;
    }
    i++ ;                     //指向下一字节
    temp = CN14x14[c][i];
    for(j=0;j<6;j++)          // 按位显示，点阵横向为14像素，
    {                         // 所以每行第2字节最后2位抛弃
      if(temp&0x80)           // 如果位值为1，显示字符色
      {
        SendByte(Color>>8);  
        SendByte(Color) ;
      }
      else                    // 如果位值为0，显示背景色
      {
        SendByte(Color_BK>>8) ;  
        SendByte(Color_BK);
      }            
      temp <<= 1 ;
    }
  }
  //LCD_CS_SET;               //液晶SPI禁止
  LCD_DC_SET;                 //数据控制信号禁止
}

/****************************************************************************
* 名    称: LCD_write_CN_string()
* 功    能: 在指定位置显示 14 ×14 点阵中文字符串   
* 入口参数: X - 显示位置左上角横坐标      Y —显示位置左上角纵坐标
*           s —字符串指针                  
* 出口参数: 无
****************************************************************************/
void LCD_write_CN_string(unsigned char X,unsigned char Y,uint8 *s,unsigned int Color,unsigned int Color_BK)
{
  unsigned char i=0;
  static unsigned char j=0;
  while (*s)
  {
    for(j=0;j<ALL_CNS;j++)                  //搜索点阵字模
    { 
      if(  *s    ==CN14x14_Index[j*2]            
         &&*(s+1)==CN14x14_Index[j*2+1])    //汉字内码存在于字模索引数组中
      {
        LCD_write_CN(X+i*14,Y,j, Color, Color_BK) ;          //显示汉字
        break; 
      }
    }
    if(j>=ALL_CNS)                          //无此字的点阵字模
    {
      DrawRectFill(X+i*14,Y,14,14,Color_BK);//显示空格
    }
    s++; s++ ;                              //字符指针移位
    i++;
    if(i>=HAL_LCD_MAX_CHARS/2)  
      return;                               //超过最大显示字符数，返回
  }
  return ;
}

/****************************************************************************
* 名    称: ShowImage()
* 功    能: 显示图片   
* 入口参数: p - 图片点阵数据  取模方式 水平扫描 从左到右 低位在前           
* 出口参数: 无
****************************************************************************/
void ShowImage(const unsigned char *p) 
{
  int i; 
  unsigned char picH,picL;
  //LCD_clear(White); //清屏
  
  LCD_set_window(24,5,80,40); //坐标设置
  for(i=0;i<80*40;i++)
  {	
    picL=*(p+i*2);	//数据低位在前
    picH=*(p+i*2+1);				
    LCD_WriteData_16Bit(picH<<8|picL);  						
  }	
}