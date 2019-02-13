#ifndef __LDC_H_
#define __LDC_H_

typedef unsigned char uint8  ;
typedef unsigned int  uint16 ;

// SPI TFT LCD端口定义
#define LCD_DC                 GPIO_Pin_13  // 数据/命令选择, L_R
#define LCD_RST                GPIO_Pin_0  // 控制lcd复位 
#define LCD_SDA                GPIO_Pin_2  // 双向数据, L_D  P1_6   P1_3
#define LCD_SCL                GPIO_Pin_3  // 时钟, L_C P1_2  P1_5
//#define LCD_CS                    // 片选, L_S 将片选接地
#define LCD_PORT				GPIOC


//液晶控制口置1操作语句宏定义
//#define LCD_CS_SET  	(LCD_CS_CLRx01)   
#define	LCD_DC_SET  	GPIO_SetBits(LCD_PORT,LCD_DC)
#define	LCD_SDA_SET  	GPIO_SetBits(LCD_PORT,LCD_SDA)
#define	LCD_SCL_SET  	GPIO_SetBits(LCD_PORT,LCD_SCL)
#define	LCD_RST_SET  	GPIO_SetBits(LCD_PORT,LCD_RST)

//液晶控制口置0操作语句宏定义
//#define LCD_CS_CLR  	(LCD_CS_CLRx00)    
#define	LCD_DC_CLR  	GPIO_ResetBits(LCD_PORT,LCD_DC)
#define	LCD_SDA_CLR  	GPIO_ResetBits(LCD_PORT,LCD_SDA)   
#define	LCD_SCL_CLR  	GPIO_ResetBits(LCD_PORT,LCD_SCL)
#define	LCD_RST_CLR  	GPIO_ResetBits(LCD_PORT,LCD_RST)


#define LCD_COMMAND 0x00
#define LCD_DATA    0x01

#define LCD_MAX_BUF 25

#define SOFTWARE_SPI //使用软件软件SPI

#define COL_SPACE   2   //字符间距
#define ROW_SPACE   4   //行间距
#define ALL_CNS     128


//LCD 颜色
#define  WHITE          0xFFFF
#define  BLACK          0x0000
#define  GREY           0xF7DE
#define  GREY2          0xF79E
#define  DARK_GREY      0x6B4D
#define  DARK_GREY2     0x52AA
#define  LIGHT_GREY     0xE71C
#define  BLUE           0x001F
#define  BLUE2          0x051F
#define  RED            0xF800
#define  MAGENTA        0xF81F
#define  GREEN          0x07E0
#define  CYAN           0x7FFF
#define  YELLOW         0xFFE0


//显示界面颜色
#define WINDOW_BK_COLOR 0xDFFF //窗口背景色
#define WINDOW_COLOR    0x11FA //窗口前景色
#define TITLE_BK_COLOR  0x11FA //标题栏背景色
#define TITLE_COLOR     0xDFFF //标题栏前景色
#define STATUS_BK_COLOR 0x0014 //状态栏背景色
#define STATUS_COLOR    0xDFFF //状态栏前景色

//硬件SPI模式时数据发送宏
/*#define SendByte(Data) {  U1DBUF = Data;}  */

#define HAL_LCD_MAX_CHARS   18    // 每行最大字符数
#define HAL_LCD_MAX_BUFF    25
#define X_MAX_PIXEL	    128
#define Y_MAX_PIXEL	    128

extern unsigned int  Color;      // 前景颜色
extern unsigned int  Color_BK;   // 背景颜色

void  SendByte(unsigned char Data);

void  SPI_WriteData(unsigned char Data);

void Lcd_WriteIndex(unsigned char Index);

void Lcd_WriteData(unsigned char Data);

void LCD_WriteData_16Bit(unsigned short Data);

void Lcd_Reset(void);

void LCD_clear(unsigned int color);

void InitLcd(void);

void LCD_set_window(unsigned int X    , unsigned int Y, 
                           unsigned int Width, unsigned int Height);

void DrawPixel(unsigned int x, unsigned int y, unsigned int color);

void DrawVerticalLine(unsigned int x, unsigned int y, 
                      unsigned int Length,unsigned int Color) ;

void DrawHorizonLine(unsigned int x, unsigned int y, 
                     unsigned int Length,unsigned int Color);

void DrawRect(unsigned int Xpos, unsigned int Ypos, unsigned int Width, 
              unsigned int Height,unsigned Color);

void DrawRectFill(unsigned int Xpos  , unsigned int Ypos, unsigned int Width, 
                  unsigned int Height, unsigned Color                        );
void LCD_write_EN(unsigned int X, unsigned int Y, unsigned char c,unsigned int Color,unsigned int Color_BK);
void LCD_write_EN_string(unsigned char X,unsigned char Y,uint8 *s,unsigned int Color,unsigned int Color_BK);
//void LCD_write_EN_string(unsigned char X,unsigned char Y,uint8 *s);
void LCD_write_CN_string(unsigned char X,unsigned char Y,uint8 *s,unsigned int Color,unsigned int Color_BK);
//void LCD_write_CN_string(unsigned char X,unsigned char Y,uint8 *s);

void ShowImage(const unsigned char *p);

#endif