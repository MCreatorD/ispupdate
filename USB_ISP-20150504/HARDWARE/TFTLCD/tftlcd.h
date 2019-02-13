#ifndef __LDC_H_
#define __LDC_H_

typedef unsigned char uint8  ;
typedef unsigned int  uint16 ;

// SPI TFT LCD�˿ڶ���
#define LCD_DC                 GPIO_Pin_13  // ����/����ѡ��, L_R
#define LCD_RST                GPIO_Pin_0  // ����lcd��λ 
#define LCD_SDA                GPIO_Pin_2  // ˫������, L_D  P1_6   P1_3
#define LCD_SCL                GPIO_Pin_3  // ʱ��, L_C P1_2  P1_5
//#define LCD_CS                    // Ƭѡ, L_S ��Ƭѡ�ӵ�
#define LCD_PORT				GPIOC


//Һ�����ƿ���1�������궨��
//#define LCD_CS_SET  	(LCD_CS_CLRx01)   
#define	LCD_DC_SET  	GPIO_SetBits(LCD_PORT,LCD_DC)
#define	LCD_SDA_SET  	GPIO_SetBits(LCD_PORT,LCD_SDA)
#define	LCD_SCL_SET  	GPIO_SetBits(LCD_PORT,LCD_SCL)
#define	LCD_RST_SET  	GPIO_SetBits(LCD_PORT,LCD_RST)

//Һ�����ƿ���0�������궨��
//#define LCD_CS_CLR  	(LCD_CS_CLRx00)    
#define	LCD_DC_CLR  	GPIO_ResetBits(LCD_PORT,LCD_DC)
#define	LCD_SDA_CLR  	GPIO_ResetBits(LCD_PORT,LCD_SDA)   
#define	LCD_SCL_CLR  	GPIO_ResetBits(LCD_PORT,LCD_SCL)
#define	LCD_RST_CLR  	GPIO_ResetBits(LCD_PORT,LCD_RST)


#define LCD_COMMAND 0x00
#define LCD_DATA    0x01

#define LCD_MAX_BUF 25

#define SOFTWARE_SPI //ʹ��������SPI

#define COL_SPACE   2   //�ַ����
#define ROW_SPACE   4   //�м��
#define ALL_CNS     128


//LCD ��ɫ
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


//��ʾ������ɫ
#define WINDOW_BK_COLOR 0xDFFF //���ڱ���ɫ
#define WINDOW_COLOR    0x11FA //����ǰ��ɫ
#define TITLE_BK_COLOR  0x11FA //����������ɫ
#define TITLE_COLOR     0xDFFF //������ǰ��ɫ
#define STATUS_BK_COLOR 0x0014 //״̬������ɫ
#define STATUS_COLOR    0xDFFF //״̬��ǰ��ɫ

//Ӳ��SPIģʽʱ���ݷ��ͺ�
/*#define SendByte(Data) {  U1DBUF = Data;}  */

#define HAL_LCD_MAX_CHARS   18    // ÿ������ַ���
#define HAL_LCD_MAX_BUFF    25
#define X_MAX_PIXEL	    128
#define Y_MAX_PIXEL	    128

extern unsigned int  Color;      // ǰ����ɫ
extern unsigned int  Color_BK;   // ������ɫ

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