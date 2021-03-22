/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include "w25qxx.h"
#include "stm32_adafruit_sd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
void retarget_put_char(char p)
{
	//UartReadyT = RESET;
	HAL_UART_Transmit(&huart3, &p, 1,0xffff); // send message via UART
	//while(UartReadyT != SET)
	//{

	//}
}

int _write(int fd, char* ptr, int len)
{
    (void)fd;
    int i = 0;
    while (ptr[i] && (i < len))
    {
    	if (ptr[i] == '\r')
    	{

    	}
    	else
    	{
    		retarget_put_char((int)ptr[i]);
			if (ptr[i] == '\n')
			{
				retarget_put_char((int)'\r');
			}
    	}
        i++;
    }
    return len;
}
uint8_t      SD_BUFF[512];
#define SD_TIMEOUT 30 * 1000


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SW 64
#define SH 64
uint8_t matrix[SH][SW][3];
uint16_t matrix_comp[SH][SW];

/*
 * PIN IN
 * R1  G1
 * B1  GRN
 * R2  G2
 * B2  GND
 * A   B
 * C   D
 * CLK LAT
 * OE  GND
 */

volatile int ss = 0;
#define SDEL(N) {for(int k=0;k<N;k++) ss++;}
void tick(int p)
{
	SDEL(1);
	HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,p?GPIO_PIN_SET:GPIO_PIN_RESET);
	SDEL(1);
}
void latch(int p)
{
	SDEL(1);
	HAL_GPIO_WritePin(LATCH_GPIO_Port,LATCH_Pin,p?GPIO_PIN_SET:GPIO_PIN_RESET);
	SDEL(1);
}
void blank(int p)
{
	SDEL(1);
	HAL_GPIO_WritePin(OE_BLANK_GPIO_Port,OE_BLANK_Pin,p?GPIO_PIN_SET:GPIO_PIN_RESET);
	SDEL(1);
}


void clear()
{
	for(int y=0;y<SH;y++)
	{
		for(int x=0;x<SW;x++)
		{
			matrix[y][x][0] = 0;
			matrix[y][x][1] = 0;
			matrix[y][x][2] = 0;
		}
	}
}
uint16_t LCD_getWidth() {
    return SW;
}

uint16_t LCD_getHeight() {
    return SH;
}
#define MAX(X,Y) ((X>Y)?X:Y)
#define MIN(X,Y) ((X<Y)?X:Y)
inline int clamp(int val,int minval,int maxval)
{
	return MAX(MIN(maxval,val),minval);
}
uint16_t makeColor(int R,int G,int B) //565
{
	return ((R>>3)<<11)|((G>>2)<<5)|(B>>3);
}
#define BLACK           0x0000      /*   0,   0,   0 */
#define NAVY            0x000F      /*   0,   0, 128 */
#define DGREEN          0x03E0      /*   0, 128,   0 */
#define DCYAN           0x03EF      /*   0, 128, 128 */
#define MAROON          0x7800      /* 128,   0,   0 */
#define PURPLE          0x780F      /* 128,   0, 128 */
#define OLIVE           0x7BE0      /* 128, 128,   0 */
#define LGRAY           0xC618      /* 192, 192, 192 */
#define DGRAY           0x7BEF      /* 128, 128, 128 */
#define BLUE            0x001F      /*   0,   0, 255 */
#define GREEN           0x07E0      /*   0, 255,   0 */
#define CYAN            0x07FF      /*   0, 255, 255 */
#define RED             0xF800      /* 255,   0,   0 */
#define MAGENTA         0xF81F      /* 255,   0, 255 */
#define YELLOW          0xFFE0      /* 255, 255,   0 */
#define WHITE           0xFFFF      /* 255, 255, 255 */
#define ORANGE          0xFD20      /* 255, 165,   0 */
//#define BGROUND   makeColor(16,0,16)
#define BGROUND   makeColor(0,0,0)
uint16_t colorTableA[] =
{
		NAVY,
		YELLOW,
		WHITE,
		MAGENTA,
		RED,
		CYAN,
		GREEN,
		BLUE,
		OLIVE,
		PURPLE,
		MAROON,
		NAVY,
		DGREEN
};

void splitColor(uint16_t COLOR,uint8_t* RGB,int BR)
{
	RGB[0] = (((COLOR>>11)&0x1f)<<3)>>BR;
	RGB[1] = (((COLOR>>5)&0x3f)<<2)>>BR;
	RGB[2] = (((COLOR)&0x1f)<<3)>>BR;
}
inline void LCD_fillRect(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h, uint16_t color)
{
	x1 = clamp(x1,0,SW);
	int x2 = clamp(x1+w,0,SW);
	y1 = clamp(y1,0,SH);
	int y2 = clamp(y1+h,0,SH);
	//printf("%d,%d %d,%d\n",x1,y1,x2,y2);
	for(int y = y1; y<y2;y++)
	{
		for(int x = x1; x<x2;x++)
		{
			matrix_comp[y][x] = color;
		}
	}
}
inline void LCD_fillRectP(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h, uint16_t *color)
{
	x1 = clamp(x1,0,SW);
	int x2 = clamp(x1+w,0,SW);
	y1 = clamp(y1,0,SH);
	int y2 = clamp(y1+h,0,SH);
	//printf("%d,%d %d,%d\n",x1,y1,x2,y2);
	for(int y = y1; y<y2;y++)
	{
		for(int x = x1; x<x2;x++)
		{
			matrix_comp[y][x] = *color++;
		}
	}
}
inline void LCD_DrawHLine(uint16_t x1,uint16_t y1,uint16_t length,uint16_t color)
{
	LCD_fillRect(x1,y1,length,1,color);
}
inline void LCD_DrawVLine(uint16_t x1,uint16_t y1,uint16_t length,uint16_t color)
{
	LCD_fillRect(x1,y1,1,length,color);
}
inline void LCD_DrawPixel(uint16_t x1, uint16_t y1,uint16_t color)
{
	LCD_fillRect(x1, y1, 1, 1,color);
}
#define ABS(X)  ((X) > 0 ? (X) : -(X))
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
  curpixel = 0;

  deltax = ABS(x2 - x1);        /* The difference between the x's */
  deltay = ABS(y2 - y1);        /* The difference between the y's */
  x = x1;                       /* Start x off at the first pixel */
  y = y1;                       /* Start y off at the first pixel */

  if (x2 >= x1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (y2 >= y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         /* There are more y-values than x-values */
  }

  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    LCD_DrawPixel(x, y , color);  /* Draw the current pixel */
    num += numadd;                            /* Increase the numerator by the top of the fraction */
    if (num >= den)                           /* Check if numerator >= denominator */
    {
      num -= den;                             /* Calculate the new numerator value */
      x += xinc1;                             /* Change the x as appropriate */
      y += yinc1;                             /* Change the y as appropriate */
    }
    x += xinc2;                               /* Change the x as appropriate */
    y += yinc2;                               /* Change the y as appropriate */
  }
}

void LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius,uint16_t color)
{
   if(Radius==0)
   {
	   LCD_DrawPixel(Xpos,Ypos,color);
	   return;
   }

  int32_t  D;       /* Decision Variable */
  uint32_t  CurX;   /* Current X Value */
  uint32_t  CurY;   /* Current Y Value */

  D = 3 - (Radius << 1);
  CurX = 0;
  CurY = Radius;

  while (CurX <= CurY)
  {
    LCD_DrawPixel((Xpos + CurX), (Ypos - CurY), color);

    LCD_DrawPixel((Xpos - CurX), (Ypos - CurY), color);

    LCD_DrawPixel((Xpos + CurY), (Ypos - CurX), color);

    LCD_DrawPixel((Xpos - CurY), (Ypos - CurX), color);

    LCD_DrawPixel((Xpos + CurX), (Ypos + CurY), color);

    LCD_DrawPixel((Xpos - CurX), (Ypos + CurY), color);

    LCD_DrawPixel((Xpos + CurY), (Ypos + CurX), color);

    LCD_DrawPixel((Xpos - CurY), (Ypos + CurX), color);


    if (D < 0)
    {
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
}
void LCD_DrawCircleFull(uint16_t Xpos, uint16_t Ypos, uint16_t Radius,uint16_t color)
{
   if(Radius==0)
   {
	   LCD_DrawPixel(Xpos,Ypos,color);
	   return;
   }

  int32_t  D;       /* Decision Variable */
  uint32_t  CurX;   /* Current X Value */
  uint32_t  CurY;   /* Current Y Value */

  D = 3 - (Radius << 1);
  CurX = 0;
  CurY = Radius;

  while (CurX <= CurY)
  {
//    LCD_DrawPixel((Xpos + CurX), (Ypos - CurY), color);
//    LCD_DrawPixel((Xpos - CurX), (Ypos - CurY), color);
    LCD_DrawHLine(Xpos - CurX,Ypos - CurY,CurX*2,color);

//    LCD_DrawPixel((Xpos + CurY), (Ypos - CurX), color);
//    LCD_DrawPixel((Xpos - CurY), (Ypos - CurX), color);
    LCD_DrawHLine(Xpos - CurY,Ypos - CurX,CurY*2,color);


//    LCD_DrawPixel((Xpos + CurX), (Ypos + CurY), color);
//    LCD_DrawPixel((Xpos - CurX), (Ypos + CurY), color);
    LCD_DrawHLine(Xpos - CurX,Ypos + CurY,CurX*2,color);


//    LCD_DrawPixel((Xpos + CurY), (Ypos + CurX), color);
//    LCD_DrawPixel((Xpos - CurY), (Ypos + CurX), color);
    LCD_DrawHLine(Xpos - CurY,Ypos + CurX,CurY*2,color);


    if (D < 0)
    {
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
}

void LCD_DrawEllipse(int Xpos, int Ypos, int XRadius, int YRadius,uint16_t color)
{
  int x = 0, y = -YRadius, err = 2-2*XRadius, e2;
  float K = 0, rad1 = 0, rad2 = 0;

  rad1 = XRadius;
  rad2 = YRadius;

  K = (float)(rad2/rad1);

  do {
	  uint16_t dt = (uint16_t)(x/K);
    LCD_DrawPixel(Xpos-dt, (Ypos+y), color);
    LCD_DrawPixel(Xpos+dt, (Ypos+y), color);
    LCD_DrawPixel(Xpos+dt, (Ypos-y), color);
    LCD_DrawPixel(Xpos-dt, (Ypos-y), color);

    e2 = err;
    if (e2 <= x) {
      err += ++x*2+1;
      if (-y == x && e2 <= y) e2 = 0;
    }
    if (e2 > y) err += ++y*2+1;
  }
  while (y <= 0);
}
void LCD_DrawEllipseFull(int Xpos, int Ypos, int XRadius, int YRadius,uint16_t color)
{
  int x = 0, y = -YRadius, err = 2-2*XRadius, e2;
  float K = 0, rad1 = 0, rad2 = 0;

  rad1 = XRadius;
  rad2 = YRadius;

  K = (float)(rad2/rad1);

  do {
	  uint16_t dt = (uint16_t)(x/K);
	  LCD_DrawHLine(Xpos-dt, (Ypos+y),dt*2, color);
	  LCD_DrawHLine(Xpos-dt, (Ypos-y),dt*2, color);

    e2 = err;
    if (e2 <= x) {
      err += ++x*2+1;
      if (-y == x && e2 <= y) e2 = 0;
    }
    if (e2 > y) err += ++y*2+1;
  }
  while (y <= 0);
}


#include "fonts.h"
sFONT* FONT_CURR =  &Font12;
#define CHAR_WIDTH (FONT_CURR->Width)
#define CHAR_HEIGHT (FONT_CURR->Height)


void LCD_Draw_Char2(char Character, int16_t X, int16_t Y, uint16_t Colour, uint16_t SizeX,uint16_t SizeY, uint16_t Background_Colour)
{
	//~ flagReinit = 1;
	uint8_t 	function_char;
    int16_t 	i,j;

	function_char = Character;

	// Draw pixels
	LCD_fillRect(X, Y, CHAR_WIDTH*SizeX, CHAR_HEIGHT*SizeY, Background_Colour);

    if (function_char <= 0x20)
	{

	}
	else
	{
		function_char -= 0x20;
		int rw = (CHAR_WIDTH+7)/8;
		for (j=0; j<FONT_CURR->Height; j++)
		{
			uint8_t* pnt =  FONT_CURR->table+j*rw+function_char*FONT_CURR->Height*rw;
			for (i=0; i<FONT_CURR->Width; i++)
			{
				uint8_t bt = pnt[i/8];
				if(bt&(1<<(7-(i&7))))
				{
					LCD_fillRect(X+(i*SizeX), Y+(j*SizeY), SizeX,SizeY, Colour);
				}
			}
		}
	}
}


void ClrScr()
{
	LCD_fillRect(0,0,LCD_getWidth(),LCD_getHeight(),BGROUND);
}
void convertColors()
{
	for(int y=0;y<SH;y++)
	{
		for(int x=0;x<SW;x++)
		{
			splitColor(matrix_comp[y][x],&matrix[y][x][0],4);
		}
	}
}
#define HAL_GPIO_WritePinAI(X,Y,Z)  (X)->BSRR= (Y)<<(Z)
/*
volatile uint32_t timeCount = 0;
void rDelayMks(int mks)
{
	volatile uint32_t timeCount_stop = timeCount+mks/10;
	while((timeCount_stop-timeCount)<0x800000)
	{
		//nothing
	}

}
*/
volatile static y_count2  = 0;
volatile static y_cnt     = 0;

void shiftData(int ON_OFF,int y_base)
{
	//for(int y=0;y<SH/2;y++)
	if(y_base&1)
	{
		blank(1);
		return;
	}
	else
	{
		int y = y_base>>1;
			latch(1);
				tick(0);
				tick(1);
				tick(0);
				tick(1);
				tick(0);
				tick(1);
				tick(0);
			latch(0);

			for(int i=0;i<SW;i++)
			{
				//tick(0);
				HAL_GPIO_WritePinAI(CLK_GPIO_Port,CLK_Pin,16);
				HAL_GPIO_WritePinAI(CLK_GPIO_Port,CLK_Pin,16);

				HAL_GPIO_WritePinAI(R1_GPIO_Port,R1_Pin,((matrix[y][i][0]>ON_OFF)?0:16));
				HAL_GPIO_WritePinAI(G1_GPIO_Port,G1_Pin,((matrix[y][i][1]>ON_OFF)?0:16));
				HAL_GPIO_WritePinAI(B1_GPIO_Port,B1_Pin,((matrix[y][i][2]>ON_OFF)?0:16));

				HAL_GPIO_WritePinAI(R2_GPIO_Port,R2_Pin,((matrix[y+SH/4][i][0]>ON_OFF)?0:16));
				HAL_GPIO_WritePinAI(G2_GPIO_Port,G2_Pin,((matrix[y+SH/4][i][1]>ON_OFF)?0:16));
				HAL_GPIO_WritePinAI(B2_GPIO_Port,B2_Pin,((matrix[y+SH/4][i][2]>ON_OFF)?0:16));

				HAL_GPIO_WritePinAI(SR1_GPIO_Port,SR1_Pin,((matrix[y+2*SH/4][i][0]>ON_OFF)?0:16));
				HAL_GPIO_WritePinAI(SG1_GPIO_Port,SG1_Pin,((matrix[y+2*SH/4][i][1]>ON_OFF)?0:16));
				HAL_GPIO_WritePinAI(SB1_GPIO_Port,SB1_Pin,((matrix[y+2*SH/4][i][2]>ON_OFF)?0:16));

				HAL_GPIO_WritePinAI(SR2_GPIO_Port,SR2_Pin,((matrix[y+3*SH/4][i][0]>ON_OFF)?0:16));
				HAL_GPIO_WritePinAI(SG2_GPIO_Port,SG2_Pin,((matrix[y+3*SH/4][i][1]>ON_OFF)?0:16));
				HAL_GPIO_WritePinAI(SB2_GPIO_Port,SB2_Pin,((matrix[y+3*SH/4][i][2]>ON_OFF)?0:16));

				//tick(0);
				//HAL_GPIO_WritePinAI(CLK_GPIO_Port,CLK_Pin,16);
				//HAL_GPIO_WritePinAI(CLK_GPIO_Port,CLK_Pin,16);
				HAL_GPIO_WritePinAI(CLK_GPIO_Port,CLK_Pin,16);
				HAL_GPIO_WritePinAI(CLK_GPIO_Port,CLK_Pin,16);
			//	HAL_GPIO_WritePinAI(CLK_GPIO_Port,CLK_Pin,16);
			//	HAL_GPIO_WritePinAI(CLK_GPIO_Port,CLK_Pin,16);
				//tick(1);
				//HAL_GPIO_WritePinAI(CLK_GPIO_Port,CLK_Pin,0);
				HAL_GPIO_WritePinAI(CLK_GPIO_Port,CLK_Pin,0);
				HAL_GPIO_WritePinAI(CLK_GPIO_Port,CLK_Pin,0);
			}
			tick(0);
			latch(1);
			latch(0);
		HAL_GPIO_WritePin(LINE_A_GPIO_Port,LINE_A_Pin,((y)&(1))?GPIO_PIN_SET:GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LINE_B_GPIO_Port,LINE_B_Pin,((y)&(2))?GPIO_PIN_SET:GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LINE_C_GPIO_Port,LINE_C_Pin,((y)&(4))?GPIO_PIN_SET:GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LINE_D_GPIO_Port,LINE_D_Pin,((y)&(8))?GPIO_PIN_SET:GPIO_PIN_RESET);
		blank(0);
		//rDelayMks(10);
		//rDelayMks(10);
	}
}

void HAL_TIM_PeriodElapsedCallback1(TIM_HandleTypeDef *htim)
{
	shiftData(y_cnt , y_count2);
	y_count2++;
	if(y_count2>=SH/2)
	{
		y_count2 = 0;
		y_cnt = (y_cnt+1)%16;
	}
}
void  LCD_Draw_Text2(const char* Text, int16_t X, int16_t Y, uint16_t Colour, uint16_t SizeX, uint16_t SizeY,uint16_t Background_Colour)
{
    while (*Text) {
        LCD_Draw_Char2(*Text, X, Y, Colour, SizeX,SizeY, Background_Colour);
        X += CHAR_WIDTH*SizeX;
	Text++;
    }
}



char* MontTable[] = {"   ","Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
char* VeryLongText = "          Privet Jenechka. Pogoda segodnya xoroshaya .Poidem gulat' skoree!!!        ";
void dTest()
{
	  {
			uint32_t trt = HAL_GetTick();
			int k;
			//setBK_imp(99);

			for(k=0;k<1000;k++)
			{
				int IW = rand()%(LCD_getWidth()/2);
				int IH = rand()%(LCD_getHeight()/2);
				int xPos =rand()%(LCD_getWidth()-IW);
				int yPos =rand()%(LCD_getHeight()-IH);

				LCD_fillRect(xPos, yPos, IW, IH,rand());
				convertColors();
				//HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port,BOARD_LED_Pin);
			}
			trt = (HAL_GetTick()-trt);
			printf("Rect Full %d ms\n",trt);
			HAL_Delay(1000);
			LCD_fillRect(0, 0, LCD_getWidth(), LCD_getHeight(), BLACK);
	  }
	  {
			uint32_t trt = HAL_GetTick();
			int k;
			//setBK_imp(99);

			for(k=0;k<1000;k++)
			{
				int IW = rand()%(LCD_getWidth()/2);
				int IH = rand()%(LCD_getHeight()/2);
				int xPos =rand()%(LCD_getWidth()-IW);
				int yPos =rand()%(LCD_getHeight()-IH);

				LCD_fillRect(xPos, yPos, IW, IH,rand());
				convertColors();
				//HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port,BOARD_LED_Pin);
			}
			trt = (HAL_GetTick()-trt);
			printf("Rect %d ms\n",trt);
			HAL_Delay(1000);
			LCD_fillRect(0, 0, LCD_getWidth(), LCD_getHeight(), BLACK);
	  }
	  {
			uint32_t trt = HAL_GetTick();
			int k;
			//setBK_imp(99);

			for(k=0;k<1000;k++)
			{
				int xPos =rand()%(LCD_getWidth());
				int yPos =rand()%(LCD_getHeight());
				int xPos1 =rand()%(LCD_getWidth());
				int yPos1 =rand()%(LCD_getHeight());

				LCD_DrawLine(xPos, yPos,xPos1,yPos1,rand());
				convertColors();
				//HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port,BOARD_LED_Pin);
			}
			trt = (HAL_GetTick()-trt);
			printf("Lines %d ms\n",trt);
			HAL_Delay(1000);
			LCD_fillRect(0, 0, LCD_getWidth(), LCD_getHeight(), BLACK);
	  }
	  {
			uint32_t trt = HAL_GetTick();
			int k;
			//setBK_imp(99);

			for(k=0;k<1000;k++)
			{
				int R = (rand()%(LCD_getHeight()/4));
				int xPos =rand()%(LCD_getWidth()-R*2);
				int yPos =rand()%(LCD_getHeight()-R*2);

				LCD_DrawCircle(xPos+R, yPos+R, R ,rand());
				convertColors();
				//HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port,BOARD_LED_Pin);
			}
			trt = (HAL_GetTick()-trt);
			printf("Circle %d ms\n",trt);
			HAL_Delay(1000);
			LCD_fillRect(0, 0, LCD_getWidth(), LCD_getHeight(), BLACK);
	  }
	  {
			uint32_t trt = HAL_GetTick();
			int k;
			//setBK_imp(99);

			for(k=0;k<1000;k++)
			{
				int R = (rand()%(LCD_getHeight()/4));
				int xPos =rand()%(LCD_getWidth()-R*2);
				int yPos =rand()%(LCD_getHeight()-R*2);

				LCD_DrawCircleFull(xPos+R, yPos+R, R ,rand());
				convertColors();
				//HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port,BOARD_LED_Pin);
			}
			trt = (HAL_GetTick()-trt);
			printf("Circle Full %d ms\n",trt);
			HAL_Delay(1000);
			LCD_fillRect(0, 0, LCD_getWidth(), LCD_getHeight(), BLACK);
	  }
	  {
			uint32_t trt = HAL_GetTick();
			int k;
			//setBK_imp(99);

			for(k=0;k<1000;k++)
			{
				int R = (rand()%(LCD_getHeight()/4));
				int xPos =rand()%(LCD_getWidth()-R*2);
				int yPos =rand()%(LCD_getHeight()-R*2);

				LCD_DrawEllipse(xPos+R, yPos+R, R ,R+1,rand());
				convertColors();
				//HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port,BOARD_LED_Pin);
			}
			trt = (HAL_GetTick()-trt);
			printf("Ellipse %d ms\n",trt);
			HAL_Delay(1000);
			LCD_fillRect(0, 0, LCD_getWidth(), LCD_getHeight(), BLACK);
	  }
	  {
			uint32_t trt = HAL_GetTick();
			int k;
			//setBK_imp(99);

			for(k=0;k<1000;k++)
			{
				int R = (rand()%(LCD_getHeight()/4));
				int xPos =rand()%(LCD_getWidth()-R*2);
				int yPos =rand()%(LCD_getHeight()-R*2);

				LCD_DrawEllipseFull(xPos+R, yPos+R, R ,R+1,rand());
				convertColors();
				//HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port,BOARD_LED_Pin);
			}
			trt = (HAL_GetTick()-trt);
			printf("Ellipse Full %d ms\n",trt);
			HAL_Delay(1000);
			LCD_fillRect(0, 0, LCD_getWidth(), LCD_getHeight(), BLACK);
			convertColors();
	  }

}
typedef struct v2ff
{
	float x;
	float y;
}vec2f;

void rotate_mat(float angle,float x,float y,float * mat)
{
	float SIN = sin(angle);
	float COS = cos(angle);
	mat[0] = COS; mat[1]  = -SIN;  mat[2] =x;
	mat[3] = SIN;  mat[4]  =  COS; mat[5] =y;
}

vec2f  apply(float * mat,vec2f  vec)
{
	vec2f res;
	res.x =	vec.x*mat[0] +vec.y*mat[1] +mat[2] ;
	res.y =	vec.x*mat[3] +vec.y*mat[4] +mat[5] ;
	return res;
}

vec2f vec2f_cr (float x,float y)
{
	vec2f res;
	res.x = x;
	res.y = y;
	return res;
}

FIL      MyFile;
#define BSIZE 1024
uint8_t dbl_b[BSIZE];
int head=0;
int tail=0;
void int_h()
{
	head = 0;
	tail = 0;
}
int in_s()
{
	return (head-tail)&(BSIZE-1);
}
int flag_stop = 0;
inline uint8_t int_g()
{
	uint8_t res;
	if(!(in_s()&512))
	{
		size_t BytesReadfile = 0;
		f_read (&MyFile, &dbl_b[head], 512 , &BytesReadfile);
		head = (head + BytesReadfile)&(BSIZE-1);
		if(BytesReadfile==0)
			flag_stop = 1;
	}
	res = dbl_b[tail];
	tail= (tail+1)&(BSIZE-1);
	return res;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  printf("\n\n\nProgram start!\n");
  W25qxx_Init();
  //extern w25qxx_t	w25qxx;
  printf("FLASH ID = %x\n",w25qxx.ID);
  printf("FLASH BlockSize  = 0x%x\n",w25qxx.BlockSize);
  printf("FLASH BlockCount = 0x%x\n",w25qxx.BlockCount);
  printf("FLASH SectorSize  = 0x%x\n",w25qxx.SectorSize);
  printf("FLASH SectorCount = 0x%x\n",w25qxx.SectorCount);
  printf("FLASH PageSize  = 0x%x\n",w25qxx.PageSize);
  printf("FLASH PageCount = 0x%x\n",w25qxx.PageCount);
  printf("FLASH CapacityInKiloByte = %ldKB\n",w25qxx.CapacityInKiloByte);
  printf("\n");
  SD_CardInfo CardInfo;
  uint8_t res = BSP_SD_Init();
  //if(res==MSD_OK)
  {
  BSP_SD_GetCardInfo(&CardInfo);
  printf("hsd.CardCapacity =%d\n",CardInfo.CardCapacity);
  //printf("hsd.SdCard.BlockNbr=%d\n",hsd.SdCard.BlockNbr);
  //printf("hsd.SdCard.BlockSize=%d\n",hsd.SdCard.BlockSize);
  //printf("hsd.SdCard.CardType=%d\n",hsd.SdCard.CardType);
  //printf("hsd.SdCard.CardVersion=%d\n",hsd.SdCard.CardVersion);
  //printf("hsd.SdCard.Class=%d\n",hsd.SdCard.Class);
  printf("hsd.CardBlockSize =%d\n",CardInfo.CardBlockSize);
  printf("hsd.SdCard.LogBlockNbr=%d\n",CardInfo.LogBlockNbr);
  printf("hsd.SdCard.LogBlockSize=%d\n",CardInfo.LogBlockSize);
  uint64_t size_k = CardInfo.LogBlockNbr;
  size_k = size_k*CardInfo.LogBlockSize;

  HAL_TIM_Base_Start_IT(&htim1);

  printf("CardInfo size %d MB\n",(int)(size_k/1024/1024));
  {
	  int32_t tk = HAL_GetTick();
	  int stat = 0;
	  int kk;
	  for(kk=0;(kk<2*64)&&!stat;kk++)
	  {
  			stat = BSP_SD_ReadBlocks(&SD_BUFF[0],kk,1, SD_TIMEOUT);
	  }
	  tk = HAL_GetTick()-tk;
	  printf("Read %d blocks (bytes = %d) in %d ms st = %d\n",kk,kk*512,tk,stat);
	//  paint();
  }
}
  //dTest();
  f_mount(NULL,USERPath, 0);
 	if(f_mount(&USERFatFS,USERPath, 0) != FR_OK)
 	{
 		printf("f_mount Error!\r\n");
 		return 0;
 	}
	//disassemble_program();


  ClrScr();
  convertColors();
  LCD_fillRect(0,0,LCD_getWidth(),LCD_getHeight(),BGROUND);
  convertColors();
 // color_init();
  int_h();

  uint16_t *table;
  if(f_open(&MyFile, "image2u.dat", FA_READ) == FR_OK)
  //if(f_open(&MyFile, "image4u.dat", FA_READ) == FR_OK)
  //if(0)
 // if(f_open(&MyFile, "ima3u.dat", FA_READ) == FR_OK)
  {

	  int W;
	  int H;
	  int SSIZE;
	  size_t BytesReadfile=10 ;

	  f_read (&MyFile, &W, 4 , &BytesReadfile);
	  f_read (&MyFile, &H, 4 , &BytesReadfile);
	  f_read (&MyFile, &SSIZE, 4 , &BytesReadfile);
	  printf("W=%d H=%d size=%d\n",W,H,SSIZE);
	  table = malloc(SSIZE*2);
	  for(int k=0;k<SSIZE;k++)
	  {
		  uint8_t rb[2];
		  f_read (&MyFile, &rb[0], 2 , &BytesReadfile);
		  table[k] = rb[0]*256+rb[1];
		  //f_read (&MyFile, &table[k], 2 , &BytesReadfile);

	  }

	  uint8_t   bbuff[240];
	  uint16_t lbuff[240];
	  int frame = 0;
	  int err = 0;
	  while(!err)
	  {
		  uint32_t trt = HAL_GetTick();
		  frame++;
		  for(int y=0;y<H;y++)
		  {
			  //
			  f_read (&MyFile, bbuff, 240 , &BytesReadfile);
			  if(!BytesReadfile) err = 1;

			  for(int x=0;x<240;x++)
			  {
				  //lbuff[x]= ct[int_g()];
				  //lbuff[x]= ct[bbuff[x]];
				  lbuff[x/2]= table[bbuff[x]];

			  }
			  if((y<128)&&(y&1))
			  {
				  //LCD_sendLineRect((240-H)/2+y,(uint8_t*)lbuff);
				  LCD_fillRectP(0,y/2,64,1,&lbuff[120/2-64/2]);
			  }
		  }
		  convertColors();
		  if(flag_stop) err = 1;
		  trt = (HAL_GetTick()-trt);
		  if(trt<40)
		  {
			  HAL_Delay(40-trt);
		  }
		  //char buff[0x20];

		 // sprintf(buff,"frame in %02d ms\n",trt);
		 // while(hspi4.State!=HAL_SPI_STATE_READY){};
		//  LCD_Draw_Text2(buff,0,0,GREEN,2,3,BLACK);
		  if(frame%100==0)
			  printf("rd %d mS\n",trt);
	  }
	  f_close(&MyFile);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int cnt = -1;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		 RTC_TimeTypeDef sTime;
		 HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		 RTC_DateTypeDef sDate;
		 HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			static int pos = 0;
				 if(sTime.Seconds!=cnt)
				 {
					//pos = (pos+2)%32;
					 char Buff[0x20];
					 ClrScr();
//					 LCD_fillRect(0, 0, LCD_getWidth(), LCD_getHeight(), RED);
//					 LCD_fillRect(1, 1, LCD_getWidth()-2, LCD_getHeight()-2, GREEN);
//					 LCD_fillRect(2, 2, LCD_getWidth()-4, LCD_getHeight()-4, BLUE);
//					 LCD_fillRect(3, 3, LCD_getWidth()-6, LCD_getHeight()-6, BLACK);
					 //LCD_fillRect(0, 0, LCD_getWidth(), LCD_getHeight(), makeColor(255,0,0));
					 LCD_fillRect(0, 0, LCD_getWidth(), LCD_getHeight(), makeColor(16,16,16));
					// LCD_fillRect(1, 1, LCD_getWidth()-2, LCD_getHeight()-2, GREEN);
					 LCD_fillRect(2, 2, LCD_getWidth()-4, LCD_getHeight()-4, BGROUND);
					 //LCD_fillRect(3, 3, LCD_getWidth()-6, LCD_getHeight()-6, BLACK);
						int R = SW/2-1;
					//	LCD_DrawCircle(SH/2, SW/2, R ,GREEN);

					 cnt = sTime.Seconds;

					 sprintf(Buff,"%02d:%02d:%02d",sTime.Hours,sTime.Minutes,sTime.Seconds);
					vec2f center = vec2f_cr(SW/2-0.5,SH/2-0.5);

					 float mat[6];
					 FONT_CURR =  &Font12;
					// LCD_Draw_Text2(Buff,4,0,GREEN,1,1,BGROUND);

					 FONT_CURR =  &Font8;
					 sprintf(Buff,"%02d %s 20%02d",sDate.Date,MontTable[sDate.Month],sDate.Year);
					 uint16_t scolor = ORANGE;//makeColor(127,127,255);
					 LCD_Draw_Text2(Buff,SW/2-FONT_CURR->Width*strlen(Buff)/2,SH/2-23,scolor,1,1,BGROUND);

					// sprintf(Buff,"%02d:%02d:%02d",sTime.Hours,sTime.Minutes,sTime.Seconds);
					 //FONT_CURR =  &Font8;
					 //LCD_Draw_Text2(Buff,SW/2-FONT_CURR->Width*strlen(Buff)/2,17+SH/2,makeColor(127,127,255),1,1,BGROUND);
					 float temp = 25.4f;
					 FONT_CURR =  &Font12;
					 sprintf(Buff,"%+2.1fC",temp);
					 int hrr = sTime.Hours%12;
					 if(hrr>=4&&hrr<=8)
					 {
						 LCD_Draw_Text2(Buff,SW/2-FONT_CURR->Width*strlen(Buff)/2,SH/2-14,scolor,1,1,BGROUND);
					 }
					 else
					 {
						LCD_Draw_Text2(Buff,SW/2-FONT_CURR->Width*strlen(Buff)/2,SH/2-14+28-pos,scolor,1,1,BGROUND);
					 }


					for(int s=0;s<12;s++)
					 {
						vec2f  start = vec2f_cr(0,SH/2-3.5);
						vec2f  end  = vec2f_cr(0,SH/2-0.5);
						float angle =- (s*5%60)*2*M_PI/60;
						rotate_mat(angle,center.x,center.y,mat);
						vec2f  startn = apply(mat,start);
						vec2f  endn = apply(mat,end);
						LCD_DrawLine(startn.x, SH-1-startn.y,endn.x,SH-1-endn.y,CYAN);
					}


					 for(int k=-1;k<=1;k++)
					 {
						vec2f  start = vec2f_cr(k,0);
						vec2f  end  = vec2f_cr(0.0,SH/2*0.55);
						float angle =- ((sTime.Hours%12)+sTime.Minutes/60.0f)*2*M_PI/12;
						rotate_mat(angle,center.x,center.y,mat);
						vec2f  startn = apply(mat,start);
						vec2f  endn = apply(mat,end);
						LCD_DrawLine(startn.x, SH-1-startn.y,endn.x,SH-1-endn.y,GREEN);

					 }
					 for(int k=-1;k<=1;k+=2)
					 {
						vec2f  start = vec2f_cr(k,0);
						vec2f  end  = vec2f_cr(0,SH/2*0.8);
						float angle =- (sTime.Minutes%60)*2*M_PI/60;
						rotate_mat(angle,center.x,center.y,mat);
						vec2f  startn = apply(mat,start);
						vec2f  endn = apply(mat,end);
						LCD_DrawLine(startn.x, SH-1-startn.y,endn.x,SH-1-endn.y,makeColor(127+64,255,64));
					 }
					 {

						vec2f  start = vec2f_cr(0,0);
						vec2f  end  = vec2f_cr(0,SH/2-3.5);
						float angle =- (sTime.Seconds%60)*2*M_PI/60;
						rotate_mat(angle,center.x,center.y,mat);
						vec2f  startn = apply(mat,start);
						vec2f  endn = apply(mat,end);
						LCD_DrawLine(startn.x, SH-1-startn.y,endn.x,SH-1-endn.y,makeColor(127+64,127+64,64));
					 }
					LCD_fillRect(SH/2-3,SH/2-3,5,5,CYAN);//makeColor(128+64,0,0));
					LCD_fillRect(SH/2-2,SH/2-2,3,3,BLUE);//makeColor(128+64,0,0));
					 //LCD_DrawCircleFull(SH/2-1, SW/2-1, 2 ,RED);



					 FONT_CURR =  &Font12;
					 //LCD_Draw_Text2("DurDom!",0,20+SH/2,BLUE,1,1,BGROUND);

					 convertColors();
				 }
#if 0
		 if(sTime.Seconds!=cnt)
		 {
			 char Buff[0x20];
			 ClrScr();
				int R = SW/2-1;
				LCD_DrawCircle(SH/2, SW/2, R ,GREEN);

			 cnt = sTime.Seconds;

			 sprintf(Buff,"%02d:%02d:%02d",sTime.Hours,sTime.Minutes,sTime.Seconds);



			 FONT_CURR =  &Font12;
			// LCD_Draw_Text2(Buff,4,0,GREEN,1,1,BGROUND);

			 FONT_CURR =  &Font8;
			 sprintf(Buff,"%02d %s 20%02d",sDate.Date,MontTable[sDate.Month],sDate.Year);
			 LCD_Draw_Text2(Buff,4,SH/2-4,YELLOW,1,1,BGROUND);
			 {
				 float angle = -(sTime.Hours%12)*2*M_PI/12+M_PI/2;
				 int yPos = -sin(angle)*R/2+SW/2;
				 int xPos = cos(angle)*R/2+SH/2;
				 printf("Angle = %f xPos = %d yPos = %d  ",angle,xPos,yPos);

				 LCD_DrawLine(SH/2, SW/2,xPos,yPos,GREEN);
			 }
			 {
				 float angle = -(sTime.Minutes%60)*2*M_PI/60+M_PI/2;
				 int yPos = -sin(angle)*R+SW/2;
				 int xPos = cos(angle)*R+SH/2;
				 printf("Angle = %f xPos = %d yPos = %d  ",angle,xPos,yPos);
				 LCD_DrawLine(SH/2, SW/2,xPos,yPos,BLUE);
			 }
			 {
				 float angle = -(sTime.Seconds%60)*2*M_PI/60+M_PI/2;
				 int yPos = -sin(angle)*R+SW/2;
				 int xPos = cos(angle)*R+SH/2;
				 printf("Angle = %f xPos = %d yPos = %d  \n",angle,xPos,yPos);
				 LCD_DrawLine(SH/2, SW/2,xPos,yPos,RED);
			 }


			 FONT_CURR =  &Font12;
			 //LCD_Draw_Text2("DurDom!",0,20,BLUE,1,1,BGROUND);

			 sprintf(Buff,"%02d:%02d:%02d",sTime.Hours,sTime.Minutes,sTime.Seconds);
			 FONT_CURR =  &Font8;
			 LCD_Draw_Text2(Buff,8,12+SH/2,GREEN,1,1,BGROUND);

			 FONT_CURR =  &Font8;
			 sprintf(Buff,"%02d %s 20%02d",sDate.Date,MontTable[sDate.Month],sDate.Year);
			 //LCD_Draw_Text2(Buff,4,12+SH/2,YELLOW,1,1,BGROUND);

			 FONT_CURR =  &Font12;
			 //LCD_Draw_Text2("DurDom!",0,20+SH/2,BLUE,1,1,BGROUND);

			 convertColors();
		 }
#endif
		 HAL_Delay(10);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /**Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /**Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 168-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 40-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|SB2_Pin|SR2_Pin|SG2_Pin 
                          |OE_BLANK_Pin|LATCH_Pin|CLK_Pin|FLASH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LINE_D_Pin|LINE_C_Pin|SD_CS_Pin|SD_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LINE_B_Pin|LINE_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SB1_Pin|B2_Pin|G2_Pin|R2_Pin 
                          |SG1_Pin|B1_Pin|G1_Pin|R1_Pin 
                          |SR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_MOSI_GPIO_Port, SD_MOSI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 
                           PE6 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC0 PC1 PC2 
                           PC3 PC6 PC7 PC9 
                           PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2 
                          |GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9 
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY1_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SB2_Pin SR2_Pin SG2_Pin OE_BLANK_Pin 
                           LATCH_Pin CLK_Pin FLASH_CS_Pin */
  GPIO_InitStruct.Pin = SB2_Pin|SR2_Pin|SG2_Pin|OE_BLANK_Pin 
                          |LATCH_Pin|CLK_Pin|FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LINE_D_Pin LINE_C_Pin */
  GPIO_InitStruct.Pin = LINE_D_Pin|LINE_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LINE_B_Pin LINE_A_Pin */
  GPIO_InitStruct.Pin = LINE_B_Pin|LINE_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB13 PB14 
                           PB15 PB6 PB7 PB8 
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SB1_Pin B2_Pin G2_Pin R2_Pin 
                           SG1_Pin B1_Pin G1_Pin R1_Pin 
                           SR1_Pin */
  GPIO_InitStruct.Pin = SB1_Pin|B2_Pin|G2_Pin|R2_Pin 
                          |SG1_Pin|B1_Pin|G1_Pin|R1_Pin 
                          |SR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 
                           PD12 PD13 PD14 PD15 
                           PD0 PD1 PD3 PD4 
                           PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_MISO_Pin */
  GPIO_InitStruct.Pin = SD_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11 
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CS_Pin SD_CLK_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin|SD_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_MOSI_Pin */
  GPIO_InitStruct.Pin = SD_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SD_MOSI_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  	  if(htim->Instance == TIM1)
  		  HAL_TIM_PeriodElapsedCallback1(htim);
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
