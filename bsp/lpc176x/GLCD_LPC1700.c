/******************************************************************************/
/* GLCD_SPI_LPC1700.c: LPC1700 low level Graphic LCD (320x240 pixels) driven  */
/*                     with SPI functions                                     */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2009 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/


#include <lpc17xx.h>
#include "GLCD.h"

//#define  FONT8_16
#ifdef FONT8_16
#include "lcd_flash.h"
#else
#include "Font_24x16.h"
#endif

/*********************** Hardware specific configuration **********************/

/* 8bit to 16bit LCD Interface 
   
   PINS: 
   - EN      = P0.19
   - LE      = P0.20
   - DIR     = P0.21
   - CS      = P0.22
   - RS      = P0.23 
   - WR      = P0.24 
   - RD      = P0.25
   - DB[0.7] = P2.0...P2.7
   - DB[8.15]= P2.0...P2.7                                                     */

#define PIN_EN		(1 << 19)
#define PIN_LE		(1 << 20)
#define PIN_DIR		(1 << 21)
#define PIN_CS      (1 << 22)
#define PIN_RS		(1 << 23)
#define PIN_WR		(1 << 24)
#define PIN_RD		(1 << 25)   

/*------------------------- Speed dependant settings -------------------------*/

/* If processor works on high frequency delay has to be increased, it can be 
   increased by factor 2^N by this constant                                   */
#define DELAY_2N    18

/*---------------------- Graphic LCD size definitions ------------------------*/

#define WIDTH       240                 /* Screen Width (in pixels)           */
#define HEIGHT      320                 /* Screen Hight (in pixels)           */
#define BPP         16                  /* Bits per pixel                     */
#define BYPP        ((BPP+7)/8)         /* Bytes per pixel                    */

/*--------------- Graphic LCD interface hardware definitions -----------------*/

/* Pin EN setting to 0 or 1                                                   */
#define LCD_EN(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_EN) : (LPC_GPIO0->FIOCLR = PIN_EN));
/* Pin LE setting to 0 or 1                                                   */
#define LCD_LE(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_LE) : (LPC_GPIO0->FIOCLR = PIN_LE));
/* Pin DIR setting to 0 or 1                                                   */
#define LCD_DIR(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_DIR) : (LPC_GPIO0->FIOCLR = PIN_DIR));
/* Pin CS setting to 0 or 1                                                   */
#define LCD_CS(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_CS) : (LPC_GPIO0->FIOCLR = PIN_CS));
/* Pin RS setting to 0 or 1                                                   */
#define LCD_RS(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_RS) : (LPC_GPIO0->FIOCLR = PIN_RS));
/* Pin WR setting to 0 or 1                                                   */
#define LCD_WR(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_WR) : (LPC_GPIO0->FIOCLR = PIN_WR));
/* Pin RD setting to 0 or 1                                                   */
#define LCD_RD(x)   ((x) ? (LPC_GPIO0->FIOSET = PIN_RD) : (LPC_GPIO0->FIOCLR = PIN_RD));
 
/*---------------------------- Global variables ------------------------------*/

/******************************************************************************/
static volatile unsigned short TextColor = Black, BackColor = White;
/************************ Local auxiliary functions ***************************/

/*******************************************************************************
* Delay in while loop cycles                                                   *
*   Parameter:    cnt:    number of while cycles to delay                      *
*   Return:                                                                    *
*******************************************************************************/

static void delay (int cnt) {

  cnt <<= DELAY_2N;
  while (cnt--);
}

__asm void wait()
{
    nop
    BX lr
}

void wait_delay(int count)
{
  while(count--);
}

/*******************************************************************************
* Send 1 byte over serial communication                                        *
*   Parameter:    byte:   byte to be sent                                      *
*   Return:                                                                    *
*******************************************************************************/

static __inline unsigned char lcd_send (unsigned short byte) {

   
  LPC_GPIO2->FIODIR |= 0x000000ff;  //P2.0...P2.7 Output
  LCD_DIR(1)		   				//Interface A->B
  LCD_EN(0)	                        //Enable 2A->2B
  
  LPC_GPIO2->FIOPIN =  byte;        //Write D0..D7
  LCD_LE(1)
  LCD_LE(0)							//latch D0..D7
  LPC_GPIO2->FIOPIN =  byte >> 8;   //Write D8..D15 
  return(1);
}


/*******************************************************************************
* Write command to LCD controller                                              *
*   Parameter:    c:      command to be written                                *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_cmd (unsigned char c) {

  LCD_RS(0)
  LCD_RD(1)
  lcd_send(c);
  LCD_WR(0)
  wait();
  LCD_WR(1)
}


/*******************************************************************************
* Write data to LCD controller                                                 *
*   Parameter:    c:      data to be written                                   *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_dat (unsigned short c) {

  LCD_RS(1)
  LCD_RD(1)
  lcd_send(c);
  LCD_WR(0)
  wait();
  LCD_WR(1)
}


/*******************************************************************************
* Start of data writing to LCD controller                                      *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_dat_start (void) {

  LCD_RS(1)
}


/*******************************************************************************
* Stop of data writing to LCD controller                                       *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_dat_stop (void) {

  LCD_CS(1)
}


/*******************************************************************************
* Data writing to LCD controller                                               *
*   Parameter:    c:      data to be written                                   *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_dat_only (unsigned short c) {

  lcd_send(c);
  LCD_WR(0)
  wait();
  wait();
  LCD_WR(1)
  wait();
}


/*******************************************************************************
* Write to LCD register                                                        *
*   Parameter:    reg:    register to be read                                  *
*                 val:    value to write to register                           *
*******************************************************************************/

static __inline void wr_reg (unsigned char reg, unsigned short val) {

  LCD_CS(0)
  wr_cmd(reg);
  wr_dat(val);
  LCD_CS(1)
}
/*******************************************************************************
* read 1 byte over serial communication                                        *
*   Parameter:    byte:   byte to be sent                                      *
*   Return:                                                                    *
*******************************************************************************/
/*
static __inline unsigned short lcd_read (void) {
  unsigned short id;
  LPC_GPIO2->FIODIR &= 0xffffff00;                //P2.0...P2.7 Input
  LCD_DIR(0)		   				              //Interface B->A
  LCD_EN(0)	                                      //Enable 2B->2A
  wait_delay(80);							      //delay some times
  id = LPC_GPIO2->FIOPIN & 0x00ff;                //Read D8..D15                         
  LCD_EN(1)	                                      //Enable 1B->1A
  wait_delay(80);							      //delay some times
  id = (id << 8) | (LPC_GPIO2->FIOPIN & 0x00ff);  //Read D0..D7                         
  LCD_DIR(1)						
  return(id); 
}       */
/*******************************************************************************
* Read data from LCD controller                                                *
*   Parameter:                                                                 *
*   Return:               read data                                            *
*******************************************************************************/
/*
static __inline unsigned short rd_dat (void) {
  unsigned short val = 0;

  LCD_RS(1)
  LCD_WR(1)
  LCD_RD(0)
  val = lcd_read();
  LCD_RD(1)
  return val;
}
  */
/*******************************************************************************
* Read from LCD register                                                       *
*   Parameter:    reg:    register to be read                                  *
*   Return:               value read from register                             *
*******************************************************************************/
/*
static unsigned short rd_reg (unsigned short reg) {
  unsigned short val = 0;

  LCD_CS(0)
  wr_cmd(reg);
  val = rd_dat(); 
  LCD_CS(1)
  return (val);
}
    */

/************************ Exported functions **********************************/

/*******************************************************************************
* Initialize the Graphic LCD controller                                        *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_Init (void) { 

  /* Configure the LCD Control pins                                           */
  LPC_GPIO0->FIODIR   |= 0x03f80000;
  LPC_GPIO0->FIOSET    = 0x03f80000;	

  delay(5);                             /* Delay 50 ms                        */
  
  /* Set GRAM write direction and BGR = 1
     I/D=10 (Horizontal : increment, Vertical : increment)
     AM=1 (address is updated in vertical writing direction)                  */
  wr_reg(0x03, 0x1038);

  wr_reg(0x07, 0x0173);                 /* 262K color and display ON          */ 
  //3.2" TFT LCD Module,DriverIC is SSD1289
  {
  	wr_reg(0x0000,0x0001);    delay(5);  //打开晶振
    	wr_reg(0x0003,0xA8A4);    delay(5);   //0xA8A4
    	wr_reg(0x000C,0x0000);    delay(5);   
    	wr_reg(0x000D,0x080C);    delay(5);   
    	wr_reg(0x000E,0x2B00);    delay(5);   
    	wr_reg(0x001E,0x00B0);    delay(5);   
    	wr_reg(0x0001,0x6b3F);    delay(5);     //驱动输出控制320*240  0x6B3F
    	wr_reg(0x0002,0x0600);    delay(5);
    	wr_reg(0x0010,0x0000);    delay(5);
    	wr_reg(0x0011,0x6030);    delay(5);        //0x4030           //定义数据格式  16位色 		横屏 0x6058
    	wr_reg(0x0005,0x0000);    delay(5);
    	wr_reg(0x0006,0x0000);    delay(5);
    	wr_reg(0x0016,0xEF1C);    delay(5);
    	wr_reg(0x0017,0x0003);    delay(5);
    	wr_reg(0x0007,0x0233);    delay(5);        //0x0233       
    	wr_reg(0x000B,0x0000);    delay(5);
    	wr_reg(0x000F,0x0000);    delay(5);        //扫描开始地址
    	wr_reg(0x0041,0x0000);    delay(5);
    	wr_reg(0x0042,0x0000);    delay(5);
    	wr_reg(0x0048,0x0000);    delay(5);
    	wr_reg(0x0049,0x013F);    delay(5);
    	wr_reg(0x004A,0x0000);    delay(5);
    	wr_reg(0x004B,0x0000);    delay(5);
    	wr_reg(0x0044,0xEF00);    delay(5);
    	wr_reg(0x0045,0x0000);    delay(5);
    	wr_reg(0x0046,0x013F);    delay(5);
    	wr_reg(0x0030,0x0707);    delay(5);
    	wr_reg(0x0031,0x0204);    delay(5);
    	wr_reg(0x0032,0x0204);    delay(5);
    	wr_reg(0x0033,0x0502);    delay(5);
    	wr_reg(0x0034,0x0507);    delay(5);
    	wr_reg(0x0035,0x0204);    delay(5);
    	wr_reg(0x0036,0x0204);    delay(5);
    	wr_reg(0x0037,0x0502);    delay(5);
    	wr_reg(0x003A,0x0302);    delay(5);
    	wr_reg(0x003B,0x0302);    delay(5);
    	wr_reg(0x0023,0x0000);    delay(5);
    	wr_reg(0x0024,0x0000);    delay(5);
    	wr_reg(0x0025,0x8000);    delay(5);
    	wr_reg(0x004f,0);        //行首址0
    	wr_reg(0x004e,0);        //列首址0
  }
}


/*******************************************************************************
* Set draw window region to whole screen                                       *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_WindowMax (void) {

  wr_reg(0x50, 0);                      /* Horizontal GRAM Start Address      */
  wr_reg(0x51, HEIGHT-1);               /* Horizontal GRAM End   Address (-1) */
  wr_reg(0x52, 0);                      /* Vertical   GRAM Start Address      */
  wr_reg(0x53, WIDTH-1);                /* Vertical   GRAM End   Address (-1) */
}


/*******************************************************************************
* Draw a pixel in foreground color                                             *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_PutPixel (unsigned int x, unsigned int y) {

  wr_reg(0x4e, y);
  wr_reg(0x4f, WIDTH-1-x);

  LCD_CS(0)
  wr_cmd(0x22);
  wr_dat(TextColor);
  LCD_CS(1)
}


/*******************************************************************************
* Set foreground color                                                         *
*   Parameter:      color:    foreground color                                 *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_SetTextColor (unsigned short color) {

  TextColor = color;
}


/*******************************************************************************
* Set background color                                                         *
*   Parameter:      color:    background color                                 *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_SetBackColor (unsigned short color) {

  BackColor = color;
}


/*******************************************************************************
* Clear display                                                                *
*   Parameter:      color:    display clearing color                           *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_Clear (unsigned short color) {
  unsigned int   i;

  GLCD_WindowMax();

  wr_reg(0x4e, 0);
  wr_reg(0x4f, 0);

  LCD_CS(0)
  wr_cmd(0x22);
  wr_dat_start();
  for(i = 0; i < (WIDTH*HEIGHT); i++)
    wr_dat_only(color);
  wr_dat_stop();
}


/*******************************************************************************
* Draw character on given position                                             *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   c:        pointer to character bitmap                      *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_DrawChar (unsigned int x, unsigned int y, unsigned char c) {
  int i, j;

  x = WIDTH-x-CHAR_W;

 // wr_reg(0x44, x);                   /* Horizontal GRAM Start Address      */
  wr_reg(0x44, x |((x+CHAR_W-1)<<8));           /* Horizontal GRAM End   Address (-1) */
  wr_reg(0x45, y);                      /* Vertical   GRAM Start Address      */
  wr_reg(0x46, y+CHAR_H-1);                  /* Vertical   GRAM End   Address (-1) */
 
 
  wr_reg(0x4e, x);
  wr_reg(0x4f, y);

  LCD_CS(0)
  wr_cmd(0x22);
  wr_dat_start();
  for (j = 0; j < CHAR_H/2; j++) {
       
        for (i = CHAR_W -1 ; i >=0 ; i--) {
          #ifdef FONT8_16
          if((ASIC[c][j*2+1] &(0x80 >> i)) == 0x00) {
          #else
          if((Font_24x16[c][j*2+1] &(0x01 << i)) == 0x00) {
          #endif
            wr_dat_only(BackColor);
          } else {
            wr_dat_only(TextColor);
          }  
        }

         for (i = CHAR_W -1 ; i >=0 ; i--) {
          #ifdef FONT8_16
          if((ASIC[c][j*2] &(0x80 >> i)) == 0x00) {
          #else
          if((Font_24x16[c][j*2] &(0x01 << i)) == 0x00) {
          #endif
            wr_dat_only(BackColor);
          } else {
            wr_dat_only(TextColor);
          }  
        }
  }    
  wr_dat_stop();
}


/*******************************************************************************
* Disply character on given line                                               *
*   Parameter:      ln:       line number                                      *
*                   col:      column number                                    *
*                   c:        ascii character                                  *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_DisplayChar (unsigned int ln, unsigned int col, unsigned char c) {

  c -= 32;
  GLCD_DrawChar(col * CHAR_W, ln * CHAR_H, c);
}


/*******************************************************************************
* Disply string on given line                                                  *
*   Parameter:      ln:       line number                                      *
*                   col:      column number                                    *
*                   s:        pointer to string                                *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_DisplayString (unsigned int ln, unsigned int col, unsigned char *s) {

  GLCD_WindowMax();
  while (*s) {
    GLCD_DisplayChar(ln, col++, *s++);
  }
}


/*******************************************************************************
* Clear given line                                                             *
*   Parameter:      ln:       line number                                      *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_ClearLn (unsigned int ln) {

  GLCD_WindowMax();
  GLCD_DisplayString(ln, 0, "                    ");
}

/*******************************************************************************
* Draw bargraph                                                                *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        maximum width of bargraph (in pixels)            *
*                   val:      value of active bargraph (in 1/1024)             *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_Bargraph (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int val) {
  int i,j;

  x = WIDTH-x-w;
  wr_reg(0x44, y);                   /* Horizontal GRAM Start Address      */
  wr_reg(0x44, y |((y+CHAR_H-1)<<8));           /* Horizontal GRAM End   Address (-1) */
  wr_reg(0x45, x);                      /* Vertical   GRAM Start Address      */
  wr_reg(0x46, x+w-1);                  /* Vertical   GRAM End   Address (-1) */

  val = (val * w) >> 10;                /* Scale value for 24x12 characters   */

  wr_reg(0x4e, y);
  wr_reg(0x4f, WIDTH-1-x);

  LCD_CS(0)
  wr_cmd(0x22);
  wr_dat_start();
  for (i = 0; i < h; i++) {
    for (j = w-1; j >= 0; j--) {
      if(j >= val) {
        wr_dat_only(BackColor);
      } else {
        wr_dat_only(TextColor);
      }
    }
  }
  wr_dat_stop();
}


/*******************************************************************************
* Display graphical bitmap image at position x horizontally and y vertically   *
* (This function is optimized for 16 bits per pixel format, it has to be       *
*  adapted for any other bits per pixel format)                                *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        width of bitmap                                  *
*                   h:        height of bitmap                                 *
*                   bitmap:   address at which the bitmap data resides         *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_Bitmap (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned char *bitmap) {
  unsigned int    i, j;
  unsigned short *bitmap_ptr = (unsigned short *)bitmap;

  x = WIDTH-x-w;

  wr_reg(0x44, y);                   /* Horizontal GRAM Start Address      */
  wr_reg(0x44, y |((y+h-1)<<8));           /* Horizontal GRAM End   Address (-1) */ 
  wr_reg(0x45, x);                      /* Vertical   GRAM Start Address      */
  wr_reg(0x46, x+w-1);                  /* Vertical   GRAM End   Address (-1) */
  wr_reg(0x4e, y);
  wr_reg(0x4f, WIDTH-1-x);

  LCD_CS(0)
  wr_cmd(0x22);
  wr_dat_start();
  for (j = 0; j < h; j++) {
    bitmap_ptr += w-1;
    for (i = 0; i < w; i++) {
      wr_dat_only(*bitmap_ptr--);
    }
    bitmap_ptr += w+1;
  }
  wr_dat_stop();
}


/*******************************************************************************
* Display graphical bmp file image at position x horizontally and y vertically *
* (This function is optimized for 16 bits per pixel format, it has to be       *
*  adapted for any other bits per pixel format)                                *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        width of bitmap                                  *
*                   h:        height of bitmap                                 *
*                   bmp:      address at which the bmp data resides            *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_Bmp (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned char *bmp) {
  unsigned int    i, j;
  unsigned short *bitmap_ptr = (unsigned short *)bmp;

  x = WIDTH-x-w;

  	wr_reg(0x44, y);                   /* Horizontal GRAM Start Address      */
    wr_reg(0x44, y |((y+h-1)<<8));           /* Horizontal GRAM End   Address (-1) */
    wr_reg(0x45, x);                      /* Vertical   GRAM Start Address      */
    wr_reg(0x46, x+w-1);                  /* Vertical   GRAM End   Address (-1) */
  	wr_reg(0x4e, y);
  	wr_reg(0x4f, WIDTH-1-x);      
  
  LCD_CS(0)
  wr_cmd(0x22);
  wr_dat_start();
  bitmap_ptr += (h*w)-1;
  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      wr_dat_only(*bitmap_ptr--);
    }
  }
  wr_dat_stop();
}

/******************************************************************************/
