/*************************************/
//文件名：lcd_flash.h
//描述：用来存放字模，图模等
//适合单片机：MSP430单片机2系列
/**************************************/

#ifdef flash
#define EXTERN
#else
#define EXTERN extern
#endif

#ifndef _lcd_flash_
#define _lcd_flash_
#define CHAR_H      16                  /* Character Height (in pixels)       */
#define CHAR_W      8                  /* Character Width (in pixels)        */
extern const unsigned char ASIC[][16];
#endif
