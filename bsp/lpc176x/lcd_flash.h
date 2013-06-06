/*************************************/
//�ļ�����lcd_flash.h
//���������������ģ��ͼģ��
//�ʺϵ�Ƭ����MSP430��Ƭ��2ϵ��
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
