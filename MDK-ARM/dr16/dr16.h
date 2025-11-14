#ifndef __DR16_H
#define __DR16_H

#include <stdint.h>

typedef struct 
{
	struct 
	{
		int16_t ch0;
		int16_t ch1;
		int16_t ch2;
		int16_t ch3;
		int8_t s1;
		int8_t s2;
	}remote;
	
	struct 
	{
		int16_t x;
		int16_t y;
		int16_t z;
		int8_t press_left;
		int8_t press_right;
		
	}mouse;

	union {
		uint16_t key_code;
		struct 
		{
			uint16_t W :1;
			uint16_t S :1;
			uint16_t A :1;
			uint16_t D :1;
			uint16_t Q :1;
			uint16_t E :1;
			uint16_t Shift :1;
			uint16_t Ctrl :1;
		}bit;
	}key;
}RC_Ctrl_t;

#define REMOTE_CH_MAX 1684;
#define REMOTE_CH_MIN 364;
#define REMOTE_CH_MID 1024;
#define REMOTE_S_DOWN 2;
#define REMOTE_S_UP 1;
#define REMOTE_S_MID 3;
#define KEY_W 0x01;
#define KAY_S 0x01<<1;
#define KAY_A 0x01<<2;
#define KAY_D 0x01<<3;
#define KAY_Q 0x01<<4;
#define KAY_E 0x01<<5;
#define KAY_Shift 0x01<<6;
#define KAY_Ctrl 0x01<<7;

extern RC_Ctrl_t RC_CtrlData;
void Get_DR16_Data(uint8_t *Buff);
	
#endif