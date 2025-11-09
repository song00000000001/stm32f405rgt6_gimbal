#include "dr16.h"
#include <stdint.h>
#include "main.h"

RC_Ctrl_t RC_CtrlData; 
//uint8_t Buff[16]={0x00,0x00,0x00,0x00,0x00,0x00};

void Get_DR16_Data(uint8_t *Buff)
{
		RC_CtrlData.remote .ch0 =((int16_t)Buff[0] | ((int16_t)Buff[1] << 8)) & 0x07FF;
		RC_CtrlData.remote .ch0 -=1024;
	    RC_CtrlData.remote .ch1 =(((int16_t)Buff[1] >> 3) | ((int16_t)Buff[2] << 5)) & 0x07FF; 
		RC_CtrlData.remote .ch1 -=1024;
		RC_CtrlData.remote .ch2 =(((int16_t)Buff[2] >> 6) | ((int16_t)Buff[3] << 2) | ((int16_t)Buff[4] << 10)) &0x07FF; 
		RC_CtrlData.remote .ch2 -=1024;
		RC_CtrlData.remote .ch3 =(((int16_t)Buff[4] >> 1) | ((int16_t)Buff[5]<<7)) &0x07FF;
		RC_CtrlData.remote .ch3 -=1024;
		RC_CtrlData.remote .s1 =(Buff[5]>>4&0x000C)>>2;
		RC_CtrlData.remote .s2 =(Buff[5]>>4&0x003);
		RC_CtrlData.mouse .x =(Buff[6]|Buff[7]<<8);
		RC_CtrlData.mouse .y =(Buff[8]|Buff[9]<<8);
		RC_CtrlData.mouse .z =(Buff[10]|Buff[11]<<8);
		RC_CtrlData.mouse .press_left =(Buff[12]);
		RC_CtrlData.mouse .press_right =(Buff[13]);
		//RC_CtrlData.key.v = ((int16_t)Buff[14]);// | ((int16_t)pData[15] << 8); 
  
		//your control code ….  
} 
