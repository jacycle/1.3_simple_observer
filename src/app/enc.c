#include "enc.h"

int lora_enc(char *input , int inputLen)
{
	int  i=0;

	if(!input || inputLen<=0)
		return -1;

	for(i = 0;i<inputLen;i++)
		input[i] ^= 0x55; /*异或0x55,还原出0xAA 0x57 00 01 00 Dlen D0 ...Dlen-1 Crc*/
	return 0;             /*Crc计算并判断数据包是否完整*/
}