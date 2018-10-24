#include "crc.h"

int crc8(char *input , int inputLen, char* outCrc)
{
	int i=0;

	if(!input || inputLen<=0 || !outCrc)
		return -1;

	*outCrc= 0;

	for(i=0;i<inputLen;i++)
		*outCrc += input[i];

	return  0;
}

int CalCrc(char *input , int len)
{
	int  i=0;
    int  crc=0;

	if(!input || len<=0)
		return -1;

	for(i=0;i<len;i++)
		crc += input[i];

	return  crc;
}