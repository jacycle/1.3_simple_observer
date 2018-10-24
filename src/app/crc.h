#ifndef __CRC_H__
#define __CRC_H__

#ifdef __cplusplus
extern "C"
{
#endif
  
int crc8(char *input , int inputLen, char* outCrc);
int CalCrc(char *input , int len);
  
#endif
  