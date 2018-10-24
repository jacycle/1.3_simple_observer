#ifndef __GUA_RTC_H__
#define __GUA_RTC_H__
 
/*********************头文件************************/ 
#include "UTC_Clock.h"
 
/*********************函数声明************************/ 
extern void GUA_RTC_Init(void);
extern void GUA_RTC_Set(UTCTimeStruct *pGUA_Timer);
extern void GUA_RTC_Get(UTCTimeStruct *pGUA_Timer);

#endif

 