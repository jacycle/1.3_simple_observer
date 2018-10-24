#include <ti/display/Display.h>
#include "GUA_RTC.h"
 
void GUA_RTC_Init(void)
{
  //��ʼ��UTC.
  UTC_init(); 
}

void GUA_RTC_Set(UTCTimeStruct *pGUA_Timer)
{
  UTCTimeStruct SetTime; 
  UTCTime SetTime_seconds;  
  
  //ת������
  SetTime.year = pGUA_Timer->year;              
  SetTime.month = pGUA_Timer->month - 1;        
  SetTime.day = pGUA_Timer->day - 1;           
  SetTime.hour = pGUA_Timer->hour - 1;          
  SetTime.minutes = pGUA_Timer->minutes;       
  SetTime.seconds = pGUA_Timer->seconds; 
 
  //��ʱ��ת��Ϊ��
  SetTime_seconds = UTC_convertUTCSecs(&SetTime);
    
  //����ʱ��
  UTC_setClock(SetTime_seconds);  
}

void GUA_RTC_Get(UTCTimeStruct *pGUA_Timer)
{
  UTCTimeStruct GetTime;
  
  //��ȡ��ǰ����
  UTC_convertUTCTime(&GetTime, UTC_getClock());
    
  //ץ������
  pGUA_Timer->year = GetTime.year;                      
  pGUA_Timer->month = GetTime.month + 1;                
  pGUA_Timer->day = GetTime.day + 1;                    
  pGUA_Timer->hour = GetTime.hour + 1;                  
  pGUA_Timer->minutes = GetTime.minutes;                
  pGUA_Timer->seconds = GetTime.seconds;                
}
