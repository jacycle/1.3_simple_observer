#include <string.h>
#include <stdlib.h>
#include <ti/display/Display.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include "observer.h"
#include "board.h"
#include "station.h"
#include "crc.h"
#include "enc.h"
#include "radio.h"
#include "hw_gpio.h"
#include "sx1276-Hal.h"
#include "sx1276.h"

#define BUFFER_SIZE     256                          // Define the payload size here

static tRadioDriver *Radio = NULL;
static uint16_t BufferSize = BUFFER_SIZE;			// RF buffer size
static uint8_t  Buffer[BUFFER_SIZE];				// RF buffer

static uint8_t station_mode;
static uint8_t station_seq = 0;
static BleDevID_t ble_device_group[BLE_DEVICE_NUM];

extern Display_Handle dispHandle;

void OnMaster( void );
void OnSlave( void );

#define SX1278_RX
//#define SX1278_TX

int station_device_isexist(uint16_t id, uint8_t c)
{
    uint16_t i;
    
    for (i=0; i<BLE_DEVICE_NUM; i++)
    {
        if ((ble_device_group[i].aging_flag) && 
            (ble_device_group[i].id == id) && 
            (ble_device_group[i].c == c))
        {
            return i;
        }
    }
    
    return -1;
}

void station_device_add(uint16_t index, uint16_t id, uint8_t c)
{
    uint8_t temp;
    uint8_t ctemp;
    
    if (index < BLE_DEVICE_NUM)
    {
        ble_device_group[index].aging_flag = 1;
//        ble_device_group[index].aging_time = 12;
        ble_device_group[index].id = id;
        ctemp = ble_device_group[index].c;
        temp = c | (ctemp & 0x03);
        ble_device_group[index].c = temp;
    }
}

void station_seq_plusplus(void)
{
    station_seq++;
}

void station_device_update(uint16_t id, uint8_t c)
{
    int ret;
    uint16_t i;
//    uint8_t min_time;
//    uint8_t index;
    
//    index = 0;
//    min_time = 0xff;
    ret = station_device_isexist(id, c);
    if (ret == -1)
    {
        for (i=0; i<BLE_DEVICE_NUM; i++)
        {
            if ((ble_device_group[i].aging_flag == 0))
            {
                station_device_add(i, id, c);
                break;
            }
            else
            {
                  /* find the min aging time */
//                if (min_time > ble_device_group[i].aging_time)
//                {
//                    min_time = ble_device_group[i].aging_time;
//                    index = i;
//                }
            }
        }
//        if (i == BLE_DEVICE_NUM)
//        {
//            station_device_add(index, id, c);
//        }
    }
    else
    {
        i = ret;
        station_device_add(i, id, c);
    }
}

void station_device_aging(void)
{
/*    uint16_t i;
      
    for (i=0; i<BLE_DEVICE_NUM; i++)
    {
        if ((ble_device_group[i].aging_flag))
        {
            if (ble_device_group[i].aging_time)
            {
                ble_device_group[i].aging_time--;
            }
            else
            {
                ble_device_group[i].aging_flag = 0;
                ble_device_group[i].id = 0;
            }
        }
    }*/
}

void station_device_delete(void)
{
    uint16_t i;
      
    for (i=0; i<BLE_DEVICE_NUM; i++)
    {
        ble_device_group[i].aging_flag = 0;
        ble_device_group[i].id = 0;
        ble_device_group[i].c = 0;
    }
}

int station_device_getbuf(uint8_t *pbuf, uint8_t buflen, uint16_t index, uint8_t offset)
{
    uint16_t i;
    uint8_t len;

    len = 0;
    if (pbuf == NULL)
    {
        return len;
    }
      
    for (i=index; i<BLE_DEVICE_NUM; i++)
    {
        if ( (i-index) >= offset)
        {
            return len;
        }
        if (ble_device_group[i].aging_flag && (len+1) < buflen)
        {
            pbuf[len++] = ble_device_group[i].id;
            pbuf[len++] = ble_device_group[i].id >> 8;
            pbuf[len++] = ble_device_group[i].c;
        }
    }
    
    return len;
}

int station_device_num(void)
{
    uint16_t i;
    uint16_t num;
      
    num = 0;
    for (i=0; i<BLE_DEVICE_NUM; i++)
    {
        if ((ble_device_group[i].aging_flag))
        {
            num++;
        }
    }
    
    return num;
}

void station_mode_set(uint8_t mode)
{
    station_mode = mode;
    Display_print1(dispHandle, 0, 0, "mode=%d", mode);
}

uint8_t station_mode_get(void)
{
    return station_mode;
}

void station_init(void)
{
    Radio = RadioDriverInit();
    
    Radio->Init();
    
#if defined (SX1278_RX)
    Radio->StartRx( );   //RFLR_STATE_RX_INIT
#elif defined (SX1278_TX)
    Radio->SetTxPacket( MY_TEST_Msg, 17 );
#endif
}

void station_send_data(uint8_t *pbuf, uint8_t len)
{
    uint32_t state;
    uint32_t timeout;
    uint32_t startTick;  
    
    Radio->SetTxPacket( pbuf, len );
//    timeout = 100;
//    while(1)
    startTick = GET_TICK_COUNT( );
    while( ( GET_TICK_COUNT( ) - startTick ) < TICK_RATE_MS( 500 ) )
    {
        state = Radio->Process();
        if(state == RF_TX_DONE)
        {
            break;
        }
/*        else
        {
            Task_sleep(10 * (1000 / Clock_tickPeriod));
            if (timeout)  timeout--;
            else  break;
        }*/
    }
}

void station_dump(uint8_t *pbuf, uint16_t len)
{
    char temp[50];
    int totlen;
    char *pstr;
    uint16_t i, j, k;
    int send_len;
    
    totlen = sizeof(temp);
    k = len / 32;
    for (j=0; j<k; j++)
    {
         pstr = temp;
         for (i=0; i<32; i++)
         {
            send_len = snprintf(pstr, totlen, "%02x ", pbuf[j*32 + i]);
            pstr += send_len;
            totlen -= send_len;
         }
         Display_print0(dispHandle, 0, 0, temp);
    }
    len = len % 32;
    if (len)
    {
        pstr = temp;
        for (i=0; i<len; i++)
        {
            send_len = snprintf(pstr, totlen, "%02x ", pbuf[j*32 + i]);
            pstr += send_len;
            totlen -= send_len;
        }
        Display_print0(dispHandle, 0, 0, temp);
    }
}

void station_start_rx(void)
{
    Radio->StartRx();
}

int station_recv_data(uint8_t **pbuf, uint16_t *len)
{
    double pktrssi;
    int rssi;
    if( Radio->Process( ) == RF_RX_DONE)
    {
        Radio->GetRxPacket( Buffer, ( uint16_t* )&BufferSize );
        Radio->StartRx( );
        Display_print1(dispHandle, 0, 0, "recvlen=%d", BufferSize);
        *len = BufferSize;
        *pbuf = Buffer;
        pktrssi = SX1276GetPacketRssi();
        rssi = pktrssi;
        Display_print1(dispHandle, 0, 0, "rssi=%d", rssi);
        station_dump(Buffer, BufferSize);
        return BufferSize;
    }
    else
    {
        return 0;
    }
}


void station_access_request(uint16_t devid)
{
    uint8_t buffer[16];
    uint8_t index;
    uint8_t crc;
    
    index = 0;
    buffer[index++] = 0xaa;
    buffer[index++] = 0x55;
    buffer[index++] = 0x02;
    buffer[index++] = devid;       // device id lsb
    buffer[index++] = devid >> 8;  // device id hsb
//    crc8((char *)buffer, index, (char *)&crc);
    crc = CalCrc(&buffer[2], index - 2);
    buffer[index++]  = crc;
    station_send_data(buffer, index);
//    station_dump(buffer, index);
}

void station_upload(uint8_t left, uint16_t devid, uint16_t time)
{
    uint8_t buffer[256];
    uint8_t index;
    uint8_t crc;
    uint8_t i, j;
    uint16_t advValue;
    uint32_t volValue;
    double pktrssi;
    char rssi;
    int devnum;
    uint8_t groupnum;
    
    advValue = HwADCRead();
    volValue = (advValue * 43) / 4095;
    pktrssi = SX1276GetPacketRssi();
    rssi = pktrssi;
    
    devnum = station_device_num();
    groupnum = (devnum + 37) / 38;
    
    /* if no device, should send the data too */
    if (groupnum == 0)
    {
        groupnum = 1;
    }
    
    /*  how many times to send */
    for (j=0; j<groupnum; j++)
    {
        left  = (groupnum - j - 1) & 0x0f;  // data left
        left |= (station_seq & 0x0f) << 4; 
        Display_print1(dispHandle, 0, 0, "left=%x", left);
        index = 0;
        buffer[index++] = 0xaa;
        buffer[index++] = 0x57;
        buffer[index++] = left;
        buffer[index++] = devid;       // device id lsb
        buffer[index++] = devid >> 8;  // device id hsb
        buffer[index++] = 0;       // datalen
        if (j == 0)
        {
            buffer[index++] = volValue & 0xff;
            buffer[index++] = rssi;
            buffer[index++] = time;
            buffer[index++] = time >> 8;
        }
        if (j == (groupnum - 1))
        {
            for(i=0; i<(devnum % 38); i++)
            {
                buffer[index++] = ble_device_group[j*38 + i].id;
                buffer[index++] = ble_device_group[j*38 + i].id >> 8;
                buffer[index++] = ble_device_group[j*38 + i].c;
            }
            /* first packet */
            if (j == 0)
            {
                buffer[5] = ((devnum % 38) * 3) + 4;
            }
            else
            {
                buffer[5] = ((devnum % 38) * 3);
            }
        }
        else
        {
            for(i=0; i<38; i++)
            {
                buffer[index++] = ble_device_group[j*38 + i].id;
                buffer[index++] = ble_device_group[j*38 + i].id >> 8;
                buffer[index++] = ble_device_group[j*38 + i].c;
            }
            /* first packet */
            if (j == 0)
            {
                buffer[5] = (38 * 3) + 4;
            }
            else
            {
                buffer[5] = (38 * 3);
            }
        }
    //    crc8((char *)buffer, index, (char *)&crc);
        crc = CalCrc(&buffer[2], index - 2);
        buffer[index++]  = crc;
        Display_print1(dispHandle, 0, 0, "station_send_len=%d", index);
        //station_dump(buffer, index);
        station_send_data(buffer, index);
        if (j != (groupnum - 1))
        {
            Task_sleep(500 * (1000 / Clock_tickPeriod));
        }
    }
}
