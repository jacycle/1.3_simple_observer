#ifndef __STATION_H__
#define __STATION_H__

#ifdef __cplusplus
extern "C"
{
#endif
  
#define A1_MODE             0
#define A2_MODE             1
  
#define RADIO_NO_RESP       0
#define A1_ACESS_RESP       1
#define A2_UPLOADE_RESP     2
 
#define BLE_DEVICE_NUM      (30 * 10)
  
typedef struct
{
  uint16_t id;    //!< Device's Address
  uint8_t c;
//  uint8_t aging_time;
  uint8_t aging_flag;
} BleDevID_t;
  
void station_mode_set(uint8_t mode);
uint8_t station_mode_get(void);
void station_access_request(uint16_t devid);
int station_recv_data(uint8_t **pbuf, uint16_t *len);
void station_start_rx(void);

void station_init(void);
void station_seq_plusplus(void);
void station_device_add(uint16_t index, uint16_t id, uint8_t c);
void station_device_update(uint16_t id, uint8_t c);
int station_device_isexist(uint16_t id, uint8_t c);
void station_device_aging(void);
void station_device_delete(void);
int station_device_num(void);
int station_device_getbuf(uint8_t *pbuf, uint8_t buflen, uint16_t index, uint8_t offset);
void station_upload(uint8_t left, uint16_t devid, uint16_t time);
  
#endif
  