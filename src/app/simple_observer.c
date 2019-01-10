/******************************************************************************

 @file       simple_observer.c

 @brief This file contains the Simple Observer sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2011-2017, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc2640r2_sdk_1_40_00_45
 Release Date: 2017-07-20 17:16:59
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/display/Display.h>

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "observer.h"
#include "iotboard_key.h"
#include "board.h"

#include "simple_observer.h"

#include "hw_gpio.h"
#include "hw_uart.h"
#include "hw_spi.h"
#include "hw_adc.h"

//sx127x
#include "platform.h"
#include "radio.h"
#include "spi.h"
#include "station.h"
#include "crc.h"
#include "sx1276-Hal.h"
#include "sx1276.h"
   
/*********************************************************************
 * MACROS
 */
#define BEACON_FEATURE

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  32

// Scan duration in ms
//#define DEFAULT_SCAN_DURATION                 4000
#define DEFAULT_SCAN_DURATION                 300

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               3000

// Discovery mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         FALSE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// Type of Display to open
#if !defined(Display_DISABLE_ALL)
  #if defined(BOARD_DISPLAY_USE_LCD) && (BOARD_DISPLAY_USE_LCD!=0)
    #define SBO_DISPLAY_TYPE Display_Type_LCD
  #elif defined (BOARD_DISPLAY_USE_UART) && (BOARD_DISPLAY_USE_UART!=0)
    #define SBO_DISPLAY_TYPE Display_Type_UART
  #else // !BOARD_DISPLAY_USE_LCD && !BOARD_DISPLAY_USE_UART
    #define SBO_DISPLAY_TYPE 0 // Option not supported
  #endif // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
#else // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
  #define SBO_DISPLAY_TYPE 0 // No Display
#endif // Display_DISABLE_ALL

// Task configuration
#define SBO_TASK_PRIORITY                     1
#define RADIO_TASK_PRIORITY                   1

#ifndef SBO_TASK_STACK_SIZE
#define SBO_TASK_STACK_SIZE                   660
#endif

#ifndef RADIO_TASK_STACK_SIZE
#define RADIO_TASK_STACK_SIZE                 1000
#endif

#define SBO_STATE_CHANGE_EVT                  0x0001
#define SBO_KEY_CHANGE_EVT                    0x0002

// Internal Events for RTOS application
#define SBO_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SBO_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define SBP_PERIODIC_EVT                      Event_Id_00
#define SBP_TIMER_PERIODIC_EVT                Event_Id_01
#define SBP_START_DICOVER_EVT                 Event_Id_02
#define SBP_STOP_DICOVER_EVT                  Event_Id_03

#define SBO_ALL_EVENTS                        (SBO_ICALL_EVT           | \
                                               SBO_QUEUE_EVT           | \
                                               SBP_TIMER_PERIODIC_EVT  | \
                                               SBP_START_DICOVER_EVT   | \
                                               SBP_STOP_DICOVER_EVT)

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data
} sboEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;
Watchdog_Handle hWDT;
uint16_t device_id;
uint32_t device_freq;
uint32_t device_freq1;
uint8_t device_sf;
uint8_t device_sf1;
uint16_t t_temp;       // set to 0 when upload data 
uint8_t radio_led_display;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

static ICall_SyncHandle syncEvent;

// Clock object used to signal timeout
static Clock_Struct keyChangeClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct sboTask;
Char sboTaskStack[SBO_TASK_STACK_SIZE];

// Task configuration
Task_Struct radioTask;
Char radioTaskStack[RADIO_TASK_STACK_SIZE];

// GAP GATT Attributes
//static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple Observer";

// Number of scan results and scan result index
static uint8 scanRes = 0 ;
//static int8 scanIdx = -1;

// Scan result list
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static uint8 scanning = FALSE;

/* Semaphore handle*/
static Event_Handle  myEventHandle = NULL;

//static Queue_Handle myQueueHandle = NULL;

// Clock instances for internal periodic events.
//static Clock_Struct periodicClock;
static Clock_Struct TIMER_periodicClock;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void SimpleBLEObserver_init(void);
static void SimpleBLEObserver_taskFxn(UArg a0, UArg a1);
static void radio_taskFxn(UArg a0, UArg a1);

static void SimpleBLEObserver_handleKeys(uint8_t shift, uint8_t keys);
static void SimpleBLEObserver_processStackMsg(ICall_Hdr *pMsg);
static void SimpleBLEObserver_processAppMsg(sboEvt_t *pMsg);
static void SimpleBLEObserver_processRoleEvent(gapObserverRoleEvent_t *pEvent);
static void SimpleBLEObserver_addDeviceInfo(uint8_t *pAddr, uint8_t eventType, uint8_t addrType, uint8_t dataLen, uint8_t *pRawData);

static uint8_t SimpleBLEObserver_eventCB(gapObserverRoleEvent_t *pEvent);

static uint8_t SimpleBLEObserver_enqueueMsg(uint8_t event, uint8_t status,
                                            uint8_t *pData);

void SimpleBLEObserver_initKeys(void);

void SimpleBLEObserver_keyChangeHandler(uint8 keys);
static void SimpleBLEPeripheral_clockHandler(UArg arg);
static void radio_led_indicate(uint8_t display);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapObserverRoleCB_t simpleBLERoleCB =
{
  SimpleBLEObserver_eventCB  // Event callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

//******************************************************************************
// fn : EVENT_Create
//
// brief : 创建事件
//
// param : none
//
// return : none
static void radio_event_create(void)
{
  Event_Params  eventParams;
  
  Event_Params_init(&eventParams);

  myEventHandle = Event_create(&eventParams, NULL);
  if(myEventHandle == NULL)
  {
//    infoStr = "fail";
  }
}

static Void radio_queue_create(void)
{
//  myQueueHandle = Queue_create(NULL,NULL);
}

//static void watchdog_callback(UArg a0)
//{
//  Watchdog_clear(hWDT);
//}
static void wdt_feed(void)
{
  Watchdog_clear(hWDT);
}

static void wdt_init(void)
{
  int t;
  
  Watchdog_Params wp;
  Watchdog_Params_init(&wp);
//  wp.callbackFxn    = watchdog_callback;
  wp.debugStallMode = Watchdog_DEBUG_STALL_ON;
  wp.resetMode      = Watchdog_RESET_ON;

  Watchdog_init();
  hWDT = Watchdog_open(CC2640R2_LAUNCHXL_WATCHDOG0, &wp);
  if (hWDT == NULL) 
  {
    /* Error opening Watchdog */
    return;
  }
  t = Watchdog_convertMsToTicks(hWDT, SBP_PERIODIC_EVT_PERIOD + 1000);
  Watchdog_setReload(hWDT, t); // 1sec (WDT runs always at 48MHz/32)
}

/*********************************************************************
 * @fn      SimpleBLEObserver_createTask
 *
 * @brief   Task creation function for the Simple Observer.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleBLEObserver_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sboTaskStack;
  taskParams.stackSize = SBO_TASK_STACK_SIZE;
  taskParams.priority = SBO_TASK_PRIORITY;

  Task_construct(&sboTask, SimpleBLEObserver_taskFxn, &taskParams, NULL);
}

void radio_createTask(void)
{
  Task_Params taskParams;
  
  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = radioTaskStack;
  taskParams.stackSize =  RADIO_TASK_STACK_SIZE;
  taskParams.priority = RADIO_TASK_PRIORITY;

  Task_construct(&radioTask, radio_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_clockHandler(UArg arg)
{
    // Wake up the application.
    Event_post(syncEvent, arg);
}

/*********************************************************************
 * @fn      SimpleBLEObserver_init
 *
 * @brief   Initialization function for the Simple Observer App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
void SimpleBLEObserver_init(void)
{
  // ******************************************************************
  // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Hard code the DB Address till CC2650 board gets its own IEEE address
  //uint8 bdAddress[B_ADDR_LEN] = { 0x44, 0x44, 0x44, 0x44, 0x44, 0x44 };
  //HCI_EXT_SetBDADDRCmd(bdAddress);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  Board_initKeys(SimpleBLEObserver_keyChangeHandler);

  dispHandle = Display_open(SBO_DISPLAY_TYPE, NULL);

  // Setup Observer Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPObserverRole_SetParameter(GAPOBSERVERROLE_MAX_SCAN_RES, sizeof(uint8_t),
                                 &scanRes );
  }

  // Setup GAP
  GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
  GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION);

  // Start the Device
  VOID GAPObserverRole_StartDevice((gapObserverRoleCB_t *)&simpleBLERoleCB);
  
  Util_constructClock(&TIMER_periodicClock, SimpleBLEPeripheral_clockHandler,
                      SBP_PERIODIC_EVT_PERIOD, SBP_PERIODIC_EVT_PERIOD, true, SBP_TIMER_PERIODIC_EVT);

  Display_print0(dispHandle, 0, 0, "BLE Observer");
  
  HwGPIOInit();
//  HwGPIOSet(Board_RLED, 0);
  
//  HwUARTInit();
//  HwUARTWrite("observer init\r\n",13);
  SpiInit();
  radio_event_create();
  radio_queue_create();
  radio_createTask();
  
  HwADCInit();
  
  wdt_init();
}

/*********************************************************************
 * @fn      SimpleBLEObserver_taskFxn
 *
 * @brief   Application task entry point for the Simple Observer.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEObserver_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBLEObserver_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    events = Event_pend(syncEvent, Event_Id_NONE, SBO_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          SimpleBLEObserver_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }
    }

    // If RTOS queue is not empty, process app message
    if (events & SBO_QUEUE_EVT)
    {
      while (!Queue_empty(appMsgQueue))
      {
        sboEvt_t *pMsg = (sboEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message
          SimpleBLEObserver_processAppMsg(pMsg);

          // Free the space from the message
          ICall_free(pMsg);
        }
      }
    }
    
    if (events & SBP_TIMER_PERIODIC_EVT)
    {
        wdt_feed();
        t_temp += SBP_PERIODIC_EVT_PERIOD / 1000;
        Display_print1(dispHandle, 0, 0, "t_temp=%d", t_temp);
        if (scanning)
        {
            GAPObserverRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                           DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                           DEFAULT_DISCOVERY_WHITE_LIST);        
        }
        if (radio_led_display)
        {
            radio_led_indicate(radio_led_display);
            radio_led_display--;
        }
        else
        {
            radio_led_indicate(0);
        }
    }
    if (events & SBP_STOP_DICOVER_EVT)
    {
        Display_print0(dispHandle, 0, 0, "stop discover");
        if (scanning)
        {
            Display_print0(dispHandle, 0, 0, "CancelDiscovery");
            GAPObserverRole_CancelDiscovery();
            scanning = FALSE;
        }
    }
    if (events & SBP_START_DICOVER_EVT)
    {
        Display_print0(dispHandle, 0, 0, "start discover");
        if (!scanning)
        {
            Display_print0(dispHandle, 0, 0, "Discover...");
            GAPObserverRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                           DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                           DEFAULT_DISCOVERY_WHITE_LIST);
            scanning = TRUE;
      }
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEObserver_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLEObserver_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SimpleBLEObserver_processRoleEvent((gapObserverRoleEvent_t *)pMsg);
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEObserver_processAppMsg
 *
 * @brief   Central application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLEObserver_processAppMsg(sboEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBO_STATE_CHANGE_EVT:
      SimpleBLEObserver_processStackMsg((ICall_Hdr *)pMsg->pData);

      // Free the stack message
      ICall_freeMsg(pMsg->pData);
      break;

    case SBO_KEY_CHANGE_EVT:
      SimpleBLEObserver_handleKeys(0, pMsg->hdr.state);
      break;

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEObserver_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void SimpleBLEObserver_handleKeys(uint8 shift, uint8 keys)
{
  static int key1_counter;
  (void)shift;  // Intentionally unreferenced parameter

  // Left key determines action to take
//  if (keys & KEY_BTN2)
/*  {
//    if (!scanning)
    {
      // Increment index
      scanIdx++;

        if (scanIdx >= scanRes)
        {
          // Prompt the user to begin scanning again.
          scanIdx = -1;
          Display_print0(dispHandle, 2, 0, "");
          Display_print0(dispHandle, 3, 0, "");
          Display_print0(dispHandle, 5, 0, "Discover ->");
        }
        else
        {
          // Display the indexed scanned device.
          Display_print1(dispHandle, 2, 0, "Device %d", (scanIdx + 1));
          Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(devList[scanIdx].addr));
          Display_print0(dispHandle, 5, 0, "");
          Display_print0(dispHandle, 6, 0, "<- Next Option");
        }
    }
  }*/

  // Right key takes the actio the user has selected.
  if (keys & KEY_BTN1)
  {
    key1_counter++;
//    if (scanIdx == -1)
    if (key1_counter == 10)
    {
      Event_post(myEventHandle, Event_Id_00);
      Display_print0(dispHandle, 0, 0, "key event...");
      if (!scanning)
      {
//        scanRes = 0;
//        Display_print0(dispHandle, 2, 0, "Discovering...");
//        Display_print0(dispHandle, 3, 0, "");
//        Display_print0(dispHandle, 4, 0, "");
//        Display_print0(dispHandle, 5, 0, "Cancel Discovery ->");
//        Display_print0(dispHandle, 6, 0, "");

//        HwUARTWrite("Discovering...\r\n", strlen("Discovering...\r\n"));
//        GAPObserverRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
//                                       DEFAULT_DISCOVERY_ACTIVE_SCAN,
//                                       DEFAULT_DISCOVERY_WHITE_LIST);
//        scanning = TRUE;
      }
//      else
//      {
//        // Cancel Scanning
//        GAPObserverRole_CancelDiscovery();
//        Display_print0(dispHandle, 5, 0, "Cancel Discovery");
//      }
    }
  }
  else
  {
    if (key1_counter >= 1 && key1_counter <= 5)
    {
      radio_led_display = 2;
      // display immediately
      radio_led_indicate(radio_led_display);
    }
    key1_counter = 0;
  }
}

/*********************************************************************
 * @fn      SimpleBLEObserver_processRoleEvent
 *
 * @brief   Observer role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLEObserver_processRoleEvent(gapObserverRoleEvent_t *pEvent)
{
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(pEvent->initDone.devAddr));
        device_id = pEvent->initDone.devAddr[4] + (pEvent->initDone.devAddr[5] << 8);
        device_freq = (pEvent->initDone.devAddr[1] << 0) |
                      (pEvent->initDone.devAddr[2] << 8) |
                      (pEvent->initDone.devAddr[3] << 16);
        device_freq *= 1000;
        device_sf = pEvent->initDone.devAddr[0] & 0x0f;
        Display_print1(dispHandle, 2, 0, "devid = 0x%04x", device_id);
        Display_print1(dispHandle, 2, 0, "freq = %d", device_freq);
        // Prompt user to begin scanning.
        Display_print0(dispHandle, 5, 0, "Discover ->");
//        HwUARTWrite("Discover ->\r\n", strlen("Discover ->\r\n"));
//        GAPObserverRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
//                                      DEFAULT_DISCOVERY_ACTIVE_SCAN,
//                                      DEFAULT_DISCOVERY_WHITE_LIST);
        
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        SimpleBLEObserver_addDeviceInfo(pEvent->deviceInfo.addr,
                                        pEvent->deviceInfo.eventType,
                                        pEvent->deviceInfo.addrType,
                                        pEvent->deviceInfo.dataLen,
                                        pEvent->deviceInfo.pEvtData);
      }
      break;

    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        uint16_t device_num;
        // Discovery complete.
        // Copy results.
        scanRes = pEvent->discCmpl.numDevs;
        memcpy(devList, pEvent->discCmpl.pDevList,
               (sizeof(gapDevRec_t) * pEvent->discCmpl.numDevs));

        Display_print1(dispHandle, 2, 0, "Devices Found %d", scanRes);
        station_device_aging();
        device_num = station_device_num();
        Display_print1(dispHandle, 0, 0, "Devices = %d", device_num);
        if (scanning)
        {
//        GAPObserverRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
//                                       DEFAULT_DISCOVERY_ACTIVE_SCAN,
//                                       DEFAULT_DISCOVERY_WHITE_LIST);
        }
        
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEObserver_eventCB
 *
 * @brief   Observer event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t SimpleBLEObserver_eventCB(gapObserverRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (SimpleBLEObserver_enqueueMsg(SBO_STATE_CHANGE_EVT,
                                   SUCCESS, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }

  // Caller should free the event
  return TRUE;
}

/*********************************************************************
 * @fn      SimpleBLEObserver_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void SimpleBLEObserver_addDeviceInfo(uint8_t *pAddr, uint8_t eventType, uint8_t addrType, uint8_t dataLen, uint8_t *pRawData)
{
#if 1
  uint16_t i;
  uint8_t bat_alarm;
  uint8_t key_alarm;
  uint16_t id;
  uint8_t c;
#ifndef BEACON_FEATURE
  const uint8_t advdata[8] = {0x02, 0x01, 0x04, 0x04, 0xff, 0xaa, 0x00, 0x00};
#else
  const uint8_t advdata[30] = {0x02, 0x01, 0x04, 0x1a, 0xff, 0x0d, 0x00, 0x02, 
                               0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x01, 0x00, 0x01, 0xc5};
#endif

  id = (pAddr[1] << 8) + pAddr[0];
  if (
      addrType == ADDRTYPE_PUBLIC &&
#ifndef BEACON_FEATURE
      dataLen == 8 &&
      eventType == GAP_ADRPT_ADV_SCAN_IND && 
      memcmp(pRawData, advdata, 6) == 0
#else
      dataLen == 30 &&
      eventType == GAP_ADRPT_ADV_NONCONN_IND && 
      memcmp(pRawData, advdata, 25) == 0 
#endif
     )
  {
#ifndef BEACON_FEATURE
      bat_alarm = pRawData[6];
      key_alarm = pRawData[7];
#else
      bat_alarm = pRawData[25];
      key_alarm = pRawData[27];
#endif
      c = (t_temp / 10) << 2;
      if (bat_alarm)
        c |= 0x01;
      if (key_alarm)
        c |= 0x02;
      station_device_update(id, c);
      
      Display_print2(dispHandle, 2, 0, "update id=0x%04x, c=%02x", 
                   id, c);
  }
  
  // If result count not at max
  if ( scanRes < DEFAULT_MAX_SCAN_RES )
  { 
     // Check if device is already in scan results
    for ( i = 0; i < scanRes; i++ )
    {
      if (memcmp(pAddr, devList[i].addr, B_ADDR_LEN) == 0)
      {
        return;
      }
    }

    // Add addr to scan result list
    memcpy(devList[scanRes].addr, pAddr, B_ADDR_LEN );
    devList[scanRes].addrType = addrType;

    // Increment scan result count
    scanRes++;
  }
#else
  uint8_t i, j;
  uint16_t id;
  const uint8_t advdata[8] = {0x02, 0x01, 0x04, 0x04, 0xff, 0x01, 0x02, 0x03}; 

  // If result count not at max
  if ( scanRes < DEFAULT_MAX_SCAN_RES )
  {
    gapAdvDataToken_t *token;
    // add by pjq, 20180826
    //for ( i = 0; i < scanRes; i++ )
    {
      if (devList[i].eventType == GAP_ADRPT_ADV_SCAN_IND && 
          devList[i].addrType == ADDRTYPE_PUBLIC)
      {
          id = (devList[i].addr[1] << 8) + devList[i].addr[0];
          Display_print4(dispHandle, 2, 0, "update id=0x%04x, eventType=%d, addrType=%d, len=%d", 
                         id, devList[i].eventType, devList[i].addrType, dataLen);
          for ( j = 0; j < dataLen; j++ )
          {
              Display_print1(dispHandle, 2, 0, "%02x ", pRawData[j]);
          }
          station_device_update(id, 0);
      }
    }
    // Check if device is already in scan results
    for ( i = 0; i < scanRes; i++ )
    {
      if (memcmp(pAddr, devList[i].addr, B_ADDR_LEN) == 0)
      {
        return;
      }
    }

    // Add addr to scan result list
    memcpy(devList[scanRes].addr, pAddr, B_ADDR_LEN );
    devList[scanRes].addrType = addrType;

    // Increment scan result count
    scanRes++;
  }
#endif
}

/*********************************************************************
 * @fn      SimpleBLEObserver_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   keys pressed
 *
 * @return  none
 */
void SimpleBLEObserver_keyChangeHandler(uint8 keys)
{
  SimpleBLEObserver_enqueueMsg(SBO_KEY_CHANGE_EVT, keys, NULL);
}


/*********************************************************************
 * @fn      SimpleBLEObserver_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimpleBLEObserver_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
  sboEvt_t *pMsg;

  // Create dynamic pointer to message.
  if (pMsg = ICall_malloc(sizeof(sboEvt_t)))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }

  return FALSE;
}

/*********************************************************************
 * @fn      radio_taskFxn
 *
 */
static uint16_t a2_mode_retry;
static uint16_t next_time;
//static uint16_t radio_freq;
static uint16_t time_offset;  // just return back to host

int station_access_resp(uint8_t *pbuf, uint8_t len)
{
    uint8_t datalen;
    uint16_t deviceid;
    
    datalen = pbuf[0];
    if (datalen >= 6 && len >= 7)
    {
        deviceid = (pbuf[1] << 0) + (pbuf[2] << 8);

        if (device_id == deviceid)
        {
            next_time = (pbuf[3] << 0) + (pbuf[4] << 8);
            if (next_time > 3600)
            {
                next_time = 3600;
            }
            device_sf1 = pbuf[5];
            device_freq1 = (pbuf[6] << 0) + (pbuf[7] << 8) + (pbuf[8] << 16);
            device_freq1 *= 1000;
            time_offset = (pbuf[9] << 0) + (pbuf[10] << 8);
            Display_print1(dispHandle, 2, 0, "id=%04x", deviceid);
            Display_print1(dispHandle, 2, 0, "next_time=%d", next_time);
            Display_print1(dispHandle, 2, 0, "device_sf1=%d", device_sf1);
            Display_print1(dispHandle, 2, 0, "device_freq1=%d", device_freq1/1000);
            Display_print1(dispHandle, 2, 0, "time_offset=%d", time_offset);
            return A1_ACESS_RESP;
        }
    }
    
    return RADIO_NO_RESP;
}

int station_upload_resp(uint8_t *pbuf, uint8_t len)
{
    uint8_t datalen;
    uint16_t deviceid;
    
    datalen = pbuf[0];
    if (datalen >= 6 && len >= 7)
    {
        deviceid = (pbuf[1] << 0) + (pbuf[2] << 8);
        if (device_id == deviceid)
        {
            next_time = (pbuf[3] << 0) + (pbuf[4] << 8);
            if (next_time > 3600)
            {
                next_time = 3600;
            }
            time_offset = (pbuf[5] << 0) + (pbuf[6] << 8);
            Display_print1(dispHandle, 2, 0, "devid=%04x", deviceid);
            Display_print1(dispHandle, 2, 0, "next_time=%d", next_time);
            Display_print1(dispHandle, 2, 0, "time_offset=%d", time_offset);
            return A2_UPLOADE_RESP;
        }
    }
    
    return RADIO_NO_RESP;
}

int station_recv_handle(uint8_t *pbuf, uint8_t len)
{
    uint8_t crc = -1;
    
    if (len < 7 || pbuf == NULL)
    {
        return -1;
    }
    //crc8(pbuf, len, &crc);
    crc = CalCrc(&pbuf[2], len -3);
    if (crc != pbuf[len-1])
    {
        return -1;
    }
    if ((pbuf[0] == 0xaa) && (pbuf[1] == 0x56))
    {
          return station_access_resp(&pbuf[2], len-3); 
    }
    if ((pbuf[0] == 0xaa) && (pbuf[1] == 0x58))
    {
          return station_upload_resp(&pbuf[2], len-3); 
    }
    
    return RADIO_NO_RESP;
}

static void radio_led_indicate(uint8_t display)
{
    uint16_t advValue;
    uint32_t volValue;
    double pktrssi;
    char rssi;
    
    if (display == 0)
    {
        HwGPIOSet(Board_LED1, 0);
        HwGPIOSet(Board_LED2, 0);
        HwGPIOSet(Board_LED3, 0);
        HwGPIOSet(Board_LED4, 0);
        HwGPIOSet(Board_LED5, 0);
        return;
    }
    
    advValue = HwADCRead();
    volValue = (advValue * 43) / 4095;
    pktrssi = SX1276GetPacketRssi();
    rssi = pktrssi;
    
    if (volValue < 22)
    {
        HwGPIOSet(Board_LED5, 1);
    }
    else
    {
        HwGPIOSet(Board_LED5, 0);
    }
    if (rssi == 0)
    {  // not register
        HwGPIOSet(Board_LED1, 1);
        HwGPIOSet(Board_LED2, 0);
        HwGPIOSet(Board_LED3, 0);
        HwGPIOSet(Board_LED4, 0);
    }
    if (rssi > 0 && rssi <= 0x80)
    {  // low
        HwGPIOSet(Board_LED1, 0);
        HwGPIOSet(Board_LED2, 0);
        HwGPIOSet(Board_LED3, 0);
        HwGPIOSet(Board_LED4, 1);
    }
    if (rssi>0x80 && rssi<=0x9f)
    {  // middle
        HwGPIOSet(Board_LED1, 0);
        HwGPIOSet(Board_LED2, 0);
        HwGPIOSet(Board_LED3, 1);
        HwGPIOSet(Board_LED4, 1);
    }
    if (rssi>=0xa0 && rssi<=0xb0)
    {  // strong
        HwGPIOSet(Board_LED1, 0);
        HwGPIOSet(Board_LED2, 1);
        HwGPIOSet(Board_LED3, 1);
        HwGPIOSet(Board_LED4, 1);
    }
    if (rssi > 0xb0)
    {
        HwGPIOSet(Board_LED1, 1);
        HwGPIOSet(Board_LED2, 1);
        HwGPIOSet(Board_LED3, 1);
        HwGPIOSet(Board_LED4, 1);
    }
    Display_print1(dispHandle, 0, 0, "rssi=%d", rssi);
}

/*
 * device register to network then led twinkle
*/
void radio_led_twinkle(void)
{
    uint8_t i;
    
    for(i=0; i<3; i++)
    {
        HwGPIOSet(Board_LED1, 1);
        HwGPIOSet(Board_LED2, 1);
        HwGPIOSet(Board_LED3, 1);
        HwGPIOSet(Board_LED4, 1);
        Task_sleep(200 * (1000 / Clock_tickPeriod));
        HwGPIOSet(Board_LED1, 0);
        HwGPIOSet(Board_LED2, 0);
        HwGPIOSet(Board_LED3, 0);
        HwGPIOSet(Board_LED4, 0);
        Task_sleep(200 * (1000 / Clock_tickPeriod));
    }
}

static void radio_taskFxn(UArg a0, UArg a1)
{
    int a1_wait_counter;
    static uint32_t wait_time;
    int ret;
    uint8_t i;
    uint8_t *pbuf;
    uint16_t rlen;
    uint32_t events;
    uint32_t startTick;
    uint32_t chipid;
    
    a1_wait_counter = 0;
    wait_time = 0;
    
//    chipid = 0;
//    Display_print1(dispHandle, 0, 0, "chipid=%x", chipid);
    station_init();
    Display_print0(dispHandle, 0, 0, "radio_taskFxn start");
    while(device_id == 0)
    {
        Task_sleep(100 * (1000 / Clock_tickPeriod));
    }
    srand((int)device_id);
/*    while(1)
    {
        HwGPIOSet(Board_RLED, 1);
        Display_print0(dispHandle, 0, 0, "station_access_request");
        station_access_request(device_id);
        Display_print0(dispHandle, 0, 0, "station_access_request complete");
        HwGPIOSet(Board_RLED, 0);
    }*/
#if 1
    while(1)
    {
start:
          if(station_mode_get() == A1_MODE)
          {
              station_goto_sleep();
              Display_print0(dispHandle, 0, 0, "A1_MODE");
              if (a1_wait_counter < 10)
              {
                  wait_time = (rand() % 60) + 1;
                  a1_wait_counter++;
              }
              else
              {
                  wait_time = (rand() % 1000) + 1000;
                  station_init();
              }
              Display_print1(dispHandle, 0, 0, "wait_time=%d", wait_time);
              wait_time = ((wait_time * 1000) / Clock_tickPeriod) * 1000;
//              station_deinit();
              events = Event_pend(myEventHandle, Event_Id_NONE, Event_Id_00 | Event_Id_01, wait_time);
//              station_init();
              if (events & Event_Id_00)
              {
                  a1_wait_counter = 0;
                  Display_print0(dispHandle, 0, 0, "goto start");
                  radio_led_twinkle();
                  goto start;
              }
              HwGPIOSet(Board_RLED, 1);
              Display_print0(dispHandle, 0, 0, "station_access_request");
              //if (LoRaGetRFFrequency() != LoRaGetRFFrequencyC1())
              {
                  SX1276LoRaSetRFFrequencyC1();
                  Task_sleep(100 * (1000 / Clock_tickPeriod));
              }
              station_access_request(device_id);
              station_start_rx();
              wait_time = 1000;
              startTick = GET_TICK_COUNT( );
              while( ( GET_TICK_COUNT( ) - startTick ) < TICK_RATE_MS( 2000 ) )
//              while(wait_time)
              {
                  ret = station_recv_data(&pbuf, &rlen);
                  if (ret)
                  {
                      ret = station_recv_handle(pbuf, rlen);
                      if (ret == A1_ACESS_RESP)
                      {
                          station_mode_set(A2_MODE);
                          Event_post(syncEvent, SBP_START_DICOVER_EVT);
                          t_temp = 0;
                          break;
                      }
                  }
//                  Task_sleep(1 * (1000 / Clock_tickPeriod));
//                  wait_time -= 1;
              }
              HwGPIOSet(Board_RLED, 0);
          }
          else if(station_mode_get() == A2_MODE)
          {
              a1_wait_counter = 0;
              
              Display_print0(dispHandle, 0, 0, "A2_MODE");
device_delete:
              station_goto_sleep();
              t_temp = 0;
              station_device_delete();
              if (i >= 3)
              {
                  a2_mode_retry++;
              }
              if (a2_mode_retry >= 10)
              {
                  a2_mode_retry = 0;
                  station_mode_set(A1_MODE);
                  Event_post(syncEvent, SBP_STOP_DICOVER_EVT);
              }
              //wait_time =(((device_id & 0x1f)-1)*20 + 600 + 20) * 1000 * (1000 / Clock_tickPeriod);
              wait_time =(next_time) * 1000 * (1000 / Clock_tickPeriod);
//              station_deinit();
              events = Event_pend(myEventHandle, Event_Id_NONE, Event_Id_00 | Event_Id_01, wait_time);
//              station_init();
              if (events & Event_Id_00)
              {
                  a1_wait_counter = 0;
                  station_mode_set(A1_MODE);
                  Event_post(syncEvent, SBP_STOP_DICOVER_EVT);
                  Display_print0(dispHandle, 0, 0, "goto start");
                  a2_mode_retry = 0;
                  radio_led_twinkle();
                  goto start;
              }
              station_seq_plusplus();
              //if (LoRaGetRFFrequency() != LoRaGetRFFrequencyT1())
              {
                  SX1276LoRaSetRFFrequencyT1();
                  Task_sleep(100 * (1000 / Clock_tickPeriod));
              }
              for (i=0; i<3; i++)
              {
                  Display_print0(dispHandle, 0, 0, "station_upload");
                  station_upload(0, device_id, time_offset);
                  station_start_rx();
//                  wait_time = 1000;
//                  while(wait_time)
                  startTick = GET_TICK_COUNT( );
                  while( ( GET_TICK_COUNT( ) - startTick ) < TICK_RATE_MS( 2000 ) )
                  {
                      ret = station_recv_data(&pbuf, &rlen);
                      if (ret)
                      {
                          ret = station_recv_handle(pbuf, rlen);
                          if (ret == A2_UPLOADE_RESP)
                          {
                              a2_mode_retry = 0;
                              goto device_delete;
                          }
                      }
//                      Task_sleep(10 * (1000 / Clock_tickPeriod));
//                      wait_time -= 10;
                  }
              }
          }
          else
          {
              Display_print0(dispHandle, 0, 0, "A3_MODE"); 
              a1_wait_counter = 0;
          }
          Task_sleep(50 * (1000 / Clock_tickPeriod));
    }
#endif
}
/*********************************************************************
*********************************************************************/
