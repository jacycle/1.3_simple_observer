#include "board.h"

#include "hw_gpio.h"

/*********************************************************************
 * LOCAL PARAMETER
 */   
PIN_Handle GPIOHandle;
PIN_State GPIOState;
const PIN_Config GPIOTable[] =
{
  Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//  Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
//  Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
 
  PIN_TERMINATE
};

static uint8_t ledBlinkFlag[3] = {0};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GY_GPIO_Init
 *
 * @brief   GPIO初始化
 *
 * @param   .
 *
 * @return  None.
 */
void HwGPIOInit(void)
{
  GPIOHandle = PIN_open(&GPIOState, GPIOTable);
}

/*********************************************************************
 * @fn      GY_GPIO_SET
 *
 * @brief   GPIO配置函数
 *
 * @param   pin -> GPIO引脚
 *          flag -> GPIO电平
 *
 * @return  None.
 */
void HwGPIOSet(uint32_t pin, uint8_t flag)
{
  switch (pin)
  {
  case Board_LED0:
    ledBlinkFlag[0] = flag;
    break;
//  case Board_LED1:
//    ledBlinkFlag[1] = flag;
//    break;
//  case Board_LED2:
//    ledBlinkFlag[2] = flag;
//    break;
  }
  PIN_setOutputValue(GPIOHandle, pin, flag);
}

/*********************************************************************
 * @fn      GY_GPIO_SET
 *
 * @brief   GPIO配置函数
 *
 * @param   pin -> GPIO引脚
 *          flag -> GPIO电平
 *
 * @return  None.
 */
void HwGPIOToggle(uint32_t pin)
{
  uint8_t flag;
  
  switch (pin)
  {
  case Board_LED0:
    ledBlinkFlag[0] = !ledBlinkFlag[0];
    flag = ledBlinkFlag[0];
    break;
//  case Board_LED1:
//    ledBlinkFlag[1] = !ledBlinkFlag[1];
//    flag = ledBlinkFlag[1];
//    break;
//  case Board_LED2:
//    ledBlinkFlag[2] = !ledBlinkFlag[2];
//    flag = ledBlinkFlag[2];
//    break;
  }
  PIN_setOutputValue(GPIOHandle, pin, flag);
}

