#include "board.h"

#include "hw_gpio.h"

/*********************************************************************
 * LOCAL PARAMETER
 */   
PIN_Handle GPIOHandle;
PIN_State GPIOState;
const PIN_Config GPIOTable[] =
{
  Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  Board_LED3 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  Board_LED4 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  Board_LED5 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  IOID_5 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  IOID_6 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  IOID_13  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
 
  PIN_TERMINATE
};

static uint8_t ledBlinkFlag[5] = {0};

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
  case Board_LED1:
    ledBlinkFlag[0] = flag;
    break;
  case Board_LED2:
    ledBlinkFlag[1] = flag;
    break;
  case Board_LED3:
    ledBlinkFlag[2] = flag;
    break;
  case Board_LED4:
    ledBlinkFlag[3] = flag;
    break;
  case Board_LED5:
    ledBlinkFlag[4] = flag;
    break;
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
  case Board_LED1:
    ledBlinkFlag[0] = !ledBlinkFlag[0];
    flag = ledBlinkFlag[0];
    break;
  case Board_LED2:
    ledBlinkFlag[1] = !ledBlinkFlag[1];
    flag = ledBlinkFlag[1];
    break;
  case Board_LED3:
    ledBlinkFlag[2] = !ledBlinkFlag[2];
    flag = ledBlinkFlag[2];
    break;
  case Board_LED4:
    ledBlinkFlag[3] = !ledBlinkFlag[3];
    flag = ledBlinkFlag[3];
    break;
  case Board_LED5:
    ledBlinkFlag[4] = !ledBlinkFlag[4];
    flag = ledBlinkFlag[4];
    break;
  }
  PIN_setOutputValue(GPIOHandle, pin, flag);
}

