/**
  * @file    main.c
  * @author  WANG Kyle
  * @email   x-box361@live.com
  * @version V0.1
  * @date    28-July-2019
  * @brief   Basic development environment.
  */

#include "stm32f4xx.h"
#include "bsp_systick.h"
#include "bsp_usart.h"
#include "imu_usart6.h"

int main(void)
{
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  USART_Config();
  IMU_USART_Config();

  printf("Just to test IMU data tansfer\n");
  DelayMs(1000);

  imu_test();
}
