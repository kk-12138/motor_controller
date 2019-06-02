/**
  * @file    main.c
  * @author  WANG Kyle
  * @email   x-box361@live.com
  * @version V0.1
  * @date    1-June-2019
  * @brief   Basic development environment.
  */

#include "stm32f4xx.h"
#include "bsp_systick.h"
#include "bsp_usart.h"

int main(void)
{
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  USART_Config();

  printf("just a UART test...\n");
  DelayMs(1000);

  while(1)
  {
    printf("You will receive this msg once per second\n");
    DelayMs(1000);
  }
}
