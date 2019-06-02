/**
  * @file    bsp_systick.c
  * @author  WANG Kyle
  * @email   x-box361@live.com
  * @version V0.1
  * @date    1-June-2019
  * @brief   This file provides the functions of delay. 
  */

#include "bsp_systick.h"

/**
  * @brief This function provides a microsecond delay
  * @param  us: Microseconds to be delay.
  * @retval None
  */
void SysTickDelayUs(__IO uint32_t us)
{
  uint32_t i;

  SysTick_Config(SystemCoreClock/1000000);

  for (i = 0; i < us; i++)
  {
    /* When the valus of counter reduce to 0, 
        bit 16 of CTRL will be reset to 1. */
    while (!((SysTick->CTRL) & (1<<16)));
  }

  /* Close SysTick timer. */
  SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;
}

/**
  * @brief This function provides a millisecond delay
  * @param  ms: Milliseconds to be delay.
  * @retval None
  */
void SysTickDelayMs(__IO uint32_t ms)
{
  uint32_t i;

  SysTick_Config(SystemCoreClock/1000);

  for (i = 0; i < ms; i++)
  {
    /* When the valus of counter reduce to 0, 
        bit 16 of CTRL will be reset to 1. */
    while (!((SysTick->CTRL) & (1<<16)));
  }

  /* Close SysTick timer. */
  SysTick->CTRL &=~ SysTick_CTRL_ENABLE_Msk;
}

/********************************END OF FILE**********************************/
