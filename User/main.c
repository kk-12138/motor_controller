/**
  * @file    main.c
  * @author  WANG Kyle
  * @email   x-box361@live.com
  * @version V0.1
  * @date    21-September-2019
  * @brief   Test IMU data transfer using USART6 RX DMA.
  */

#include "stm32f4xx.h"
#include "bsp_systick.h"
#include "bsp_usart.h"
#include "imu_usart6.h"

int imu_cpy_flag = 0;
int imu_display_flag = 0;
uint8_t imu_data[11] = {0x0};

int main(void)
{
  uint8_t sum;

  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  USART_SysDebugInit();
  USART_ImuInit();

  printf("Just to test IMU data tansfer using DMA\n");
  DelayMs(1000);

  reset_imu();

  while (1)
  {
    /* If the IMU data has been completely copied(imu_cpy_flag == 1), 
     * and this frame data has never been displayed before(imu_display_flag == 1), 
     * it will be showed.
     */
    if ((imu_cpy_flag == 1) && (imu_display_flag == 1))
    {
      sum = (imu_data[0]+imu_data[1]+imu_data[2]+imu_data[3]+imu_data[4]+ \
             imu_data[5]+imu_data[6]+imu_data[7]+imu_data[8]+imu_data[9]);
      if (imu_data[10] == sum)
      {
        parse_imu_data(imu_data);
      }
      imu_display_flag = 0;
    }
  }
}
