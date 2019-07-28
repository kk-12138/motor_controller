/**
  * @file    imu_usart6.c
  * @author  WANG Kyle
  * @email   x-box361@live.com
  * @version V0.1
  * @date    27-July-2019
  * @brief   This file provides the functions of IMU. 
  */

#include <string.h>

#include "imu_usart6.h"
#include "bsp_usart.h"

imu_time_t imu_time;
imu_acc_t imu_acc;
imu_gyro_t imu_gyro;
imu_angle_t imu_angle;

 /**
  * @brief  Configs the NVIC.
  * @param  None
  * @retval None
  */
//static void NVIC_Configuration(void)
//{
//  NVIC_InitTypeDef NVIC_InitStructure;

//  /* Enable the USARTx Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = IMU_USART_IRQ;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

//  NVIC_Init(&NVIC_InitStructure);
//}

 /**
  * @brief  Configs the USART6 parameter.
  * @param  None
  * @retval None
  */
void IMU_USART_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /* Enable GPIOA and USART1 clock. */
  RCC_AHB1PeriphClockCmd(IMU_USART_RX_GPIO_CLK|IMU_USART_TX_GPIO_CLK,ENABLE);
  RCC_APB2PeriphClockCmd(IMU_USART_CLK, ENABLE);

  /* USARTx GPIO configuration -----------------------------------------------*/
  /* Change the mapping of the specified pin. */
  GPIO_PinAFConfig(IMU_USART_RX_GPIO_PORT, IMU_USART_RX_SOURCE, IMU_USART_RX_AF);
  GPIO_PinAFConfig(IMU_USART_TX_GPIO_PORT, IMU_USART_TX_SOURCE, IMU_USART_TX_AF);

  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

  GPIO_InitStructure.GPIO_Pin = IMU_USART_TX_PIN;
  GPIO_Init(IMU_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = IMU_USART_RX_PIN;
  GPIO_Init(IMU_USART_RX_GPIO_PORT, &GPIO_InitStructure);

  /* Enable the USART OverSampling by 8 */
  USART_OverSampling8Cmd(IMU_USART, ENABLE);

  /* USARTx configuration ----------------------------------------------------*/
  /* USARTx configured as follows:
      - BaudRate = 5250000 baud
      - Maximum BaudRate that can be achieved when using the Oversampling by 8
        is: (USART APB Clock / 8) 
     Example: 
      - (USART3 APB1 Clock / 8) = (42 MHz / 8) = 5250000 baud
      - (USART1 APB2 Clock / 8) = (84 MHz / 8) = 10500000 baud
      - Maximum BaudRate that can be achieved when using the Oversampling by 16
        is: (USART APB Clock / 16) 
     Example: (USART3 APB1 Clock / 16) = (42 MHz / 16) = 2625000 baud
     Example: (USART1 APB2 Clock / 16) = (84 MHz / 16) = 5250000 baud
      - Word Length = 8 Bits
      - one Stop Bit
      - No parity
      - Hardware flow control disabled (RTS and CTS signals)
      - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = IMU_USART_BAUDRATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(IMU_USART, &USART_InitStructure); 

//  /* Config NVIC. */
//  NVIC_Configuration();

//  /* Enable USART interrupt. */
//  USART_ITConfig(IMU_USART, USART_IT_RXNE, ENABLE);

  /* Enable USART. */
  USART_Cmd(IMU_USART, ENABLE);
}

void imu_test(void)
{
  static uint8_t rx_buffer[512];
  static uint8_t count;
  uint8_t sum;

  while (1)
  {
    while (USART_GetFlagStatus(IMU_USART, USART_FLAG_RXNE) == RESET)
    {}
    rx_buffer[count++] = USART_ReceiveData(IMU_USART);

    /* Wrong data header, need to search again. */
    if (rx_buffer[0] != 0x55)
    {
      count = 0;
      continue;
    }

    if (count == 11)
    {
      sum = (rx_buffer[0]+rx_buffer[1]+rx_buffer[2]+rx_buffer[3]+rx_buffer[4]+ \
             rx_buffer[5]+rx_buffer[6]+rx_buffer[7]+rx_buffer[8]+rx_buffer[9]);

      if (rx_buffer[10] == sum)
      {
        parse_imu_data(rx_buffer);
        count = 0;
      }
      else
      { 
        count = 0;
      }
    }
  }
}

void parse_imu_data(uint8_t *data_buf)
{
  switch(data_buf[1])
  {
    case 0x50:
    memcpy(&imu_time, &data_buf[2], 8);
    printf("Time: 20%d-%d-%d %d:%d:%.3f\r\n", imu_time.year, imu_time.month, imu_time.day, \
                                              imu_time.hour, imu_time.minute, \
                                              (float)imu_time.second+(float)imu_time.milli_second/1000);
    break;

    case 0x51:
    memcpy(&imu_acc, &data_buf[2], 8);
    printf("acc: %.3f, %.3f, %.3f\r\n", (float)imu_acc.acc[0]/32768*16, \
                                        (float)imu_acc.acc[1]/32768*16, \
                                        (float)imu_acc.acc[2]/32768*16);
    break;

    case 0x52:
    memcpy(&imu_gyro, &data_buf[2], 8);
    printf("ang_vel: %.3f, %.3f, %.3f\r\n", (float)imu_gyro.ang_vel[0]/32768*2000, \
                                            (float)imu_gyro.ang_vel[1]/32768*2000, \
                                            (float)imu_gyro.ang_vel[2]/32768*2000);
    break;

    case 0x53:
    memcpy(&imu_angle, &data_buf[2], 8);
    printf("angle: %.3f, %.3f, %.3f\r\n", (float)imu_angle.angle[0]/32768*180, \
                                          (float)imu_angle.angle[1]/32768*180, \
                                          (float)imu_angle.angle[2]/32768*180);
    break;

    default :
    break;
  }
}
/********************************END OF FILE**********************************/
