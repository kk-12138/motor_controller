/**
  * @file    imu_usart6.c
  * @author  WANG Kyle
  * @email   x-box361@live.com
  * @version V0.1
  * @date    21-September
  * @brief   This file provides the functions of IMU. 
  */

#include <string.h>

#include "imu_usart6.h"
#include "bsp_usart.h"

imu_time_t imu_time;
imu_acc_t imu_acc;
imu_gyro_t imu_gyro;
imu_angle_t imu_angle;

uint8_t imu_rx_buf[IMU_RX_BUF_SIZE] = {0x0};

 /**
  * @brief  Configs the USART6 parameter.
  * @param  None
  * @retval None
  */
static void USART_ImuConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /* Enable GPIOA and USART6 clock. */
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

  /* Enable USART. */
  USART_Cmd(IMU_USART, ENABLE);
}

static void DMA_ImuConfig(void)
{
  DMA_InitTypeDef   DMA_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;

  /* Enable the DMA clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  DMA_DeInit(DMA2_Stream1);

  /* Configure DMA Initialization Structure */
  DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)&(USART6->DR);
  DMA_InitStructure.DMA_BufferSize          = IMU_RX_BUF_SIZE;
  DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;
//  DMA_InitStructure.DMA_Mode                = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Mode                = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority            = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_Single;

  /* Configure RX DMA: DMA2_Stream1 Channel_5*/
  DMA_InitStructure.DMA_Channel             = DMA_Channel_5;
  DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)&imu_rx_buf;
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);

  /* Configure the NVIC initialization structure of RX DMA.*/
  NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable RX DMA. */
//  DMA_Cmd(DMA2_Stream1, ENABLE);
  /* Disable the RX DMA here and enable it after getting the IMU header data. */
  DMA_Cmd(DMA2_Stream1, DISABLE);
//  USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
  USART_DMACmd(USART6, USART_DMAReq_Rx, DISABLE); 

  DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
  DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
}

void USART_ImuInit(void)
{
  USART_ImuConfig();
  DMA_ImuConfig();
}
void reset_imu(void)
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
        /* Got the IMU deater data, enable the RX DMA. */
        DMA_Cmd(DMA2_Stream1, ENABLE);
        USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);

        return;
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
