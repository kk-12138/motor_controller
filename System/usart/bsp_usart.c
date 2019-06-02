/**
  * @file    bsp_usart.c
  * @author  WANG Kyle
  * @email   x-box361@live.com
  * @version V0.1
  * @date    1-June-2019
  * @brief   This file provides the functions of USART. 
  */

#include "bsp_usart.h"

 /**
  * @brief  Configs the NVIC.
  * @param  None
  * @retval None
  */
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);
}

 /**
  * @brief  Configs the USART1 parameter.
  * @param  None
  * @retval None
  */
void USART_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /* Enable GPIOA and USART1 clock. */
  RCC_AHB1PeriphClockCmd(DEBUG_USART_RX_GPIO_CLK|DEBUG_USART_TX_GPIO_CLK,ENABLE);
  RCC_APB2PeriphClockCmd(DEBUG_USART_CLK, ENABLE);

  /* USARTx GPIO configuration -----------------------------------------------*/
  /* Change the mapping of the specified pin. */
  GPIO_PinAFConfig(DEBUG_USART_RX_GPIO_PORT, DEBUG_USART_RX_SOURCE, DEBUG_USART_RX_AF);
  GPIO_PinAFConfig(DEBUG_USART_TX_GPIO_PORT, DEBUG_USART_TX_SOURCE, DEBUG_USART_TX_AF);

  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_PIN;
  GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_PIN;
  GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);

  /* Enable the USART OverSampling by 8 */
  USART_OverSampling8Cmd(DEBUG_USART, ENABLE);

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
  USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(DEBUG_USART, &USART_InitStructure); 

  /* Config NVIC. */
  NVIC_Configuration();

  /* Enable USART interrupt. */
  USART_ITConfig(DEBUG_USART, USART_IT_RXNE, ENABLE);

  /* Enable USART. */
  USART_Cmd(DEBUG_USART, ENABLE);
}

/**
  * @brief  Transmits single data through the USARTx peripheral.
  * @param  pUSARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  data: the data to transmit.
  * @retval None
  */
void USART_SendByte(USART_TypeDef *pUSARTx, uint8_t data)
{
  USART_SendData(pUSARTx, data);

  /* wait until the transmition complete. */
  while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}

/**
  * @brief  Transmits array through the USARTx peripheral.
  * @param  pUSARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  array: the data to transmit.
  * @param  num: the number of members in the array.
  * @retval None
  */
void USART_SendArray(USART_TypeDef *pUSARTx, uint8_t *array, uint16_t num)
{
  uint8_t i;

  for (i = 0; i < num; i++)
  {
    USART_SendByte(pUSARTx, array[i]);
  }

  /* wait until the transmition complete. */
  while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET);
}

/**
  * @brief  Transmits string through the USARTx peripheral.
  * @param  pUSARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  str: Point to the data for transmit.
  * @retval None
  */
void USART_SendString(USART_TypeDef *pUSARTx, char *str)
{
  unsigned int k = 0;

  do 
  {
    USART_SendByte(pUSARTx, *(str + k));
    k++;
  } while (*(str + k) != '\0');
  
  /* wait until the transmition complete. */
  while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET);
}

/**
  * @brief  Transmits single data through the USARTx peripheral.
  * @param  pUSARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  data: the data to transmit.
  * @retval None
  */
void USART_SendHalfWord(USART_TypeDef *pUSARTx, uint16_t data)
{
  uint8_t temp_h, temp_l;

  temp_h = (data & 0XFF00) >> 8;
  temp_l = data & 0XFF;

  /* Sends high eight bits of data. */
  USART_SendData(pUSARTx, temp_h);	
  while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);

  /* Sends low eight bits of data. */
  USART_SendData(pUSARTx, temp_l);
  while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}

/* Redirecs C library fputc to USART1, and then you can use printf. */
int fputc(int ch, FILE *f)
{
  USART_SendData(DEBUG_USART, (uint8_t) ch);
  while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);

  return (ch);
}

/* Redirecs C library fgetc to USART1. */
int fgetc(FILE *f)
{
  /* Waits for USART1 input data. */
  while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE) == RESET);

  return (int)USART_ReceiveData(DEBUG_USART);
}

/********************************END OF FILE**********************************/
