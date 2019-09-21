/**
  * @file    imu_usart6.h
  * @author  WANG Kyle
  * @email   x-box361@live.com
  * @version V0.1
  * @date    21-September
  * @brief   This file provides the functions of IMU. 
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IMU_USART6_H
#define __IMU_USART6_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/ 

typedef struct {
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t milli_second;
} imu_time_t;

typedef struct {
  int16_t acc[3];
  int16_t tempeture;
} imu_acc_t;

typedef struct {
  int16_t ang_vel[3];
  int16_t tempeture;
} imu_gyro_t;

typedef struct {
  int16_t angle[3];
  int16_t tempeture;
} imu_angle_t;

/* Exported constants --------------------------------------------------------*/

/** 
  * @brief USART configurtion.
  *
  * You should flow below setups to config USART:
  * 1. Config bus clock macro: USART1 and USART6 append on AHB2 bus,
  * and others append on APB1 bus;
  * 2. Config GPIO macro.
  */
#define IMU_USART                             USART6
#define IMU_USART_CLK                         RCC_APB2Periph_USART6
#define IMU_USART_BAUDRATE                    115200

#define IMU_USART_RX_GPIO_PORT                GPIOC
#define IMU_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOC
#define IMU_USART_RX_PIN                      GPIO_Pin_7
#define IMU_USART_RX_AF                       GPIO_AF_USART6
#define IMU_USART_RX_SOURCE                   GPIO_PinSource7

#define IMU_USART_TX_GPIO_PORT                GPIOC
#define IMU_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOC
#define IMU_USART_TX_PIN                      GPIO_Pin_6
#define IMU_USART_TX_AF                       GPIO_AF_USART6
#define IMU_USART_TX_SOURCE                   GPIO_PinSource6

#define IMU_USART_IRQHandler                  USART6_IRQHandler
#define IMU_USART_IRQ                         USART6_IRQn

#define IMU_RX_BUF_SIZE                       11
/* Exported functions --------------------------------------------------------*/ 

void USART_ImuInit(void);

void reset_imu(void);
void parse_imu_data(uint8_t *data_buf);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_USART6_H */

/********************************END OF FILE**********************************/
