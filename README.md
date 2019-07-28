Hardware:<br>
--------
`CPU`: STM32 F407VET6<br>
`Oscillator`: 8 MHz<br>

`IMU`: Connected to USART6
>GND ----> GND<br>
>VCC ----> 3.3V<br>
>RXD ----> PC6<br>
>TXD ----> PC7<br>

Software:<br>
--------
* Basic delay function using systick;<br>
* Debug port using USART1(GPIOA_9: TX; GPIOA_10: RX);<br>
* Simple IMU data transfer test using USART6(GPIOC_6: TX; GPIOC_7: RX);<br>

IDE:<br>
--------
Keil uVersion5.<br>

Attention:<br>
--------
You should config system clock according to your oscillator.<br>
In this project, I use `8` MHz oscillator. Therefore it should be configured as:<br>
* Line 144 in `stm32f4xx.h`:<br>
>#define    HSE_VALUE    ((uint32_t)`8000000`) /*!< Value of the External oscillator in Hz */<br>
* Line 371 in `system_stm32f4xx.c`:<br>
>#define    PLL_M    `8`<br>
