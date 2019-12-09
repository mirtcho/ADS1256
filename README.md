# ADS1256

STM32H7 library for ADS1256
 - stm32CubeIDE1.0.3 project
  
HW configuration
1. SPI4 Pins 
  - PE6 - MOSI
  - PE5 - MISO
2. GPIIO
 - PE3 - NSS- output controlled by software
 - PE4 - ADS_DRDY - input. conncted to ADS1256 DRDY
