# Immigrate rosserial to genericSTM32F103CB

---

## junk code don`t read

this repo is just a practice of the course `embedded system`.Aiming at getting familiar with the `fancy` tools and `fashion` framework, I **don`t** take care of the **quality** of code.

## features

use a opensource flight controller of quadrotor, which named `CC3D`.
use `cubemx` and `platformio` to replace rubbish software `keil5`.

## MPU6000 SPI driver of stm32 HAL library

I really take a hard time at writing the `driver` and `initialization` code of `MPU6000` (who want to write the initialization code of `MPU6000` should take care of the follow code).

(```)
  // disable I2C and enable SPI first
  // we should do this at the first time power on
  MPU6000_set_reg(MPU6000_USER_CTRL,BIT_I2C_IF_DIS,hspi);
(```)

## rosserial immigration

remember to set uart as `DMA` mode.
change the source code of `STM32Hardware.h`.

## redirection of function `printf`

the low level interface is nolonger
(```)
    int fputc(int char, FILE *stream);
(```)

and it changes to
(```)
    int _write (int fd, char *pBuffer, int size);
(```)

and remember to check your toolchains first!

## Each Branch of this Repository

***origin***    the basic implement of rosserial on genericSTM32F103CB
***SPI-test***  use `SPI1` to read `MPU6000`, then use `UART1` to send out data.
***MPU_6050_DMP*** the implement of DMP on `SPI` `MPU6000` and messages to ros via `UART1`.
