#include "spi_map_to_i2c.h"

/**
 *  @brief      Write to a device register.
 *
 *  @param[in]  slave_addr  Slave address of device.
 *  @param[in]  reg_addr	Slave register to be written to.
 *  @param[in]  length      Number of bytes to write.
 *  @param[out] data        Data to be written to register.
 *
 *  @return     0 if successful.
 */
int stm32_i2c_write(unsigned char slave_addr,unsigned char reg_addr,
                    unsigned char length,unsigned char const *data){
    // just for avoid warning
    UNUSED(slave_addr);
    return MPU6000_set_reg(reg_addr,(*data),&hspi1);
}

/**
 *  @brief      Read from a device.
 *
 *  @param[in]  slave_addr  Slave address of device.
 *  @param[in]  reg_addr	Slave register to be read from.
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Data from register.
 *
 *  @return     0 if successful.
 */
int stm32_i2c_read(unsigned char slave_addr,unsigned char reg_addr,
                    unsigned char length,unsigned char *data){
    // just for avoid warning
    UNUSED(slave_addr);
    return MPU6000_get_reg(reg_addr,data,&hspi1);
}


/**
 *  @brief	Set up the I2C port and configure the stm32 as the master.
 *  @return	0 if successful.
 */
int stm32_i2c_enable(void){
    HAL_SPI_Init(&hspi1);
}
/**
 *  @brief  Disable I2C communication.
 *  This function will disable the I2C hardware and should be called prior to
 *  entering low-power mode.
 *  @return 0 if successful.
 */
int stm32_i2c_disable(void){
    HAL_SPI_DeInit(&hspi1);
}