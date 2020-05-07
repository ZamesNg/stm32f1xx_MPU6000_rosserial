#include "MPU6000.h"
#include "math.h"


SPI_HandleTypeDef *MPU6000_SPI;
uint8_t MPU6000_rx_buff[14];
int16_t gyro_16[3],accel_16[3];
uint8_t gyro_8[6],accel_8[6];
float gyro_16_bias[3];
float theta;
float theta_acc;

HAL_StatusTypeDef write(SPI_HandleTypeDef *hspi,uint8_t address,uint8_t data)
{
	uint8_t w_data[2]={address,data};
	HAL_StatusTypeDef status;
	__START_SPI;
	status=HAL_SPI_Transmit(hspi,w_data,2,0x2);
	__STOP_SPI;
	return status;
}

uint8_t spiTransfer(SPI_HandleTypeDef *hspi,uint8_t data)
{
	uint8_t d;
	HAL_SPI_TransmitReceive(hspi,&data,&d,1,0x1);
	return d;
}

HAL_StatusTypeDef MPU6000_set_reg(uint8_t reg,uint8_t value,SPI_HandleTypeDef *hspi)
{
	uint8_t data[2]={reg,value};
	HAL_StatusTypeDef status;
	__START_SPI;
	status	=	HAL_SPI_Transmit(hspi,data,2,0x01);
	__STOP_SPI;
	return status;
}

HAL_StatusTypeDef MPU6000_get_reg(uint8_t reg,uint8_t *	value,SPI_HandleTypeDef *hspi)
{
	uint8_t data[1]={reg};
	HAL_StatusTypeDef status;
	__START_SPI;
	HAL_SPI_Transmit(hspi,data,1,0x01);
	status	=	HAL_SPI_Receive(hspi,value,1,0x01);
	__STOP_SPI;
	return status;
}


int initMPU6000(SPI_HandleTypeDef *hspi)
{
  // Initialize MPU6050 device
  // wake up device
  MPU6000_set_reg(MPU6000_USER_CTRL,BIT_I2C_IF_DIS,hspi);
  //uint8_t data[2]={PWR_MGMT_1, 0x00};
  //HAL_I2C_Master_Transmit(hi2c,MPU6050_ADDRESS,data,2,1); 
  HAL_Delay(100);
  MPU6000_set_reg(PWR_MGMT_1,BIT_H_RESET,hspi);// Clear sleep mode bit (6), enable all sensors 
  HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  
  MPU6000_set_reg(MPU6000_USER_CTRL,BITS_AS_RESET,hspi);// Clear sleep mode bit (6), enable all sensors 
  HAL_Delay(100);
  // get stable time source
  MPU6000_set_reg(PWR_MGMT_1,MPU_CLK_SEL_PLLGYROX,hspi);// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  
  // Configure Gyro and Accelerometer
  // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
  // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
  // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
  MPU6000_set_reg(CONFIG,BITS_DLPF_CFG_42HZ,hspi);
  
  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)	
  MPU6000_set_reg(SMPLRT_DIV,BITS_SAMPLE_RATE_200HZ,hspi);  // Use a 200 Hz rate; the same rate set in CONFIG above
  
  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  MPU6000_set_reg(GYRO_CONFIG,BITS_FS_250DPS,hspi); //± 500 °/s
  MPU6000_set_reg(ACCEL_CONFIG,BITS_FS_4G,hspi); //± 4g

  
  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
  MPU6000_set_reg(INT_PIN_CFG,0x22,hspi);
  MPU6000_set_reg(INT_ENABLE,0x01,hspi);

  MPU6000_SPI = hspi;
  /*
  gyro_nise_max = 1000; 
  float callibrate_gyr[3],callibrate_accel[3];
  for (int i=0;i<1000;i++)
  readAccelGyroData6000(callibrate_,callibrate_accel);
  gyro_nise_max = 0.01; 
  */
  return 0;
}

void readGyroData6000(int16_t * destination)
{
  uint8_t gyr[6];
  __START_SPI;
  spiTransfer(MPU6000_SPI, MPU6000_GYRO_XOUT_H | 0x80);
  HAL_SPI_Receive(MPU6000_SPI,gyr,6,0x1);
  __STOP_SPI;
  destination[0] = (int16_t)(((int16_t)gyr[0] << 8) | gyr[1]); 
  destination[1] = (int16_t)(((int16_t)gyr[2] << 8) | gyr[3]);  
  destination[2] = (int16_t)(((int16_t)gyr[4] << 8) | gyr[5]); 
}
		
void readAccelData6000(int16_t * destination)
{
  uint8_t acc[6];
  __START_SPI;
  spiTransfer(MPU6000_SPI, MPU6000_ACCEL_XOUT_H | 0x80);
  HAL_SPI_Receive(MPU6000_SPI,acc,6,0x1);
  __STOP_SPI;
  destination[0] = (int16_t)(((int16_t)acc[0] << 8) | acc[1]); 
  destination[1] = (int16_t)(((int16_t)acc[2] << 8) | acc[3]);  
  destination[2] = (int16_t)(((int16_t)acc[4] << 8) | acc[5]); 
}


void readAccelGyroData6000(void)
{
  MPU6000_rx_buff[0]	=	 MPU6000_ACCEL_XOUT_H | 0x80;
  __START_SPI;
  HAL_SPI_Transmit(MPU6000_SPI,MPU6000_rx_buff,1,0x01);
  HAL_SPI_Receive(MPU6000_SPI,MPU6000_rx_buff,14,0x01);
  __STOP_SPI;
  
  /*
  for (int i=0; i<6; i++)
  {
    accel_8[i]=MPU6000_rx_buff[i];
    gyro_8[i]=MPU6000_rx_buff[i+8];
  }
  */
  /*
  destination_acc[0] = (int16_t)(((int16_t)MPU6000_rx_buff[0] << 8) | MPU6000_rx_buff[1]); 
  destination_acc[1] = (int16_t)(((int16_t)MPU6000_rx_buff[2] << 8) | MPU6000_rx_buff[3]);  
  destination_acc[2] = (int16_t)(((int16_t)MPU6000_rx_buff[4] << 8) | MPU6000_rx_buff[5]); 
  destination_gyr[0] = (int16_t)(((int16_t)MPU6000_rx_buff[8] << 8) | MPU6000_rx_buff[9]); 
  destination_gyr[1] = (int16_t)(((int16_t)MPU6000_rx_buff[10] << 8) | MPU6000_rx_buff[11]);  
  destination_gyr[2] = (int16_t)(((int16_t)MPU6000_rx_buff[12] << 8) | MPU6000_rx_buff[13]);
  */
  gyro_16[0] =  (int16_t)(((int16_t)MPU6000_rx_buff[8] << 8) | MPU6000_rx_buff[9])-(int16_t)gyro_16_bias[0];
  gyro_16[1] =  (int16_t)(((int16_t)MPU6000_rx_buff[10] << 8) | MPU6000_rx_buff[11])-(int16_t)gyro_16_bias[1];
  gyro_16[2] =  (int16_t)(((int16_t)MPU6000_rx_buff[12] << 8) | MPU6000_rx_buff[13])-(int16_t)gyro_16_bias[2];
  accel_16[0] = -(int16_t)(((int16_t)MPU6000_rx_buff[0] << 8) | MPU6000_rx_buff[1]); 
  accel_16[1] = (int16_t)(((int16_t)MPU6000_rx_buff[2] << 8) | MPU6000_rx_buff[3]);  
  accel_16[2] = (int16_t)(((int16_t)MPU6000_rx_buff[4] << 8) | MPU6000_rx_buff[5]); 
  /*
  for (int i=0;i<3;i++)
  {
    destination_gyr[i] = destination_gyr[i]*RAW_TO_RAD - gyro_noise[i]; 
    if (destination_gyr[i]<gyro_nise_max&&destination_gyr[i]>-gyro_nise_max) gyro_noise[i] = gyro_noise[i] + 0.05*destination_gyr[i];
  }
  */
}

void calibrateGyroData6000(void)
{
  MPU6000_rx_buff[0]	=	 MPU6000_ACCEL_XOUT_H | 0x80;
  __START_SPI;
  HAL_SPI_Transmit(MPU6000_SPI,MPU6000_rx_buff,1,0x01);
  HAL_SPI_Receive(MPU6000_SPI,MPU6000_rx_buff,14,0x01);
  __STOP_SPI;
  
  gyro_16[0] =  (int16_t)(((int16_t)MPU6000_rx_buff[8] << 8) | MPU6000_rx_buff[9]);
  gyro_16[1] =  (int16_t)(((int16_t)MPU6000_rx_buff[10] << 8) | MPU6000_rx_buff[11]);
  gyro_16[2] =  (int16_t)(((int16_t)MPU6000_rx_buff[12] << 8) | MPU6000_rx_buff[13]);
  
  accel_16[0] = -(int16_t)(((int16_t)MPU6000_rx_buff[0] << 8) | MPU6000_rx_buff[1]); 
  accel_16[1] = (int16_t)(((int16_t)MPU6000_rx_buff[2] << 8) | MPU6000_rx_buff[3]);  
  accel_16[2] = (int16_t)(((int16_t)MPU6000_rx_buff[4] << 8) | MPU6000_rx_buff[5]); 

  gyro_16_bias[0] = 0.05f*gyro_16[0]+0.95f*gyro_16_bias[0]; 
  gyro_16_bias[1] = 0.05f*gyro_16[1]+0.95f*gyro_16_bias[1];  
  gyro_16_bias[2] = 0.05f*gyro_16[2]+0.95f*gyro_16_bias[2]; 
  
  float acc=(accel_16[0]+accel_16[1])*0.707107f;
  float theta_acc=(atan2f(acc,accel_16[2]))+2.6E-2f;
  theta=0.95f*theta+0.05f*theta_acc; 
}


