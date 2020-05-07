#include "stm32f1xx_hal.h"

#define MPU6000_SMPLRT_DIV	    	0x19
#define MPU6000_GYRO_CONFIG	    	0x1B
#define MPU6000_ACCEL_CONFIG  		0x1C
#define MPU6000_FIFO_EN		    	0x23
#define MPU6000_INT_PIN_CFG	    	0x37
#define MPU6000_INT_ENABLE	    	0x38
#define MPU6000_INT_STATUS	    	0x3A
#define MPU6000_ACCEL_XOUT_H 		0x3B
#define MPU6000_ACCEL_XOUT_L 		0x3C
#define MPU6000_ACCEL_YOUT_H 		0x3D
#define MPU6000_ACCEL_YOUT_L 		0x3E
#define MPU6000_ACCEL_ZOUT_H 		0x3F
#define MPU6000_ACCEL_ZOUT_L    	0x40
#define MPU6000_TEMP_OUT_H	    	0x41
#define MPU6000_TEMP_OUT_L	    	0x42
#define MPU6000_GYRO_XOUT_H	    	0x43
#define MPU6000_GYRO_XOUT_L	    	0x44
#define MPU6000_GYRO_YOUT_H	    	0x45
#define MPU6000_GYRO_YOUT_L	     	0x46
#define MPU6000_GYRO_ZOUT_H	    	0x47
#define MPU6000_GYRO_ZOUT_L	    	0x48
#define MPU6000_USER_CTRL	    	0x6A
#define MPU6000_PWR_MGMT_1	    	0x6B
#define MPU6000_PWR_MGMT_2	    	0x6C
#define MPU6000_FIFO_COUNTH	    	0x72
#define MPU6000_FIFO_COUNTL	    	0x73
#define MPU6000_FIFO_R_W		    0x74
#define MPU6000_WHOAMI		    	0x75

// Bits

#define BIT_SLEEP			            0x40
#define BIT_H_RESET			            0x80
#define BITS_AS_RESET                   0x01
#define BITS_CLKSEL			            0x07
#define MPU_CLK_SEL_PLLGYROX	        0x01
#define MPU_CLK_SEL_PLLGYROZ	        0x03
#define MPU_EXT_SYNC_GYROX		        0x02
#define BITS_FS_250DPS                  0x00
#define BITS_FS_500DPS                  0x08
#define BITS_FS_1000DPS                 0x10
#define BITS_FS_2000DPS                 0x18
#define BITS_FS_2G                      0x00
#define BITS_FS_4G                      0x08
#define BITS_FS_8G                      0x10
#define BITS_FS_16G                     0x18
#define BITS_FS_MASK                    0x18
#define BITS_DLPF_CFG_MASK              0x07
#define BIT_INT_ANYRD_2CLEAR            0x10
#define BIT_RAW_RDY_EN			        0x01
#define BIT_I2C_IF_DIS                  0x10
#define BIT_INT_STATUS_DATA		        0x01

#define BITS_DLPF_CFG_256HZ_NOLPF2      0x00
#define BITS_DLPF_CFG_188HZ             0x01
#define BITS_DLPF_CFG_98HZ              0x02
#define BITS_DLPF_CFG_42HZ              0x03
#define BITS_DLPF_CFG_20HZ              0x04
#define BITS_DLPF_CFG_10HZ              0x05
#define BITS_DLPF_CFG_5HZ               0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF      0x07

#define BITS_SAMPLE_RATE_1000HZ         0x00
#define BITS_SAMPLE_RATE_500HZ          0x01
#define BITS_SAMPLE_RATE_333HZ          0x02
#define BITS_SAMPLE_RATE_250HZ          0x03
#define BITS_SAMPLE_RATE_200HZ          0x04
#define BITS_SAMPLE_RATE_167HZ          0x05
#define BITS_SAMPLE_RATE_143HZ          0x06
#define BITS_SAMPLE_RATE_125HZ          0x07

#define MPUREG_CONFIG                   0x1A

#define SMPLRT_DIV                      0x19
#define CONFIG                          0x1A
#define GYRO_CONFIG                     0x1B
#define ACCEL_CONFIG                    0x1C
#define INT_PIN_CFG                     0x37
#define INT_ENABLE                      0x38
#define PWR_MGMT_1                      0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2                      0x6C

#define RAW_TO_RAD                      0.001064*0.5f

#define __START_SPI     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define __STOP_SPI      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)


extern float gyro_nise_max;
extern int16_t gyro_16[3],accel_16[3];
extern float gyro_16_bias[3];
extern uint8_t gyro_8[6],accel_8[6];
extern float theta;
extern float theta_acc;
//int initMPU6000(SPI_HandleTypeDef *hspi);
void readGyroData6000(int16_t * destination);
void readAccelData6000(int16_t * destination);
void readAccelGyroData6000(void);
void calibrateGyroData6000(void);
int initMPU6000(SPI_HandleTypeDef *hspi);
