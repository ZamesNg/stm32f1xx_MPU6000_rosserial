/*CODED by Bruno Alfano on 07/03/2014
www.xene.it

USAGE (example program):
#include "mbed.h"
#include "MPU6000.h"        //Include library
SPI spi(p11, p12, p13);     //define the SPI (mosi, miso, sclk)
mpu6000_spi imu(spi,p22);   //define the mpu6000 object
int main(){
    if(imu.init(1,BITS_DLPF_CFG_5HZ)){  //INIT the mpu6000
        printf("\nCouldn't initialize MPU6000 via SPI!");
    }    
    wait(0.1);    
    printf("\n\nWHOAMI=%u\n",imu.whoami()); //output the I2C address to know if SPI is working, it should be 104
    wait(0.1);    
    printf("\nGyro_scale=%u\n",imu.set_gyro_scale(BITS_FS_2000DPS));    //Set full scale range for gyros
    wait(1);  
    printf("\nAcc_scale=%u\n",imu.set_acc_scale(BITS_FS_16G));          //Set full scale range for accs
    wait(0.1);    
    while(1) {
        myled = 1;
        wait(0.3);
        myled = 0;
        wait(0.3);
        printf("\nT=%.3f",imu.read_temp());
        printf(" X=%.3f",imu.read_acc(0));  
        printf(" Y=%.3f",imu.read_acc(1)); 
        printf(" Z=%.3f",imu.read_acc(2));   
        printf(" rX=%.3f",imu.read_rot(0));  
        printf(" rY=%.3f",imu.read_rot(1)); 
        printf(" rZ=%.3f",imu.read_rot(2));
    }
}
*/


#ifndef MPU6000_h
#define MPU6000_h
#include "mbed.h"


class mpu6000_spi
{
    SPI& spi;
    DigitalOut cs;
    
  public:
    mpu6000_spi(SPI& _spi, PinName _cs);
    bool init(int sample_rate_div,int low_pass_filter);
    float read_acc(int axis);
    float read_rot(int axis);
    unsigned int set_gyro_scale(int scale);
    unsigned int set_acc_scale(int scale);
    int calib_acc(int axis);
    float read_temp();
    void select();
    void deselect();
    unsigned int whoami();
    
    float acc_divider;
    float gyro_divider;
    
  private:
    PinName _CS_pin;
    PinName _SO_pin;
    PinName _SCK_pin;
    float _error;
};

#endif



// MPU6000 registers
#define MPUREG_XG_OFFS_TC 0x00
#define MPUREG_YG_OFFS_TC 0x01
#define MPUREG_ZG_OFFS_TC 0x02
#define MPUREG_X_FINE_GAIN 0x03
#define MPUREG_Y_FINE_GAIN 0x04
#define MPUREG_Z_FINE_GAIN 0x05
#define MPUREG_XA_OFFS_H 0x06
#define MPUREG_XA_OFFS_L 0x07
#define MPUREG_YA_OFFS_H 0x08
#define MPUREG_YA_OFFS_L 0x09
#define MPUREG_ZA_OFFS_H 0x0A
#define MPUREG_ZA_OFFS_L 0x0B
#define MPUREG_PRODUCT_ID 0x0C
#define MPUREG_SELF_TEST_X 0x0D
#define MPUREG_SELF_TEST_Y 0x0E
#define MPUREG_SELF_TEST_Z 0x0F
#define MPUREG_SELF_TEST_A 0x10
#define MPUREG_XG_OFFS_USRH 0x13
#define MPUREG_XG_OFFS_USRL 0x14
#define MPUREG_YG_OFFS_USRH 0x15
#define MPUREG_YG_OFFS_USRL 0x16
#define MPUREG_ZG_OFFS_USRH 0x17
#define MPUREG_ZG_OFFS_USRL 0x18
#define MPUREG_SMPLRT_DIV 0x19
#define MPUREG_CONFIG 0x1A
#define MPUREG_GYRO_CONFIG 0x1B
#define MPUREG_ACCEL_CONFIG 0x1C
#define MPUREG_INT_PIN_CFG 0x37
#define MPUREG_INT_ENABLE 0x38
#define MPUREG_ACCEL_XOUT_H 0x3B
#define MPUREG_ACCEL_XOUT_L 0x3C
#define MPUREG_ACCEL_YOUT_H 0x3D
#define MPUREG_ACCEL_YOUT_L 0x3E
#define MPUREG_ACCEL_ZOUT_H 0x3F
#define MPUREG_ACCEL_ZOUT_L 0x40
#define MPUREG_TEMP_OUT_H 0x41
#define MPUREG_TEMP_OUT_L 0x42
#define MPUREG_GYRO_XOUT_H 0x43
#define MPUREG_GYRO_XOUT_L 0x44
#define MPUREG_GYRO_YOUT_H 0x45
#define MPUREG_GYRO_YOUT_L 0x46
#define MPUREG_GYRO_ZOUT_H 0x47
#define MPUREG_GYRO_ZOUT_L 0x48
#define MPUREG_USER_CTRL 0x6A
#define MPUREG_PWR_MGMT_1 0x6B
#define MPUREG_PWR_MGMT_2 0x6C
#define MPUREG_BANK_SEL 0x6D
#define MPUREG_MEM_START_ADDR 0x6E
#define MPUREG_MEM_R_W 0x6F
#define MPUREG_DMP_CFG_1 0x70
#define MPUREG_DMP_CFG_2 0x71
#define MPUREG_FIFO_COUNTH 0x72
#define MPUREG_FIFO_COUNTL 0x73
#define MPUREG_FIFO_R_W 0x74
#define MPUREG_WHOAMI 0x75

// Configuration bits MPU6000
#define BIT_SLEEP 0x40
#define BIT_H_RESET 0x80
#define BITS_CLKSEL 0x07
#define MPU_CLK_SEL_PLLGYROX 0x01
#define MPU_CLK_SEL_PLLGYROZ 0x03
#define MPU_EXT_SYNC_GYROX 0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10

#define READ_FLAG   0x80


