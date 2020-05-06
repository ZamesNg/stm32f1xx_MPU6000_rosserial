/*CODED by Bruno Alfano on 07/03/2014
www.xene.it
*/

#include <mbed.h>
#include "MPU6000.h"

mpu6000_spi::mpu6000_spi(SPI& _spi, PinName _cs) : spi(_spi), cs(_cs) {}

/*-----------------------------------------------------------------------------------------------
                                    INITIALIZATION
usage: call this function at startup, giving the sample rate divider (raging from 0 to 255) and
low pass filter value; suitable values are:
BITS_DLPF_CFG_256HZ_NOLPF2
BITS_DLPF_CFG_188HZ
BITS_DLPF_CFG_98HZ
BITS_DLPF_CFG_42HZ
BITS_DLPF_CFG_20HZ
BITS_DLPF_CFG_10HZ 
BITS_DLPF_CFG_5HZ 
BITS_DLPF_CFG_2100HZ_NOLPF
returns 1 if an error occurred
-----------------------------------------------------------------------------------------------*/
bool mpu6000_spi::init(int sample_rate_div,int low_pass_filter){
    unsigned int response;
    spi.format(8,0);
    spi.frequency(1000000);
    //FIRST OF ALL DISABLE I2C
    select();
    response=spi.write(MPUREG_USER_CTRL);
    response=spi.write(BIT_I2C_IF_DIS);
    deselect();
    //RESET CHIP
    select();
    response=spi.write(MPUREG_PWR_MGMT_1);
    response=spi.write(BIT_H_RESET); 
    deselect();
    wait(0.15);
    //WAKE UP AND SET GYROZ CLOCK
    select();
    response=spi.write(MPUREG_PWR_MGMT_1);
    response=spi.write(MPU_CLK_SEL_PLLGYROZ); 
    deselect();
    //DISABLE I2C
    select();
    response=spi.write(MPUREG_USER_CTRL);
    response=spi.write(BIT_I2C_IF_DIS);
    deselect();
    //WHO AM I?
    select();
    response=spi.write(MPUREG_WHOAMI|READ_FLAG);
    response=spi.write(0x00);
    deselect();
    if(response<100){return 0;}//COULDN'T RECEIVE WHOAMI
    //SET SAMPLE RATE
    select();
    response=spi.write(MPUREG_SMPLRT_DIV);
    response=spi.write(sample_rate_div); 
    deselect();
    // FS & DLPF
    select();
    response=spi.write(MPUREG_CONFIG);
    response=spi.write(low_pass_filter);
    deselect();
    //DISABLE INTERRUPTS
    select();
    response=spi.write(MPUREG_INT_ENABLE);
    response=spi.write(0x00);
    deselect();
    return 0;
}

/*-----------------------------------------------------------------------------------------------
                                ACCELEROMETER SCALE
usage: call this function at startup, after initialization, to set the right range for the
accelerometers. Suitable ranges are:
BITS_FS_2G
BITS_FS_4G
BITS_FS_8G
BITS_FS_16G
returns the range set (2,4,8 or 16)
-----------------------------------------------------------------------------------------------*/
unsigned int mpu6000_spi::set_acc_scale(int scale){
    unsigned int temp_scale;
    select();
    spi.write(MPUREG_ACCEL_CONFIG);
    spi.write(scale);  
    deselect();    
    switch (scale){
        case BITS_FS_2G:
            acc_divider=16384;
        break;
        case BITS_FS_4G:
            acc_divider=8192;
        break;
        case BITS_FS_8G:
            acc_divider=4096;
        break;
        case BITS_FS_16G:
            acc_divider=2048;
        break;   
    }
    wait(0.01);
    select();
    temp_scale=spi.write(MPUREG_ACCEL_CONFIG|READ_FLAG);
    temp_scale=spi.write(0x00);  
    deselect();
    switch (temp_scale){
        case BITS_FS_2G:
            temp_scale=2;
        break;
        case BITS_FS_4G:
            temp_scale=4;
        break;
        case BITS_FS_8G:
            temp_scale=8;
        break;
        case BITS_FS_16G:
            temp_scale=16;
        break;   
    }
    return temp_scale;
}


/*-----------------------------------------------------------------------------------------------
                                GYROSCOPE SCALE
usage: call this function at startup, after initialization, to set the right range for the
gyroscopes. Suitable ranges are:
BITS_FS_250DPS
BITS_FS_500DPS
BITS_FS_1000DPS
BITS_FS_2000DPS
returns the range set (250,500,1000 or 2000)
-----------------------------------------------------------------------------------------------*/
unsigned int mpu6000_spi::set_gyro_scale(int scale){
    unsigned int temp_scale;
    select();
    spi.write(MPUREG_GYRO_CONFIG);
    spi.write(scale);  
    deselect();    
    switch (scale){
        case BITS_FS_250DPS:
            gyro_divider=131;
        break;
        case BITS_FS_500DPS:
            gyro_divider=65.5;
        break;
        case BITS_FS_1000DPS:
            gyro_divider=32.8;
        break;
        case BITS_FS_2000DPS:
            gyro_divider=16.4;
        break;   
    }
    wait(0.01);
    select();
    temp_scale=spi.write(MPUREG_GYRO_CONFIG|READ_FLAG);
    temp_scale=spi.write(0x00);  
    deselect();
    switch (temp_scale){
        case BITS_FS_250DPS:
            temp_scale=250;
        break;
        case BITS_FS_500DPS:
            temp_scale=500;
        break;
        case BITS_FS_1000DPS:
            temp_scale=1000;
        break;
        case BITS_FS_2000DPS:
            temp_scale=2000;
        break;   
    }
    return temp_scale;
}


/*-----------------------------------------------------------------------------------------------
                                WHO AM I?
usage: call this function to know if SPI is working correctly. It checks the I2C address of the
mpu6000 which should be 104 when in SPI mode.
returns the I2C address (104)
-----------------------------------------------------------------------------------------------*/
unsigned int mpu6000_spi::whoami(){
    unsigned int response;
    select();
    response=spi.write(MPUREG_WHOAMI|READ_FLAG);
    response=spi.write(0x00);
    deselect();
    return response;
}


/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER
usage: call this function to read accelerometer data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
returns the value in Gs
-----------------------------------------------------------------------------------------------*/
float mpu6000_spi::read_acc(int axis){
    uint8_t responseH,responseL;
    int16_t bit_data;
    float data;
    select();
    switch (axis){
        case 0:
        responseH=spi.write(MPUREG_ACCEL_XOUT_H | READ_FLAG);
        break;
        case 1:
        responseH=spi.write(MPUREG_ACCEL_YOUT_H | READ_FLAG);
        break;
        case 2:
        responseH=spi.write(MPUREG_ACCEL_ZOUT_H | READ_FLAG);
        break;
    }
    responseH=spi.write(0x00);
    responseL=spi.write(0x00);
    bit_data=((int16_t)responseH<<8)|responseL;
    data=(float)bit_data;
    data=data/acc_divider;
    deselect();
    return data;
}

/*-----------------------------------------------------------------------------------------------
                                READ GYROSCOPE
usage: call this function to read gyroscope data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
returns the value in Degrees per second
-----------------------------------------------------------------------------------------------*/
float mpu6000_spi::read_rot(int axis){
    uint8_t responseH,responseL;
    int16_t bit_data;
    float data;
    select();
    switch (axis){
        case 0:
        responseH=spi.write(MPUREG_GYRO_XOUT_H | READ_FLAG);
        break;
        case 1:
        responseH=spi.write(MPUREG_GYRO_YOUT_H | READ_FLAG);
        break;
        case 2:
        responseH=spi.write(MPUREG_GYRO_ZOUT_H | READ_FLAG);
        break;
    }
    responseH=spi.write(0x00);
    responseL=spi.write(0x00);
    bit_data=((int16_t)responseH<<8)|responseL;
    data=(float)bit_data;
    data=data/gyro_divider;
    deselect();
    return data;
}

/*-----------------------------------------------------------------------------------------------
                                READ TEMPERATURE
usage: call this function to read temperature data. 
returns the value in °C
-----------------------------------------------------------------------------------------------*/
float mpu6000_spi::read_temp(){
    uint8_t responseH,responseL;
    int16_t bit_data;
    float data;
    select();
    responseH=spi.write(MPUREG_TEMP_OUT_H | READ_FLAG);
    responseH=spi.write(0x00);
    responseL=spi.write(0x00);
    bit_data=((int16_t)responseH<<8)|responseL;
    data=(float)bit_data;
    data=(data/340)+36.53;
    deselect();
    return data;
}

/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER CALIBRATION
usage: call this function to read accelerometer data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
returns Factory Trim value
-----------------------------------------------------------------------------------------------*/
int mpu6000_spi::calib_acc(int axis){
    uint8_t responseH,responseL,calib_data;
    int temp_scale;
    //READ CURRENT ACC SCALE
    select();
    responseH=spi.write(MPUREG_ACCEL_CONFIG|READ_FLAG);
    temp_scale=spi.write(0x00);  
    deselect();
    wait(0.01);
    set_acc_scale(BITS_FS_8G);
    wait(0.01);
    //ENABLE SELF TEST
    select();
    responseH=spi.write(MPUREG_ACCEL_CONFIG);
    temp_scale=spi.write(0x80>>axis);  
    deselect();
    wait(0.01);
    select();
    responseH=spi.write(MPUREG_SELF_TEST_X|READ_FLAG);
    switch(axis){
        case 0:
            responseH=spi.write(0x00);
            responseL=spi.write(0x00);
            responseL=spi.write(0x00);
            responseL=spi.write(0x00);
            calib_data=((responseH&11100000)>>3)|((responseL&00110000)>>4);
        break;
        case 1:
            responseH=spi.write(0x00);
            responseH=spi.write(0x00);
            responseL=spi.write(0x00);
            responseL=spi.write(0x00);
            calib_data=((responseH&11100000)>>3)|((responseL&00001100)>>2);
        break;
        case 2:
            responseH=spi.write(0x00);
            responseH=spi.write(0x00);
            responseH=spi.write(0x00);
            responseL=spi.write(0x00);
            calib_data=((responseH&11100000)>>3)|((responseL&00000011));
        break;
    }
    deselect();
    wait(0.01);
    set_acc_scale(temp_scale);
    return calib_data;
} 

/*-----------------------------------------------------------------------------------------------
                                SPI SELECT AND DESELECT
usage: enable and disable mpu6000 communication bus
-----------------------------------------------------------------------------------------------*/
void mpu6000_spi::select() {
    //Set CS low to start transmission (interrupts conversion)
    cs = 0;
}
void mpu6000_spi::deselect() {
    //Set CS high to stop transmission (restarts conversion)
    cs = 1;
}