/*
 * main.cpp

 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include <ros.h>
#include <sensor_msgs\Imu.h>

#ifdef __cplusplus
 extern "C" {
#endif

#include <MPU6000.h>
#include <usart.h>
#include <spi.h>
#include "inv_mpu.h"

#ifdef __cplusplus
}
#endif

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu_mpu6000",&imu_msg);
float pitch,yaw,roll;
int status;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void)
{
  initMPU6000(&hspi1);
  nh.initNode();
  nh.advertise(imu_pub);
  pitch = yaw = roll = 0;
  status = 0;
  for(status = mpu_dmp_init();status;){
  printf("dmp init error!\r\n");
  printf("error code: %d\r\n",status);
  HAL_Delay(500);
  }
}

void loop(void)
{
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);

  readAccelGyroData6000();

  imu_msg.angular_velocity.x = gyro_16[0];
  imu_msg.angular_velocity.y = gyro_16[1];
  imu_msg.angular_velocity.z = gyro_16[2];

  imu_msg.linear_acceleration.x = accel_16[0];
  imu_msg.linear_acceleration.y = accel_16[1];
  imu_msg.linear_acceleration.z = accel_16[2];

  while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0);

  imu_msg.orientation.x = pitch;
  imu_msg.orientation.y = roll;
  imu_msg.orientation.z = yaw;

  imu_pub.publish(&imu_msg);
  nh.spinOnce();
  HAL_Delay(200);
}
