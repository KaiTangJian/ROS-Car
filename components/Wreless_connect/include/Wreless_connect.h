#ifndef __Wreless_connect_H__
#define __Wreless_connect_H__

#include "mpu6050.h"
/**
 * @brief MPU6050
 * 
 */
typedef struct {
    mpu6050_handle_t sensor;            ///< MPU6050传感器句柄
    mpu6050_raw_acce_value_t acce_cal;  ///< 加速度校准值
    mpu6050_raw_gyro_value_t gyro_cal;  ///< 陀螺仪校准值
    mpu6050_raw_acce_value_t curr_acce; ///< 当前加速度读数
    mpu6050_raw_gyro_value_t curr_gyro; ///< 当前陀螺仪读数
}mpu6050_context_t;

void func();

#endif
