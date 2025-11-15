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

/**
 * @brief 全局上下文包结构体
 * 
 * 整合所有硬件模块的上下文信息和机器人物理参数
 */
typedef struct {
    //motor_control_context_t mc;     ///< 电机控制上下文
    mpu6050_context_t mpu6050;      ///< MPU6050传感器上下文
    float wheel_space;              ///< 轮距（mm）
    float wheel_perimeter;          ///< 轮子周长（mm）
} context_pack_t;

#endif
