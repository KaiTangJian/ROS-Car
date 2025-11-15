#include <stdio.h>
#include "mdns.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp32_i2c_rw.h"
#include "Oled.h"
#include "MPU6050_APP.h"
#include "Wreless_connect.h"

//系统参数配置
/**
 * @brief I2c设备配置
 * 用于MPU6050和OLED
 */
i2c_device_config_t i2c_device_config = {
    .scl_pin = 42,
    .sda_pin = 41,
    .i2c_num = 0
};
// 机械参数配置
#define MOTOR_WHEEL_SPACING        128.6    // 轮距，单位：mm
#define PLUSE_PER_ROUND            1040     // 电机转动一圈产生的脉冲数量：减速比13*20*编码器4倍频
#define MOTOR_WHEEL_CIRCLE         150.8    // 轮子周长，单位：mm (直径48mm)

// 电机驱动选择（可选TB6612FNG或DRV8833）
#define USE_TB6612_DRIVER

// PID控制器参数
#define PID_P   1.15    // 比例系数
#define PID_I   1.1     // 积分系数  
#define PID_D   0.01    // 微分系数

/**
 * @brief 全局上下文结构体
 * 包含所有硬件组件的配置和状态信息
 */
static context_pack_t ctx = {
    .wheel_space = MOTOR_WHEEL_SPACING,
    .wheel_perimeter = MOTOR_WHEEL_CIRCLE,
};


/**
 * @brief 硬件模块初始化函数
 * 1.MPU6050
 * 2。OLED显示器
 */
bool hard_ware_init(void)
{
    if(!set_i2c_device_config(&i2c_device_config))
    return false;

    if(!i2c_device_init())
    return false;

    if(!oled_init())
    return false;

    if (!mpu6050_hardware_init(&ctx.mpu6050))           // 初始化MPU6050传感器
    return false;
   
    return true;
}

void app_main(void)
{
    while(!hard_ware_init());
    printf("Start");
     oled_disp_task();
      mpu6050_task(&ctx.mpu6050);
}