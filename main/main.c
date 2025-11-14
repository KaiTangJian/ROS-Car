#include <stdio.h>
#include "mdns.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp32_i2c_rw.h"
#include "Oled.h"

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

    return true;
}

void app_main(void)
{
    while(!hard_ware_init());
    printf("Start");
    xTaskCreatePinnedToCore(oled_disp,"oled_disp",8*1024,NULL,10,NULL,0);

}