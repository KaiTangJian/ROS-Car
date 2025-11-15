#ifndef __MPU6050_APP__H
#define __MPU6050_APP__H

#include <stdbool.h>
#include "driver/i2c.h"
#include "Wreless_connect.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


bool mpu6050_hardware_init(mpu6050_context_t *ctx);
void mpu6050_sample_task(void *pvParameters);
void  mpu6050_task(mpu6050_context_t *ctx);
#endif