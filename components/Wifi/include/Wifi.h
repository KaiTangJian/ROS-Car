#ifndef __Wifi__H
#define __Wifi__H

#include "esp_wifi.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "lwip/ip4_addr.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include "nvs.h"


#define WIF_MAX_AP_CONNECTION       2               ///< AP模式最大连接设备数
#define DEFAULT_WIFI_SSID           "Xiaomi"   ///< 默认连接的WiFi网络名称
#define DEFAULT_WIFI_PASSWORD       "Aa18928337280"   ///< 默认WiFi网络密码
#define EXAMPLE_ESP_MAXIMUM_RETRY   10              ///< STA模式最大重连次数
#define DEFAULT_WIFI_AP_PSWD        "87654321"      ///< AP模式热点密码
#define DEFAULT_WIFI_AP_SSID_PREFIX "micu_ros"      ///< AP模式热点名称前缀

/**
 * @brief WiFi连接状态枚举类型
 * 
 * 描述ESP32 WiFi子系统的当前连接状态
 */
typedef enum
{
    WIFI_STATUS_STA_DISCONECTED = 0,    ///< STA模式未连接或连接丢失
    WIFI_STATUS_STA_CONNECTED = 1,      ///< STA模式已连接并获取IP地址
    WIFI_STATUS_AP_READY,               ///< AP模式就绪，可接受设备连接
} wifi_status_t;

void func(void);
#endif
