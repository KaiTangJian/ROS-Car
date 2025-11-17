# ESP32-S3 IMU传感器与OLED显示项目

这是一个基于ESP32-S3的嵌入式项目，集成了MPU6050六轴IMU传感器和SSD1306 OLED显示屏，支持实时姿态解算和图形化显示。项目采用模块化组件架构，提供了I2C通信抽象层、多种AHRS姿态解算算法和完整的显示驱动。

## 📋 目录

- [项目特性](#-项目特性)
- [硬件配置](#-硬件配置)
- [项目结构](#-项目结构)
- [快速开始](#-快速开始)
- [组件说明](#-组件说明)
- [API参考](#-api参考)
- [开发指南](#-开发指南)
- [技术栈](#-技术栈)
- [许可证](#-许可证)

## 🚀 项目特性

### 核心功能
- ✅ **MPU6050 IMU传感器** - 三轴加速度计和三轴陀螺仪数据采集
- ✅ **SSD1306 OLED显示** - 128×64像素图形显示，支持ASCII字符
- ✅ **实时姿态解算** - 集成Madgwick和Mahony AHRS算法
- ✅ **I2C通信协议** - 自定义抽象层，简化硬件通信
- ✅ **FreeRTOS多任务** - 并行处理传感器数据和显示更新
- ✅ **模块化架构** - 高度解耦的组件设计，易于扩展

### 传感器特性
- 🎯 加速度计：±2g/±4g/±8g/±16g 可配置量程
- 🎯 陀螺仪：±250/±500/±1000/±2000 °/s 可配置量程
- 🎯 姿态解算：实时四元数输出（Roll/Pitch/Yaw）
- 🎯 数据更新率：最高1000Hz（可配置）

### 显示特性
- 📟 分辨率：128×64 像素
- 📟 接口：I2C（默认地址：0x3C）
- 📟 字体：内置ASCII字库（5×7, 8×16等）
- 📟 刷新率：可配置（默认10Hz）

## ⚙️ 硬件配置

### 主控芯片
- **芯片**: ESP32-S3
- **架构**: Xtensa 32位LX7双核
- **频率**: 240MHz (最高)
- **内存**: 512KB SRAM, 8MB PSRAM
- **存储**: 4MB Flash

### 引脚分配

#### I2C总线
| 功能 | GPIO引脚 | 说明 |
|------|----------|------|
| SDA  | GPIO 41  | 数据线 |
| SCL  | GPIO 42  | 时钟线 |

#### 预留引脚（扩展用途）
| 引脚 | 功能 | 状态 |
|------|------|------|
| GPIO 1 | UART TX | 可用 |
| GPIO 2 | UART RX | 可用 |
| GPIO 3 | PWM输出 | 可用 |
| GPIO 4 | ADC输入 | 可用 |
| GPIO 21 | I2C_SDA_ALT | 可用 |
| GPIO 48 | I2C_SCL_ALT | 可用 |

### 硬件连接
```
ESP32-S3          MPU6050
--------          -------
GPIO 41  <------> SDA
GPIO 42  <------> SCL
3.3V    <------> VCC
GND     <------> GND
```

```
ESP32-S3          SSD1306 OLED
--------          --------------
GPIO 41  <------> SDA
GPIO 42  <------> SCL
3.3V    <------> VCC
GND     <------> GND
```

## 📁 项目结构

```
├── main/                        # 主应用代码
│   ├── main.c                   # 应用入口和任务管理
│   ├── CMakeLists.txt           # 主组件构建配置
│   └── idf_component.yml        # 组件依赖声明
│
├── components/                  # 自定义组件
│   ├── MPU6050_APP/            # MPU6050应用层封装
│   │   ├── MPU6050_APP.c
│   │   ├── include/
│   │   └── CMakeLists.txt
│   │
│   ├── Oled/                   # OLED显示驱动
│   │   ├── Oled.c              # 核心驱动代码
│   │   ├── Oled.h              # 公共接口
│   │   ├── Oledfonts.h         # 字体定义
│   │   ├── include/
│   │   └── CMakeLists.txt
│   │
│   ├── Wreless_connect/        # 无线连接模块（开发中）
│   │   ├── Wreless_connect.c
│   │   ├── include/
│   │   └── CMakeLists.txt
│   │
│   └── lib/                    # 底层库组件
│       ├── esp32_i2c_rw/       # I2C通信抽象层
│       │   ├── esp32_i2c_rw.c
│       │   ├── esp32_i2c_rw.h
│       │   └── README.md
│       │
│       ├── imu_ahrs/           # 姿态解算算法
│       │   ├── MadgwickAHRS.c  # Madgwick滤波算法
│       │   ├── MadgwickAHRS.h
│       │   ├── MahonyAHRS.c    # Mahony滤波算法
│       │   ├── MahonyAHRS.h
│       │   └── CMakeLists.txt
│       │
│       ├── mcu_dmp/            # DMP运动处理器
│       │   └── mcu_dmp.h
│       │
│       ├── rotary_encoder/     # 旋转编码器支持
│       │   └── rotary_encoder.h
│       │
│       └── WP_Math/            # 数学工具库
│           ├── WP_Math.c
│           ├── WP_Math.h
│           ├── WP_DataType.h
│           └── CMakeLists.txt
│
├── managed_components/          # 托管组件（通过idf_component.yml管理）
│   ├── espressif__mdns/        # mDNS服务发现 (v1.9.0)
│   └── espressif__mpu6050/     # MPU6050官方驱动 (v1.2.0)
│
├── build/                      # 构建输出目录
│   ├── bootloader/
│   ├── template-app.bin        # 编译固件
│   └── elf_utils/
│
├── ROS-Car/                    # ROS机器人车集成（子模块）
├── .vscode/                    # VSCode配置
│   ├── settings.json           # 编辑器设置
│   ├── launch.json             # 调试配置
│   └── c_cpp_properties.json   # C/C++扩展配置
│
├── sdkconfig                   # ESP-IDF配置
├── sdkconfig.old               # 配置备份
├── dependencies.lock           # 依赖版本锁定
├── CMakeLists.txt              # 根级CMake配置
└── README.md                   # 项目说明文档
```

## 🏁 快速开始

### 前置要求
- ESP-IDF v5.1.6 或更高版本
- Python 3.8+
- CMake 3.5+
- Ninja 或 Make
- ESP32-S3 开发板
- MPU6050 传感器模块
- SSD1306 OLED显示屏

### 安装ESP-IDF

```bash
# 克隆ESP-IDF仓库
git clone https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.1.6
git submodule update --init --recursive

# 设置环境变量（Linux/macOS）
source export.sh

# 设置环境变量（Windows PowerShell）
.\export.ps1
```

### 配置项目

```bash
# 进入项目目录
cd d:\espproject\template-app

# 设置目标芯片
idf.py set-target esp32s3

# 配置项目参数
idf.py menuconfig
```

### 编译与烧录

```bash
# 编译项目
idf.py build

# 烧录到开发板
idf.py flash

# 监控串口输出
idf.py monitor
```

### 合并命令（推荐）

```bash
# 一键编译、烧录并打开监控
idf.py flash monitor
```

### 调试

```bash
# 使用VSCode调试器
# F5启动调试，或在VSCode中使用"ESP32-S3 Debug"配置
```

## 🔧 组件说明

### 主应用组件 (main)

负责整体系统初始化和任务管理：
- 初始化I2C总线（GPIO 41/42）
- 配置MPU6050传感器
- 初始化OLED显示屏
- 创建FreeRTOS显示任务
- 定期更新姿态数据并显示

### MPU6050_APP组件

MPU6050传感器的应用层封装：
```c
// 示例：读取加速度和陀螺仪数据
void mpu6050_app_read_data(float* accel, float* gyro) {
    // 读取原始数据并转换为物理量
    // 返回加速度(g)和角速度(°/s)
}
```

**状态**: ⚠️ 开发中（当前为占位符实现）

### OLED组件

SSD1306 OLED显示驱动，提供完整的显示功能：

```c
// 初始化OLED
void OLED_Init(void);

// 清空屏幕
void OLED_Clear(void);

// 显示文本
void OLED_ShowString(uint8_t x, uint8_t y, char* str, uint8_t size);

// 显示数字
void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);

// 绘制像素点
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color);

// 绘制线条
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);
```

**字体支持**:
- 5×7 ASCII字符
- 8×16 ASCII字符
- 可扩展自定义字体

### I2C抽象层 (esp32_i2c_rw)

提供简化的I2C通信接口：

```c
// I2C初始化
esp_err_t i2c_master_init(void);

// 写入数据到设备
esp_err_t i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data);

// 从设备读取数据
esp_err_t i2c_read_bytes(uint8_t device_addr, uint8_t reg_addr, uint8_t* data, size_t len);

// 连续读取多个寄存器
esp_err_t i2c_read_continuous(uint8_t device_addr, uint8_t start_reg, uint8_t* buffer, size_t size);
```

### 姿态解算库 (imu_ahrs)

#### Madgwick算法
```c
// 初始化Madgwick滤波器
MadgwickInit(float beta);

// 更新四元数
void MadgwickUpdate(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float mx, float my, float mz);

// 获取欧拉角
void MadgwickGetEulerAngles(float* roll, float* pitch, float* yaw);
```

#### Mahony算法
```c
// 初始化Mahony滤波器
MahonyInit(float kp, float ki);

// 更新四元数
void MahonyUpdate(float gx, float gy, float gz,
                 float ax, float ay, float az,
                 float mx, float my, float mz);

// 获取四元数
void MahonyGetQuat(float* qw, float* qx, float* qy, float* qz);
```

### 无线连接模块 (Wreless_connect)

预留给WiFi/Bluetooth功能：
- WiFi STA/AP模式
- mDNS服务发现
- WebSocket通信
- OTA固件升级

**状态**: ⚠️ 开发中（当前为占位符）

## 📚 API参考

### MPU6050驱动

```c
// 初始化MPU6050
esp_err_t mpu6050_init(void);

// 配置量程
esp_err_t mpu6050_set_accel_full_scale(mpu6050_accel_fs_t fs);
esp_err_t mpu6050_set_gyro_full_scale(mpu6050_gyro_fs_t fs);

// 读取原始数据
esp_err_t mpu6050_get_accel_raw(int16_t* ax, int16_t* ay, int16_t* az);
esp_err_t mpu6050_get_gyro_raw(int16_t* gx, int16_t* gy, int16_t* gz);

// 读取缩放数据（物理量）
esp_err_t mpu6050_get_accel_data(float* ax, float* ay, float* az);
esp_err_t mpu6050_get_gyro_data(float* gx, float* gy, float* gz);

// 读取温度
esp_err_t mpu6050_get_temperature(float* temperature);
```

### OLED显示API

```c
// 初始化和清屏
void OLED_Init(void);
void OLED_Clear(void);
void OLED_Refresh(void);

// 文本显示
void OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t size);
void OLED_ShowString(uint8_t x, uint8_t y, char* str, uint8_t size);
void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);

// 图形绘制
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);
void OLED_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r, uint8_t color);

// 位图显示
void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t* bmp);
```

## 💻 开发指南

### 添加新传感器

1. 在 `components/` 下创建新组件目录
2. 实现传感器驱动（遵循ESP-IDF组件规范）
3. 在 `main/idf_component.yml` 中添加依赖
4. 在 `main.c` 中初始化并创建任务

### 自定义显示内容

编辑 `main/main.c` 中的显示任务：

```c
static void oled_display_task(void* params) {
    while(1) {
        // 读取传感器数据
        float accel[3], gyro[3];
        mpu6050_get_accel_data(&accel[0], &accel[1], &accel[2]);
        mpu6050_get_gyro_data(&gyro[0], &gyro[1], &gyro[2]);

        // 清空屏幕
        OLED_Clear();

        // 显示自定义内容
        OLED_ShowString(0, 0, "MPU6050 Test", 12);
        OLED_ShowString(0, 16, "Accel:", 12);
        OLED_ShowNumber(50, 16, (int)(accel[0]*100), 5, 12);
        // ...更多显示

        // 刷新显示
        OLED_Refresh();

        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz刷新率
    }
}
```

### 配置编译选项

在 `sdkconfig` 中可调整：
- **CONFIG_MPU6050_I2C_ADDRESS**: MPU6050 I2C地址（默认0x68）
- **CONFIG_OLED_I2C_ADDRESS**: OLED I2C地址（默认0x3C）
- **CONFIG_DISPLAY_FPS**: 显示刷新率（默认10Hz）
- **CONFIG_SENSOR_SAMPLERATE**: 传感器采样率（默认100Hz）

### 调试技巧

1. **启用详细日志**:
   ```bash
   idf.py menuconfig > Component config > Log output > Default log verbosity > Debug
   ```

2. **使用逻辑分析仪**:
   - 连接GPIO 41/42到逻辑分析仪
   - 设置I2C协议解析
   - 分析总线通信波形

3. **串口监控**:
   ```bash
   idf.py monitor --baud 115200
   ```

4. **GDB调试**:
   ```bash
   idf.py openocd
   # 在VSCode中启动调试会话
   ```

## 🔍 故障排除

### 常见问题

**Q: MPU6050初始化失败**
- 检查I2C接线是否正确
- 确认电源电压为3.3V
- 验证I2C地址（默认0x68，备用0x69）
- 串口查看错误日志：`mpu6050: probe failed`

**Q: OLED不显示内容**
- 检查I2C接线
- 验证I2C地址（默认0x3C）
- 确认电源和对比度设置
- 测试I2C通信：`i2cdetect -y 0`

**Q: 传感器数据异常**
- 检查传感器安装方向
- 验证量程配置
- 进行校准（imu_ahrs组件提供校准函数）
- 确认采样率设置合理

**Q: 编译错误**
- 确认ESP-IDF版本：v5.1.6+
- 清理构建缓存：`idf.py clean`
- 重新同步子模块：`git submodule update --init --recursive`

## 📊 技术栈

### 开发框架
- **ESP-IDF**: v5.1.6 (Espressif IoT Development Framework)
- **CMake**: 3.5+
- **FreeRTOS**: 内核任务调度

### 组件依赖
- **mpu6050**: espressif/mpu6050 v1.2.0
- **mdns**: espressif/mdns v1.9.0
- **driver**: ESP-IDF内置GPIO/I2C驱动
- **log**: ESP-IDF日志系统

### 开发工具
- **IDE**: VSCode + ESP-IDF Extension
- **编译器**: xtensa-esp32s3-elf-gcc
- **调试器**: GDB + OpenOCD
- **版本控制**: Git

### 算法库
- **Madgwick滤波器**: 实时姿态解算
- **Mahony滤波器**: 互补滤波算法
- **DMP**: MPU6050内置数字运动处理器

### 通信协议
- **I2C**: 主模式，标准模式（100kHz）/快速模式（400kHz）
- **UART**: 调试输出，115200 baud
- **SPI**: 可扩展（ESP32-S3硬件支持）

## 📈 性能指标

| 指标 | 数值 | 单位 |
|------|------|------|
| 启动时间 | <500 | ms |
| 姿态解算频率 | 100 | Hz |
| 显示刷新率 | 10 | Hz |
| I2C通信速率 | 400 | kHz |
| 功耗（活跃） | 80-120 | mA |
| 功耗（深度睡眠） | 10 | μA |

## 🛣️ 路线图

### v1.0 (当前版本) ✅
- [x] MPU6050传感器集成
- [x] SSD1306 OLED显示
- [x] I2C通信抽象层
- [x] AHRS姿态解算算法
- [x] FreeRTOS任务管理
- [x] VSCode开发环境配置

### v1.1 (开发中) 🚧
- [ ] 完成MPU6050_APP组件
- [ ] 实现WiFi连接功能
- [ ] 添加Web界面控制
- [ ] OTA固件升级支持
- [ ] 数据记录到Flash

### v1.2 (规划中) 📋
- [ ] 支持更多传感器（磁力计、气压计）
- [ ] 添加蓝牙功能
- [ ] ROS集成（通过ROS-Car子模块）
- [ ] 机器学习姿态识别
- [ ] 移动端APP控制

### v2.0 (远期规划) 🎯
- [ ] 多传感器融合
- [ ] 实时SLAM功能
- [ ] 云端数据分析
- [ ] AI辅助姿态预测
- [ ] 完整机器人控制系统

## 🤝 贡献

欢迎贡献代码！请遵循以下步骤：

1. Fork 本仓库
2. 创建特性分支：`git checkout -b feature/new-feature`
3. 提交更改：`git commit -am 'Add new feature'`
4. 推送分支：`git push origin feature/new-feature`
5. 创建Pull Request

### 提交规范
- **feat**: 新功能
- **fix**: 错误修复
- **docs**: 文档更新
- **style**: 代码格式化
- **refactor**: 代码重构
- **test**: 测试相关
- **chore**: 构建或辅助工具

## 📄 许可证
本项目采用MIT许可证。详见 [LICENSE](LICENSE) 文件。

## 🙏 致谢

- [Espressif Systems](https://www.espressif.com/) - 提供优秀的ESP-IDF框架
- [MPU6050社区](https://invensense.tdk.com/developers-center/design-files/) - 姿态解算算法参考
- 开源社区 - 各种库和工具的支持

## 📞 联系方式

- **项目地址**: [GitHub Repository]
- **问题反馈**: [GitHub Issues]
- **技术讨论**: [GitHub Discussions]

---