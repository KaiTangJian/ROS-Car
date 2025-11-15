idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp32_i2c_rw Oled log MPU6050_APP Wreless_connect)