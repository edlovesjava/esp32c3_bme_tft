// include/Config.h
#pragma once

// I2C Configuration
#define I2C_SDA_PIN         8
#define I2C_SCL_PIN         9
#define I2C_FREQUENCY       100000

// SPI/Display Configuration
#define TFT_SCLK            4
#define TFT_MOSI            6
#define TFT_CS              7
#define TFT_DC              5
#define TFT_RST             10
#define TFT_BL              3

// Display Settings
#define DISPLAY_WIDTH       160
#define DISPLAY_HEIGHT      80
#define DISPLAY_ROTATION    1

// System Configuration
#define SENSOR_POLL_INTERVAL_MS  2000
#define HISTORY_SIZE             50
#define MAX_DISPLAY_VALUES       6
#define MAX_SENSORS              4
