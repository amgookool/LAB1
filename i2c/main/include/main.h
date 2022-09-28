#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO 2         // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO 0         // GPIO number for I2C master data

#define I2C_MASTER_NUM I2C_NUM_0    // I2C port number for master dev
#define I2C_MASTER_TX_BUF_DISABLE 0 // I2C master do not need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0 // I2C master do not need buffer

#define WRITE_BIT I2C_MASTER_WRITE // I2C master write
#define READ_BIT I2C_MASTER_READ   // I2C master read
#define ACK_CHECK_EN 0x1           // I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0          // I2C master will not check ack from slave
#define ACK_VAL 0x0                // I2C ack value
#define NACK_VAL 0x1               // I2C nack value
#define LAST_NACK_VAL 0x2          // I2C last_nack value

// Definitions for ADS1115 ADDR Pin possible address
#define ADS1115_ADDR_GND 0x48
#define ADS11115_ADDR_VDD 0x49
#define ADS1115_ADDR_SDA 0x4A
#define ADS1115_ADDR_SCL 0x4B

// Definitions for Conv, Config, Lo_Thresh, High_Thresh Register Addressess
#define ADS1115_CONV_REG 0x00
#define ADS1115_CONFIG_REG 0x01
#define ADS1115_LOTHRESH_REG 0x02
#define ADS1115_HITHRESH_REG 0x03

typedef struct config_fields
{
    uint8_t OS;             // Operational Status 1-bit 
    uint8_t MUX;            // 3-bits
    uint8_t PGA;            // 3-bits
    uint8_t MODE;           // 1-bit
    uint8_t DR;             // Data Rate 3-bits
    uint8_t COMP_MODE;      // Comparator Mode 1-bit
    uint8_t COMP_POL;       // Comparator Polarity 1-bit
    uint8_t COMP_LAT;       // Latching Comparator 1-bit
    uint8_t COMP_QUE;       // Comparator Queue and Disable 2-bits
    uint16_t configuration; // 16-bit configuration of all above fields 
} ADS1115_CONFIG_FIELDS;

// Function Prototypes
static void get_16bit_config(ADS1115_CONFIG_FIELDS* config);

static esp_err_t ads1115_write_bytes(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data, uint16_t data_len);

static esp_err_t ads1115_write_data(i2c_port_t i2c_num, uint8_t reg_addr, uint16_t data);

static esp_err_t ads1115_read_bytes(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data, uint16_t data_len);

static esp_err_t ads1115_read_data(i2c_port_t i2c_num, uint8_t reg_addr, uint16_t *data);

#endif