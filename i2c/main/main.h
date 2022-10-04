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

#define I2C_MASTER_SCL_IO 2 // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO 0 // GPIO number for I2C master data

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

// Struct Definition for configuration bits for Configuration Register
typedef struct config_fields
{
    uint8_t OS;             // Operational Status (1-bit)
    uint8_t MUX;            // Input Multiplexer (3-bits)
    uint8_t PGA;            // Programmable Gain Amplifier (3-bits)
    uint8_t MODE;           // Mode (1-bit)
    uint8_t DR;             // Data Rate (3-bits)
    uint8_t COMP_MODE;      // Comparator Mode (1-bit)
    uint8_t COMP_POL;       // Comparator Polarity (1-bit)
    uint8_t COMP_LAT;       // Latching Comparator (1-bit)
    uint8_t COMP_QUE;       // Comparator Queue and Disable (2-bits)
    uint16_t configuration; // 16-bit configuration of all above fields
} ADS1115_CONFIG_FIELDS;


/**
 * @brief Function to create 16-bit data for configuration register
 *
 * Bit Order from MSB to LSB

 * Operational Status(1-bit) -> Multiplexer(3-bits) -> Prog. Gain Amp(3-bits) -> MODE(1-bit) -> DataRate(3-bits) -> Comparator Mode (1-bit)
 * --> Comparator Polarity (1-bit) -> Comparator Latch (1-bit) -> Comparator Queue(2-bit)

 * @param config - refers to the ADS1115_CONFIG_FIELDS data type
 *
 * @return
 *     - NULL
 */
static void get_16bit_config(ADS1115_CONFIG_FIELDS *config);

/**
 * @brief Function to write by byte(8-bits) to the ADS1115
 *
 * 1. send data
 * __________________________________________________________________________-_______________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | write data + ack  | stop |
 * --------|---------------------------|-------------------------|-------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_addr slave specific register address
 * @param data data to send
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
static esp_err_t ads1115_write_bytes(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data, uint16_t data_len);

/**
 * @brief Function to write to 16-bit register (Configuration) on the ADS1115
 *
 * 1. Right Shift 8-bits of 0th index on the write array buffer and AND operation with 0b11111111 to set the LSB
 * 2. Right Shift 0-bits of the 1st index on the write array buffer and AND operation with 0b11111111 to set the MSB
 *
 * @param i2c_num I2C port number
 * @param reg_addr slave specific register address
 * @param data data to send
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
static esp_err_t ads1115_write_data(i2c_port_t i2c_num, uint8_t reg_addr, uint16_t data);

/**
 * @brief Function to read by byte from ADS1115
 *
 * 1. send reg address (conversion register)
 * ______________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | stop |
 * --------|---------------------------|-------------------------|------|
 *
 * 2. read data
 * ___________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read data_len byte + ack(last nack)  | stop |
 * --------|---------------------------|--------------------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_addr slave register address
 * @param data data to read
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
static esp_err_t ads1115_read_bytes(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data, uint16_t data_len);

/**
 * @brief Function to read from 16-bit register (Conversion) on the ADS1115
 *
 * 1. Left shift 8 bits of the 0th index of the read_data array to get the MSB and OR the 1st index of the array to get the LSB
 *
 * @param i2c_num I2C port number
 * @param reg_addr slave specific register address
 * @param data data to send
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
static esp_err_t ads1115_read_data(i2c_port_t i2c_num, uint8_t reg_addr, uint16_t *data);

#endif