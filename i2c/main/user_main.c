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

static const char *TAG = "main";

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

// ADS1115 Register Definitions
#define ADS1115_ADDR                    0x48       // 0b1001000 -> GND
#define ADS1115_CONFIG_REG              0x1        // 0b00000001
#define ADS1115_CONFIG_REG_MSB          0x84       // 0b10000100
#define ADS1115_CONFIG_REG_LSB          0x83       // 0b10000011
#define ADS1115_CONVERSION_REG          0x0        // 0b00000000

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300; // !!!300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}

/**
 * @brief test code to write ADS1115
 *
 * 1. send data
 * __________________________________________________________________________-_______________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | write data + ack  | stop |
 * --------|---------------------------|-------------------------|-------------------|------|
 *
 * @param i2c_num I2C port number
 * @param register_addr slave specific register address
 * @param data data to send
 * @param data_length data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
static esp_err_t i2c_ads1115_write(i2c_port_t i2c_num, uint8_t register_addr, uint8_t *data, size_t data_length)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                   // start condition
    i2c_master_write_byte(cmd, ADS1115_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN); // address frame + read/write bit
    i2c_master_write_byte(cmd, register_addr, ACK_CHECK_EN);                 // accessing register
    i2c_master_write(cmd, data, data_length, ACK_CHECK_EN);                  // writing data to register
    i2c_master_stop(cmd);                                                    // stop condition
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief test code to read ADS1115
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
 * @param reg_address slave register address
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
static esp_err_t i2c_ads1115_read(i2c_port_t i2c_num, uint8_t register_addr, uint8_t *data, size_t data_length)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, register_addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_length, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_ads1115_init(i2c_port_t i2c_num)
{
    uint8_t cmd_data;
    vTaskDelay(200 / portTICK_RATE_MS);
    i2c_master_init();

    // reset ADS1115 config register -> 0b00000110
    cmd_data = 0x6;
    ESP_ERROR_CHECK(i2c_ads1115_write(i2c_num, ADS1115_CONFIG_REG, &cmd_data, (uint8_t)8));

    // write to Config Register
    cmd_data = (ADS1115_CONFIG_REG_MSB << 8 | ADS1115_CONFIG_REG_LSB);
    ESP_ERROR_CHECK(i2c_ads1115_write(i2c_num, ADS1115_CONFIG_REG, &cmd_data, (u_int16_t)16));

    return ESP_OK;
}

static void i2c_task(void *arg)
{
    uint8_t sensor_data[2];
    int ret;
    i2c_master_ads1115_init(I2C_MASTER_NUM);

    while(1)
    {   
        memset(sensor_data,0,(uint8_t) 2);
        ret = i2c_ads1115_read(I2C_MASTER_NUM,ADS1115_CONVERSION_REG,&sensor_data,(uint8_t)2);
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG,"Successfully read ADS115...\n");
            double voltage = ((double)(int16_t)((sensor_data[7] << 8) | sensor_data[8]));
            ESP_LOGI(TAG,"Voltage:%f",voltage);
        }
        else 
        {
            ESP_LOGE(TAG, "No ack, sensor not connected...skip...\n");
        }
        vTaskDelay(100/portTICK_RATE_MS);
    }
    i2c_driver_delete(I2C_MASTER_NUM);
}

void app_main(void)
{
    // start i2c task
    xTaskCreate(i2c_task, "i2c_task_example", 2048, NULL, 10, NULL);
}