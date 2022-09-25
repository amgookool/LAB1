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

static void get_16bit_config(ADS1115_CONFIG_FIELDS* config)
{
    uint16_t data;

    data = (config->OS << 3) | config->MUX;
    data = (data << 3) | config->PGA;
    data = (data << 1) | config->MODE;
    data = (data << 3) | config->DR;
    data = (data << 1) | config->COMP_MODE;
    data = (data << 1) | config->COMP_POL;
    data = (data << 1) | config->COMP_LAT;
    data = (data << 2) | config->COMP_QUE;
    config->configuration = data;
}

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
    conf.clk_stretch_tick = 500; // !!!500 ticks, Clock stretch is about 300us, you can make changes according to the actual situation.
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
static esp_err_t ads1115_write_bytes(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data, uint16_t data_len)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // start condition 
    i2c_master_write_byte(cmd,(ADS1115_ADDR_GND << 1) | WRITE_BIT,ACK_CHECK_EN); // address frame + read/write bit
    i2c_master_write_byte(cmd,reg_addr,ACK_CHECK_EN); // accessing register
    i2c_master_write(cmd,data,data_len,ACK_CHECK_EN); // writing to register
    i2c_master_stop(cmd); // stop condition
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}


static esp_err_t ads1115_write_data(i2c_port_t i2c_num, uint8_t reg_addr, uint16_t data)
{
    esp_err_t ret;
    uint8_t write_buff[2];

    write_buff[0] = (data >> 8) & 0xFF;
    write_buff[1] = (data >> 0 ) & 0xFF;

    ret = ads1115_write_bytes(i2c_num, reg_addr,write_buff, 2);

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
static esp_err_t ads1115_read_bytes(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data, uint16_t data_len)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR_GND << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr,ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG,"Coul not read bytes from ADS1115!");
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR_GND << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd,data,data_len,LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    for(int i = 0; i < data_len; i++)
    {
        ESP_LOGI(TAG,"Byte:%d: %d",i,data[data_len]);
    }
    return ret;
}


static esp_err_t ads1115_read_data(i2c_port_t i2c_num, uint8_t reg_addr, uint16_t *data)
{
    esp_err_t ret;
    uint16_t sensor_data = 0;
    uint8_t read_buff[2];

    ret = ads1115_read_bytes(i2c_num,reg_addr,read_buff, 2);
    sensor_data = (read_buff[0] << 8) | read_buff[1];
    *data = sensor_data;
    return ret;
}


static esp_err_t i2c_master_ads1115_init(i2c_port_t i2c_num,ADS1115_CONFIG_FIELDS* config_fields)
{
    vTaskDelay(200 / portTICK_RATE_MS);
    i2c_master_init();

    config_fields->OS = 0x00;
    config_fields->MUX = 0x00;
    config_fields->PGA = 0x00;
    config_fields->MODE = 0x00;
    config_fields->DR = 0x04;
    config_fields->COMP_MODE = 0x00;
    config_fields->COMP_POL = 0x00;
    config_fields->COMP_LAT = 0x00;
    config_fields->COMP_QUE = 0x03;
    
    get_16bit_config(config_fields);

    // writing to config register
    ESP_ERROR_CHECK(ads1115_write_data(i2c_num,ADS1115_CONFIG_REG,config_fields->configuration));

    return ESP_OK;
}


static void i2c_read_task(void *config_params)
{
    ADS1115_CONFIG_FIELDS *config_fields;
    config_fields = (ADS1115_CONFIG_FIELDS*)config_params;

    i2c_master_ads1115_init(I2C_MASTER_NUM,config_fields);
    uint16_t sensor_data;
    esp_err_t ret;

    while(1)
    {   

        ret = (ads1115_read_data(I2C_MASTER_NUM,ADS1115_CONV_REG,&sensor_data));
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Successfully read ADS1115...\n");
            ESP_LOGI(TAG,"Sensor Data: %d", (int)&sensor_data);
            vTaskDelay(500/portTICK_PERIOD_MS);
        }
        else
        {
            ESP_LOGE(TAG, "Could not read the ADS1115\n");

        }
        config_fields->OS=0x01;
        get_16bit_config(config_fields);
        ads1115_write_data(I2C_MASTER_NUM,ADS1115_CONFIG_REG,config_fields->configuration);

        vTaskDelay(5000/portTICK_PERIOD_MS);
    }
    i2c_driver_delete(I2C_MASTER_NUM);
}

void app_main(void)
{
    ADS1115_CONFIG_FIELDS* config_fields;
    xTaskCreate(i2c_read_task, "i2c_read_task", 2048, (void *)&config_fields, 5, NULL);
   
}