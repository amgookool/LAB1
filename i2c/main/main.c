#include "main.h"

static const char *TAG = "main";

/**
 * @brief i2c master initialization
 * Configuring the ESP01 GPIO for I2C communication
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
 * @brief Function to initialize ESP in I2C mode and write the bits in the Configuration register on the ADS1115
 * 
 * @param i2c_num 
 * @return esp_err_t 
 */
static esp_err_t i2c_master_ads1115_init(i2c_port_t i2c_num)
{
    vTaskDelay(200 / portTICK_RATE_MS);
    i2c_master_init();
    ESP_LOGI(TAG, "Starting ESP Master Initialization for ADS1115\n");

    ADS1115_CONFIG_FIELDS config_fields;
    // MSB - 0b 0 100 001 0
    // LSB - 0b 100 0 0 0 10
    config_fields.OS = 0x00;
    config_fields.MUX = 0x04;
    config_fields.PGA = 0x01;
    config_fields.MODE = 0x00;
    config_fields.DR = 0x04;
    config_fields.COMP_MODE = 0x00;
    config_fields.COMP_POL = 0x00;
    config_fields.COMP_LAT = 0x00;
    config_fields.COMP_QUE = 0x02;

    get_16bit_config(&config_fields);

    ESP_LOGI(TAG, "Config Field: %d\n", (int)config_fields.configuration);
    // writing to config register
    // uint16_t conf_data = 0b0100001010000010;
    ESP_ERROR_CHECK(ads1115_write_data(i2c_num, ADS1115_CONFIG_REG, config_fields.configuration));
    return ESP_OK;
}

/**
 * @brief Task Funnction for xTaskCreate FreeRTOS function
 * 
 * Initialize the ESP in I2C mode and starts process to continously read the ADS1115
 * @param config_params ->  NULL 
 */
static void i2c_ads1115_task(void *config_params)
{
    uint16_t sensor_data;
    esp_err_t ret;

    ret = i2c_master_ads1115_init(I2C_MASTER_NUM);
    if (ret == ESP_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Succussfully Initialized ADS1115 and I2C Protocol\n");
    }
    else
    {
        i2c_driver_delete(I2C_MASTER_NUM);
        ESP_LOGI(TAG, "Error: %s", (char *)ret);
    }
    while (1)
    {
        ret = (ads1115_read_data(I2C_MASTER_NUM, ADS1115_CONV_REG, &sensor_data));
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Successfully read ADS1115...\n");
            ESP_LOGI(TAG, "Sensor Data: %d\n", (int)sensor_data);
            vTaskDelay(3000 / portTICK_PERIOD_MS);
        }
        else
        {
            ESP_LOGI(TAG, "Could not read the ADS1115\nError: %s", (char *)ret);
            vTaskDelete(NULL);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    xTaskCreate(i2c_ads1115_task, "i2c_ads1115_task", 2048, NULL, 5, NULL);
}

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
static void get_16bit_config(ADS1115_CONFIG_FIELDS *config)
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
static esp_err_t ads1115_write_bytes(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data, uint16_t data_len)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                         // start condition
    i2c_master_write_byte(cmd, (ADS1115_ADDR_GND << 1) | WRITE_BIT, ACK_CHECK_EN); // address frame + read/write bit
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);                            // accessing register
    i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);                           // writing to register
    i2c_master_stop(cmd);                                                          // stop condition
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

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
static esp_err_t ads1115_write_data(i2c_port_t i2c_num, uint8_t reg_addr, uint16_t data)
{
    esp_err_t ret;
    uint8_t write_buff[2];

    write_buff[0] = (data >> 8) & 0xFF;
    write_buff[1] = (data >> 0) & 0xFF;

    ret = ads1115_write_bytes(i2c_num, reg_addr, write_buff, 2);

    return ret;
}

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
static esp_err_t ads1115_read_bytes(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data, uint16_t data_len)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR_GND << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "Could not read bytes from ADS1115!");
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR_GND << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    for (int i = 0; i < data_len; i++)
    {
        ESP_LOGI(TAG, "Byte:%d: %d", i, data[i]);
    }
    return ret;
}

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
static esp_err_t ads1115_read_data(i2c_port_t i2c_num, uint8_t reg_addr, uint16_t *data)
{
    esp_err_t ret;
    uint16_t sensor_data = 0;
    uint8_t read_buff[2];

    ret = ads1115_read_bytes(i2c_num, reg_addr, read_buff, 2);
    sensor_data = (read_buff[0] << 8) | read_buff[1];
    *data = sensor_data;
    return ret;
}
