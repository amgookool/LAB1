#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "main";

#define GPIO_OUTPUT_IO 2
#define GPIO_INPUT_IO 0

static xQueueHandle gpio_evt_queue = NULL;

static void gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_led_task(void *arg)
{
    uint32_t io_num;

    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            if (gpio_get_level(io_num) == 0)
            {
                ESP_LOGI(TAG, "Turning the LED on...\n");
                gpio_set_level(GPIO_OUTPUT_IO, 1);
            }
            else if (gpio_get_level(io_num) == 1)
            {
                ESP_LOGI(TAG, "Turning the LED off...\n");
                gpio_set_level(GPIO_OUTPUT_IO, 0);
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

void app_main(void)
{
    gpio_config_t io_conf;
    // OUTPUT IO Configuration
    io_conf.intr_type = GPIO_INTR_DISABLE;           // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;                 // set as output mode
    io_conf.pin_bit_mask = (1ULL << GPIO_OUTPUT_IO); // bit mask of the pin that you want to set
    io_conf.pull_down_en = 0;                        // disable pull-down mode
    io_conf.pull_up_en = 0;                          // disable pull-up mode
    gpio_config(&io_conf);                           // configure GPIO with the given settings

    // INPUT IO Configuration
    io_conf.intr_type = GPIO_INTR_NEGEDGE;          // interrupt of falling edge
    io_conf.pin_bit_mask = (1ULL << GPIO_INPUT_IO); // bit mask of the pins
    io_conf.mode = GPIO_MODE_INPUT;                 // set as input mode
    io_conf.pull_up_en = 0;                         // disable pull-up mode
    gpio_config(&io_conf);

    // change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO, GPIO_INTR_NEGEDGE);

    // create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // start gpio task
    xTaskCreate(gpio_led_task, "gpio_led_task", 2048, NULL, 10, NULL);

    // install gpio isr service
    gpio_install_isr_service(0);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO, gpio_isr_handler, (void *)GPIO_INPUT_IO);

    // remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_IO);
    // hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_IO, gpio_isr_handler, (void *)GPIO_INPUT_IO);

    int cnt = 0;

    while (1)
    {
        ESP_LOGI(TAG, "cnt: %d\n", cnt++);
        vTaskDelay(1000 / portTICK_RATE_MS);
        gpio_set_level(GPIO_OUTPUT_IO, 0);
    }
}
