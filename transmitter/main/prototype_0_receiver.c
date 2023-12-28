#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_wifi_types.h"

#define BUTTON1                 15
#define BUTTON2                 23
#define TOGGLE                  22
#define WIFI_CHANNEL            0
#define RECEIVER_MAC            {0x40, 0x4C, 0xCA, 0x51, 0x57, 0xF0}
#define ESP_INR_FLAG_DEFAULT    0

static const char *TAG = "espnow_transmitter";
static const char* PMK_KEY = "789eebEkksXswqwe";
static const char* LMK_KEY = "36ddee7ae14htdi6";

typedef struct Data_t {
    bool button_one_state;
    bool button_two_state;
    bool toggle_state;
} generic_data_t;


static generic_data_t TEST_DATA = {0, false, false};

void IRAM_ATTR brightness_isr_handler(void *arg)
{
    ((generic_data_t *)arg)->button_one_state = !((generic_data_t *)arg)->button_one_state;
}

void IRAM_ATTR momentary_isr_handler(void *arg)
{
    ((generic_data_t *)arg)->button_two_state = !((generic_data_t *)arg)->button_two_state;
}

void IRAM_ATTR momentary_toggle_isr_handler(void *arg)
{
    ((generic_data_t *)arg)->toggle_state = !((generic_data_t *)arg)->toggle_state;
}

void wifi_init() {
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());
    // esp_netif_create_default_wifi_sta();

    // wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    // ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    // ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    // // Enable the long range protocol
    // ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR) );
    // ESP_ERROR_CHECK(esp_wifi_start());
    // ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84));
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_LR) );
}

void espnow_send_task(void *pvParameter) {
    uint8_t receiver_mac[] = RECEIVER_MAC;
    generic_data_t *data = (generic_data_t *)pvParameter;

    esp_now_init();
    esp_now_set_pmk((uint8_t *)PMK_KEY);
    esp_now_peer_info_t peer_info;
    memcpy(peer_info.peer_addr, receiver_mac, 6);
    peer_info.channel = WIFI_CHANNEL;
    for (uint8_t i = 0; i < 16; i++) {
        peer_info.lmk[i] = LMK_KEY[i];
    }
    peer_info.encrypt = true;

    if (esp_now_add_peer(&peer_info) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer");
        vTaskDelete(NULL);
    }

    int sensor_value = 0;
    uint8_t brightness_button_level;
    uint8_t momentary_button_level;
    uint8_t toggle_switch_level;

    while (1) {
        brightness_button_level = gpio_get_level(BUTTON1);
        momentary_button_level = gpio_get_level(BUTTON2);
        toggle_switch_level = gpio_get_level(TOGGLE);
        if (brightness_button_level == 1 && momentary_button_level == 1 && toggle_switch_level == 1)
        {
            sensor_value = 7;
        } 
        else if(brightness_button_level == 0 && momentary_button_level == 1 && toggle_switch_level == 1)
        {
            sensor_value = 3;
        }
        else if(brightness_button_level == 0 && momentary_button_level == 0 && toggle_switch_level == 1)
        {
            sensor_value = 1;
        }
        else if(brightness_button_level == 1 && momentary_button_level == 1 && toggle_switch_level == 0)
        {
            sensor_value = 6;
        }
        else if(brightness_button_level == 0 && momentary_button_level == 1 && toggle_switch_level == 0)
        {
            sensor_value = 2;
        }
        else if(brightness_button_level == 1 && momentary_button_level == 0 && toggle_switch_level == 0)
        {
            sensor_value = 4;
        } 
        else if(brightness_button_level == 1 && momentary_button_level == 0 && toggle_switch_level == 1)
        {
            sensor_value = 8;
        }
        else {
            sensor_value = 0;
        }

        //either 0, 100, 001, 010, 110, 011, 111

        // Read sensor value or any data to be sent
        //sensor_value = 100; // Replace this with your sensor reading logic

        esp_err_t result = esp_now_send(receiver_mac, (uint8_t *)&sensor_value, sizeof(sensor_value));
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "Data sent successfully: %d", sensor_value);
        } else {
            ESP_LOGE(TAG, "Error sending data");
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); // Send data every 100 milliseconds
    }
}

static void set_gpio()
{
    gpio_reset_pin(BUTTON1);
    gpio_reset_pin(BUTTON2);
    gpio_reset_pin(TOGGLE);
    
    gpio_set_direction(BUTTON1, GPIO_MODE_INPUT);
    gpio_set_direction(BUTTON2, GPIO_MODE_INPUT);
    gpio_set_direction(TOGGLE, GPIO_MODE_INPUT);
    gpio_pullup_dis(BUTTON1);
    gpio_pulldown_en(BUTTON1);
    gpio_pullup_dis(TOGGLE);
    gpio_pulldown_en(TOGGLE);
    gpio_pullup_dis(BUTTON2);
    gpio_pulldown_en(BUTTON2);
}

void app_main() {
    set_gpio();
    // gpio_set_intr_type(TOGGLE, GPIO_INTR_ANYEDGE);
    // gpio_install_isr_service(ESP_INR_FLAG_DEFAULT);
    // gpio_isr_handler_add(TOGGLE, momentary_toggle_isr_handler, (void *)&TEST_DATA);
    
    // // Set and install interrupt service for rising and falling edges for the momentary switch
    // gpio_set_intr_type(BUTTON2, GPIO_INTR_POSEDGE);
    // gpio_isr_handler_add(BUTTON2, momentary_isr_handler, (void *)&TEST_DATA);

    // gpio_set_intr_type(BUTTON1, GPIO_INTR_POSEDGE);
    // gpio_isr_handler_add(BUTTON2, brightness_isr_handler, (void *)&TEST_DATA);

    if (gpio_get_level(TOGGLE) == 0)
    {
        TEST_DATA.toggle_state = false;
    }
    else
    {
        TEST_DATA.toggle_state = true;
    }

    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();
    xTaskCreate(&espnow_send_task, "espnow_send_task", 2048, (void *)&TEST_DATA, 5, NULL);
}