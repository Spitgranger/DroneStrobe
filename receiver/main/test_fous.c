/* LEDC (LED Controller) basic example
    GPIO 4 = LED
    GPIO 23 = Button
    GPIO 15 = BUTTOn
    GPIO 22 = Toggle switch
*/
// TODO Debounce buttons, fix logic when momentary state is changed
#include <stdio.h>
#include <string.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "espnow_example.h"
#include "esp_wifi_types.h"



#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ESP_INR_FLAG_DEFAULT    0
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (4) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (255) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_DUTY_15            (32)
#define LEDC_DUTY_35            (128)
#define LEDC_DUTY_0             0
#define LEDC_FREQUENCY          (800) // Frequency in Hertz. Set frequency at 4 kHz
#define BUTTON1                 15
#define BUTTON2                 23
#define TOGGLE                  22
#define ESP_MAXDELAY            512
#define TRANSMITTER_MAC         {0x40, 0x4C, 0xCA, 0x51, 0x58, 0x1C}
#define WIFI_CHANNEL            0

TaskHandle_t listener_handle;

typedef struct Data_t {
    uint8_t current_state;
    bool momentary_state;
    bool on_state;
    bool button_one_state;
    bool button_two_state;
    bool toggle_state;
    bool strobe_state;
} generic_data_t;

static generic_data_t TEST_DATA = {0, false, false, false, false, false, false};

static const char* PMK_KEY = "789eebEkksXswqwe";
static const char* LMK_KEY = "36ddee7ae14htdi6";
static const char *TAG = "Prototype 1 receiever";

static SemaphoreHandle_t bin_sem;

void listener(void *xStruct);
void momentary_listener(void *xStruct);

void IRAM_ATTR toggle_isr_handler(void *arg)
{
    
    ((generic_data_t *)arg)->momentary_state = !((generic_data_t *)arg)->momentary_state;
    //ESP_LOGI("INTERR", "INTTERRIPS");
}

void IRAM_ATTR momentary_isr_handler(void *arg)
{
    ((generic_data_t *)arg)->on_state = !((generic_data_t *)arg)->on_state;
}

static void wifi_init(void)
{
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());
    // esp_netif_create_default_wifi_sta();

    // wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    // ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    // ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    // ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR));
    // ESP_ERROR_CHECK(esp_wifi_start());
    // ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84));
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_LR) );
}

void on_data_receive(const uint8_t *mac, const uint8_t *data, int len) {
    int received_value = 0;

    if (len == sizeof(int)) {
        memcpy(&received_value, data, sizeof(int));
        ESP_LOGI(TAG, "Received value: %d", received_value);
        // Perform actions based on the received data here
        switch (received_value) {
                case 7:
                    // Case when all buttons are pressed, in the case adjust the brightness
                    TEST_DATA.strobe_state = true;
                    break;
                case 3:
                    // This is the case where the toggle and momentary button are pressed
                    TEST_DATA.on_state = true;
                    TEST_DATA.momentary_state = true;                    
                    break;
                case 1:
                    TEST_DATA.momentary_state = true;
                    TEST_DATA.on_state = false;
                    TEST_DATA.strobe_state = false;
                    TEST_DATA.button_two_state = false;
                    break;
                case 6:
                    TEST_DATA.strobe_state = true;
                    break;
                case 2:
                    // Set duty to 35%
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_35));
                    // Update duty to apply the new value
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                    break;
                case 4:
                    TEST_DATA.button_two_state = true;
                    break;
                case 8:
                    //This is the case when the brightness adjustment is pressed and the led is not on. 
                    //(Momentary toggle on and momentary switch not pressed)
                    //Advance the state in the background
                    ++TEST_DATA.current_state;
                    break;
                default:
                    TEST_DATA.button_two_state = false;
                    TEST_DATA.button_one_state = false;
                    TEST_DATA.toggle_state = false;
                    TEST_DATA.on_state = true;
                    TEST_DATA.strobe_state = false;
                    break;
        }
    }
    vTaskResume(listener_handle);
}

void espnow_receive_task(void *pvParameter) {
    uint8_t transmitter_mac[] = TRANSMITTER_MAC;

    esp_now_init();
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)PMK_KEY) );

    esp_now_peer_info_t peer_info;
    memcpy(peer_info.peer_addr, transmitter_mac, 6);
    peer_info.channel = WIFI_CHANNEL;
    for (uint8_t i = 0; i < 16; i++) {
        peer_info.lmk[i] = LMK_KEY[i];
    }
    peer_info.encrypt = true;

    if (esp_now_add_peer(&peer_info) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer");
        vTaskDelete(NULL);
    }
    esp_now_register_recv_cb(on_data_receive);
    vTaskDelete(NULL);
}

/* Warning:
 * For ESP32, ESP32S2, ESP32S3, ESP32C3, ESP32C2, ESP32C6, ESP32H2, ESP32P4 targets,
 * when LEDC_DUTY_RES selects the maximum duty resolution (i.e. value equal to SOC_LEDC_TIMER_BIT_WIDTH),
 * 100% duty cycle is not reachable (duty cannot be set to (2 ** SOC_LEDC_TIMER_BIT_WIDTH)).
 */

static void example_ledc_init(void)
    {
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
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


void app_main(void)
{
    // Configure GPIO pins
    set_gpio();
    example_ledc_init();

    // Set and install interrupt for both rising and falling edges for the toggle switch 
    gpio_set_intr_type(TOGGLE, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(ESP_INR_FLAG_DEFAULT);
    gpio_isr_handler_add(TOGGLE, toggle_isr_handler, (void *)&TEST_DATA);
    
    // Set and install interrupt service for rising and falling edges for the momentary switch
    gpio_set_intr_type(BUTTON2, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(BUTTON2, momentary_isr_handler, (void *)&TEST_DATA);

    if (gpio_get_level(TOGGLE) == 0)
    {
        TEST_DATA.momentary_state = false;
    }
    else
    {
        TEST_DATA.momentary_state = true;
    }

    xTaskCreate(listener, "Listener", 4000, (void *)&TEST_DATA, 1, &listener_handle);
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();
    xTaskCreate(&espnow_receive_task, "espnow_receive_task", 2048, NULL, 5, NULL);
    //xTaskCreate(momentary_listener, "ListenerMomentary", 3000, (void *)&TEST_DATA, 2, NULL);

    // while (1)
    // {
    //     uint8_t toggle_switch_state = gpio_get_level(TOGGLE);
    //     if (toggle_switch_state == 1){
    //         char* test = (char *)malloc(sizeof(uint8_t));
    //         itoa(toggle_switch_state, test, 10);
    //         printf("%s\n", test);
    //         free(test);
    //     }
    //     vTaskDelay(100/portTICK_PERIOD_MS);

    // }
    // uint8_t brightness_state = 0;
    // // Current board state ranges from 0 to 7
    // uint8_t current_state = 0;
    // // On startup updated duty cycle to 100%
    // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

// Add the control for the toggle switch to momentary;

void listener(void * xStruct) {
    generic_data_t *data = (generic_data_t *)xStruct;
    bool brightness_button_state = 0;
    while(1){
        //uint8_t brightness_button_state = gpio_get_level(BUTTON1);
        //vTaskDelay(100/portTICK_PERIOD_MS);
        brightness_button_state = data->button_two_state;
        // If the led is currently on and momentary state is false or the momentary button is pressed and the momentary state is true
        
        if (data->strobe_state == 1)
        {
            
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
                // Update duty to apply the new value
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                 //Set to 0, essentially turning LED off
                vTaskDelay(50/portTICK_PERIOD_MS);
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_0));
                // Update duty to apply the new value
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
            
        }
        else if (brightness_button_state == 1 /*&& (data->on_state == 1 || data->momentary_state == 0)*/) {
            //printf("%i", data->momentary_state);
            switch (data->current_state % 3) {
                case 0:
                    // Set duty to 100%
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
                    // Update duty to apply the new value
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                    break;
                case 1:
                    // Set duty to 35%
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_35));
                    // Update duty to apply the new value
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                    break;
                case 2:
                    // Set duty to 15%
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_15));
                    // Update duty to apply the new value
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                    break;
                default:
                    break;
            }
            ++data->current_state;
            // char* test = (char *)malloc(sizeof(uint8_t));
            // itoa(brightness_button_state, test, 10);
            // //printf("%s\n", test);
            // free(test);
        }
        else if (data->on_state == 0 && data->momentary_state == 1)
        {
            //Set to 0, essentially turning LED off
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_0));
            // Update duty to apply the new value
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        }
        else if (data->on_state == 1 && data->momentary_state == 1)
        {
            switch (data->current_state % 3) {
                case 0:
                    // Set duty to 100%
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
                    // Update duty to apply the new value
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                    break;
                case 1:
                    // Set duty to 35%
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_35));
                    // Update duty to apply the new value
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                    break;
                case 2:
                    // Set duty to 15%
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_15));
                    // Update duty to apply the new value
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                    break;
                default:
                    break;
            }
        }
        vTaskSuspend(NULL);
    }
}

void momentary_listener(void *xStruct)
{
    while(1)
    {
        generic_data_t *data = (generic_data_t *)xStruct;
        uint8_t toggle_switch_state = gpio_get_level(TOGGLE);
        vTaskDelay(50/portTICK_PERIOD_MS);
        if (toggle_switch_state == 1) 
        {
            data->momentary_state = true;
            printf("%i", data->momentary_state);
        }
        vTaskDelay(50/portTICK_PERIOD_MS);
    }
}