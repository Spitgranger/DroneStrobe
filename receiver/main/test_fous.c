/* LEDC (LED Controller) basic example
    GPIO 4 = LED
    GPIO 23 = Button
    GPIO 15 = BUTTOn
    GPIO 22 = Toggle switch
*/
// TODO Debounce buttons, fix logic when momentary state is changed, implement rolling code decryption
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
#include "esp_mac.h"
#include "esp_wifi_types.h"
#include "mbedtls/aes.h"
#include "lora.h"
#include "esp_mac.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#define ESP_INR_FLAG_DEFAULT 0
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO (10) // Define the output GPIO
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (255)                // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_DUTY_15 (32)
#define LEDC_DUTY_35 (128)
#define LEDC_DUTY_0 0
#define LEDC_FREQUENCY (800) // Frequency in Hertz. Set frequency at 4 kHz
#define BUTTON1 15
#define BUTTON2 23
#define TOGGLE 22
#define ESP_MAXDELAY 512
#define ESP_QUEUE_SIZE 10

TaskHandle_t listener_handle;
TaskHandle_t lora_receiver_handle;
// Instead of blocking the cpu on the received callback, send the data to a queue to be processed by a lower priority task
static QueueHandle_t s_example_espnow_queue;
static unsigned char paired_mac[6];

typedef struct Data_t
{
    uint8_t current_state;
    bool momentary_state;
    bool on_state;
    bool strobe_state;
} generic_data_t;

typedef struct received_t
{
    bool button_one_state;
    bool button_two_state;
    bool toggle_state;
    unsigned char mac[6];
    char pairing_key[6]
} received_data_t;

static generic_data_t TEST_DATA = {0, false, false, false, false, false, false};

static const char *PMK_KEY = "789eebEkksXswqwe";
static const char *LMK_KEY = "36ddee7ae14htdi6";
static const char *TAG = "Prototype 1 receiever";

void listener(void *xStruct);
void print_mac(const unsigned char *mac);

static void received_data_processor(void *pvParameter)
{
    received_data_t evt;
    generic_data_t *data = (generic_data_t *)pvParameter;
    // Loop to process receieved data and update the programs state
    while (1)
    {
        // If there are messages in the queue, processes them else yield the CPU back to the scheduler
        while (xQueueReceive(s_example_espnow_queue, &evt, ESP_MAXDELAY) == pdTRUE)
        {
            if (memcmp(evt.mac, paired_mac, sizeof(evt.mac)) != 0)
            {
                ESP_LOGI(pcTaskGetName(NULL), "Invalid mac or unpaired device, message not read");
                vTaskDelay(100 / portTICK_PERIOD_MS);
                continue;
            }

            if (evt.button_one_state == 1 && evt.button_two_state == 1)
            {
                data->strobe_state = true;
            }
            else if (evt.button_one_state == 0 && evt.button_two_state == 1 && evt.toggle_state == 1)
            {
                data->on_state = true;
                data->momentary_state = true;
            }
            else if (evt.button_one_state == 0 && evt.button_two_state == 0 && evt.toggle_state == 1)
            {
                data->momentary_state = true;
                data->on_state = false;
                data->strobe_state = false;
            }
            else if (evt.button_one_state == 1 && evt.button_two_state == 0 && evt.toggle_state == 0)
            {
                data->on_state = true;
                data->momentary_state = false;
                ++data->current_state;
            }
            else if (evt.button_one_state == 1)
            {
                ++data->current_state;
            }
            else if (evt.button_one_state == 0 && evt.button_two_state == 0 && evt.toggle_state == 0)
            {
                data->on_state = true;
                data->strobe_state = false;
            }
            else
            { // If the input is not valid, ignore it
                vTaskResume(listener_handle);
                continue;
            }
            vTaskResume(listener_handle);
        }
        // Else if there are no messages in the queue, yield the CPU back to the scheduler and wait 100 for next message / connection to be reestablished
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
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
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 4 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void lora_task_receiver(void *pvParameter)
{
    uint64_t messages_received = 0;
    ESP_LOGI(pcTaskGetName(NULL), "Start receiving data");
    uint8_t buf[sizeof(received_data_t)]; // Maximum Payload size of SX1276/77/78/79 is 255
    while (1)
    {
        lora_receive(); // put into receive mode
        if (lora_received())
        {
            int rxLen = lora_receive_packet(buf, sizeof(received_data_t));
            int rssi = lora_packet_rssi();
            ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received:[%.*s] at %ddbm", rxLen, rxLen, buf, rssi);
            received_data_t *evt = (received_data_t *)buf;
            if (messages_received == 0)
            {
                ESP_LOGI(pcTaskGetName(NULL), "Paired with device: [%.2X:%.2X:%.2X:%.2X:%.2X:%.2X]", evt->mac[0], evt->mac[1], evt->mac[2], evt->mac[3], evt->mac[4], evt->mac[5]);
                memcpy(paired_mac, evt->mac, sizeof(evt->mac));
            }
            ++messages_received;
            if (xQueueSend(s_example_espnow_queue, evt, ESP_MAXDELAY) != pdTRUE)
            {
                ESP_LOGE(TAG, "Failed to send data to queue");
            }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS); // Avoid WatchDog alerts, receieve data every 10ms
    }
}

void print_mac(const unsigned char *mac)
{
    printf("%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void app_main(void)
{
    // Initialize led pwm channel settings
    example_ledc_init();

    /*
    Initialize LoRA here
    */
    if (lora_init() == 0)
    {
        ESP_LOGE(pcTaskGetName(NULL), "Does not recognize the module");
        while (1)
        {
            vTaskDelay(1);
        }
    }
    ESP_LOGI(pcTaskGetName(NULL), "Frequency is 915MHz");
    lora_set_frequency(915e6); // 915MHz

    lora_enable_crc();

    int cr = 1;
    int bw = 7;
    int sf = 7;

    lora_set_coding_rate(cr);
    // lora_set_coding_rate(CONFIG_CODING_RATE);
    // cr = lora_get_coding_rate();
    ESP_LOGI(pcTaskGetName(NULL), "coding_rate=%d", cr);

    lora_set_bandwidth(bw);
    // lora_set_bandwidth(CONFIG_BANDWIDTH);
    // int bw = lora_get_bandwidth();
    ESP_LOGI(pcTaskGetName(NULL), "bandwidth=%d", bw);

    lora_set_spreading_factor(sf);
    // lora_set_spreading_factor(CONFIG_SF_RATE);
    // int sf = lora_get_spreading_factor();
    ESP_LOGI(pcTaskGetName(NULL), "spreading_factor=%d", sf);

    s_example_espnow_queue = xQueueCreate(ESP_QUEUE_SIZE, sizeof(received_data_t));
    if (s_example_espnow_queue == NULL)
    {
        ESP_LOGE(TAG, "Create mutex fail");
    }
    xTaskCreate(listener, "Listener", 4000, (void *)&TEST_DATA, 2, &listener_handle);
    xTaskCreate(&lora_task_receiver, "RX", 1024 * 3, (void *)&TEST_DATA, 5, &lora_receiver_handle);
    xTaskCreate(received_data_processor, "example_espnow_task", 2048, (void *)&TEST_DATA, 4, NULL);
}

// Add the control for the toggle switch to momentary;

void listener(void *xStruct)
{
    generic_data_t *data = (generic_data_t *)xStruct;
    while (1)
    {
        // ESP_LOGI(pcTaskGetName(NULL), "Start processing");
        //  Handle hardware logic updates according to the state set by the data processor
        if (data->strobe_state == 1)
        {
            while (data->strobe_state == 1)
            {
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
                // Update duty to apply the new value
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                // Set to 0, essentially turning LED off
                vTaskDelay(50 / portTICK_PERIOD_MS);
                vTaskResume(lora_receiver_handle);
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_0));
                // Update duty to apply the new value
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                vTaskDelay(50 / portTICK_PERIOD_MS);
                vTaskResume(lora_receiver_handle);
            }
        }
        else if (data->on_state == 0)
        {
            // Set to 0, essentially turning LED off
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_0));
            // Update duty to apply the new value
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        }
        else if (data->on_state == 1)
        {
            switch (data->current_state % 3)
            {
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