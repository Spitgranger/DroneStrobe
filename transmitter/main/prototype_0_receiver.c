// TODO implement rolling code encryption using counter
// Note full wavelength is 32,76 cm
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
#include "mbedtls/aes.h"
#include "espnow_example.h"
#include "lora.h"

#define BUTTON1 15
#define BUTTON2 23
#define TOGGLE 22
#define WIFI_CHANNEL 0
#define RECEIVER_MAC                       \
    {                                      \
        0x40, 0x4C, 0xCA, 0x51, 0x57, 0xF0 \
    }
#define ESP_INR_FLAG_DEFAULT 0

static const char *TAG = "espnow_transmitter";
static const char *PMK_KEY = "789eebEkksXswqwe";
static const char *LMK_KEY = "36ddee7ae14htdi6";
static char enc_key[32] = "12345678901234567890123456789012";
static char enc_iv[16] = "1234567890123456";

/*
generic_data_t struct is used to store the button states and the counter used for rolling code encryption
Counter between the receiever and transmitter must be synchronized in order to accept the message as valid
*/
typedef struct Data_t
{
    bool button_one_state;
    bool button_two_state;
    bool toggle_state;
    // uint64_t counter;
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

unsigned char *encrypt_any_length_string(const char *input, uint8_t *key, uint8_t *iv)
{
    int padded_input_len = 0;
    int input_len = strlen(input) + 1;
    int modulo16 = input_len % 16;
    if (input_len < 16)
    {
        padded_input_len = 16;
    }
    else
    {
        padded_input_len = (strlen(input) / 16 + 1) * 16;
    }

    char *padded_input = (char *)malloc(padded_input_len);
    if (!padded_input)
    {
        printf("Failed to allocate memory\n");
        return NULL;
    }
    memcpy(padded_input, input, strlen(input));
    uint8_t pkc5_value = (17 - modulo16);
    for (int i = strlen(input); i < padded_input_len; i++)
    {
        padded_input[i] = pkc5_value;
    }
    unsigned char *encrypt_output = (unsigned char *)malloc(padded_input_len);
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_enc(&aes, key, 256);
    mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, padded_input_len, iv,
                          (unsigned char *)padded_input, encrypt_output);
    ESP_LOG_BUFFER_HEX("cbc_encrypt", encrypt_output, padded_input_len);
    return encrypt_output;
}
void wifi_init()
{
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
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_LR));
}

/*
    FreeRTOS task that handles the button presses and toggles
    Utilizes the generic_data_t struct to store the button states and polling to update the
    states every 50 milliseconds
*/
void process_input(void *pvParameter)
{
    generic_data_t *data = (generic_data_t *)pvParameter;
    uint8_t brightness_button_level;
    uint8_t momentary_button_level;
    uint8_t toggle_switch_level;
    // either 0, 100, 001, 010, 110, 011, 111
    while (1)
    {
        brightness_button_level = gpio_get_level(BUTTON1);
        momentary_button_level = gpio_get_level(BUTTON2);
        toggle_switch_level = gpio_get_level(TOGGLE);
        data->button_one_state = brightness_button_level;
        data->button_two_state = momentary_button_level;
        data->toggle_state = toggle_switch_level;
        vTaskDelay(50 / portTICK_PERIOD_MS);
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

void lora_task_transmitter(void *pvParameter)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start");
    // uint8_t buf[256]; // Maximum Payload size of SX1276/77/78/79 is 255. We are sending 11 bytes.
    while (1)
    {
        lora_send_packet((uint8_t *)pvParameter, sizeof(generic_data_t));
        ESP_LOGI(pcTaskGetName(NULL), "%d byte packet sent...", sizeof(generic_data_t));
        int lost = lora_packet_lost();
        if (lost != 0)
        {
            ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main()
{
    set_gpio();

    if (gpio_get_level(TOGGLE) == 0)
    {
        TEST_DATA.toggle_state = false;
    }
    else
    {
        TEST_DATA.toggle_state = true;
    }

    /*Initialize LoRA here*/
    /*
    LoRa has the following three communication parameters.
    1.Signal Bandwidth (= BW)
    2.Error Cording Rate (= CR)
    3.Spreading Factor (= SF)
    The communication speed is faster when BW is large, CR is small, and SF is small.
    However, as the communication speed increases, the reception sensitivity deteriorates, need to select the parameters that best suit needs...
    */

    if (lora_init() == 0)
    {
        ESP_LOGE(pcTaskGetName(NULL), "Module is not recongnized");
        while (1)
        {
            vTaskDelay(1);
        }
    }
    ESP_LOGI(pcTaskGetName(NULL), "Frequency is 915MHz");
    lora_set_frequency(915e6); // 915MHz

    lora_enable_crc();

    int cr = 1; // 4/5
    int bw = 7; // 125khz
    int sf = 7; // 7:128 chips / symbol

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
    xTaskCreate(&lora_task_transmitter, "TX", 1024 * 3, (void *)&TEST_DATA, 5, NULL);

    xTaskCreate(&process_input, "process_input", 2048, (void *)&TEST_DATA, 6, NULL);
}