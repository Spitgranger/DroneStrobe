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
#include "lora.h"
#include "esp_mac.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#define BUTTON1 4
#define BUTTON2 5
#define TOGGLE 6
#define PAIRING_LED 1
#define ESP_INTR_FLAG_DEFAULT 0

static const char *TAG = "espnow_transmitter";
static const char *PAIRING_KEY = "789eebEkksXswqwe";
// static const char *LMK_KEY = "36ddee7ae14htdi6";
// static char enc_key[32] = "12345678901234567890123456789012";
// static char enc_iv[16] = "1234567890123456";

TaskHandle_t transmitter_task_handle;
TaskHandle_t input_processor_task_handle;
TaskHandle_t pairing_task_handle;
TaskHandle_t blink_pairing_led_task_handle;
TaskHandle_t heartbeat_receiver_task_handle;

SemaphoreHandle_t xSemaphore = NULL;

static QueueHandle_t event_action_queue = NULL;

/*
generic_data_t struct is used to store the button states and the counter used for rolling code encryption
Counter between the receiever and transmitter must be synchronized in order to accept the message as valid
*/
typedef struct Data_t
{
    bool button_one_state;
    bool button_two_state;
    bool toggle_state;
    unsigned char mac[6];
    // char pairing_key[6]
    // uint64_t counter;
} generic_data_t;

typedef struct Pairing_Data_t
{
    unsigned char mac[6];
    char pairing_key[17];
} pairing_data_t;

typedef struct Heartbeat_Data_t
{
    int rssi;
} heartbeat_data_t;

static generic_data_t TEST_DATA = {false, false, false, {0}};
static pairing_data_t PAIRING_DATA = {{0}, "789eebEkksXswqwe"};
static bool paired = false;

void IRAM_ATTR brightness_isr_handler(void *arg)
{
    // ((generic_data_t *)arg)->button_one_state = !((generic_data_t *)arg)->button_one_state;
    uint8_t event = 1;
    xQueueSendFromISR(event_action_queue, &event, NULL);
    xTaskResumeFromISR(input_processor_task_handle);
}

void IRAM_ATTR momentary_isr_handler(void *arg)
{
    //((generic_data_t *)arg)->button_two_state = !((generic_data_t *)arg)->button_two_state;
    uint8_t event = 2;
    xQueueSendFromISR(event_action_queue, &event, NULL);
    xTaskResumeFromISR(input_processor_task_handle);
}

void IRAM_ATTR momentary_toggle_isr_handler(void *arg)
{
    //((generic_data_t *)arg)->toggle_state = !((generic_data_t *)arg)->toggle_state;
    uint8_t event = 3;
    xQueueSendFromISR(event_action_queue, &event, NULL);
    xTaskResumeFromISR(input_processor_task_handle);
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
/* WIFI init function. Not currently used but may be useful in the future
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
*/

/*
    FreeRTOS task that handles the button presses and toggles
    Utilizes the generic_data_t struct to store the button states and polling to update the
    states every 50 milliseconds
*/
// void process_input(void *pvParameter)
// {
//     generic_data_t *data = (generic_data_t *)pvParameter;
//     uint8_t brightness_button_level;
//     uint8_t momentary_button_level;
//     uint8_t toggle_switch_level;
//     unsigned long pressed_time = 0;
//     unsigned long released_time = 0;
//     uint8_t last_pressed = 0;
//     uint8_t last_toggle = 0;
//     uint8_t last_momentary = 0;
//     uint8_t last_brightness = 0;
//     uint32_t time_difference = 0;
//     bool send = false;
//     // either 0, 100, 001, 010, 110, 011, 111
//     while (1)
//     {
//         brightness_button_level = gpio_get_level(BUTTON1);
//         momentary_button_level = gpio_get_level(BUTTON2);
//         toggle_switch_level = gpio_get_level(TOGGLE);

//         if (brightness_button_level != last_brightness || momentary_button_level != last_momentary || toggle_switch_level != last_toggle)
//         {
//             if (brightness_button_level)
//             {
//                 ESP_LOGI(pcTaskGetName(NULL), "Button 1 pressed");
//             }
//             if (momentary_button_level)
//             {
//                 ESP_LOGI(pcTaskGetName(NULL), "Button 2 pressed");
//             }
//             if (toggle_switch_level)
//             {
//                 ESP_LOGI(pcTaskGetName(NULL), "Toggle switch pressed");
//             }
//             last_brightness = brightness_button_level;
//             last_momentary = momentary_button_level;
//             last_toggle = toggle_switch_level;
//             send = true;
//         }
//         if (last_pressed == 0 && brightness_button_level == 1)
//         {
//             pressed_time = esp_timer_get_time();
//         }
//         else if (last_pressed == 1 && brightness_button_level == 0)
//         {
//             released_time = esp_timer_get_time();
//             time_difference = released_time - pressed_time;
//             if (time_difference / 1000ULL > 3000 && !paired)
//             {
//                 // Momentary press, enter pairing mode
//                 ESP_LOGI(pcTaskGetName(NULL), "Pairing mode activated");
//                 vTaskSuspend(transmitter_task_handle);
//                 vTaskResume(pairing_task_handle);
//                 vTaskSuspend(input_processor_task_handle);
//             }
//         }
//         last_pressed = brightness_button_level;
//         if (send)
//         {
//             data->button_one_state = brightness_button_level;
//             data->button_two_state = momentary_button_level;
//             data->toggle_state = toggle_switch_level;
//             vTaskResume(transmitter_task_handle);
//             send = false;
//         }
//         vTaskDelay(80 / portTICK_PERIOD_MS);
//     }
// }

void process_input(void *pvParameter)
{
    generic_data_t *data = (generic_data_t *)pvParameter;
    bool brightness_button_level = 0;
    uint8_t momentary_button_level;
    uint8_t toggle_switch_level;
    unsigned long pressed_time = 0;
    unsigned long released_time = 0;
    uint8_t last_pressed = 0;
    uint8_t last_toggle = 0;
    uint8_t last_momentary = 0;
    uint8_t current_pressed = 0;
    uint8_t current_toggle = 0;
    uint8_t current_momentary = 0;
    uint8_t evt;
    bool send = false;
    // either 0, 100, 001, 010, 110, 011, 111
    while (1)
    {
        vTaskSuspend(heartbeat_receiver_task_handle);
        while (xQueueReceive(event_action_queue, &evt, 512))
        {
            switch (evt)
            {
            case 1:
                ESP_LOGI(pcTaskGetName(NULL), "Button 1 pressed");
                data->button_one_state = !data->button_one_state;
                send = true;
                break;
            case 2:
                ESP_LOGI(pcTaskGetName(NULL), "Button 2 pressed");
                data->button_two_state = !data->button_two_state;
                brightness_button_level = !brightness_button_level;
                if (last_pressed == 0 && brightness_button_level == 1)
                {
                    pressed_time = esp_timer_get_time();
                }
                else if (last_pressed == 1 && brightness_button_level == 0)
                {
                    released_time = esp_timer_get_time();
                    uint32_t time_difference = released_time - pressed_time;
                    if (time_difference / 1000ULL > 3000 && !paired)
                    {
                        // Momentary press, enter pairing mode
                        ESP_LOGI(pcTaskGetName(NULL), "Pairing mode activated");
                        vTaskSuspend(transmitter_task_handle);
                        vTaskResume(pairing_task_handle);
                        vTaskSuspend(input_processor_task_handle);
                    }
                }
                last_pressed = brightness_button_level;
                send = true;
                break;
            case 3:
                ESP_LOGI(pcTaskGetName(NULL), "Toggle switch pressed");
                data->toggle_state = !data->toggle_state;
                send = true;
                break;
            default:
                break;
            }
            if (send)
            {
                vTaskResume(transmitter_task_handle);
                send = false;
            }
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
        vTaskResume(heartbeat_receiver_task_handle);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/*
    Function that initializes the GPIO pins for the buttons and toggle switch
    Sets the direction of the pins to input and enables the pull down resistors
*/
static void set_gpio()
{
    gpio_reset_pin(BUTTON1);
    gpio_reset_pin(BUTTON2);
    gpio_reset_pin(TOGGLE);
    gpio_reset_pin(PAIRING_LED);

    gpio_set_direction(BUTTON1, GPIO_MODE_INPUT);
    gpio_set_direction(BUTTON2, GPIO_MODE_INPUT);
    gpio_set_direction(TOGGLE, GPIO_MODE_INPUT);
    gpio_set_direction(PAIRING_LED, GPIO_MODE_OUTPUT);
    gpio_pullup_dis(BUTTON1);
    gpio_pulldown_en(BUTTON1);
    gpio_pullup_dis(TOGGLE);
    gpio_pulldown_en(TOGGLE);
    gpio_pullup_dis(BUTTON2);
    gpio_pulldown_en(BUTTON2);
}

void lora_task_transmitter(void *pvParameter)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start transmitting button state packets...");
    // uint8_t buf[256]; // Maximum Payload size of SX1276/77/78/79 is 255. We are sending 11 bytes.
    while (1)
    {
        lora_send_packet((uint8_t *)pvParameter, sizeof(generic_data_t));
        // ESP_LOGI(pcTaskGetName(NULL), "%d byte packet sent...", sizeof(generic_data_t));
        int lost = lora_packet_lost();
        if (lost != 0)
        {
            ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
        }
        // Since the we cannot send and receive at the same time, need to listen for heartbeats immediately after sending
        // lora_receive();
        // if (lora_received())
        // {
        //     int rxLen = lora_receive_packet(buf, sizeof(generic_data_t));
        //     // ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received:[%.*s] at %ddbm", rxLen, rxLen, buf, rssi);
        //     heartbeat_data_t *evt = (heartbeat_data_t *)buf;
        //     if (xQueueSend(heartbeat_queue, evt, 512) != pdTRUE)
        //     {
        //         ESP_LOGE(pcTaskGetName(NULL), "Failed to send heartbeat data to queue");
        //     }
        // }
        vTaskSuspend(NULL);
    }
}

void pairing_task(void *pvParameter)
{
    vTaskSuspend(NULL);
    ESP_LOGI(pcTaskGetName(NULL), "Start pairing...");
    vTaskResume(blink_pairing_led_task_handle);
    uint8_t buf[sizeof(pairing_data_t)];
    bool received = false;
    while (!paired)
    {
        lora_send_packet((uint8_t *)&PAIRING_DATA, sizeof(pairing_data_t));
        int lost = lora_packet_lost();
        if (lost != 0)
        {
            ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
        }
        lora_receive();
        if (lora_received())
        {
            int rxLen = lora_receive_packet(buf, sizeof(pairing_data_t));
            int rssi = lora_packet_rssi();
            ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received:[%.*s] at %ddbm", rxLen, rxLen, buf, rssi);
            pairing_data_t *evt = (pairing_data_t *)buf;
            if (memcmp(evt->pairing_key, PAIRING_KEY, sizeof(evt->pairing_key)) == 0)
            {
                ESP_LOGI(pcTaskGetName(NULL), "Paired with %02x:%02x:%02x:%02x:%02x:%02x",
                         evt->mac[0], evt->mac[1], evt->mac[2], evt->mac[3], evt->mac[4], evt->mac[5]);
                paired = true;
                break;
            }
        }
        else
        {
            ESP_LOGI(pcTaskGetName(NULL), "Pairing Acknolwedgement not received");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    vTaskResume(transmitter_task_handle);
    vTaskResume(input_processor_task_handle);
    vTaskResume(heartbeat_receiver_task_handle);
    vTaskDelete(NULL);
}

void blink_pairing_led_task()
{
    vTaskSuspend(NULL);
    while (1)
    {
        if (paired)
        {
            gpio_set_level(PAIRING_LED, 1);
            break;
        }
        gpio_set_level(PAIRING_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(PAIRING_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}

static void lora_module_setup()
{
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
}
/*
Function that handles that handles the receiving of heartbeat packets, which send the state of the battery.
This task is suspended when the transmitter is sending data and resumed when the transmitter is not sending data.
The default state is to always listen for heartbeats until a button is pressed, in which case the the interrupt will suspend this task until the
data processor task resumes it.
*/
static void heartbeat_receiver_task()
{
    vTaskSuspend(NULL);
    heartbeat_data_t evt;
    uint8_t buf[sizeof(heartbeat_data_t)];
    while (1)
    {
        lora_receive(); // put into receive mode
        if (lora_received())
        {
            int rxLen = lora_receive_packet(buf, sizeof(heartbeat_data_t));
            int rssi = lora_packet_rssi();
            ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received:[%.*s] at %ddbm", rxLen, rxLen, buf, rssi);
            heartbeat_data_t *evt = (heartbeat_data_t *)&buf;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main()
{
    set_gpio();
    // Initialize the pairing led off
    gpio_set_level(PAIRING_LED, 0);
    // This initializes the test data structure with the appropriate values of the toggle on start.
    if (gpio_get_level(TOGGLE) == 0)
    {
        TEST_DATA.toggle_state = false;
    }
    else
    {
        TEST_DATA.toggle_state = true;
    }

    event_action_queue = xQueueCreate(10, sizeof(uint8_t));

    if (event_action_queue == NULL)
    {
        ESP_LOGE(pcTaskGetName(NULL), "Failed to create queue");
        return;
    }

    xSemaphore = xSemaphoreCreateBinary();

    if (xSemaphore == NULL)
    {
        // The semaphore was not created successfully.
        // Handle error here.
        ESP_LOGE(pcTaskGetName(NULL), "Failed to create semaphore");
        return;
    }

    // Get the MAC of the ESP board and store it in the test data structure
    esp_read_mac(PAIRING_DATA.mac, ESP_MAC_WIFI_STA);
    esp_read_mac(TEST_DATA.mac, ESP_MAC_WIFI_STA);

    gpio_set_intr_type(BUTTON1, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(BUTTON2, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(TOGGLE, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(BUTTON1, brightness_isr_handler, (void *)&TEST_DATA);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(BUTTON2, momentary_isr_handler, (void *)&TEST_DATA);
    gpio_isr_handler_add(TOGGLE, momentary_toggle_isr_handler, (void *)&TEST_DATA);

    // Setup lora module
    lora_module_setup();

    xTaskCreatePinnedToCore(&lora_task_transmitter, "TX", 1024 * 3, (void *)&TEST_DATA, 5, &transmitter_task_handle, 1);

    xTaskCreatePinnedToCore(&process_input, "process_input", 4096, (void *)&TEST_DATA, 6, &input_processor_task_handle, 0);

    xTaskCreatePinnedToCore(&pairing_task, "pairing_task", 2048, NULL, 7, &pairing_task_handle, 1);

    xTaskCreatePinnedToCore(&blink_pairing_led_task, "pairing_task", 1024, NULL, 7, &blink_pairing_led_task_handle, 1);
    xTaskCreatePinnedToCore(&heartbeat_receiver_task, "heartbeat_receiver_task", 2048, NULL, 3, &heartbeat_receiver_task_handle, 0);
}