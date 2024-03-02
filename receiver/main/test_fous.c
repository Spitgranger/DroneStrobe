/* LEDC (LED Controller) basic example
    GPIO 11 = LED Pairing status
    GPIO 23 = Button
    GPIO 15 = BUTTOn
    GPIO 22 = Toggle switch
    GPIO 7  = ADC1 CHANNEL 6 PIN used for battery voltage measuring the full voltage 12.6 is divided by 5.
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
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_wifi_types.h"
#include "mbedtls/aes.h"
#include "lora.h"
#include "esp_mac.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#define ESP_INR_FLAG_DEFAULT 0
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO (10) // Define the output GPIO for lower power
#define LEDC_HIGH_POWER_OUTPUT (11)
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_HIGH_POWER_CHANNEL LEDC_CHANNEL_1
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT // Set duty resolution to 8 bits
#define LEDC_DUTY (255)                // Set duty to 100%. (2 ** 8) * 100% = 255
#define LEDC_DUTY_15 (32)
#define LEDC_DUTY_35 (128)
#define LEDC_DUTY_0 0
#define LEDC_FREQUENCY (800) // Frequency in Hertz. Set frequency at 800 Hz
#define BUTTON1 15
#define BUTTON2 23
#define TOGGLE 22
#define ESP_MAXDELAY 512
#define ESP_QUEUE_SIZE 10
#define BATTERY_ADC1_CHAN0 ADC_CHANNEL_6
#define ADC_ATTEN ADC_ATTEN_DB_12
#define BATTERY_MAX 12.6
#define BATTERY_LOW 10.8
#define BATTERY_MIN 9.9

TaskHandle_t listener_handle;
TaskHandle_t lora_receiver_handle;
TaskHandle_t received_data_processor_handle;
TaskHandle_t heartbeat_sender_handle;
TaskHandle_t voltage_reader_handle;
// Instead of blocking the cpu on the received callback, send the data to a queue to be processed by a lower priority task
static QueueHandle_t s_example_espnow_queue;
static unsigned char paired_mac[6];
static const char *PAIRING_KEY = "789eebEkksXswqwe";
static uint64_t last_brightness_press = 0;
static int receiver_battery_voltages[10] = {0};

// adc battery voltage monitoring
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);
static adc_oneshot_unit_handle_t adc1_handle;
static adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
};
//-------------ADC1 Config---------------//
static adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN,
};
//-------------ADC1 Calibration Init---------------//
static adc_cali_handle_t adc1_cali_chan0_handle = NULL;
static bool do_calibration1_chan0;

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
    char pairing_key[6];
} received_data_t;

typedef struct Pairing_Data_t
{
    unsigned char mac[6];
    char pairing_key[17];
} pairing_data_t;

typedef struct Heartbeat_Data_t
{
    int adc_raw;
    int voltage;
    uint8_t mac[6];
} heartbeat_data_t;

static generic_data_t TEST_DATA = {0, false, false, false};
static pairing_data_t PAIRING_DATA = {{0}, "789eebEkksXswqwe"};

// static const char *PMK_KEY = "789eebEkksXswqwe";
// static const char *LMK_KEY = "36ddee7ae14htdi6";
static const char *TAG = "Prototype 1 receiever";

void listener(void *xStruct);
void print_mac(const unsigned char *mac);
static void adc_init();

static void received_data_processor(void *pvParameter)
{
    vTaskSuspend(NULL);
    received_data_t evt;
    generic_data_t *data = (generic_data_t *)pvParameter;
    uint64_t brightness_button_time_difference = 0;
    // Loop to process receieved data and update the programs state
    while (1)
    {
        // If there are messages in the queue, processes them else yield the CPU back to the scheduler
        while (xQueueReceive(s_example_espnow_queue, &evt, ESP_MAXDELAY) == pdTRUE)
        {
            vTaskSuspend(voltage_reader_handle);
            if (memcmp(evt.mac, paired_mac, sizeof(evt.mac)) != 0)
            {
                ESP_LOGI(pcTaskGetName(NULL), "Invalid mac or unpaired device, message not read");
                vTaskDelay(10 / portTICK_PERIOD_MS);
                continue;
            }

            if (evt.button_one_state == 1 && evt.button_two_state == 1)
            {
                ESP_LOGI(pcTaskGetName(NULL), "Strobing");
                data->strobe_state = true;
            }
            else if (evt.button_one_state == 0 && evt.button_two_state == 1 && evt.toggle_state == 1)
            {
                ESP_LOGI(pcTaskGetName(NULL), "Momentary state ON LED ON");
                data->on_state = true;
                data->momentary_state = true;
            }
            else if (evt.button_one_state == 0 && evt.button_two_state == 0 && evt.toggle_state == 1)
            {
                ESP_LOGI(pcTaskGetName(NULL), "MOMENTARY STATE LED OFF");
                data->momentary_state = true;
                data->on_state = false;
                data->strobe_state = false;
            }
            else if (evt.button_one_state == 1 && evt.button_two_state == 0 && evt.toggle_state == 0)
            {
                // This timeout is needed as the transmitter sends quickly (75ms) pressing the brightness button can cause the state to change to quickly.
                brightness_button_time_difference = esp_timer_get_time() - last_brightness_press;
                if (brightness_button_time_difference / 1000ULL > 100)
                {
                    ESP_LOGI(pcTaskGetName(NULL), "STATE CHANGED BRIGHTNESS");
                    data->on_state = true;
                    data->momentary_state = false;
                    ++data->current_state;
                }
                // TODO Note if the statement below is put in the if statement above, it will change brightness every seconds instead of
                // the user needing to release the button for at least a second to change state again same below
                last_brightness_press = esp_timer_get_time();
            }
            else if (evt.button_one_state == 1)
            {
                brightness_button_time_difference = esp_timer_get_time() - last_brightness_press;
                if (brightness_button_time_difference / 1000ULL > 100)
                {
                    ESP_LOGI(pcTaskGetName(NULL), "Brigntness state changed");
                    ++data->current_state;
                }
                last_brightness_press = esp_timer_get_time();
            }
            else if (evt.button_one_state == 0 && evt.button_two_state == 0 && evt.toggle_state == 0)
            {
                ESP_LOGI(pcTaskGetName(NULL), "MOMENTARY MODE, LIGHTS ON");
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
        vTaskResume(voltage_reader_handle);
        // Else if there are no messages in the queue, yield the CPU back to the scheduler and wait 100 for next message / connection to be reestablished
        vTaskDelay(10 / portTICK_PERIOD_MS);
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

static void high_power_ledc_init(void)
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
        .channel = LEDC_HIGH_POWER_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_HIGH_POWER_OUTPUT,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static void pairing_task(void *pvParameter)
{
    uint8_t buf[sizeof(pairing_data_t)]; // Maximum Payload size of SX1276/77/78/79 is 255
    while (1)
    {
        lora_receive(); // put into receive mode
        if (lora_received())
        {
            int rxLen = lora_receive_packet(buf, sizeof(buf));
            int rssi = lora_packet_rssi();
            ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received:[%.*s] at %ddbm", rxLen, rxLen, buf, rssi);
            pairing_data_t *evt = (pairing_data_t *)buf;
            if (memcmp(evt->pairing_key, PAIRING_KEY, sizeof(evt->pairing_key)) != 0)
            {
                ESP_LOGI(pcTaskGetName(NULL), "Invalid pairing key, message not read");
                vTaskDelay(100 / portTICK_PERIOD_MS);
                continue;
            }
            memcpy(paired_mac, evt->mac, sizeof(evt->mac));
            ESP_LOGI(pcTaskGetName(NULL), "Paired with %02x:%02x:%02x:%02x:%02x:%02x",
                     evt->mac[0], evt->mac[1], evt->mac[2], evt->mac[3], evt->mac[4], evt->mac[5]);
            // Send the received mac address back to the transmitter, to acknowledge the pairing (do this 10 times to ensure that it is received)
            for (uint8_t i = 0; i < 20; ++i)
            {
                lora_send_packet((uint8_t *)&PAIRING_DATA, sizeof(pairing_data_t));
            }
            // After pairing, flash led for 1 second
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_0));
            // Update duty to apply the new value
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
            break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Avoid WatchDog alerts
    }
    vTaskResume(lora_receiver_handle);
    vTaskResume(listener_handle);
    vTaskResume(received_data_processor_handle);
    vTaskResume(voltage_reader_handle);
    // vTaskResume(heartbeat_sender_handle);
    vTaskDelete(NULL);
}

void lora_task_receiver(void *pvParameter)
{
    vTaskSuspend(NULL);
    // uint64_t messages_received = 0;
    ESP_LOGI(pcTaskGetName(NULL), "Start receiving data");
    heartbeat_data_t data;

    uint8_t buf[sizeof(received_data_t)]; // Maximum Payload size of SX1276/77/78/79 is 255
    unsigned long last_sent = 0;
    while (1)
    {
        lora_receive(); // put into receive mode
        if (lora_received())
        {
            int rxLen = lora_receive_packet(buf, sizeof(received_data_t));
            int rssi = lora_packet_rssi();
            ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received:[%.*s] at %ddbm", rxLen, rxLen, buf, rssi);
            received_data_t *evt = (received_data_t *)buf;
            if (xQueueSend(s_example_espnow_queue, evt, ESP_MAXDELAY) != pdTRUE)
            {
                ESP_LOGE(TAG, "Failed to send data to queue");
            }
        }
        if (xTaskGetTickCount() - last_sent > pdMS_TO_TICKS(20000))
        {
            last_sent = xTaskGetTickCount();
            // Send heartbeat every 30 seconds
            vTaskResume(heartbeat_sender_handle);
            vTaskSuspend(NULL);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Avoid WatchDog alerts, receieve data every 10ms
    }
}

void print_mac(const unsigned char *mac)
{
    printf("%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void initialize_lora()
{
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
}

static void heartbeat_sender_task(void *pvParameter)
{
    vTaskSuspend(NULL);
    ESP_LOGI(pcTaskGetName(NULL), "Start sending heartbeat");
    heartbeat_data_t data;
    int voltage;
    esp_read_mac(data.mac, ESP_MAC_WIFI_STA);

    while (1)
    {
        voltage = 0;
        // // Do a oneshot read of the eaa
        // ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BATTERY_ADC1_CHAN0, &adc_raw));
        // ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, BATTERY_ADC1_CHAN0, adc_raw);
        // if (do_calibration1_chan0)
        // {
        //     ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw, &voltage));
        //     ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, BATTERY_ADC1_CHAN0, voltage);
        // }
        for (uint8_t i = 0; i < 10; ++i)
        {
            voltage += receiver_battery_voltages[i];
        }
        voltage /= 10;
        ESP_LOGI(pcTaskGetName(NULL), "Sending heartbeat");
        data.adc_raw = 0;
        data.voltage = voltage;
        for (int i = 0; i < 5; i++)
        {
            lora_send_packet((uint8_t *)&data, sizeof(heartbeat_data_t));
        }
        vTaskResume(lora_receiver_handle);
        vTaskSuspend(NULL);
    }
}

static void voltage_reader_task()
{
    vTaskSuspend(NULL);
    int counter = 0;
    int adc_raw;
    int voltage;
    while (1)
    {
        // Do a oneshot read of the eaa
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BATTERY_ADC1_CHAN0, &adc_raw));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, BATTERY_ADC1_CHAN0, adc_raw);
        if (do_calibration1_chan0)
        {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw, &voltage));
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, BATTERY_ADC1_CHAN0, voltage);
            receiver_battery_voltages[counter % 10] = voltage;
            ++counter;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    // Initialize led pwm channel settings
    example_ledc_init();
    high_power_ledc_init();
    adc_init();

    // Initialize lora module
    initialize_lora();

    s_example_espnow_queue = xQueueCreate(ESP_QUEUE_SIZE, sizeof(received_data_t));
    if (s_example_espnow_queue == NULL)
    {
        ESP_LOGE(TAG, "Create mutex fail");
    }

    esp_read_mac(PAIRING_DATA.mac, ESP_MAC_WIFI_STA);
    xTaskCreatePinnedToCore(pairing_task, "Pairing", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(listener, "Listener", 4000, (void *)&TEST_DATA, 5, &listener_handle, 1);
    xTaskCreatePinnedToCore(&lora_task_receiver, "RX", 1024 * 3, (void *)&TEST_DATA, 6, &lora_receiver_handle, 0);
    xTaskCreatePinnedToCore(received_data_processor, "Received_data_processor", 2048, (void *)&TEST_DATA, 6, &received_data_processor_handle, 1);
    xTaskCreatePinnedToCore(heartbeat_sender_task, "Heartbeat_sender", 1024 * 3, NULL, 5, &heartbeat_sender_handle, 0);
    xTaskCreatePinnedToCore(voltage_reader_task, "Voltage Reader", 1024 * 3, NULL, 2, &voltage_reader_handle, 1);
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
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_HIGH_POWER_CHANNEL, LEDC_DUTY));

                // Update duty to apply the new value
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_HIGH_POWER_CHANNEL));
                // Set to 0, essentially turning LED off
                vTaskDelay(50 / portTICK_PERIOD_MS);
                vTaskResume(lora_receiver_handle);
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_0));
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_HIGH_POWER_CHANNEL, LEDC_DUTY_0));
                // Update duty to apply the new value
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_HIGH_POWER_CHANNEL));
                vTaskDelay(50 / portTICK_PERIOD_MS);
                vTaskResume(lora_receiver_handle);
            }
        }
        else if (data->on_state == 0)
        {
            // Set to 0, essentially turning LED off
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_0));
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_HIGH_POWER_CHANNEL, LEDC_DUTY_0));
            // Update duty to apply the new value
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_HIGH_POWER_CHANNEL));
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

static void adc_init()
{
    // Initialize unit
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    // Configure the ADC CHANNEL for GPIO 7
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BATTERY_ADC1_CHAN0, &config));
    // Check calibration
    do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, BATTERY_ADC1_CHAN0, ADC_ATTEN, &adc1_cali_chan0_handle);
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}