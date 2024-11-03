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
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "Message/Message.h"

#define BUTTON1 4
#define BUTTON2 5
#define TOGGLE 6
#define PAIRING_LED 11
#define BATTERY_STATUS_LED 10
#define ESP_INTR_FLAG_DEFAULT 0
#define BATTERY_MAX 8400
#define BATTERY_LOW 7400
#define BATTERY_MIN 6400
#define BATTERY_ADC1_CHAN0 ADC_CHANNEL_6
#define ADC_ATTEN ADC_ATTEN_DB_12
#define TRANSMITTER_BATTERY_MAX 4200
#define TRANSMITTER_BATTERY_LOW 3600
#define TRANSMITTER_BATTERY_MIN 3200

static const char *TAG = "espnow_transmitter";
static const char *PAIRING_KEY = "789eebEkksXswqw";

// Task handles for freertos tasks
TaskHandle_t transmitter_task_handle;
TaskHandle_t input_processor_task_handle;
TaskHandle_t pairing_task_handle;
TaskHandle_t blink_pairing_led_task_handle;
TaskHandle_t heartbeat_receiver_task_handle;
TaskHandle_t voltage_status_led_task;
TaskHandle_t heartbeat_processor_task_handle;

SemaphoreHandle_t xSemaphore = NULL;
SemaphoreHandle_t transmitter_semaphore = NULL;

// adc battery voltage monitoring
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);
static void adc_init(void);
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

// Static variable initialization
static generic_data_t TEST_DATA = {false, false, false, {0}, {0}};
static pairing_data_t PAIRING_DATA = {{0}, "789eebEkksXswqwe"};
static bool paired = false;
static QueueHandle_t event_action_queue = NULL;
static uint64_t started = 0;
static uint8_t paired_receiver_mac[6] = {0};
static uint8_t own_mac[6] = {0};
static uint8_t voltage_state = 0;
uint8_t message[33] = {0};

void IRAM_ATTR button_isr_handler(void *arg)
{
    started = esp_timer_get_time();
    xTaskResumeFromISR(input_processor_task_handle);
}

/*
since individual transmission of messages is too unreliable make it like this:
interrupt resets counter and resumes the below task, the task polls buttons for 10 seconds from the last button press
which should do the trick
*/
void process_input(void *pvParameter)
{
    // This is the method to poll
    generic_data_t *data = (generic_data_t *)pvParameter;
    uint8_t brightness_button_level;
    uint8_t momentary_button_level;
    uint8_t toggle_switch_level;
    unsigned long pressed_time = 0;
    unsigned long released_time = 0;
    uint8_t last_pressed = 0;
    uint64_t time_difference = 0;
    // either 0, 100, 001, 010, 110, 011, 111
    while (1)
    {
        // Calculate the time elapsed between the last button press and the current time. If it's greater than
        // 10 seconds suspend sending listening to inputs and resume listening to heartbeats
        ESP_LOGI(pcTaskGetName(NULL), "Input processor woken up");
        if (xSemaphore != NULL)
        {
            /* See if we can obtain the semaphore.  If the semaphore is not
            available wait 10 ticks to see if it becomes free. */
            if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                xSemaphoreGive(transmitter_semaphore);
                while (1)
                {
                    vTaskSuspend(heartbeat_processor_task_handle);
                    vTaskSuspend(heartbeat_receiver_task_handle);
                    // Increment the transmitter semaphore so it can start sending messages
                    time_difference = esp_timer_get_time() - started;
                    if (time_difference / 1000ULL > 10000)
                    {
                        if (transmitter_semaphore != NULL)
                        {
                            /* See if we can obtain the semaphore.  If the semaphore is not
                            available wait 10 ticks to seidfe if it becomes free. */
                            if (xSemaphoreTake(transmitter_semaphore, pdMS_TO_TICKS(10)) == pdTRUE)
                            {
                                xSemaphoreGive(xSemaphore);
                                vTaskSuspend(transmitter_task_handle);
                                vTaskResume(heartbeat_processor_task_handle);
                                // vTaskResume(voltage_status_led_task);
                                vTaskResume(heartbeat_receiver_task_handle);
                                break;
                            }
                            else
                            {
                                // Transmitter is not done transmitting. Wait by continuing to next iteration of loop
                                continue;
                            }
                        }
                    }
                    vTaskResume(transmitter_task_handle);
                    brightness_button_level = !gpio_get_level(BUTTON1);
                    momentary_button_level = !gpio_get_level(BUTTON2);
                    toggle_switch_level = !gpio_get_level(TOGGLE);
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
                            vTaskSuspend(heartbeat_receiver_task_handle);
                            vTaskSuspend(heartbeat_processor_task_handle);
                            vTaskSuspend(voltage_status_led_task);
                            xSemaphoreGive(xSemaphore);
                            vTaskResume(pairing_task_handle);
                            break;
                            // vTaskSuspend(input_processor_task_handle);
                        }
                    }
                    last_pressed = brightness_button_level;

                    data->button_one_state = brightness_button_level;
                    data->button_two_state = momentary_button_level;
                    data->toggle_state = toggle_switch_level;
                    xSemaphoreGive(xSemaphore);
                    vTaskDelay(30 / portTICK_PERIOD_MS);
                }
                vTaskSuspend(NULL);
            }
        }
        else
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
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
    gpio_reset_pin(BATTERY_STATUS_LED);

    gpio_set_direction(BUTTON1, GPIO_MODE_INPUT);
    gpio_set_direction(BUTTON2, GPIO_MODE_INPUT);
    gpio_set_direction(TOGGLE, GPIO_MODE_INPUT);
    gpio_set_direction(PAIRING_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(BATTERY_STATUS_LED, GPIO_MODE_OUTPUT);
    gpio_pullup_dis(BUTTON1);
    gpio_pulldown_en(BUTTON1);
    gpio_pullup_dis(TOGGLE);
    gpio_pulldown_en(TOGGLE);
    gpio_pullup_dis(BUTTON2);
    gpio_pulldown_en(BUTTON2);
}

// This task is only responsible for sending state packets to the receiver
void lora_task_transmitter(void *pvParameter)
{
    vTaskSuspend(NULL);
    ESP_LOGI(pcTaskGetName(NULL), "Start transmitting button state packets...");
    // uint8_t buf[256]; // Maximum Payload size of SX1276/77/78/79 is 255. We are sending 11 bytes.
    while (1)
    {
        if (transmitter_semaphore != NULL)
        {
            /* See if we can obtain the semaphore.  If the semaphore is not
            available wait 10 ticks to see if it becomes free. */
            if (xSemaphoreTake(transmitter_semaphore, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                ESP_LOGI(pcTaskGetName(NULL), "Transmitting Packet");
                build_transmission_message((generic_data_t *)pvParameter);
                lora_send_packet(&message[0], sizeof(message));
                // lora_send_packet((uint8_t *)pvParameter, sizeof(generic_data_t));
                // ESP_LOGI(pcTaskGetName(NULL), "%d byte packet sent...", sizeof(generic_data_t));
                int lost = lora_packet_lost();
                if (lost != 0)
                {
                    ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
                }
                // vTaskResume(heartbeat_receiver_task_handle);
                xSemaphoreGive(transmitter_semaphore);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }
}

void pairing_task(void *pvParameter)
{
    vTaskSuspend(NULL);
    ESP_LOGI(pcTaskGetName(NULL), "Start pairing...");
    vTaskResume(blink_pairing_led_task_handle);
    uint8_t buf[sizeof(message)];
    while (!paired)
    {
        build_pairing_message(PAIRING_KEY);
        lora_send_packet(message, sizeof(message));
        int lost = lora_packet_lost();
        if (lost != 0)
        {
            ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
        }
        lora_receive();
        if (lora_received())
        {
            int rxLen = lora_receive_packet(buf, sizeof(message));
            int rssi = lora_packet_rssi();
            ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received:[%.*s] at %ddbm", rxLen, rxLen, buf, rssi);
            if (memcmp(buf + 13, PAIRING_KEY, 16) == 0 && buf[0] == 1)
            {
                ESP_LOGI(pcTaskGetName(NULL), "Paired with %02x:%02x:%02x:%02x:%02x:%02x",
                         buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);
                memcpy(paired_receiver_mac, buf + 1, sizeof(paired_receiver_mac));
                memcpy(message + 7, paired_receiver_mac, 6);
                paired = true;
            }
        }
        else
        {
            ESP_LOGI(pcTaskGetName(NULL), "Pairing Acknolwedgement not received");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    // vTaskResume(transmitter_task_handle);
    vTaskResume(input_processor_task_handle);
    vTaskResume(voltage_status_led_task);
    // vTaskResume(heartbeat_receiver_task_handle);
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
            vTaskSuspend(NULL);
        }
        gpio_set_level(PAIRING_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(PAIRING_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    // vTaskDelete(NULL);
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
    // vTaskSuspend(NULL);
    uint8_t buf[sizeof(message)];
    while (1)
    {
        if (xSemaphore != NULL)
        {
            /* See if we can obtain the semaphore.  If the semaphore is not
            available wait 50ms to see if it becomes free. */
            if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                lora_receive(); // put into receive mode
                if (lora_received())
                {
                    int rxLen = lora_receive_packet(buf, sizeof(message));
                    int rssi = lora_packet_rssi();
                    if (xQueueSend(event_action_queue, buf, 128) != pdTRUE)
                    {
                        ESP_LOGE(TAG, "Failed to send data to queue");
                    }
                    ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received:[Message Type %u] at %ddbm", rxLen, buf[0], rssi);
                }
                xSemaphoreGive(xSemaphore);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }
}

static void heartbeat_processor_task()
{
    uint8_t buf[33] = {0};
    int transmitter_battery_voltages[10] = {0};
    int adc_raw;
    int transmitter_voltage;
    int receiver_voltage;
    uint32_t transmitter_counter = 0;
    uint16_t average_transmitter_voltages;
    while (1)
    {
        // Receive the heartbeat containing the receivers battery voltage
        while (xQueueReceive(event_action_queue, buf, 128) == pdTRUE)
        {
            if (memcmp(buf + 7, own_mac, sizeof(own_mac)) == 0 && !paired)
            {
                ESP_LOGI(pcTaskGetName(NULL), "Connection Restarted");
                vTaskResume(blink_pairing_led_task_handle);
                paired = true;
                memcpy(paired_receiver_mac, buf + 1, sizeof(paired_receiver_mac));
                vTaskResume(input_processor_task_handle);
                vTaskDelete(pairing_task_handle);
            }
            else if (memcmp(buf + 1, paired_receiver_mac, sizeof(paired_receiver_mac)) != 0 || buf[0] != 2)
            {
                ESP_LOGI(pcTaskGetName(NULL), "Invalid mac or unpaired device, message not read");
                vTaskDelay(10 / portTICK_PERIOD_MS);
                continue;
            }
            receiver_voltage = (buf[13] << 24) + (buf[14] << 16) + (buf[15] << 8) + (buf[16]);
            ESP_LOGI(pcTaskGetName(NULL), "RECEIVED receiver VOLTAGE: %d", receiver_voltage);
            if (receiver_voltage < (int)(BATTERY_LOW / 3))
            {
                ESP_LOGI(pcTaskGetName(NULL), "BATTERY LOW");
                voltage_state |= 0x1u;
                // gpio_set_level(BATTERY_STATUS_LED, 1);
            }
            else if (receiver_voltage > (int)(BATTERY_LOW / 3))
            {
                ESP_LOGI(pcTaskGetName(NULL), "BATTERY OK");
                voltage_state &= ~(0x1u);
                // gpio_set_level(BATTERY_STATUS_LED, 0);
            }
        }
        // Read the transmitters battery voltage (this has a voltage divide by 2 i.e 3.7V/2)
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BATTERY_ADC1_CHAN0, &adc_raw));
        ESP_LOGI(TAG, "TRANSMITTER ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, BATTERY_ADC1_CHAN0, adc_raw);
        if (do_calibration1_chan0)
        {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw, &transmitter_voltage));
            ESP_LOGI(TAG, "TRANSMITTER ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, BATTERY_ADC1_CHAN0, transmitter_voltage);
        }
        if (transmitter_counter < 10)
        {
            transmitter_battery_voltages[transmitter_counter] = transmitter_voltage;
            ++transmitter_counter;
        }
        else if (transmitter_counter >= 10)
        {
            transmitter_battery_voltages[transmitter_counter % 10] = transmitter_voltage;
            transmitter_counter++;
            average_transmitter_voltages = 0;
            for (uint8_t i = 0; i < 10; ++i)
            {
                average_transmitter_voltages += transmitter_battery_voltages[i];
            }
            if ((average_transmitter_voltages / 10) < (int)(TRANSMITTER_BATTERY_MIN / 2))
            {
                ESP_LOGI(pcTaskGetName(NULL), "TRANSMITTER BATTERY LOW");
                voltage_state |= (0x1u << 1);
            }
            else if ((average_transmitter_voltages / 10) > (int)(TRANSMITTER_BATTERY_MIN / 2))
            {
                ESP_LOGI(pcTaskGetName(NULL), "TRANSMITTER BATTERY OK");
                voltage_state &= ~(0x1u << 1);
                // gpio_set_level(BATTERY_STATUS_LED, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

void set_battery_status_led_task()
{
    while (1)
    {
        // Receiver Low Voltage
        if (voltage_state & (0x1u) && !(voltage_state & (0x1u << 1)))
        {
            gpio_set_level(BATTERY_STATUS_LED, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            // gpio_set_level(BATTERY_STATUS_LED, 0);
            // vTaskDelay(pdMS_TO_TICKS(1000));
        }
        // Transmitter Low Voltage
        else if (voltage_state & (0x1u << 1) && !(voltage_state & (0x1u)))
        {
            gpio_set_level(BATTERY_STATUS_LED, 1);
            vTaskDelay(pdMS_TO_TICKS(2000));
            gpio_set_level(BATTERY_STATUS_LED, 0);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        // Both Low, blink fast
        else if ((voltage_state & (0x1u << 1)) && (voltage_state & (0x1u)))
        {
            gpio_set_level(BATTERY_STATUS_LED, 1);
            vTaskDelay(pdMS_TO_TICKS(300));
            gpio_set_level(BATTERY_STATUS_LED, 0);
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        else
        {
            // Nothing is low, shut off light
            gpio_set_level(BATTERY_STATUS_LED, 0);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
}

void app_main()
{
    set_gpio();
    // Initialize the pairing led off
    gpio_set_level(PAIRING_LED, 0);
    adc_init();
    // This initializes the test data structure with the appropriate values of the toggle on start.
    if (gpio_get_level(TOGGLE) == 0)
    {
        TEST_DATA.toggle_state = false;
    }
    else
    {
        TEST_DATA.toggle_state = true;
    }

    event_action_queue = xQueueCreate(10, sizeof(message));

    if (event_action_queue == NULL)
    {
        ESP_LOGE(pcTaskGetName(NULL), "Failed to create queue");
        return;
    }

    xSemaphore = xSemaphoreCreateBinary();
    transmitter_semaphore = xSemaphoreCreateBinary();

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
    esp_read_mac(own_mac, ESP_MAC_WIFI_STA);
    // Copy our own mac into the first 6 bytes of message to identify the sender of this message
    memcpy(message + 1, own_mac, 6);

    gpio_set_intr_type(BUTTON1, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(BUTTON2, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(TOGGLE, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(BUTTON1, button_isr_handler, (void *)&TEST_DATA);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(BUTTON2, button_isr_handler, (void *)&TEST_DATA);
    gpio_isr_handler_add(TOGGLE, button_isr_handler, (void *)&TEST_DATA);

    // Setup lora module
    lora_module_setup();

    xTaskCreatePinnedToCore(&lora_task_transmitter, "TX", 1024 * 3, (void *)&TEST_DATA, 6, &transmitter_task_handle, 1);

    xTaskCreatePinnedToCore(&process_input, "process_input", 4096, (void *)&TEST_DATA, 7, &input_processor_task_handle, 0);

    xTaskCreatePinnedToCore(&set_battery_status_led_task, "battery_led_status_task", 1024 * 2, NULL, 2, &voltage_status_led_task, 0);

    xTaskCreatePinnedToCore(heartbeat_processor_task, "heartbeat_processor", 1024 * 4, NULL, 3, &heartbeat_processor_task_handle, 0);

    xTaskCreatePinnedToCore(&pairing_task, "pairing_task", 2048, NULL, 3, &pairing_task_handle, 1);

    xTaskCreatePinnedToCore(&blink_pairing_led_task, "pairing_task_blinker", 1024 * 3, NULL, 1, &blink_pairing_led_task_handle, 0);
    xTaskCreatePinnedToCore(heartbeat_receiver_task, "heartbeat_receiver_task", 2048, NULL, 3, &heartbeat_receiver_task_handle, 1);
    xSemaphoreGive(xSemaphore);
}

static void adc_init(void)
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