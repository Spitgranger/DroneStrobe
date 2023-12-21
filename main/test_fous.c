/* LEDC (LED Controller) basic example
    GPIO 4 = LED
    GPIO 23 = Button
    GPIO 15 = BUTTOn
    GPIO 22 = Toggle switch
*/
// TODO Debounce buttons, fix logic when momentary state is changed
#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_log.h"


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
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz
#define BUTTON1                 15
#define BUTTON2                 23
#define TOGGLE                  22

typedef struct Data_t {
    uint8_t current_state;
    bool momentary_state;
    bool on_state;
} generic_data_t;

static generic_data_t TEST_DATA = {0, false, false};

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

    xTaskCreate(listener, "Listener", 4000, (void *)&TEST_DATA, 1, NULL);
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
    while(1){
        uint8_t brightness_button_state = gpio_get_level(BUTTON1);
        printf("%i", data->on_state);
        vTaskDelay(100/portTICK_PERIOD_MS);
        // If the led is currently on and momentary state is false or the momentary button is pressed and the momentary state is true
        if (brightness_button_state == 1 && (data->on_state == 1 || data->momentary_state == 0)) {
            printf("%i", data->momentary_state);
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
            char* test = (char *)malloc(sizeof(uint8_t));
            itoa(brightness_button_state, test, 10);
            printf("%s\n", test);
            free(test);
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