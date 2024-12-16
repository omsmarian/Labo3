#include <esp_log.h>
#include <driver/adc.h>
#include <driver/dac.h>
#include <driver/gpio.h>
#include <driver/i2s.h>
#include <math.h> // Include for sin()

#define BUTTON_START_STOP GPIO_NUM_23
#define BUTTON_EFFECT_1 GPIO_NUM_16
#define BUTTON_EFFECT_2 GPIO_NUM_17
#define BUTTON_EFFECT_3 GPIO_NUM_18
#define BUTTON_EFFECT_4 GPIO_NUM_19
#define BUTTON_EFFECT_5 GPIO_NUM_22

#define LED_EFFECT_1 GPIO_NUM_33
#define LED_EFFECT_2 GPIO_NUM_32
#define LED_EFFECT_3 GPIO_NUM_15
#define LED_EFFECT_4 GPIO_NUM_4
#define LED_EFFECT_5 GPIO_NUM_2

#define BUFFER_SIZE 1024
#define SAMPLE_RATE 22050

static const char *TAG = "AudioEffects";

volatile bool processing = false;
int16_t adcBuffer[BUFFER_SIZE];
int16_t dacBuffer[BUFFER_SIZE];

bool effect1Enabled = false;
bool effect2Enabled = false;
bool effect3Enabled = false;
bool effect4Enabled = false;
bool effect5Enabled = false;

void setupI2S() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S, // Updated to non-deprecated format
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = BUFFER_SIZE,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_PIN_NO_CHANGE,
        .ws_io_num = I2S_PIN_NO_CHANGE,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = GPIO_NUM_34 // ADC input
    };

    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &pin_config));
    ESP_ERROR_CHECK(i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_6));
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0));
    ESP_ERROR_CHECK(i2s_adc_enable(I2S_NUM_0));

    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX); // Updated with explicit cast
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_1, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN));
}

void processAudio() {
    size_t bytesRead, bytesWritten;

    ESP_ERROR_CHECK(i2s_read(I2S_NUM_0, adcBuffer, BUFFER_SIZE * sizeof(int16_t), &bytesRead, portMAX_DELAY));

    for (int i = 0; i < BUFFER_SIZE; i++) {
        int16_t sample = adcBuffer[i];

        if (effect1Enabled) {
            sample = sample >> 1;
        }
        if (effect2Enabled) {
            sample = sample + (sample >> 2);
        }
        if (effect3Enabled) {
            sample = (sample >> 1) + (sample >> 3);
        }
        if (effect4Enabled) {
            static float tremoloFactor = 0;
            tremoloFactor += 0.05;
            sample *= (0.5 + 0.5 * sin(tremoloFactor)); // `sin` function now resolved
        }
        if (effect5Enabled) {
            sample = sample > 32767 ? 32767 : (sample < -32768 ? -32768 : sample);
        }

        dacBuffer[i] = sample;
    }

    ESP_ERROR_CHECK(i2s_write(I2S_NUM_1, dacBuffer, BUFFER_SIZE * sizeof(int16_t), &bytesWritten, portMAX_DELAY));
}

void setupGPIO() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_START_STOP) | (1ULL << BUTTON_EFFECT_1) |
                        (1ULL << BUTTON_EFFECT_2) | (1ULL << BUTTON_EFFECT_3) |
                        (1ULL << BUTTON_EFFECT_4) | (1ULL << BUTTON_EFFECT_5),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    io_conf.pin_bit_mask = (1ULL << LED_EFFECT_1) | (1ULL << LED_EFFECT_2) |
                           (1ULL << LED_EFFECT_3) | (1ULL << LED_EFFECT_4) |
                           (1ULL << LED_EFFECT_5);
    io_conf.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

void app_main() {
    setupGPIO();
    setupI2S();

    while (true) {
        if (gpio_get_level(BUTTON_START_STOP)) {
            processing = !processing;
            ESP_LOGI(TAG, "%s", processing ? "Processing started" : "Processing stopped"); // Fixed ternary syntax
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if (gpio_get_level(BUTTON_EFFECT_1)) {
            effect1Enabled = !effect1Enabled;
            gpio_set_level(LED_EFFECT_1, effect1Enabled);
            ESP_LOGI(TAG, "Effect 1 toggled");
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if (gpio_get_level(BUTTON_EFFECT_2)) {
            effect2Enabled = !effect2Enabled;
            gpio_set_level(LED_EFFECT_2, effect2Enabled);
            ESP_LOGI(TAG, "Effect 2 toggled");
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if (gpio_get_level(BUTTON_EFFECT_3)) {
            effect3Enabled = !effect3Enabled;
            gpio_set_level(LED_EFFECT_3, effect3Enabled);
            ESP_LOGI(TAG, "Effect 3 toggled");
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if (gpio_get_level(BUTTON_EFFECT_4)) {
            effect4Enabled = !effect4Enabled;
            gpio_set_level(LED_EFFECT_4, effect4Enabled);
            ESP_LOGI(TAG, "Effect 4 toggled");
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if (gpio_get_level(BUTTON_EFFECT_5)) {
            effect5Enabled = !effect5Enabled;
            gpio_set_level(LED_EFFECT_5, effect5Enabled);
            ESP_LOGI(TAG, "Effect 5 toggled");
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if (processing) {
            processAudio();
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
