#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "driver/i2s.h"

// ================= OLED =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(
    SCREEN_WIDTH,
    SCREEN_HEIGHT,
    &Wire,
    -1
);

// ================= I2S MIC =================
#define I2S_WS   25
#define I2S_SD   32
#define I2S_SCK  33

#define SAMPLE_RATE 44100
#define BUFFER_SIZE 512

// ================= BATTERY =================
#define BATTERY_PIN 34

// Делитель 100k + 100k
const float R1 = 100000.0;
const float R2 = 100000.0;

// Калибровка ADC ESP32
const float ADC_CALIBRATION = 1.13;
const int ADC_MAX = 4095;

// Буфер микрофона
int16_t samples[BUFFER_SIZE];

// ===========================================

void setupI2S()
{
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_zero_dma_buffer(I2S_NUM_0);
}

// ===========================================
// Получение уровня сигнала с микрофона
// ===========================================
float getMicLevel()
{
    size_t bytesRead = 0;

    i2s_read(
        I2S_NUM_0,
        &samples,
        sizeof(samples),
        &bytesRead,
        portMAX_DELAY
    );

    int sampleCount = bytesRead / 2;

    long long sum = 0;

    for (int i = 0; i < sampleCount; i++)
    {
        int32_t sample = samples[i];

        // INMP441 часто приходит "смещённым"
        sample = sample >> 8;

        // убираем шумовой ноль
        sample = abs(sample);

        if (sample < 20)
            sample = 0;

        sum += sample;
    }

    float level =
        (float)sum / sampleCount;

    // усиление чувствительности
    level *= 10.0;

    return level;
}
// ===========================================
// Напряжение батареи
// ===========================================
float readBatteryVoltage()
{
    const int samplesCount = 50;

    long adcSum = 0;

    for (int i = 0; i < samplesCount; i++)
    {
        adcSum += analogRead(BATTERY_PIN);
        delay(2);
    }

    float adcRaw =
        adcSum / (float)samplesCount;

    // Напряжение на ADC
    float adcVoltage =
        (adcRaw / 4095.0) * 3.3;

    // Делитель 100k + 100k
    float batteryVoltage =
        adcVoltage * 2.0;

    // Калибровка ESP32 ADC
    batteryVoltage *= ADC_CALIBRATION;

    return batteryVoltage;
}

// ===========================================
// Процент батареи
// ===========================================
int batteryPercent(float voltage)
{
    // Для Li-Ion 18650
    if (voltage >= 4.2) return 100;
    if (voltage <= 3.0) return 0;

    return (int)((voltage - 3.0) / 1.2 * 100.0);
}

// ===========================================

void setup()
{
    Serial.begin(115200);

    Wire.begin(21, 22);

    if (!display.begin(
            SSD1306_SWITCHCAPVCC,
            0x3C))
    {
        Serial.println("OLED ERROR");
        while (true);
    }

    display.clearDisplay();
    display.setTextColor(WHITE);

    analogReadResolution(12);
    analogSetPinAttenuation(
        BATTERY_PIN,
        ADC_11db
    );

    setupI2S();

    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(10, 20);
    display.println("TEST");
    display.display();

    delay(1500);
}

void loop()
{
    float micLevel = getMicLevel();

    float batteryVoltage =
        readBatteryVoltage();

    int battery =
        batteryPercent(batteryVoltage);

    // ---------------- SERIAL ----------------
    Serial.print("Mic: ");
    Serial.print(micLevel);

    Serial.print(" | Battery: ");
    Serial.print(batteryVoltage, 2);
    Serial.print(" V");

    Serial.print(" | ");
    Serial.print(battery);
    Serial.println("%");

    // ---------------- OLED ----------------
    display.clearDisplay();

    display.setTextSize(1);

    display.setCursor(0, 0);
    display.print("Battery: ");
    display.print(batteryVoltage, 2);
    display.print("V");

    display.setCursor(0, 12);
    display.print("Charge: ");
    display.print(battery);
    display.print("%");

    display.setCursor(0, 28);
    display.print("Mic level");

    // Полоска микрофона
    int barWidth =
        constrain(micLevel / 20, 0, 120);

    display.drawRect(0, 42, 120, 12, WHITE);
    display.fillRect(2, 44,
                     barWidth,
                     8,
                     WHITE);

    display.display();

    delay(50);
}