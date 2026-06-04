#include <Arduino.h>
#include <driver/i2s.h>
#include <U8g2lib.h>

// ================= OLED =================
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
  U8G2_R0,
  U8X8_PIN_NONE
);

// ================= I2S =================
#define I2S_WS   25
#define I2S_SD   32
#define I2S_SCK  33

#define I2S_PORT I2S_NUM_0

#define bufferLen 64
int16_t sBuffer[bufferLen];

void i2s_install() {

  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format =
      i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = bufferLen,
    .use_apll = false
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin() {

  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}

void setup() {

  Serial.begin(115200);

  u8g2.begin();

  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
}

void loop() {

  size_t bytesIn = 0;

  esp_err_t result = i2s_read(
    I2S_PORT,
    &sBuffer,
    bufferLen,
    &bytesIn,
    portMAX_DELAY
  );

  float mean = 0;

  if (result == ESP_OK) {

    int16_t samples_read = bytesIn / 8;

    if (samples_read > 0) {

      for (int16_t i = 0; i < samples_read; i++) {
        mean += abs(sBuffer[i]);
      }

      mean /= samples_read;
    }
  }

  // масштаб OLED
  int level = map(mean, 0, 1500, 0, 120);
  level = constrain(level, 0, 120);

  // ---------- OLED ----------
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(2, 12, "MIC LEVEL");

  u8g2.drawFrame(4, 28, 120, 18);
  u8g2.drawBox(6, 30, level, 14);

  u8g2.setCursor(2, 58);
  u8g2.print((int)mean);

  u8g2.sendBuffer();

  delay(20);
}