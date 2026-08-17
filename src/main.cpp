// ============================================================
//  ГИТАРНЫЙ ТЮНЕР — ПРОШИВКА ДЛЯ АППАРАТНОЙ ВАЛИДАЦИИ
//  Целевая платформа : ESP32 DevKit V1
//  Фреймворк         : Arduino (PlatformIO)
//  Назначение        : Проверка OLED, микрофона INMP441,
//                      захвата звука, детекции высоты тона,
//                      определения ноты, индикатора строя
//                      и измерения заряда батареи.
//  ВНИМАНИЕ: Это НЕ финальная прошивка продукта.
// ============================================================

#include <Arduino.h>
#include <driver/i2s.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>

// ─────────────────────────────────────────────
//  OLED-дисплей SSD1306 (128x64, интерфейс I2C)
//  SDA -> GPIO21   SCL -> GPIO22
//  Используются аппаратные пины Wire ESP32 по умолчанию.
// ─────────────────────────────────────────────
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /*reset=*/U8X8_PIN_NONE,
                                           /*clock=*/22, /*data=*/21);

// ─────────────────────────────────────────────
//  Микрофон INMP441 -- цифровой I2S МЭМС-микрофон
// ─────────────────────────────────────────────
#define I2S_PORT        I2S_NUM_0   // Используем I2S-периферию №0
#define I2S_WS_PIN      25          // WS  (Word Select / LRCK)
#define I2S_SD_PIN      32          // SD  (Serial Data -- вход данных)
#define I2S_SCK_PIN     33          // SCK (битовый тактовый сигнал)
#define SAMPLE_RATE     44100       // Частота дискретизации, Гц
#define BLOCK_SIZE      2048        // Размер одного блока обработки, семплов
                                    // При 44100 Гц -> ~46 мс на блок
#define I2S_BUF_COUNT   4           // Количество DMA-буферов
#define I2S_BUF_LEN     512         // Размер одного DMA-буфера, семплов

// ─────────────────────────────────────────────
//  Измерение заряда батареи через АЦП
//
//  Схема: аккумулятор 18650 подключён через делитель 10к+10к
//  к GPIO34 (ADC1_CH6 -- только вход, безопасен для АЦП).
//
//  Коэффициент делителя: (10к+10к)/10к = 2.0
//  То есть на пине: Ubat / 2
//
//  КАЛИБРОВКА АЦП ESP32:
//  Встроенный АЦП ESP32 имеет нелинейность и смещение нуля
//  (~100-150 мВ занижение в диапазоне 3.0-4.2 В).
//  Поправочный коэффициент ADC_CORRECTION подобран
//  эмпирически по сравнению с показаниями мультиметра:
//    мультиметр 4.06 В -> устройство без поправки ~3.74 В
//    поправочный множитель = 4.06 / 3.74 = 1.086
//
//  Если показания уплывут после замены резисторов делителя --
//  скорректировать ADC_CORRECTION под свой экземпляр платы.
// ─────────────────────────────────────────────
#define BAT_ADC_PIN     34
#define ADC_REF_V       3.3f        // Опорное напряжение АЦП ESP32, В
#define ADC_RESOLUTION  4095.0f     // 12-битный АЦП: диапазон 0..4095
#define DIVIDER_RATIO   2.0f        // Коэффициент резистивного делителя (10к+10к)/10к
//
// Поправочный коэффициент нелинейности АЦП ESP32.
// АЦП ESP32 занижает показания на ~5-10% в диапазоне 3.0-4.2 В.
// Коэффициент = реальное_напряжение / показание_без_поправки.
// Измерено: мультиметр 4.06 В, без поправки 3.74 В -> 4.06/3.74 = 1.086
// При замене платы или резисторов -- перекалибровать вручную.
#define ADC_CORRECTION  1.086f
//
#define BAT_MAX_V       4.2f        // Напряжение полностью заряженного Li-ion, В
#define BAT_MIN_V       3.0f        // Напряжение разряженного Li-ion, В
#define BAT_SAMPLES     32          // 32 усредняемых отсчёта для сглаживания шума АЦП

// ─────────────────────────────────────────────
//  Таблица нот гитары
//  Частоты стандартного строя (эталон A4 = 440 Гц)
// ─────────────────────────────────────────────
struct GuitarNote {
    const char* name;   // Название ноты (отображается на дисплее)
    float       freq;   // Эталонная частота, Гц
};

static const GuitarNote NOTES[] = {
    { "E2",  82.41f  },   // 6-я струна (самая толстая)
    { "A2", 110.00f  },   // 5-я струна
    { "D3", 146.83f  },   // 4-я струна
    { "G3", 196.00f  },   // 3-я струна
    { "B3", 246.94f  },   // 2-я струна
    { "E4", 329.63f  }    // 1-я струна (самая тонкая)
};
static const int NUM_NOTES = 6;

// ─────────────────────────────────────────────
//  Пороги индикатора точности настройки
// ─────────────────────────────────────────────
// IN_TUNE_CENTS -- если отклонение в пределах +/-5 центов -> "IN TUNE"
// CLOSE_CENTS   -- в пределах +/-20 центов -> одна стрелка (< или >)
// за пределами +/-20 центов                -> двойная стрелка (<< или >>)
#define IN_TUNE_CENTS   5.0f
#define CLOSE_CENTS     20.0f

// ─────────────────────────────────────────────
//  Пороги DSP и подавления шумов
//
//  RMS_THRESHOLD  -- минимальный уровень RMS сигнала (единицы int16, диапазон 0..32767)
//                   для запуска детекции высоты тона.
//                   Снижен до 60: тюнер работает на разумном расстоянии от гитары.
//                   Если реагирует на фоновый шум в помещении -- поднять до 100-150.
//                   Если не ловит тихие струны -- опустить до 40-50.
//
//  CORR_THRESHOLD -- минимальный пик NACF (диапазон 0..1).
//                   1.0 = идеально периодический сигнал (синус).
//                   Гитарная струна даёт 0.55..0.85.
//                   Речь, щелчки и шум дают 0.1..0.35 -- отсеиваются.
//
//  STABILITY_FRAMES -- сколько последовательных блоков должны согласоваться
//                      (частота в пределах 5%) перед фиксацией ноты.
//                      4 блока x ~46 мс = ~185 мс задержки отклика.
//
//  FREQ_MIN/MAX -- диапазон поиска (с запасом за пределами нот гитары)
// ─────────────────────────────────────────────
#define RMS_THRESHOLD     60.0f
#define CORR_THRESHOLD    0.50f //было 0.45
#define STABILITY_FRAMES  4     //было 4
#define FREQ_MIN          65.0f     // Чуть ниже E2 (82 Гц) с запасом
#define FREQ_MAX          550.0f    // Чуть выше E4 (330 Гц) с запасом
#define OCTAVE_THRESHOLD  0.80f

// ─────────────────────────────────────────────
//  Буферы обработки сигнала
//
//  ВАЖНО: все массивы объявлены глобально (static).
//  Стек задачи loopTask Arduino по умолчанию = 8192 байт.
//  Один float[2048] = 8192 байт -- если объявить локально,
//  стек немедленно переполнится (Stack canary triggered).
//  Глобальные переменные размещаются в сегменте BSS (DRAM),
//  не на стеке -- переполнения нет.
// ─────────────────────────────────────────────
static int16_t   audioBuf[BLOCK_SIZE];      // Сырые int16 семплы из I2S DMA
static float     floatBuf[BLOCK_SIZE];      // Сигнал в float после удаления DC
static float     acfBuf[BLOCK_SIZE];        // Используется для передачи пика NACF
static float     windowedBuf[BLOCK_SIZE];   // Сигнал после применения окна Хэмминга

// ─────────────────────────────────────────────
//  Переменные состояния -- слой детекции
//  Обновляются каждые ~46 мс (один блок)
// ─────────────────────────────────────────────
static float     detectedFreq   = 0.0f;  // Последняя определённая частота, Гц
static int       lockedNoteIdx  = -1;    // Индекс стабилизированной ноты (-1 = нет)
static float     lockedCents    = 0.0f;  // Отклонение в центах для зафиксированной ноты
static float     lastFreq       = 0.0f;  // Частота предыдущего блока (для stability)
static int       stableCount    = 0;     // Счётчик последовательных совпадений
static int       candidateIdx   = -1;    // Текущая кандидатная нота

// ─────────────────────────────────────────────
//  Переменные состояния -- слой отображения
//
//  Отображение ОТДЕЛЕНО от детекции.
//  Последняя пойманная нота залипает на экране
//  и сбрасывается на прочерки только через NOTE_HOLD_MS
//  миллисекунд без нового сигнала.
//  displayNoteIdx == -1 -> нот ещё не было -> отображаем "--"
// ─────────────────────────────────────────────
static int       displayNoteIdx = -1;    // Что показывается на экране прямо сейчас
static float     displayCents   = 0.0f;  // Центы для отображаемой ноты
static float     displayFreq    = 0.0f;  // Частота для отображаемой ноты
static uint32_t  lastNoteSeenMs = 0;     // millis() момента последней фиксации ноты
#define NOTE_HOLD_MS  30000UL            // Время удержания ноты на экране: 30 секунд

// ─────────────────────────────────────────────
//  Прочие переменные состояния
// ─────────────────────────────────────────────
static float     batteryV       = 0.0f;  // Напряжение батареи, В
static int       batteryPct     = 0;     // Заряд батареи, %
static bool      micOk          = false; // Флаг: микрофон отвечает на запросы
static uint32_t  batUpdateMs    = 0;     // millis() последнего обновления батареи

float debugOctaveCorr = 0.0f;
float debugRatio = 0.0f;
bool  debugOctaveFix = false;

// ─────────────────────────────────────────────
//  Предварительные объявления функций
// ─────────────────────────────────────────────
void     initI2S();
bool     readBlock(int16_t* buf, int n);
float    computeRMS(const float* buf, int n);
float    detectPitch(const float* buf, int n);
int      findClosestNote(float freq, float* centsOut);
void     updateBattery();
void     drawDisplay();
void     printDebug(float rms, float corrPeak, float freq, int noteIdx, float cents);

// ============================================================
//  ИНИЦИАЛИЗАЦИЯ (выполняется один раз при включении)
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== ЗАГРУЗКА ТЮНЕРА -- АППАРАТНАЯ ВАЛИДАЦИЯ ===");

    // -- Инициализация I2C и OLED --
    Wire.begin(21, 22);
    if (!u8g2.begin()) {
        Serial.println("[ОШИБКА] u8g2.begin() не выполнен -- проверьте SDA/SCL");
    } else {
        Serial.println("[OK] OLED инициализирован");
    }

    // -- Загрузочная заставка --
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(10, 20, "GUITAR TUNER");
    u8g2.drawStr(10, 34, "HW VALIDATION");
    u8g2.drawStr(10, 48, "Initialising...");
    u8g2.sendBuffer();
    delay(1200);

    // -- Инициализация АЦП батареи --
    // GPIO34 -- только входной пин, внешняя подтяжка не нужна
    pinMode(BAT_ADC_PIN, INPUT);
    updateBattery();
    Serial.printf("[OK] АЦП батареи: %.2f В  (%d%%)\n", batteryV, batteryPct);

    // -- Инициализация I2S / микрофон INMP441 --
    initI2S();

    // -- Проверка микрофона: один тестовый захват --
    // Если DMA-буфер не заполнился за 500 мс -> ошибка подключения
    micOk = readBlock(audioBuf, BLOCK_SIZE);
    if (micOk) {
        Serial.println("[OK] I2S / INMP441 -- первое чтение успешно");
    } else {
        Serial.println("[ОШИБКА] I2S -- чтение не удалось, проверьте микрофон");
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_9x15B_tf);
        u8g2.drawStr(16, 28, "MIC ERROR");
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(10, 44, "Check INMP441 wiring");
        u8g2.sendBuffer();
        delay(4000);
    }

    Serial.println("=== ВХОД В ОСНОВНОЙ ЦИКЛ ===\n");
}

// ============================================================
//  ОСНОВНОЙ ЦИКЛ
//  Выполняется непрерывно. Один проход ≈ 46 мс при BLOCK_SIZE=2048.
// ============================================================
void loop() {
    // -- Обновление показаний батареи каждые 5 секунд --
    // Не обновляем каждый блок, чтобы не тормозить обработку звука
    if (millis() - batUpdateMs > 60000) {
        updateBattery();
        batUpdateMs = millis();
    }

    // -- Чтение блока аудио из I2S DMA --
    // Блокирующее чтение с таймаутом 500 мс
    micOk = readBlock(audioBuf, BLOCK_SIZE);
    if (!micOk) {
        // DMA-буфер не заполнился -- ошибка микрофона или I2S
        lockedNoteIdx = -1;
        stableCount   = 0;
        drawDisplay();
        delay(50);
        return;
    }

    // -- Перевод int16 -> float и удаление постоянной составляющей (DC) --
    // DC-смещение -- это ненулевое среднее значение сигнала из-за
    // несовершенства питания микрофона. Оно смещает ось сигнала
    // и ухудшает результат автокорреляции. Удаляем вычитанием среднего.
    float dcSum = 0.0f;
    for (int i = 0; i < BLOCK_SIZE; i++) dcSum += (float)audioBuf[i];
    float dc = dcSum / BLOCK_SIZE;
    for (int i = 0; i < BLOCK_SIZE; i++) floatBuf[i] = (float)audioBuf[i] - dc;

    // -- Проверка порога RMS (гейт тишины) --
    // RMS = среднеквадратическое значение сигнала.
    // Если сигнал слишком тихий -- это тишина или шум.
    // Не запускаем дорогостоящую автокорреляцию на слабом сигнале.
    float rms = computeRMS(floatBuf, BLOCK_SIZE);
    if (rms < RMS_THRESHOLD) {
        // Сигнал ниже порога -- сбрасываем счётчики детекции,
        // но НЕ сбрасываем дисплей (нота залипает на экране).
        stableCount   = 0;
        candidateIdx  = -1;
        lockedNoteIdx = -1;
        printDebug(rms, 0.0f, 0.0f, -1, 0.0f);

        // Проверяем 30-секундный таймаут удержания ноты на экране
        if (displayNoteIdx >= 0 &&
            (millis() - lastNoteSeenMs) > NOTE_HOLD_MS) {
            displayNoteIdx = -1;  // Время вышло -- возврат к прочеркам
        }
        drawDisplay();
        return;
    }

    // -- Детекция высоты тона методом NACF --
    // Нормализованная автокорреляционная функция.
    // Возвращает частоту фундаментального тона в Гц, или 0 если не найдена.
    // Пик NACF сохраняется в acfBuf[0] для отладочного вывода.
    float freq     = detectPitch(floatBuf, BLOCK_SIZE);
    float corrPeak = acfBuf[0];

    // -- Поиск ближайшей гитарной ноты --
    float cents   = 0.0f;
    int   noteIdx = -1;
    if (freq > 0.0f) {
        noteIdx = findClosestNote(freq, &cents);
    }

    printDebug(rms, corrPeak, freq, noteIdx, cents);

    // -- Логика стабилизации (антидребезг ноты) --
    // Нота считается пойманной только если STABILITY_FRAMES
    // последовательных блоков дают ту же ноту с частотой,
    // отличающейся не более чем на 5%.
    // Это предотвращает мигание между нотами в атаке/затухании.
    if (noteIdx >= 0 && freq > 0.0f) {
        // Проверяем близость текущей частоты к предыдущей (в пределах 5%)
        bool freqClose = fabsf(freq - lastFreq) < (freq * 0.05f);

        if (freqClose && noteIdx == candidateIdx) {
            stableCount++;  // Та же нота, частота стабильна -> +1 к счётчику
        } else {
            stableCount  = 1;       // Другая нота или скачок частоты -> перезапуск
            candidateIdx = noteIdx;
        }
        lastFreq = freq;

        // Если набрали нужное количество совпадений -- фиксируем ноту
        if (stableCount >= STABILITY_FRAMES) {
            lockedNoteIdx = noteIdx;
            lockedCents   = cents;
            detectedFreq  = freq;

            // Обновляем слой отображения (именно эти данные видит пользователь)
            displayNoteIdx = noteIdx;
            displayCents   = cents;
            displayFreq    = freq;
            lastNoteSeenMs = millis();  // Сбрасываем таймер удержания
        }
    } else {
        // Нота не найдена в этом блоке -- сброс счётчика кандидата.
        // Слой отображения НЕ трогаем: нота залипает до истечения таймаута.
        stableCount   = 0;
        candidateIdx  = -1;
        lockedNoteIdx = -1;
    }

    // -- Проверка 30-секундного таймаута --
    if (displayNoteIdx >= 0 &&
        (millis() - lastNoteSeenMs) > NOTE_HOLD_MS) {
        displayNoteIdx = -1;  // 30 с без ноты -- сброс на прочерки
    }

    // -- Обновление дисплея --
    drawDisplay();
}

// ============================================================
//  ИНИЦИАЛИЗАЦИЯ I2S-ПЕРИФЕРИИ
//  Конфигурация строго по параметрам микрофона INMP441.
//  Не изменять без проверки на реальном железе.
// ============================================================
void initI2S() {
    i2s_config_t cfg = {
        // Режим: мастер + только приём (микрофон -- только вход)
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate          = SAMPLE_RATE,                    // 44100 Гц
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,      // 16 бит
        // INMP441 с L/R=GND -> только левый канал
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        // Стандартный I2S (Philips): данные передаются на спаде WS
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,           // Приоритет прерывания
        .dma_buf_count        = I2S_BUF_COUNT,                  // 4 DMA-буфера
        .dma_buf_len          = I2S_BUF_LEN,                    // 512 семплов каждый
        .use_apll             = false,    // APLL не нужен при 44100 Гц
        .tx_desc_auto_clear   = false,    // Не используется (только RX)
        .fixed_mclk           = 0        // MCLK не нужен для INMP441
    };

    i2s_pin_config_t pins = {
        .bck_io_num   = I2S_SCK_PIN,        // GPIO33 -- битовый тактовый
        .ws_io_num    = I2S_WS_PIN,          // GPIO25 -- выбор слова (LRCK)
        .data_out_num = I2S_PIN_NO_CHANGE,   // Вывод данных не используется
        .data_in_num  = I2S_SD_PIN           // GPIO32 -- вход данных с микрофона
    };

    esp_err_t r1 = i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
    esp_err_t r2 = i2s_set_pin(I2S_PORT, &pins);

    if (r1 == ESP_OK && r2 == ESP_OK) {
        Serial.println("[OK] Драйвер I2S установлен");
    } else {
        Serial.printf("[ОШИБКА] I2S install: %s | pin: %s\n",
                      esp_err_to_name(r1), esp_err_to_name(r2));
    }
}

// ============================================================
//  ЧТЕНИЕ БЛОКА АУДИО ИЗ I2S DMA
//  Возвращает true если данные успешно получены.
//  Таймаут 500 мс -- при BLOCK_SIZE=2048 и 44100 Гц
//  реальное время заполнения ~46 мс, запас более чем достаточен.
// ============================================================
bool readBlock(int16_t* buf, int n) {
    size_t bytesRead = 0;
    esp_err_t err = i2s_read(I2S_PORT,
                             buf,
                             (size_t)(n * sizeof(int16_t)),
                             &bytesRead,
                             pdMS_TO_TICKS(500));
    if (err != ESP_OK || bytesRead == 0) return false;
    return true;
}

// ============================================================
//  ВЫЧИСЛЕНИЕ RMS (среднеквадратическое значение)
//  RMS = sqrt( sum(x[i]^2) / N )
//  Используется как индикатор уровня сигнала:
//  тишина -> малый RMS, музыкальный сигнал -> большой RMS.
// ============================================================
float computeRMS(const float* buf, int n) {
    float sum = 0.0f;
    for (int i = 0; i < n; i++) sum += buf[i] * buf[i];
    return sqrtf(sum / (float)n);
}

// ============================================================
//  ДЕТЕКЦИЯ ВЫСОТЫ ТОНА -- НОРМАЛИЗОВАННАЯ АКФ (NACF)
//
//  Алгоритм:
//  1. Применяем окно Хэмминга для уменьшения краевых эффектов.
//  2. Вычисляем нормализованную АКФ для лагов в диапазоне
//     [lagMin, lagMax], соответствующих [FREQ_MIN, FREQ_MAX].
//  3. Ищем лаг с максимальной корреляцией.
//  4. Уточняем лаг параболической интерполяцией (субсемпловая точность).
//  5. Возвращаем частоту: f = SAMPLE_RATE / уточнённый_лаг
//
//  Почему NACF, а не FFT-пик:
//  Гитарная струна -- негармонический источник, FFT-пик
//  может попасть на обертон, а не на фундаментальный тон.
//  NACF находит период сигнала, т.е. всегда фундаментал,
//  независимо от тембра инструмента.
//
//  Возвращает: частоту в Гц, или 0.0f если нота не найдена.
//  Побочный эффект: acfBuf[0] = значение пика NACF (для отладки).
// ============================================================
float detectPitch(const float* buf, int n) {
    // Пересчёт частотного диапазона в диапазон лагов:
    // лаг = SAMPLE_RATE / частота
    // большая частота -> малый лаг, малая частота -> большой лаг
    int lagMin = (int)(SAMPLE_RATE / FREQ_MAX);   // ~80  при FREQ_MAX=550 Гц
    int lagMax = (int)(SAMPLE_RATE / FREQ_MIN);   // ~677 при FREQ_MIN=65 Гц

    if (lagMax >= n) lagMax = n - 1;

    // -- Шаг 1: Окно Хэмминга --
    // w[i] = 0.54 - 0.46*cos(2*pi*i/(N-1))
    // Плавно обнуляет края блока, уменьшает краевые корреляции,
    // которые иначе дадут ложные пики на малых лагах.
    // Используем глобальный буфер (не локальный!) -- см. примечание
    // про стек выше в разделе буферов.
    float* windowed = windowedBuf;
    for (int i = 0; i < n; i++) {
        float w = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * i / (float)(n - 1));
        windowed[i] = buf[i] * w;
    }

    // -- Шаг 2: Проверка энергии сигнала --
    // Если сигнал вырожден в ноль после окна -- возвращаем 0
    float e0 = 0.0f;
    for (int i = 0; i < n; i++) e0 += windowed[i] * windowed[i];
    if (e0 < 1.0f) {
        acfBuf[0] = 0.0f;
        return 0.0f;
    }

    // -- Шаг 3: Поиск пика нормализованной АКФ --
    // NACF(lag) = sum(x[i]*x[i+lag]) / sqrt(sum(x[i]^2) * sum(x[i+lag]^2))
    // Нормализация на локальные энергии делает результат
    // независимым от уровня сигнала (всегда в диапазоне -1..1).
    float bestCorr = 0.0f;
    int   bestLag  = 0;

    for (int lag = lagMin; lag <= lagMax; lag++) {
        float num  = 0.0f;  // Числитель: кросс-произведение семплов
        float den1 = 0.0f;  // Энергия левого окна [0..m]
        float den2 = 0.0f;  // Энергия правого окна [lag..m+lag]
        int   m    = n - lag;
        for (int i = 0; i < m; i++) {
            num  += windowed[i] * windowed[i + lag];
            den1 += windowed[i] * windowed[i];
            den2 += windowed[i + lag] * windowed[i + lag];
        }
        float denom = sqrtf(den1 * den2);
        float corr  = (denom > 1.0f) ? (num / denom) : 0.0f;

        if (corr > bestCorr) {
            bestCorr = corr;
            bestLag  = lag;
        }
    }

        // -- Шаг 3.5: Проверка возможной октавной ошибки --
    // Если на удвоенном лаге (вдвое меньшая частота)
    // имеется почти такой же высокий пик НАКФ, считаем,
    // что глобальный максимум соответствует второй гармонике,
    // а истинный фундаментальный тон находится на большем лаге.
    // ----- Отладочные значения -----
        debugOctaveCorr = 0.0f;
        debugRatio = 0.0f;
        debugOctaveFix = false;

// ----- Проверка октавной ошибки -----
       // Частота, соответствующая найденному максимуму
float detectedFreq = (float)SAMPLE_RATE / (float)bestLag;

// Проверку субгармоники выполняем только для низких частот
        if (detectedFreq < 180.0f)
        {
            int octaveLag = bestLag * 2;

            if (octaveLag <= lagMax)
            {
                float num  = 0.0f;
                float den1 = 0.0f;
                float den2 = 0.0f;

                int m = n - octaveLag;

                for (int i = 0; i < m; i++) {
                    num  += windowed[i] * windowed[i + octaveLag];
                    den1 += windowed[i] * windowed[i];
                    den2 += windowed[i + octaveLag] * windowed[i + octaveLag];
                }

                float denom = sqrtf(den1 * den2);
                float octaveCorr = (denom > 1.0f) ? (num / denom) : 0.0f;

                debugOctaveCorr = octaveCorr;
                debugRatio = (bestCorr > 0.0f) ? octaveCorr / bestCorr : 0.0f;

                if (octaveCorr > OCTAVE_THRESHOLD * bestCorr) {
                    bestLag = octaveLag;
                    bestCorr = octaveCorr;
                    debugOctaveFix = true;
                }
            }
            else {
                debugOctaveCorr = 0.0f;
                debugRatio = 0.0f;
                debugOctaveFix = false;
            }
        }
        else {
            // Для высоких частот проверка не выполняется
            debugOctaveCorr = 0.0f;
            debugRatio = 0.0f;
            debugOctaveFix = false;
        }

    // Сохраняем пик для вывода в Serial (acfBuf[0] как канал связи с loop())
    acfBuf[0] = bestCorr;

    // -- Порог NACF: отсев шума и непериодических сигналов --
    if (bestCorr < CORR_THRESHOLD || bestLag == 0) return 0.0f;

    // -- Шаг 4: Параболическая интерполяция лага --
    // Дискретный максимум АКФ имеет погрешность +/-0.5 семпла.
    // При bestLag=80 (~550 Гц) ошибка 0.5 семпла -> ~3 Гц.
    // Параболическая интерполяция уточняет позицию до субсемпловой точности.
    // Формула: delta = 0.5 * (y0 - y2) / (2*y1 - y0 - y2)
    //          refinedLag = bestLag + delta
    if (bestLag > lagMin && bestLag < lagMax) {
        // Вычисляем NACF в соседних точках (lag-1 и lag+1) для интерполяции
        auto acfAt = [&](int lag) -> float {
            float num = 0.0f, den1 = 0.0f, den2 = 0.0f;
            int m = n - lag;
            for (int i = 0; i < m; i++) {
                num  += windowed[i] * windowed[i + lag];
                den1 += windowed[i] * windowed[i];
                den2 += windowed[i + lag] * windowed[i + lag];
            }
            float denom = sqrtf(den1 * den2);
            return (denom > 1.0f) ? (num / denom) : 0.0f;
        };

        float y0 = acfAt(bestLag - 1);  // NACF в точке lag-1
        float y1 = bestCorr;             // NACF в найденном пике
        float y2 = acfAt(bestLag + 1);  // NACF в точке lag+1

        // Знаменатель параболы: если близок к нулю -- пик плоский,
        // интерполяция нецелесообразна, оставляем целочисленный лаг
        float d = 2.0f * y1 - y0 - y2;
        float refinedLag = (float)bestLag;
        if (fabsf(d) > 1e-6f) {
            refinedLag += 0.5f * (y0 - y2) / d;
        }
        return (float)SAMPLE_RATE / refinedLag;
    }

    // Если bestLag на границе диапазона -- интерполяция невозможна
    return (float)SAMPLE_RATE / (float)bestLag;
}

// ============================================================
//  ПОИСК БЛИЖАЙШЕЙ ГИТАРНОЙ НОТЫ
//
//  Перебирает таблицу NOTES[] и находит ноту с минимальным
//  линейным отстоянием по частоте от детектированного значения.
//
//  Затем вычисляет отклонение в центах:
//  cents = 1200 * log2(f_detected / f_reference)
//  Положительное значение -> струна завышена (sharp, надо ослабить)
//  Отрицательное значение -> струна занижена (flat, надо натянуть)
// ============================================================
int findClosestNote(float freq, float* centsOut) {
    int   best  = 0;
    float bestD = 1e9f;
    for (int i = 0; i < NUM_NOTES; i++) {
        float d = fabsf(freq - NOTES[i].freq);
        if (d < bestD) { bestD = d; best = i; }
    }
    *centsOut = 1200.0f * log2f(freq / NOTES[best].freq);
    return best;
}

// ============================================================
//  ИЗМЕРЕНИЕ НАПРЯЖЕНИЯ БАТАРЕИ
//
//  АЦП ESP32 имеет нелинейность: реальное напряжение занижается
//  примерно на 5-10% в диапазоне 3.0-4.2 В.
//  Поправочный коэффициент ADC_CORRECTION получен сравнением
//  с мультиметром и применяется как финальный множитель.
//
//  Полная формула:
//  V_pin = ADC_raw / 4095 * 3.3 В
//  V_bat = V_pin * DIVIDER_RATIO * ADC_CORRECTION
// ============================================================
void updateBattery() {
    // Усредняем BAT_SAMPLES отсчётов для подавления шума АЦП
    long sum = 0;
    for (int i = 0; i < BAT_SAMPLES; i++) {
        sum += analogRead(BAT_ADC_PIN);
        delay(1);  // Небольшая задержка: RC цепь успевает зарядиться до нового замера
    }
    float adcRaw = (float)(sum / BAT_SAMPLES);

    // Пересчёт кода АЦП в напряжение на выводе делителя
    float adcV = adcRaw / ADC_RESOLUTION * ADC_REF_V;

    // Восстановление реального напряжения батареи:
    // умножаем на коэффициент делителя и на поправку нелинейности
    batteryV = adcV * DIVIDER_RATIO * ADC_CORRECTION;

    // Ограничение в физически разумных пределах Li-ion
    if (batteryV > BAT_MAX_V) batteryV = BAT_MAX_V;
    if (batteryV < BAT_MIN_V) batteryV = BAT_MIN_V;

    // Линейный пересчёт напряжения в проценты заряда
    // Примечание: реальная разрядная кривая Li-ion нелинейна,
    // для точного SOC нужна таблица или алгоритм Кулона.
    float pct  = (batteryV - BAT_MIN_V) / (BAT_MAX_V - BAT_MIN_V) * 100.0f;
    batteryPct = (int)pct;
    if (batteryPct < 0)   batteryPct = 0;
    if (batteryPct > 100) batteryPct = 100;
}

// ============================================================
//  ОТРИСОВКА ДИСПЛЕЯ
//
//  Макет (128x64 пикселей):
//  +------------------------------+
//  | HW TEST              87%    |  <- строки 7 / 10
//  |                             |
//  |           E2                |  <- строка 42 (шрифт 28px)
//  |         82.4 Hz             |  <- строка 52
//  | -12c   << <|     TOO LOW    |  <- строка 62
//  +------------------------------+
//
//  Отрисовка двухбуферная (u8g2 full-buffer режим):
//  clearBuffer() -> все команды draw -> sendBuffer()
//  Экран обновляется единым блоком -> нет мерцания.
// ============================================================
void drawDisplay() {
    u8g2.clearBuffer();  // Очищаем внутренний буфер (не сам экран)

    // -- Ошибка микрофона -- специальный экран --
    if (!micOk) {
        u8g2.setFont(u8g2_font_9x15B_tf);
        u8g2.drawStr(20, 28, "MIC ERROR");
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(10, 44, "Check INMP441 wiring");
        char bStr[20];
        snprintf(bStr, sizeof(bStr), "Bat: %d%%", batteryPct);
        u8g2.drawStr(0, 10, bStr);
        u8g2.sendBuffer();
        return;
    }

    // -- Заряд батареи (правый верхний угол) --
    char batStr[12];
    snprintf(batStr, sizeof(batStr), "%d%%", batteryPct);
    u8g2.setFont(u8g2_font_6x10_tf);
    int batW = u8g2.getStrWidth(batStr);
    u8g2.drawStr(128 - batW - 1, 10, batStr);

    // -- Метка "HW TEST" (левый верхний угол, мелкий шрифт) --
    //u8g2.setFont(u8g2_font_5x7_tf);
    //u8g2.drawStr(0, 7, "HW TEST");

    // -- Большая нота по центру ИЛИ прочерки --
    // displayNoteIdx == -1: нот ещё не было, или 30 с истекло.
    // Отображаем полный интерфейс тюнера с "--" вместо ноты.
    if (displayNoteIdx < 0) {
        // Прочерки вместо названия ноты (крупный шрифт)
        u8g2.setFont(u8g2_font_logisoso28_tf);
        int dw = u8g2.getStrWidth("--");
        u8g2.drawStr((128 - dw) / 2, 42, "--");

        // Прочерки вместо числового значения частоты
        u8g2.setFont(u8g2_font_6x10_tf);
        const char* fPh = "-- Hz";
        int fw = u8g2.getStrWidth(fPh);
        u8g2.drawStr((128 - fw) / 2, 52, fPh);

        // Нейтральный индикатор (только вертикальная черта по центру)
        u8g2.setFont(u8g2_font_6x10_tf);
        const char* iNeutral = "    |    ";
        int iw = u8g2.getStrWidth(iNeutral);
        u8g2.drawStr((128 - iw) / 2, 62, iNeutral);

        u8g2.sendBuffer();
        return;
    }

    // -- Название ноты (крупный шрифт, горизонтальное центрирование) --
    const char* noteName = NOTES[displayNoteIdx].name;
    u8g2.setFont(u8g2_font_logisoso28_tf);
    int nw = u8g2.getStrWidth(noteName);
    u8g2.drawStr((128 - nw) / 2, 42, noteName);

    // -- Числовое значение частоты --
    char freqStr[20];
    snprintf(freqStr, sizeof(freqStr), "%.1f Hz", displayFreq);
    u8g2.setFont(u8g2_font_6x10_tf);
    int fw = u8g2.getStrWidth(freqStr);
    u8g2.drawStr((128 - fw) / 2, 52, freqStr);

    // -- Стрелочный индикатор точности настройки --
    // Показывает положение струны относительно эталона:
    //   "    |    " -> IN TUNE  (в пределах +/-5 центов)
    //   "   <|    " -> TOO LOW  (-5..-20 центов) -- одна стрелка влево
    //   "<< <|    " -> TOO LOW  (< -20 центов)   -- двойная стрелка
    //   "    |>   " -> TOO HIGH (+5..+20 центов)
    //   "    |> >>" -> TOO HIGH (> +20 центов)
    const char* statusStr;
    char        indStr[20];

    if (fabsf(displayCents) <= IN_TUNE_CENTS) {
        statusStr = "IN TUNE";
        snprintf(indStr, sizeof(indStr), "    |    ");
    } else if (displayCents < -CLOSE_CENTS) {
        statusStr = "TOO LOW";
        snprintf(indStr, sizeof(indStr), "<< <|    ");
    } else if (displayCents < 0) {
        statusStr = "LOW";
        snprintf(indStr, sizeof(indStr), "   <|    ");
    } else if (displayCents > CLOSE_CENTS) {
        statusStr = "TOO HIGH";
        snprintf(indStr, sizeof(indStr), "    |> >>");
    } else {
        statusStr = "HIGH";
        snprintf(indStr, sizeof(indStr), "    |>   ");
    }

    // Стрелочный индикатор -- по горизонтальному центру
    u8g2.setFont(u8g2_font_6x10_tf);
    int iw = u8g2.getStrWidth(indStr);
    u8g2.drawStr((128 - iw) / 2, 62, indStr);

    // Текстовый статус (TOO LOW / IN TUNE / TOO HIGH) -- правый нижний угол
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(128 - u8g2.getStrWidth(statusStr) - 1, 62, statusStr);

    // Значение отклонения в центах -- левый нижний угол, со знаком
    char centsStr[10];
    snprintf(centsStr, sizeof(centsStr), "%+.0fc", displayCents);
    u8g2.drawStr(0, 62, centsStr);

    // Отправка буфера на дисплей -- единое обновление экрана, без мерцания
    u8g2.sendBuffer();
}

// ============================================================
//  ОТЛАДОЧНЫЙ ВЫВОД В SERIAL MONITOR (115200 бод)
//  Одна строка на каждый обработанный блок (~46 мс).
//  Формат:
//  RMS: [уровень] | Corr: [пик NACF] | Freq: [Гц] |
//  Note: [нота] | Cents: [+-центы] | Bat: [В] [%]
// ============================================================
void printDebug(float rms,
                float corrPeak,
                float freq,
                int noteIdx,
                float cents)
{
    Serial.printf(
        "RMS:%6.1f | "
        "Corr:%.3f | "
        "Oct:%.3f | "
        "R:%.3f | "
        "%s | "
        "F:%7.2f Hz | "
        "%-3s | "
        "%+5.1f c | "
        "%.2fV %d%%\n",

        rms,
        corrPeak,
        debugOctaveCorr,
        debugRatio,
        debugOctaveFix ? "FIX" : "---",
        freq,
        (noteIdx >= 0) ? NOTES[noteIdx].name : "---",
        cents,
        batteryV,
        batteryPct
    );
}
