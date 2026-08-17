<img width="1191" height="1322" alt="image" src="https://github.com/user-attachments/assets/ab1ad2b0-8ee8-4369-8112-a70e31e838db" /># ESP32 Guitar Tuner

Digital guitar tuner based on ESP32 with real-time pitch detection.

## Features

- Real-time pitch detection
- Guitar note recognition
- OLED SSD1306 display
- INMP441 I2S microphone
- Noise rejection
- Auto-correlation based pitch detection

## Hardware

- ESP32 DevKit V1
- INMP441 microphone
- OLED SSD1306 (I2C)
- Li-ion 18650 battery
- TP4056 charging module

## Architecture

AudioCapture
↓
SignalPreprocessor
↓
PitchDetector
↓
NoteEstimator
↓
UI

## Photos

<img width="1191" height="1322" alt="image" src="https://github.com/user-attachments/assets/cf0af7c7-1ae0-4c8a-a333-537332b9b4d8" />


## Schematic

<img width="1880" height="925" alt="image" src="https://github.com/user-attachments/assets/3a3d148a-32e2-43e7-8072-e5ea415d7f01" />


## Demo



https://github.com/user-attachments/assets/56c4f8ae-7c36-4c9b-8713-583998ae20d5


