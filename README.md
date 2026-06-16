# ESP32 Guitar Tuner

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

<img width="1536" height="1024" alt="image" src="https://github.com/user-attachments/assets/868827cc-a3c3-4c6e-8253-1e09f4ad66a6" />


## Photos

(фото позже)

## Schematic

(схема позже)

## Demo

(видео позже)

## GIT_TUTORIAL

![alt text](image.png)
