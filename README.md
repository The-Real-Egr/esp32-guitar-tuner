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

## Photos

(фото позже)

## Schematic

(схема позже)

## Demo

(видео позже)
