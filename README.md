This project uses two ESP32-based boards to monitor environmental data, detect nearby human presence, capture sound anomalies, display live status, and optionally send environmental/event data over LoRaWAN.

## Project Overview

The system is split into two devices:

### Transmitter

Board A:

- XIAO ESP32S3 Sense
- Grove sensors
- SCD41 CO₂ / temperature / humidity sensor
- Ultrasonic distance sensor
- Analog microphone
- SD card module
- Wio-E5 LoRa module
- Camera
- Speaker

Main functions:

- Reads CO₂, temperature, and humidity
- Detects nearby human presence using ultrasonic distance
- Captures analog microphone audio
- Detects loud sound anomalies
- Saves anomaly audio clips to SD card
- Sends live status to the receiver over ESP-NOW
- Sends periodic/event data over LoRaWAN
- Plays a bird-like sound when a person is detected

### Receiver

Board B:

- ESP32-based board
- TFT display

Main functions:

- Receives live status packets over ESP-NOW
- Displays time, CO₂, humidity, temperature, and status pages
- Shows a bird animation when a person is nearby
- Shows an anomaly warning when loud audio is detected
- Uses WiFi only for NTP time sync, then switches back to ESP-NOW operation

## Repository Structure

```text
.
├── transmitter/
│   └── transmitter_proposed_no_tflm_auto_channel.ino
├── receiver/
│   └── receiver_lightweight_ui.ino
└── README.md
