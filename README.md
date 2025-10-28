# ESP32 UART-TCP Bridge

A TCP to UART bridge implementation for ESP32 using ESP-IDF.

## Features
- WiFi Station mode connectivity
- TCP server on port 9000
- UART bridge with 921600 baud rate
- Non-blocking socket operations

## Building
1. Install ESP-IDF
2. Configure WiFi credentials:
   ```bash
   idf.py menuconfig
   ```
3. Build and flash:
   ```bash
   idf.py build flash
   ```