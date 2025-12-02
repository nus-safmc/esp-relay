# ESP32 UART-UDP MavLink Bridge

A UDP to UART bridge implementation for ESP32 using ESP-IDF to relay MavLink packets between computer and drone.

## Features
- WiFi Station mode connectivity
- UDP server on port 8888
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

## Using the relay
1. Make sure the wifi credentials are set correctly in udp_server.c
2. Once code is flashed to ESP32, relay should be start once WiFi connection is achieved.
3. To communicate with PX4 FC, get the IP address of the ESP32 (use idf.py monitor), and establish a MAVROS connection with the IP on port 8888