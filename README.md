# Project-Star: Survey and Terrain Analysis Robot (STAR)

[![Documentation Status](https://img.shields.io/badge/docs-latest-brightgreen.svg)](https://topographic-robot.github.io/Topographic-Robot-Documentation/html/index.html)

**STAR** is an advanced terrain-mapping robot designed to traverse various landscapes, including hills and valleys, to create detailed 3D mesh topology maps. This repository contains the firmware for the ESP32 microcontroller that powers STAR, along with configuration files, hardware information, and supporting scripts.

## Table of Contents

*   [Features](#features)
*   [Hardware](#hardware)
*   [Software](#software)
*   [Installation](#installation)
*   [Wiring](#wiring)
*   [JTAG Debugging](#jtag-debugging)
*   [Documentation](#documentation)
*   [Contributing](#contributing)
*   [License](#license)

## Features

*   **Terrain Mapping:** Generates detailed 3D mesh topology maps of diverse terrains.
*   **Autonomous Navigation:** Designed to traverse hills, valleys, and other challenging landscapes.
*   **Sensor Suite:** Equipped with a comprehensive set of sensors for data acquisition:
    *   **Environmental:** DHT22 (Temperature & Humidity), BH1750 (Ambient Light), SEN-CCS811 & MQ135 (Air Quality).
    *   **Motion & Orientation:** MPU6050 (Accelerometer & Gyroscope), QMC5883L (Magnetometer).
    *   **Localization:** GY-NEO6MV2 (GPS).
    *   **Imaging:** OV7670 Camera (controlled by DE10-Lite FPGA).
*   **Data Storage:** Utilizes a Micro-SD card for onboard data logging.
*   **Remote Communication:** Implements Wi-Fi connectivity for data transmission and potential remote control.
*   **Modular Design:** Built using ESP-IDF with a modular component structure for easy maintenance and expansion.
*   **JTAG Debugging:** Supports JTAG debugging for development and troubleshooting.
*   **Interrupt-Driven Sensor Handling:** The MPU6050 sensor implementation uses interrupts for efficient data acquisition.

## Hardware

The core of STAR is an ESP32 microcontroller. It interacts with the following hardware components:

*   **Microcontroller:** ESP32
*   **Sensors:**
    *   DHT22 (Temperature & Humidity)
    *   MPU6050 (Accelerometer & Gyroscope)
    *   BH1750 (Ambient Light)
    *   QMC5883L (Magnetometer)
    *   OV7670 (Camera, controlled by DE10-Lite)
    *   SEN-CCS811 (Air Quality - eCO2, TVOC)
    *   MQ135 (Air Quality - various gases)
    *   GY-NEO6MV2 (GPS)
*   **Storage:** Micro-SD Card Module
*   **Actuators:** Motors controlled via PCA9685 PWM Driver
*   **Communication:** Wi-Fi (ESP32 built-in)
*   **Power:** 3.3V power supply for most components, with potential external power for motors
*   **FPGA (External):** DE10-Lite (for camera control)
*   **JTAG Debugger:** CJMCU-4232 (optional, for debugging)

**Schematics:**

Refer to the images below for the schematics of the robot:

![Print Schematic-2](https://github.com/user-attachments/assets/91e3658f-626d-4d3d-8d84-060d94f8161d)
![Print Schematic-3](https://github.com/user-attachments/assets/b480c20d-7b98-4bb1-b207-027b9d4634cc)
![Print Schematic-4](https://github.com/user-attachments/assets/fd70dec2-0874-4dc0-99f8-d9a695c5698a)
![Print Schematic-5](https://github.com/user-attachments/assets/cfb79a64-f3b6-4450-9d41-8d787df71902)

## Software

The firmware is developed using the **ESP-IDF** framework and written primarily in **C**. It utilizes **FreeRTOS** for task management.

**Software Components:**

*   **`components/`:** Contains modular components:
    *   **`camera/`:** Hardware Abstraction Layer (HAL) for the OV7670 camera.
    *   **`common/`:** I2C, SPI, and UART communication drivers.
    *   **`controllers/`:** HAL for the PCA9685 PWM driver.
    *   **`sensors/`:** HALs for each sensor (BH1750, DHT22, MPU6050, QMC5883L, GY-NEO6MV2, CCS811, MQ135).
    *   **`storage/`:** HAL for SD card interactions.
*   **`main/`:**
    *   **`include/managers/`:** Time management and file writing utilities.
    *   **`include/tasks/`:** Task definitions for motor control, sensor data acquisition, Wi-Fi management, and web server communication.
    *   `main.c`: Main application entry point.

**Dependencies:**

*   **ESP-IDF:** v4.4 or later (check `idf.py --version`)
*   **cJSON:** For JSON data formatting.
*   **FATFS:** For SD card file system operations.

## Installation

1. **Install ESP-IDF:**
    Follow the official ESP-IDF installation instructions: [https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)

2. **Clone the Repository:**
    ```bash
    git clone https://github.com/Topographic-Robot/Project-Star.git
    cd Project-Star
    ```

3. **Configure Wi-Fi Credentials:**
    *   Copy the `main/include/tasks/include/wifi_credentials.txt` file to `main/include/tasks/include/wifi_credentials.h`.
    *   Edit `wifi_credentials.h` and replace the placeholder values for `wifi_ssid` and `wifi_pass` with your actual Wi-Fi network credentials. **Do not commit this file to Git.**

4. **Configure Webserver URL (Optional):**
    *   If you intend to send sensor data to a web server, copy the `main/include/tasks/include/webserver_info.txt` file to `main/include/tasks/include/webserver_info.h`.
    *   Edit `webserver_info.h` and replace the placeholder value for `webserver_url` with your server's URL. **Do not commit this file to Git.**

5. **Build and Flash:**
    ```bash
    idf.py set-target esp32
    idf.py build
    idf.py -p /dev/ttyUSB0 flash monitor  # Replace /dev/ttyUSB0 with your ESP32's port
    ```

## Wiring

The `Wiring.txt` file in the repository provides detailed pin connections between the ESP32 and each component. Here's a summary:

**Important:** Always double-check the pin connections in `Wiring.txt` before powering on the robot.

| Component       | Component Pin | ESP32 Pin       | Notes                                                                                                    |
| --------------- | ------------- | --------------- | -------------------------------------------------------------------------------------------------------- |
| DHT22           | DATA          | GPIO_NUM_4 (D4)  | Temperature and humidity sensor.                                                                         |
| MPU6050         | SDA           | GPIO_NUM_21 (D21) | Accelerometer and gyroscope. Uses I2C.                                                                 |
|                 | SCL           | GPIO_NUM_22 (D22) |                                                                                                          |
|                 | INT           | GPIO_NUM_26 (D26) | Interrupt pin for motion detection, data ready, etc.                                                    |
| BH1750          | SDA           | GPIO_NUM_21 (D21) | Ambient light sensor. Uses I2C.                                                                       |
|                 | SCL           | GPIO_NUM_22 (D22) |                                                                                                          |
| QMC5883L        | SDA           | GPIO_NUM_21 (D21) | Magnetometer. Uses I2C.                                                                               |
|                 | SCL           | GPIO_NUM_22 (D22) |                                                                                                          |
|                 | DRDY          | GPIO_NUM_18 (D18) | Data Ready pin (currently marked as TODO in the `Wiring.txt`).                                        |
| OV7670          | SDA           | GPIO_NUM_21 (D21) | Camera module. Uses I2C for configuration. Data and control signals are managed by the DE10-Lite FPGA. |
|                 | SCL           | GPIO_NUM_22 (D22) |                                                                                                          |
| Micro-SD Reader | MOSI          | GPIO_NUM_23 (D23) | Uses SPI.                                                                                               |
|                 | MISO          | GPIO_NUM_19 (D19) |                                                                                                          |
|                 | CLK           | GPIO_NUM_14 (D14) |                                                                                                          |
|                 | CS            | GPIO_NUM_5 (D5)   |                                                                                                          |
| SEN-CCS811      | SDA           | GPIO_NUM_21 (D21) | Air quality sensor (eCO2, TVOC). Uses I2C.                                                              |
|                 | SCL           | GPIO_NUM_22 (D22) |                                                                                                          |
|                 | WAKE          | GPIO_NUM_33 (D33) |                                                                                                          |
|                 | RST           | GPIO_NUM_32 (D32) |                                                                                                          |
|                 | INT           | GPIO_NUM_25 (D25) |                                                                                                          |
| MQ135           | A0            | GPIO_NUM_34 (D34) | Air quality sensor (various gases). Analog output.                                                    |
|                 | D0            | GPIO_NUM_35 (D35) | Digital output (optional).                                                                               |
| GY-NEO6MV2      | TX            | GPIO_NUM_16 (RX2) | GPS module. Uses UART.                                                                                  |
|                 | RX            | GPIO_NUM_17 (TX2) |                                                                                                          |
| PCA9685         | SDA           | GPIO_NUM_21 (D21) | PWM driver (for motor control). Uses I2C.                                                              |
|                 | SCL           | GPIO_NUM_22 (D22) |                                                                                                          |
|                 | OE            | GPIO_NUM_15 (D15) | Output Enable.                                                                                           |

## JTAG Debugging

The `JTAG-info.txt` file provides instructions for setting up JTAG debugging using a CJMCU-4232 debugger and OpenOCD. Here's a summarized procedure:

1. **Connect the CJMCU-4232:** Refer to the pin mapping in `JTAG-info.txt` to connect the debugger to the ESP32's JTAG pins.
2. **Install OpenOCD:** If you don't have OpenOCD installed, follow the instructions for your operating system.
3. **Identify the ESP32 with `lsusb` (or equivalent):** This will help you find the correct `VID` and `PID` values.
4. **Run OpenOCD:**

    ```bash
    openocd -f board/esp32-wrover-kit-3.3v.cfg -c "ftdi_vid_pid 0x0403 0x6011"
    ```

    *   Replace `0x0403` and `0x6011` with the `VID` and `PID` values for your ESP32 if necessary.

5. **Connect with GDB:**

    ```bash
    xtensa-esp32-elf-gdb -ex "target remote localhost:3333" build/Topographic-Robot.elf
    ```

## Documentation

For more detailed information about the project, including hardware specifications, software architecture, and API documentation, please refer to the online documentation:

[https://topographic-robot.github.io/Topographic-Robot-Documentation/html/index.html](https://topographic-robot.github.io/Topographic-Robot-Documentation/html/index.html)

## Contributing

Contributions to Project-Star are welcome! Please follow these guidelines:

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Write clear and concise code with proper comments.
4. Test your changes thoroughly.
5. Submit a pull request with a detailed description of your changes.

## License

This project is licensed under the [MIT License](LICENSE) - see the `LICENSE` file for details.
