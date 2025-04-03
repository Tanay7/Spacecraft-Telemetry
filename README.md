# Spacecraft Telemetry Transmitter (LoRa/CCSDS/RS-FEC)

**Author:** Tanay

## Overview

This project implements a telemetry transmitter, simulating a spacecraft system, designed to send various sensor data over a Long Range (LoRa) radio link. The system collects data from a suite of sensors, formats it into CCSDS (Consultative Committee for Space Data Systems) compliant packets, applies Reed-Solomon Forward Error Correction (RS-FEC) for enhanced reliability, and transmits the encoded data using an SX1262 LoRa module.

The project serves as an educational tool and demonstration platform for fundamental concepts in spacecraft communication, sensor integration, data packetization, and error correction, drawing parallels with real-world missions like Voyager 1 in its use of standardized protocols and error resilience techniques.

An optional U8g2-compatible monochrome display can be used to visualize real-time sensor readings, GPS status, power management information, and LoRa transmission parameters.

## Key Features

* **Multi-Sensor Integration:** Collects data from:
    * BME280: Temperature, Pressure, Humidity, Altitude
    * QMC6310: 3-Axis Magnetometer
    * QMI8658: 6-Axis IMU (3-Axis Accelerometer, 3-Axis Gyroscope) & Temperature
    * GPS: Latitude, Longitude, Altitude, Date, Time, Satellite Count, HDOP
    * Integrated PMU: Battery Voltage, Battery Percentage, Charging Status, VBUS Voltage, System Voltage
* **LoRa Communication:** Utilizes SX1262 LoRa module via the RadioLib library for long-range, low-power data transmission (433 MHz band configured).
* **CCSDS Packet Formatting:** Structures telemetry data into CCSDS Application Packets (AP) according to space communication standards for interoperability.
* **Reed-Solomon Forward Error Correction (RS-FEC):** Implements RS(255, 223) coding to add redundancy and improve data robustness against transmission errors over the wireless link.
* **CRC Checksum:** Includes a CRC-16 (CCITT-FALSE) checksum within the packet for data integrity verification at the receiver.
* **Optional Display:** Supports U8g2-compatible displays for real-time monitoring of system status and sensor data across multiple screens.
* **Modular Design:** Leverages external libraries for sensor interfacing, LoRa communication, GPS parsing, and display control.

## Prerequisites

### Hardware

* **Microcontroller:** An Arduino-compatible board supported by RadioLib and equipped with necessary I2C, SPI, and UART interfaces. The specific board and pin mappings are expected to be defined in `LoRaBoards.h`.
* **LoRa Module:** SX1262 LoRa transceiver module.
* **Sensors:**
    * Adafruit BME280 (or compatible) connected via I2C.
    * QMC6310 Magnetometer (specific module/breakout) connected via I2C.
    * QMI8658 IMU (specific module/breakout) connected via SPI (or potentially I2C depending on the board/library implementation).
    * GPS Module (e.g., u-blox) connected via UART (`SerialGPS`).
* **Power Management Unit (PMU):** An integrated PMU accessible via the `PMU` object (specific to certain boards like ESP32-S3 variants).
* **Display (Optional):** U8g2-compatible monochrome display (e.g., SSD1306 OLED) connected via I2C.
* **Wiring:** Appropriate connections for Power, Ground, I2C (SDA, SCL), SPI (MOSI, MISO, SCK, CS), UART (TX, RX), and LoRa module pins (NSS, DIO1, RST, BUSY). Refer to `LoRaBoards.h` for specific pin assignments.

### Software

* **Arduino IDE:** Or a compatible development environment (e.g., PlatformIO).
* **Libraries:**
    * `RadioLib`: For LoRa communication ([https://github.com/jgromes/RadioLib](https://github.com/jgromes/RadioLib))
    * `LoRaBoards.h`: **Custom, board-specific definitions required.** This file needs to be created or obtained for the target hardware, defining pins and board features for RadioLib and sensors.
    * `Wire`: Standard Arduino library for I2C.
    * `SPI`: Standard Arduino library for SPI.
    * `Adafruit Sensor`: ([https://github.com/adafruit/Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor))
    * `Adafruit BME280 Library`: ([https://github.com/adafruit/Adafruit_BME280_Library](https://github.com/adafruit/Adafruit_BME280_Library))
    * `SensorQMC6310.hpp`: **Custom library.** Needs to be provided or developed based on the specific QMC6310 sensor module used.
    * `SensorQMI8658.hpp`: **Custom library.** Needs to be provided or developed based on the specific QMI8658 sensor module used.
    * `U8g2`: For monochrome displays ([https://github.com/olikraus/u8g2](https://github.com/olikraus/u8g2))
    * `RS-FEC.h`: **Custom or specific library.** The Reed-Solomon implementation source needs to be available.
    * `TinyGPS++`: For GPS NMEA sentence parsing ([http://arduinogps.jjoe.org/](http://arduinogps.jjoe.org/))
    * Board Support Package (BSP): For the specific microcontroller board being used, including PMU access if applicable.

## Installation & Setup

1.  **Install Arduino IDE/PlatformIO:** Set up your preferred development environment.
2.  **Install Libraries:** Use the Arduino Library Manager (Sketch -> Include Library -> Manage Libraries...) or PlatformIO's Library Manager to install the required libraries (RadioLib, Adafruit Sensor, Adafruit BME280, U8g2, TinyGPS++).
3.  **Add Custom Libraries:** Manually add the custom/board-specific libraries (`LoRaBoards.h`, `SensorQMC6310.hpp`, `SensorQMI8658.hpp`, `RS-FEC.h`) to your Arduino sketchbook's `libraries` folder or include them in your PlatformIO project's `lib` directory.
4.  **Configure `LoRaBoards.h`:** **Crucially,** ensure `LoRaBoards.h` contains the correct pin definitions (`RADIO_CS_PIN`, `RADIO_DIO1_PIN`, `RADIO_RST_PIN`, `RADIO_BUSY_PIN`, `I2C_SDA`, `I2C_SCL`, `IMU_CS`, etc.) and board-specific initializations (`setupBoards()`, `u8g2` constructor, `SerialGPS` definition, `PMU` object access) for your target hardware.
5.  **Hardware Connections:** Wire the LoRa module, sensors, GPS, and display (if used) to the microcontroller according to the pin definitions in `LoRaBoards.h` and the component datasheets. Ensure correct voltage levels and logic compatibility.
6.  **Board Selection:** Select the correct microcontroller board in the Arduino IDE (Tools -> Board) or configure it in your `platformio.ini` file.
7.  **Port Selection:** Select the correct serial port for uploading (Tools -> Port).
8.  **Compile & Upload:** Compile the `TelemetryTx.ino` sketch and upload it to your microcontroller.
9.  **Serial Monitor:** Open the Serial Monitor (Tools -> Serial Monitor) at 115200 baud to view debug messages (if any are added) or confirm successful startup.

## Configuration

Several parameters can be adjusted within the `TelemetryTx.ino` code:

```c++
// --- General ---
#define SPACECRAFT_ID "DSN-2025-001" // Identifier for this transmitter

// --- LoRa Parameters (SX1262) ---
#define CARRIER_FREQ 433.0           // LoRa frequency in MHz
#define TX_POWER 20                  // LoRa transmit power in dBm (Check local regulations!)
#define BANDWIDTH 125.0              // LoRa bandwidth in kHz (e.g., 62.5, 125.0, 250.0, 500.0)
#define SPREADING_FACTOR 12          // LoRa spreading factor (7-12)
#define CODING_RATE 6                // LoRa coding rate (5-8)
#define SYNC_WORD 0x35               // LoRa sync word (byte, differentiates networks)

// --- Sensor & Timing ---
#define SEALEVELPRESSURE_HPA 1013.25 // Standard pressure at sea level for BME280 altitude calculation
const unsigned long sensorUpdateInterval = 100; // How often sensor data is read (milliseconds)
// Note: Display update interval is hardcoded to 2000ms (2 seconds) in updateDisplay()

// --- CCSDS ---
#define CCSDS_APID 0x7FF             // Application Process ID (example)

// --- Reed-Solomon FEC ---
// const int rsMsgLen = 223;        // Data bytes per block (defined by RS<msg, ecc> template)
// const uint8_t ECC_LENGTH = 32;   // Parity bytes per block (defined by RS<msg, ecc> template)

// --- Magnetometer Declination (Manual Adjustment Needed) ---
// Located inside updateQMC6310Data() function:
// float declination = -0.041;     // Adjust for your geographic location! (Example: Galway, Ireland)
