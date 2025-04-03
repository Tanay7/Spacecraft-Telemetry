# Spacecraft Telemetry Transmitter

**Author:** Tanay

## Overview

This Arduino code implements a telemetry transmitter designed for a simulated spacecraft. It collects data from various onboard sensors and transmits this information wirelessly using LoRa communication. The transmitted data is structured according to the Consultative Committee for Space Data Systems (CCSDS) standards, ensuring compatibility with common space communication protocols. To enhance data reliability over the radio link, Reed-Solomon Forward Error Correction (RS-FEC) is applied to the telemetry packets. An optional U8g2 display provides real-time visualization of sensor readings and system status. This project serves as an educational demonstration of fundamental spacecraft telemetry concepts, drawing inspiration from missions like Voyager 1.

## Key Features

* **Sensor Data Acquisition:** Reads data from a comprehensive suite of sensors:
    * **Environmental:** Temperature, pressure, humidity, and altitude from the BME280 sensor.
    * **Magnetic Field:** Three-axis measurements from the QMC6310 magnetometer.
    * **Inertial:** Accelerometer (X, Y, Z), gyroscope (X, Y, Z), and temperature from the QMI8658 Inertial Measurement Unit (IMU).
    * **GPS:** Location (latitude, longitude, altitude), date, time, number of satellites in view, and Horizontal Dilution of Precision (HDOP) from a GPS module.
    * **Power Management Unit (PMU):** Battery voltage, charge percentage, charging status, VBUS voltage, and system voltage.
* **CCSDS Packetization:** Structures sensor data into CCSDS Application Packets (APs) for standardized telemetry transmission.
* **LoRa Communication:** Utilizes the SX1262 LoRa transceiver and the RadioLib library for robust wireless data transmission. Configurable parameters include frequency, transmit power, bandwidth, spreading factor, and coding rate.
* **Reed-Solomon FEC:** Implements RS(255, 223) Reed-Solomon Forward Error Correction to improve data integrity by adding redundancy.
* **Optional Display:** Supports an U8g2 monochrome display for visualizing real-time sensor readings and system information through a series of rotating screens.
* **Data Integrity:** Includes a CRC-16 checksum (CCITT-FALSE algorithm) within the telemetry packet for error detection at the receiver.
* **Packet Sequencing:** Incorporates a packet sequence counter for tracking transmitted packets.
* **Error Handling:** Includes basic checks for sensor initialization and a fatal error display routine.

## Hardware Requirements

* Arduino-compatible microcontroller (specific board defined in `LoRaBoards.h`)
* SX1262 LoRa Transceiver Module
* BME280 Environmental Sensor
* QMC6310 Magnetometer
* QMI8658 Inertial Measurement Unit (IMU)
* GPS Module (with serial output)
* Power Management Unit (PMU) - Integrated PMU on the target board
* Optional: U8g2 compatible monochrome display (e.g., OLED or LCD)

**Note:** Pin assignments and board-specific configurations are likely defined in the `LoRaBoards.h` file.

## Software Requirements (Libraries)

* **RadioLib:** For LoRa radio communication ([https://github.com/jgromes/RadioLib](https://github.com/jgromes/RadioLib))
* **LoRaBoards.h:** Board-specific definitions for RadioLib (likely a custom file specific to the hardware setup).
* **Wire.h:** For I2C communication (used by BME280, QMC6310, and U8g2 display).
* **SPI.h:** For SPI communication (used by QMI8658 and potentially the LoRa module or SD card if used within the QMI8658 library).
* **Adafruit_Sensor.h & Adafruit_BME280.h:** For interfacing with the BME280 sensor ([https://github.com/adafruit/Adafruit_BME280_Library](https://github.com/adafruit/Adafruit_BME280_Library)).
* **SensorQMC6310.hpp & SensorQMI8658.hpp:** Custom sensor libraries for the QMC6310 magnetometer and QMI8658 IMU (these might be user-defined or provided by the sensor vendor).
* **U8g2lib.h:** For controlling the U8g2 monochrome display ([https://github.com/olikraus/u8g2](https://github.com/olikraus/u8g2)).
* **math.h:** For mathematical functions (e.g., `atan2`, `M_PI`, `pow`).
* **RS-FEC.h:** For Reed-Solomon Forward Error Correction (the source of this library is not explicitly mentioned and might be custom or from a specific library that needs to be identified and potentially linked here).
* **TinyGPS++.h:** For parsing data from the GPS module ([http://arduinogps.jjoe.org/](http://arduinogps.jjoe.org/)).
* **stdio.h:** For standard input/output functions, specifically `sprintf` in this code.

**Installation:**

1.  Ensure you have the Arduino IDE installed on your system.
2.  Install the required libraries using the Arduino Library Manager (Sketch > Include Library > Manage Libraries...). Search for and install:
    * RadioLib by Jan Gromes
    * Adafruit BME280 Library by Adafruit
    * Adafruit Unified Sensor by Adafruit
    * U8g2 by olikraus
    * TinyGPSPlus by Mikal Hart
3.  Obtain the custom sensor libraries (`SensorQMC6310.hpp` and `SensorQMI8658.hpp`) and the Reed-Solomon FEC library (`RS-FEC.h`). Place these files in the same directory as your `TelemetryTx.ino` sketch, or within the Arduino libraries folder.
4.  Ensure you have the `LoRaBoards.h` file configured correctly for your specific hardware setup, defining pin assignments for the LoRa module and other peripherals.
5.  Connect the sensors, LoRa module, display (if used), and GPS module to your Arduino-compatible microcontroller according to the pin definitions in `LoRaBoards.h` and the sensor/module datasheets.
6.  Open the `TelemetryTx.ino` file in the Arduino IDE.
7.  Select the correct board and port in the Arduino IDE (Tools > Board and Tools > Port).
8.  Upload the code to your microcontroller.

## Usage

Once the code is uploaded and running, the system will:

1.  Initialize the sensors and the LoRa radio.
2.  Continuously read data from the connected sensors at a defined interval.
3.  Format the sensor data into CCSDS telemetry packets.
4.  Apply Reed-Solomon Forward Error Correction to the packets.
5.  Transmit the encoded packets wirelessly using the LoRa radio.
6.  Optionally, display sensor readings and system status on the connected U8g2 display, cycling through different screens.

You will need a compatible LoRa receiver configured to the same parameters (frequency, bandwidth, spreading factor, coding rate, and sync word) to receive and decode the telemetry data. The received data will also need to be processed to remove the Reed-Solomon FEC and interpret the CCSDS packet structure.

## Configuration

The following parameters can be configured within the `TelemetryTx.ino` file:

* **Spacecraft Identification:** Modify the `SPACECRAFT_ID` macro to set a unique identifier for your simulated spacecraft.
* **CCSDS Packet Parameters:** The APID (`CCSDS_APID`), version (`CCSDS_VERSION`), type (`CCSDS_TYPE`), secondary header flag (`CCSDS_SEC_HDR_FLAG`), and sequence flags (`CCSDS_SEQ_FLAGS`) are defined as macros and can be adjusted if needed.
* **Reed-Solomon FEC Parameters:** The message length (`rsMsgLen`) and ECC length (`ECC_LENGTH`) are defined as constants. Ensure the `RS::ReedSolomon` template instantiation matches these values.
* **LoRa Radio Configuration:**
    * `CARRIER_FREQ`: LoRa carrier frequency in MHz (default: 433.0 MHz).
    * `TX_POWER`: LoRa transmit power in dBm (default: 20 dBm).
    * `BANDWIDTH`: LoRa bandwidth in kHz (default: 125.0 kHz).
    * `SPREADING_FACTOR`: LoRa spreading factor (default: 12).
    * `CODING_RATE`: LoRa coding rate (default: 6).
    * `SYNC_WORD`: LoRa sync word (network ID) (default: 0x35).
* **Sensor Update Interval:** The `sensorUpdateInterval` constant (in milliseconds) controls how frequently sensor data is read.
* **Display Screen Delay:** The `defaultScreenDelay` and `gpsScreenDelay` constants (in milliseconds) control the duration each screen is displayed on the U8g2 display.
* **GPS Serial Port:** The serial port used for the GPS module is initialized with `SerialGPS.begin(9600);`. Adjust the baud rate if your GPS module requires a different setting.
* **Sea Level Pressure:** The `SEALEVELPRESSURE_HPA` constant is used for altitude calculation by the BME280 sensor. You may need to adjust this value based on your local atmospheric conditions for more accurate altitude readings.
* **Magnetic Declination:** The `declination` variable in the `updateQMC6310Data()` function is set for Galway, Ireland. You should update this value to the magnetic declination for your specific location to obtain accurate compass heading readings.

**Note:** Refer to the datasheets of your specific hardware components for recommended configuration values.

## Error Handling

The code includes basic error handling for sensor initialization. If any of the sensors (BME280, QMC6310, or QMI8658) fail to initialize, a fatal error message will be displayed on the U8g2 display (if connected) using the `displayFatalError()` function, and the system will halt.

## Educational Purpose

This code is intended for educational purposes and serves as a demonstration of fundamental concepts in spacecraft telemetry, including:

* Sensor data acquisition from various types of sensors.
* Data structuring according to space communication standards (CCSDS).
* Wireless data transmission using LoRa technology.
* Improving data reliability through Forward Error Correction (Reed-Solomon).
* Real-time data visualization.

## Target Platform

This code is designed to be compatible with Arduino-compatible microcontrollers. The specific target board is defined in the `LoRaBoards.h` file, which likely contains pin definitions and other board-specific configurations.

## LoRa Frequency

The LoRa communication is configured to operate at a frequency of 433.0 MHz. This can be changed by modifying the `CARRIER_FREQ` macro. Ensure that the chosen frequency is compliant with the regulations in your region.

## CCSDS Packet Structure Notes

The telemetry data is encapsulated within a CCSDS Application Packet (AP). The primary header includes the following fields:

* **Packet Version Number (3 bits):** Set to `0b000` (CCSDS version).
* **Type Indicator (1 bit):** Set to `0b0` (Telemetry Packet).
* **Secondary Header Flag (1 bit):** Set to `0b1`, indicating the presence of a secondary header (although a secondary header is not explicitly implemented in this basic example).
* **Application Process Identifier (APID) (11 bits):** Set to `0x7FF` as an example value. This should be a unique identifier for the telemetry application.
* **Sequence Flags (2 bits):** Set to `0b11` (Continuation Segment), assuming packets might be segmented if the data payload becomes larger in more complex scenarios. In this example, single packets are likely sent.
* **Packet Sequence Count (14 bits):** A counter that increments with each transmitted packet, modulo 16384.
* **Packet Length Field (2 bytes):** Indicates the length of the packet data field minus one, in bytes.

The data field of the CCSDS packet includes the spacecraft ID, mission time, sensor readings, PMU data, a message string, and the CRC-16 checksum.

## Reed-Solomon FEC Notes

Reed-Solomon Forward Error Correction (RS-FEC) is implemented using an RS(255, 223) code. This means that for every 223 bytes of sensor data, 32 bytes of parity information are added. These parity bytes allow the receiver to detect and correct errors that may occur during transmission, significantly improving the reliability of the telemetry link, especially over longer distances or in noisy environments.

## Voyager 1 Context

The design of this telemetry transmitter draws inspiration from spacecraft missions like Voyager 1. Voyager 1, launched in 1977 and still operational, transmits scientific data back to Earth from interstellar space. While Voyager utilizes the Deep Space Network (DSN) for communication over vast distances, this project employs LoRa for shorter-range communication, demonstrating similar fundamental principles of telemetry data acquisition, packetization according to standards like CCSDS, and the use of error correction techniques like Reed-Solomon to ensure data integrity. The types of sensor data included in this code (environmental, magnetic, inertial, and positional) are representative of the data collected by spacecraft like Voyager, although the actual sensor suite on Voyager is far more extensive and sophisticated. This project provides a simplified yet illustrative example of the complex telemetry systems used in real-world space missions.
