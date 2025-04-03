# Spacecraft Telemetry Transmitter

## Author: Tanay

## Overview

This Arduino code implements a telemetry transmitter designed for a simulated spacecraft. It gathers data from various onboard sensors, including environmental, magnetic, inertial, GPS, and power management units. This sensor data is then structured into CCSDS (Consultative Committee for Space Data Systems) compliant packets, a standard used in real-world space missions like Voyager 1. To ensure reliable data transmission over a LoRa radio link, Reed-Solomon Forward Error Correction (RS-FEC) is applied to the telemetry packets.

The code initializes and reads data from the BME280 environmental sensor, QMC6310 magnetometer, QMI8658 Inertial Measurement Unit (IMU), a GPS module, and a Power Management Unit (PMU). This data is then formatted into a CCSDS Application Packet (AP). The packet is encoded using RS(255, 223) FEC to add redundancy for error correction during transmission. Finally, the encoded packet is transmitted using an SX1262 LoRa transceiver, managed by the RadioLib library.

An optional U8g2 display can be used to visualize the current sensor readings and system status on the transmitting end. The code also includes basic error handling for sensor initialization and a fatal error display routine for critical failures.

## Features

* **Multi-Sensor Data Acquisition:** Reads data from BME280 (temperature, pressure, humidity, altitude), QMC6310 (magnetic field), QMI8658 (accelerometer, gyroscope, temperature), GPS (location, time, satellites, HDOP), and PMU (battery voltage, percentage, charging, VBUS, system voltage).
* **CCSDS Packet Generation:** Structures the sensor data into CCSDS Application Packets (APs) following the Packet Utilization Standard for low-rate telemetry.
* **Reed-Solomon FEC Encoding:** Applies RS(255, 223) encoding to the CCSDS packets, adding 32 bytes of parity data for every 223 bytes of sensor data, enabling error correction on the receiving end.
* **LoRa Radio Transmission:** Utilizes the RadioLib library to control an SX1262 LoRa transceiver for transmitting the encoded telemetry data at 433.0 MHz.
* **Optional U8g2 Display Visualization:** Supports an optional U8g2 monochrome display to show real-time sensor readings and system information.
* **Basic Error Handling:** Includes checks for successful initialization of key sensors and a routine to display fatal error messages.
* **Packet Sequencing:** Implements a packet sequence counter within the CCSDS header for tracking transmitted packets.
* **CRC Checksum:** Calculates and includes a CRC-16 checksum (CCITT-FALSE) within the telemetry packet for data integrity verification by the receiver.

## Hardware Requirements

* Arduino-compatible microcontroller (specific board defined in `LoRaBoards.h`)
* SX1262 LoRa Transceiver module
* BME280 Environmental Sensor
* QMC6310 Magnetometer
* QMI8658 Inertial Measurement Unit (IMU)
* GPS Module (with serial output)
* PMU (Power Management Unit) compatible with the code's PMU access methods
* (Optional) U8g2 compatible monochrome display
* Wiring to connect all sensors, the LoRa transceiver, and the display to the microcontroller. Refer to `LoRaBoards.h` for pin assignments.

## Software Requirements

* Arduino IDE installed.
* The following Arduino libraries need to be installed via the Arduino Library Manager or by manually downloading and placing them in your Arduino libraries folder:
    * **RadioLib:** [https://github.com/jgromes/RadioLib](https://github.com/jgromes/RadioLib)
    * **Adafruit BME280 Library:** [https://github.com/adafruit/Adafruit_BME280_Library](https://github.com/adafruit/Adafruit_BME280_Library)
    * **U8g2lib:** [https://github.com/olikraus/u8g2](https://github.com/olikraus/u8g2)
    * **TinyGPS++:** [http://arduinogps.jjoe.org/](http://arduinogps.jjoe.org/)
* **LoRaBoards.h:** This is a board-specific header file containing pin definitions and should be included in the project. This file is likely custom and dependent on the specific hardware setup.
* **SensorQMC6310.hpp:** Custom library for the QMC6310 magnetometer. Ensure this file is in your sketch directory.
* **SensorQMI8658.hpp:** Custom library for the QMI8658 IMU. Ensure this file is in your sketch directory.
* **RS-FEC.h:** This header file provides the Reed-Solomon Forward Error Correction implementation. The source of this library is not explicitly mentioned in the code. Ensure you have the correct `RS-FEC.h` and its associated `.cpp` file (if any) in your project.
* Standard Arduino libraries (`Wire.h`, `SPI.h`, `math.h`, `stdio.h`) are typically included with the Arduino IDE.

## Installation

1.  **Install Arduino IDE:** Download and install the Arduino IDE from the official Arduino website.
2.  **Install Required Libraries:**
    * Open the Arduino IDE.
    * Go to `Sketch` > `Include Library` > `Manage Libraries...`.
    * Search for and install the following libraries: `RadioLib`, `Adafruit BME280 Library`, `U8g2`, and `TinyGPS++`.
    * Place the `LoRaBoards.h`, `SensorQMC6310.hpp`, `SensorQMI8658.hpp`, and `RS-FEC.h` (and any associated `.cpp` files) in the same directory as your `TelemetryTx.ino` sketch.
3.  **Connect Hardware:** Connect all the sensors, the LoRa transceiver, and the optional U8g2 display to your Arduino board according to the pin definitions specified in `LoRaBoards.h` and the sensor library documentation. Ensure proper power and ground connections.
4.  **Upload Code:**
    * Open the `TelemetryTx.ino` file in the Arduino IDE.
    * Select the correct board and port from the `Tools` menu.
    * Click the `Upload` button to compile and upload the code to your Arduino board.

## Configuration

The following parameters in the code can be configured:

* **Spacecraft Identification:**
    * `SPACECRAFT_ID`: Defines the identifier string for the simulated spacecraft.
* **CCSDS Packet Parameters:**
    * `CCSDS_APID`: Sets the Application Process Identifier for the CCSDS packets.
    * `CCSDS_VERSION`, `CCSDS_TYPE`, `CCSDS_SEC_HDR_FLAG`, `CCSDS_SEQ_FLAGS`: Define the respective fields in the CCSDS primary header.
* **Reed-Solomon FEC Parameters:**
    * `rsMsgLen`: Specifies the length of the data payload (must match the receiver).
    * `ECC_LENGTH`: Defines the length of the error correction code (must match the receiver).
* **LoRa Radio Configuration:**
    * `CARRIER_FREQ`: Sets the LoRa carrier frequency in MHz (must match the receiver).
    * `TX_POWER`: Configures the LoRa transmit power in dBm.
    * `BANDWIDTH`: Sets the LoRa bandwidth in kHz (must match the receiver).
    * `SPREADING_FACTOR`: Defines the LoRa spreading factor (must match the receiver).
    * `CODING_RATE`: Sets the LoRa coding rate (must match the receiver).
    * `SYNC_WORD`: Configures the LoRa sync word (network ID) (must match the receiver).
* **Sensor Update Intervals:**
    * `sensorUpdateInterval`: Controls the frequency at which sensor data is read (in milliseconds).
* **Display Settings:**
    * `defaultScreenDelay`, `gpsScreenDelay`: Determine the duration for which each screen is displayed on the optional U8g2 display (in milliseconds).
* **Sea Level Pressure:**
    * `SEALEVELPRESSURE_HPA`: Used for calculating altitude from the BME280 sensor. Adjust this value based on your local atmospheric conditions for more accurate altitude readings.

**Important:** Ensure that the LoRa radio configuration parameters (`CARRIER_FREQ`, `BANDWIDTH`, `SPREADING_FACTOR`, `CODING_RATE`, `SYNC_WORD`) and the Reed-Solomon FEC parameters (`rsMsgLen`, `ECC_LENGTH`) **exactly match** the configuration of the receiving telemetry system (`TelemetryRx.ino`).

## Usage

Once the code is uploaded and running, the microcontroller will start reading data from the connected sensors, building CCSDS telemetry packets, encoding them with Reed-Solomon FEC, and transmitting them over the LoRa radio link.

* **Serial Output:** The code initializes serial communication for debugging. You can use the Serial Monitor in the Arduino IDE to observe any debug messages or error outputs.
* **Optional U8g2 Display:** If a U8g2 display is connected and the `U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);` line is uncommented (and the correct constructor is used for your display), the display will cycle through different screens showing real-time readings from the connected sensors.
* **LoRa Transmission:** The encoded telemetry packets will be transmitted wirelessly via the LoRa radio at the configured frequency and settings. Ensure that a compatible LoRa receiver (like the `TelemetryRx.ino` code) is within range and configured to the same LoRa parameters to receive the data.

## CCSDS Packet Structure Notes

The telemetry data is encapsulated within CCSDS Application Packets (APs). The structure implemented in this code follows these guidelines:

* **Primary Header:** Consists of:
    * **Packet Version Number (3 bits):** Set to `0b000` (CCSDS version).
    * **Type Indicator (1 bit):** Set to `0b0` (Telemetry Packet).
    * **Secondary Header Flag (1 bit):** Set to `0b1` (Presence of Secondary Header - although a secondary header is not explicitly used in this basic example).
    * **Application Process Identifier (APID) (11 bits):** Set to `0x7FF` (example value, can be changed).
    * **Sequence Flags (2 bits):** Set to `0b11` (Continuation Segment - assuming packets are segmented if needed, though this example likely sends single packets).
    * **Packet Sequence Count (14 bits):** A counter that increments with each transmitted packet.
    * **Packet Length Field (16 bits):** Indicates the length of the packet data field minus one, in bytes.
* **Data Field:** Contains the sensor readings and other telemetry data as defined in the `CCSDSPacket` struct.
* **Packet Error Control (Optional):** A CRC-16 checksum (CCITT-FALSE) is calculated over the entire packet (excluding the CRC field itself) and appended for error detection.

## Reed-Solomon FEC Notes

The code employs an RS(255, 223) Reed-Solomon code for Forward Error Correction. This means that for every block of 223 bytes of telemetry data, 32 bytes of parity information are added. This redundancy allows the receiver to correct up to 16 byte errors within each 255-byte block, significantly improving the reliability of data transmission, especially in noisy environments or over longer distances.

## Voyager 1 Context

The code's use of CCSDS packet structures and error correction techniques like Reed-Solomon FEC is inspired by real spacecraft communication systems, such as the Voyager 1 mission. Voyager 1, launched in 1977 and still operational, transmits scientific data back to Earth from interstellar space. Due to the vast distances involved, the signals are extremely weak and prone to errors. Therefore, robust communication protocols and error correction codes, including CCSDS standards and variations of Reed-Solomon codes, are essential for ensuring the integrity of the received data. This simulated telemetry system demonstrates some of the fundamental principles behind spacecraft communication.

## Error Handling

The code includes basic error handling:

* **Sensor Initialization Checks:** During the `setup()` function, the code attempts to initialize the BME280, QMC6310, and QMI8658 sensors. If any of these initialization attempts fail, the `displayFatalError()` function is called.
* **Fatal Error Display:** The `displayFatalError()` function prints an error message to the serial monitor and, if a U8g2 display is connected, displays a "SYSTEM FAILURE" message along with the specific error. The program then enters an infinite loop, potentially blinking an LED (if `LED_BUILTIN` is defined) to indicate a critical failure.

## Contributing

Contributions to this project are welcome. If you find any issues or have suggestions for improvements, please feel free to open an issue or submit a pull request on the project's repository (if hosted on a platform like GitHub).

## License

[Specify the license under which this code is released, e.g., MIT License, Apache License 2.0, etc. If no license is specified, the default copyright rules apply.]

## Acknowledgments

* This code utilizes the excellent **RadioLib** library by [mention the author or organization if known] for LoRa communication.
* The **Adafruit BME280 Library** by Adafruit Industries is used for interfacing with the BME280 sensor.
* The **U8g2lib** library by [mention the author or organization, typically olikraus] is used for controlling the monochrome display.
* The **SensorQMC6310** and **SensorQMI8658** libraries are custom implementations likely provided by the sensor vendor or a third party.
* The **TinyGPS++** library by [mention the author, typically Mikal Hart] is used for parsing GPS data.
* The Reed-Solomon FEC implementation is provided by the `RS-FEC.h` library [mention the source or author if known].
* The CCSDS packet structure is based on the standards defined by the Consultative Committee for Space Data Systems.


