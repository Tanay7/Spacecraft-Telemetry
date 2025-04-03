# Spacecraft Telemetry Receiver

## Author: Tanay

## Overview

This Arduino code implements a telemetry receiver for a simulated spacecraft. It is designed to receive sensor data transmitted over a LoRa radio link. The received data is expected to be structured according to the Consultative Committee for Space Data Systems (CCSDS) packet format and encoded using Reed-Solomon Forward Error Correction (RS-FEC) for enhanced data integrity.

The receiver demodulates the incoming LoRa signals, decodes the Reed-Solomon FEC, verifies the Cyclic Redundancy Check (CRC) checksums, and processes the resulting CCSDS telemetry packets. It continuously monitors the signal quality by tracking the Received Signal Strength Indication (RSSI) and Signal-to-Noise Ratio (SNR). The code also maintains counts of successfully received packets and various error types, including CRC errors, RS-FEC errors, and data range violations.

The decoded telemetry data and system status are displayed on a connected U8g2 monochrome display, providing a real-time view of the spacecraft's sensor readings and communication link performance. Detailed packet information and debugging logs are also outputted via the serial interface.

## Features

* **LoRa Demodulation:** Utilizes the RadioLib library to demodulate LoRa signals received by an SX1262 transceiver.
* **CCSDS Packet Processing:** Parses received data according to the expected CCSDS Application Packet (AP) structure.
* **Reed-Solomon FEC Decoding:** Employs RS(255, 223) decoding to correct up to 16 byte errors per received block, ensuring data accuracy.
* **CRC Checksum Verification:** Calculates and verifies a 16-bit CRC (CCITT-FALSE) to detect any uncorrected transmission errors.
* **Comprehensive Telemetry Data Display:** Visualizes a wide range of spacecraft sensor data on a U8g2 display, including:
    * **Environmental Data (BME280):** Temperature, pressure, humidity, and altitude.
    * **Magnetic Field Data (QMC6310):** X, Y, and Z axis magnetic field readings.
    * **Inertial Data (QMI8658):** Accelerometer (X, Y, Z), gyroscope (X, Y, Z), and temperature readings.
    * **GPS Data:** Latitude, longitude, altitude, date, time, number of satellites in view, and HDOP.
    * **Power Management Unit (PMU) Data:** Battery voltage, battery percentage, charging status, VBUS voltage, and system voltage.
    * **Packet Information:** Packet sequence count and a custom message string.
* **Signal Quality Monitoring:** Displays real-time RSSI and SNR values of the received LoRa signal.
* **Packet Reception Statistics:** Tracks and displays the number of valid packets received, CRC errors, Reed-Solomon errors, and data range errors.
* **Error Handling:** Implements data range validation to ensure the received sensor data falls within expected limits.
* **Fatal Error Handling:** Includes a dedicated routine to display critical system failures on the U8g2 display and halt execution.
* **Serial Output for Logging and Debugging:** Provides detailed logging of received packets and system events via the serial interface.

## Hardware Requirements

* Arduino-compatible microcontroller (specific board defined in `LoRaBoards.h`)
* SX1262 LoRa Transceiver module
* U8g2 compatible monochrome display (SH1106 128x64 OLED recommended)
* Wiring to connect the LoRa transceiver and display to the microcontroller (refer to `LoRaBoards.h` for pin assignments).
* (Optional) TCXO (Temperature Compensated Crystal Oscillator) for the LoRa module, if supported and defined in `LoRaBoards.h`.

## Software Requirements

* Arduino IDE installed.
* The following Arduino libraries need to be installed via the Arduino Library Manager or by manually downloading and placing them in your Arduino libraries folder:
    * **RadioLib:** [https://github.com/jgromes/RadioLib](https://github.com/jgromes/RadioLib)
    * **U8g2lib:** [https://github.com/olikraus/u8g2](https://github.com/olikraus/u8g2)
* **LoRaBoards.h:** This is a board-specific header file containing pin definitions and should be included in the project. This file is likely custom and dependent on the specific hardware setup.
* **RS-FEC.h:** This header file provides the Reed-Solomon Forward Error Correction implementation. The source of this library is not explicitly mentioned in the code. Ensure you have the correct `RS-FEC.h` and its associated `.cpp` file (if any) in your project.
* Standard Arduino libraries (`Wire.h`, `SPI.h`) are typically included with the Arduino IDE.

## Installation

1.  **Install Arduino IDE:** Download and install the Arduino IDE from the official Arduino website.
2.  **Install Required Libraries:**
    * Open the Arduino IDE.
    * Go to `Sketch` > `Include Library` > `Manage Libraries...`.
    * Search for and install the following libraries: `RadioLib` and `U8g2`.
    * Locate or obtain the `LoRaBoards.h` and `RS-FEC.h` (and potentially `.cpp`) files and place them in the same directory as your `TelemetryRx.ino` sketch.
3.  **Connect Hardware:** Connect the SX1262 LoRa transceiver and the U8g2 display to your Arduino board according to the pin definitions in `LoRaBoards.h`. Ensure proper power and ground connections.
4.  **Upload Code:**
    * Open the `TelemetryRx.ino` file in the Arduino IDE.
    * Select the correct board and port from the `Tools` menu.
    * Click the `Upload` button to compile and upload the code to your Arduino board.

## Configuration

The following parameters in the code **MUST** match the configuration used by the telemetry transmitter (`TelemetryTx.ino`):

* **LoRa Radio Configuration:**
    * `CARRIER_FREQ`: LoRa carrier frequency in MHz (e.g., `433.0`).
    * `BANDWIDTH`: LoRa bandwidth in kHz (e.g., `125.0`).
    * `SPREADING_FACTOR`: LoRa spreading factor (7-12) (e.g., `12`).
    * `CODING_RATE`: LoRa coding rate (5-8) (e.g., `6`).
    * `SYNC_WORD`: LoRa sync word (network ID) (e.g., `0x35`).
* **Reed-Solomon FEC Configuration:**
    * `rsMsgLen`: Length of the Reed-Solomon message (data bytes) (e.g., `223`).
    * `ECC_LENGTH`: Length of the Reed-Solomon error correction code (parity bytes) (e.g., `32`).
* **RSSI Threshold:**
    * `rssiThreshold`: Minimum RSSI value (in dBm) for a packet to be considered valid (e.g., `-100.0`). Adjust this value based on your environment and expected signal strength.

Ensure that the pin definitions in `LoRaBoards.h` are correctly configured for your specific Arduino board and hardware connections.

## Usage

Once the code is uploaded and running, the telemetry receiver will continuously listen for LoRa packets.

* **U8g2 Display:** The received telemetry data and system status will be displayed on the connected U8g2 display. The display cycles through different screens showing various sensor readings, RF information, and error statistics every 2 seconds.
* **Serial Output:** Detailed information about each received packet, including sensor data, RSSI, SNR, and CRC status, will be printed to the Serial Monitor in the Arduino IDE. This output is useful for debugging and logging.

The receiver tracks the number of valid packets, CRC errors, Reed-Solomon errors, and data range errors. These statistics are displayed on one of the U8g2 display screens.

If a fatal error occurs (e.g., radio initialization failure), an error message will be displayed on the U8g2 screen, and the program will halt, potentially blinking an LED if `LED_BUILTIN` is defined.

## CCSDS Packet Structure Notes

The code expects the received telemetry data to be structured as a CCSDS Application Packet (AP). The structure of this packet is defined by the `CCSDSPacket` struct in the code and should match the packet structure used by the transmitting spacecraft (`TelemetryTx.ino`). Key assumptions about the CCSDS packet structure include:

* **Version, Type, APID:** The `version_type_apid` field contains the CCSDS packet version number, type, and Application Process Identifier (APID). These should be consistent with the transmitter.
* **Sequence Flags, Packet Sequence Count:** The `sequence_flags_count` field includes sequence flags and a packet sequence counter for tracking packet order.
* **Packet Length Field:** The `packet_length` field indicates the total length of the packet data minus 7 bytes (primary header size).

## Reed-Solomon FEC Notes

The receiver performs RS(255, 223) decoding. This configuration implies:

* **Total Block Size:** 255 bytes (data + parity).
* **Data Bytes:** 223 bytes (the `rsMsgLen`).
* **Parity Bytes (Error Correction Code):** 32 bytes (the `ECC_LENGTH`).

This Reed-Solomon scheme can correct up to 16 byte errors within each 255-byte block. The code tracks the number of Reed-Solomon decoding errors (`rsErrors`) to monitor the quality of the communication link.

## Signal Quality Monitoring

The code continuously monitors the following signal quality parameters:

* **RSSI (Received Signal Strength Indication):** Measured in dBm, RSSI indicates the power level of the received LoRa signal. A higher (less negative) RSSI value generally indicates a stronger signal.
* **SNR (Signal-to-Noise Ratio):** Measured in dB, SNR represents the ratio of the desired signal power to the background noise power. A higher SNR value indicates a cleaner signal with less interference.

The `rssiThreshold` is used to filter out packets received with very weak signals, preventing the processing of potentially corrupted data.

## Contributing

Contributions to this project are welcome. If you find any issues or have suggestions for improvements, please feel free to open an issue or submit a pull request on the project's repository (if hosted on a platform like GitHub).

## License

[Specify the license under which this code is released, e.g., MIT License, Apache License 2.0, etc. If no license is specified, the default copyright rules apply.]

## Acknowledgments

* This code utilizes the excellent **RadioLib** library by [mention the author or organization if known] for LoRa communication.
* The **U8g2lib** library by [mention the author or organization, typically olikraus] is used for controlling the monochrome display.
* The Reed-Solomon FEC implementation is provided by the `RS-FEC.h` library [mention the source or author if known].
* The CCSDS packet structure is based on the standards defined by the Consultative Committee for Space Data Systems.
