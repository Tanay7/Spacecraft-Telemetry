# Voyager 1 Telemetry Simulation on T-Beam SUPREME LoRa Board

## Author: Tanay

*(Revised README incorporating license and acknowledgment details)*

## Introduction

This project simulates a basic spacecraft telemetry system, demonstrating the fundamental principles of data acquisition, packetization, error correction, and wireless communication. It consists of two main components: a **transmitter** that simulates a spacecraft gathering sensor data and sending it to Earth, and a **receiver** that simulates a ground station receiving and processing this data. The system utilizes LoRa radio for communication, structures data into CCSDS (Consultative Committee for Space Data Systems) compliant packets, and employs Reed-Solomon Forward Error Correction (RS-FEC) for robust data transmission. **This project was built and tested using the LilyGO T-Beam Supreme module.**

## Voyager 1 Context

The design of this telemetry system draws inspiration from real-world spacecraft missions, most notably the Voyager 1 mission. Launched in 1977, Voyager 1 is one of humanity's most iconic deep space probes, currently exploring interstellar space billions of miles from Earth. Its communication system provides a compelling example of the challenges and solutions involved in transmitting data over vast distances.

![Voyager 1 Spacecraft](https://github.com/Tanay7/Spacecraft-Telemetry/raw/main/images/Voyager_1.jpg)
*(Image source: NASA/JPL-Caltech)*

* **Deep Space Network (DSN):** Voyager 1 communicates with Earth using NASA's Deep Space Network, a network of large parabolic radio antennas located around the world. These antennas are highly sensitive to the faint signals received from deep space probes. Our simulation uses LoRa, a lower-power, shorter-range radio technology, but the concept of a dedicated network for receiving spacecraft data is analogous.
* **Data Rates and Distances:** Due to the immense distance, the data rate from Voyager 1 is extremely low, measured in bits per second (bps). As of recent years, the downlink rate was around 160 bps. While our LoRa simulation has a much higher potential data rate, the principle of balancing data rate with range and signal strength remains relevant.
* **Importance of Error Correction:** Over such vast distances, signals are significantly attenuated and corrupted by noise. Therefore, robust error correction codes are crucial for ensuring the integrity of the received scientific data. Voyager 1 has used various coding schemes throughout its mission, including convolutional codes and Reed-Solomon codes. Our use of RS(255, 223) FEC mirrors this critical aspect of deep space communication.
* **CCSDS Standards:** Missions like Voyager adhere to CCSDS standards for telemetry and telecommand to ensure interoperability and standardized data handling across different space agencies and missions. Our simulation's adoption of CCSDS packet structures reflects this industry-wide practice.
* **Sensor Data:** Voyager 1 carries a suite of sophisticated scientific instruments. The types of sensor data simulated in our project (environmental, magnetic, inertial, GPS, power management) are representative of the kinds of information collected by spacecraft, albeit in a simplified form.

## Transmitter (`Transmit_Interrupt.ino`)

**(Full code available at: [https://github.com/Tanay7/Spacecraft-Telemetry/tree/main/Spacecraft%20Telemetry%20Transmitter](https://github.com/Tanay7/Spacecraft-Telemetry/tree/main/Spacecraft%20Telemetry%20Transmitter))**

### Overview

The transmitter code simulates a spacecraft collecting data from various sensors, packaging this data into CCSDS compliant packets, encoding it with Reed-Solomon FEC, and transmitting it wirelessly using a LoRa radio.

### Features

* Acquires data from BME280, QMC6310, QMI8658, GPS, and PMU.
* Generates CCSDS Application Packets with sensor data.
* Encodes packets using RS(255, 223) Reed-Solomon FEC.
* Transmits encoded data via SX1262 LoRa radio at 433.0 MHz.
* Optionally displays sensor readings on a U8g2 display.
* Includes a packet sequence counter and CRC-16 checksum.
* Performs basic error handling for sensor initialization.

### Hardware Requirements

* LilyGO T-Beam Supreme (or compatible Arduino board with required peripherals)
* SX1262 LoRa Transceiver module (onboard T-Beam Supreme)
* BME280 Environmental Sensor (ensure correct connection/address)
* QMC6310 Magnetometer (ensure correct connection/address)
* QMI8658 Inertial Measurement Unit (IMU) (onboard T-Beam Supreme)
* GPS Module (onboard T-Beam Supreme, requires serial communication)
* PMU (AXP2101 Power Management Unit, onboard T-Beam Supreme)
* (Optional) U8g2 compatible monochrome display (onboard T-Beam Supreme)

### Software Requirements

* Arduino IDE
* [RadioLib Library](https://github.com/jgromes/RadioLib) by JGromes
* [Adafruit BME280 Library](https://github.com/adafruit/Adafruit_BME280_Library) by Adafruit Industries
* [U8g2 Library](https://github.com/olikraus/u8g2) by olikraus
* [TinyGPS++ Library](http://arduinogps.jjoe.org/) by Mikal Hart
* `LoRaBoards.h`: Custom header for board-specific pin definitions and configurations (Must be present in the sketch directory or include path).
* `SensorQMC6310.hpp`: Custom library/header for QMC6310 interaction (Must be present. Verify source and license if obtained externally).
* `SensorQMI8658.hpp`: Custom library/header for QMI8658 interaction (Must be present. Verify source and license if obtained externally).
* `RS-FEC.h`: Custom or library header for Reed-Solomon FEC (Must be present. Specify source/library used and verify its license).

### Installation (Transmitter)

1.  Install the Arduino IDE.
2.  Install the required libraries (RadioLib, Adafruit BME280, U8g2, TinyGPS++) using the Arduino Library Manager or by downloading them from their repositories.
3.  Place the custom header files (`LoRaBoards.h`, `SensorQMC6310.hpp`, `SensorQMI8658.hpp`, `RS-FEC.h`) in the same directory as the `Transmit_Interrupt.ino` sketch, or ensure they are correctly included.
4.  Connect any external hardware components (if not using the integrated T-Beam Supreme sensors) according to the pin definitions in `LoRaBoards.h` and the sensor documentation.
5.  Select the correct board (e.g., "LilyGO T-Beam S3 Supreme") in the Arduino IDE.
6.  Upload the `Transmit_Interrupt.ino` sketch to the board.

### Configuration (Transmitter)

Key configuration parameters in `Transmit_Interrupt.ino` include:

* `SPACECRAFT_ID`: Identifier for the simulated spacecraft.
* CCSDS header parameters (`CCSDS_APID`, `CCSDS_VERSION`, etc.).
* Reed-Solomon FEC parameters (`rsMsgLen`, `ECC_LENGTH`).
* LoRa radio parameters (`CARRIER_FREQ`, `TX_POWER`, `BANDWIDTH`, `SPREADING_FACTOR`, `CODING_RATE`, `SYNC_WORD`). **These must match the receiver's configuration.**
* Sensor update interval (`sensorUpdateInterval`).
* Display settings (`defaultScreenDelay`, `gpsScreenDelay`).
* Sea level pressure for altitude calculation (`SEALEVELPRESSURE_HPA`).

### Usage (Transmitter)

Once running, the transmitter will:

1.  Initialize the connected sensors and LoRa radio.
2.  Continuously read data from the sensors at the defined interval.
3.  Format the sensor data into CCSDS packets.
4.  Encode the packets using Reed-Solomon FEC.
5.  Transmit the encoded packets via LoRa radio.
6.  Optionally display sensor readings on the U8g2 display, cycling through different screens.

## Receiver (`Receive_Interrupt.ino`)

**(Full code available at: [https://github.com/Tanay7/Spacecraft-Telemetry/tree/main/Spacecraft%20Telemetry%20Receiver](https://github.com/Tanay7/Spacecraft-Telemetry/tree/main/Spacecraft%20Telemetry%20Receiver))**

### Overview

The receiver code simulates a ground station that listens for telemetry data transmitted by the spacecraft. It demodulates the received LoRa signals, decodes the Reed-Solomon FEC, processes the CCSDS packets, and displays the extracted sensor data and system status.

### Features

* Demodulates LoRa signals using RadioLib.
* Decodes Reed-Solomon FEC (RS(255, 223)).
* Parses CCSDS Application Packets.
* Verifies CRC-16 checksums for data integrity.
* Displays telemetry data and system status on a U8g2 display.
* Monitors signal quality (RSSI and SNR).
* Counts valid packets and various error types (CRC, RS-FEC, data range).
* Logs detailed packet information via serial output.

### Hardware Requirements

* LilyGO T-Beam Supreme (or compatible Arduino board with required peripherals)
* SX1262 LoRa Transceiver module (onboard T-Beam Supreme)
* U8g2 compatible monochrome display (onboard T-Beam Supreme)

### Software Requirements

* Arduino IDE
* [RadioLib Library](https://github.com/jgromes/RadioLib) by JGromes
* [U8g2 Library](https://github.com/olikraus/u8g2) by olikraus
* `RS-FEC.h`: Custom or library header for Reed-Solomon FEC (Must be present and identical to the one used by the transmitter. Specify source/library used and verify its license).
* `LoRaBoards.h`: Custom header for board-specific pin definitions and configurations (Must be present).

### Installation (Receiver)

1.  Install the Arduino IDE.
2.  Install the required libraries (RadioLib, U8g2) using the Arduino Library Manager or by downloading them from their repositories.
3.  Place the custom header files (`LoRaBoards.h`, `RS-FEC.h`) in the same directory as the `Receive_Interrupt.ino` sketch, or ensure they are correctly included.
4.  Select the correct board (e.g., "LilyGO T-Beam S3 Supreme") in the Arduino IDE.
5.  Upload the `Receive_Interrupt.ino` sketch to the board.

### Configuration (Receiver)

Key configuration parameters in `Receive_Interrupt.ino` include:

* LoRa radio parameters (`CARRIER_FREQ`, `BANDWIDTH`, `SPREADING_FACTOR`, `CODING_RATE`, `SYNC_WORD`). **These must perfectly match the transmitter's configuration.**
* Reed-Solomon FEC parameters (`rsMsgLen`, `ECC_LENGTH`). **These must match the transmitter's configuration.**
* RSSI threshold (`rssiThreshold`) for filtering weak signals (optional).

### Usage (Receiver)

Once running, the receiver will:

1.  Initialize the LoRa radio and display.
2.  Start listening for incoming LoRa packets.
3.  Upon receiving a packet, it will demodulate the signal.
4.  Attempt to decode the Reed-Solomon FEC. Report errors if decoding fails.
5.  Verify the CRC-16 checksum if RS decoding was successful. Report errors if CRC fails.
6.  Extract the sensor data from the CCSDS packet if both RS and CRC checks pass.
7.  Display the received telemetry data, signal status (RSSI, SNR), and error counts on the U8g2 display, cycling through screens.
8.  Log detailed packet information and status to the Serial Monitor.

## Communication Protocol

### CCSDS Packet Structure

Both the transmitter and receiver utilize CCSDS Application Packets (APs). The structure is defined by the `CCSDSPacket` struct in both codes and includes:

* **Primary Header:** Contains version, type, APID, sequence flags, sequence count, and packet length.
* **Data Field:** Contains sensor readings (temperature, pressure, humidity, altitude, magnetic field (3-axis), inertial data (accel/gyro 3-axis), GPS data (lat, lon, alt, speed, course, sats), PMU data (voltage, current), and a message string).
* **Packet Error Control:** A 16-bit CRC (CCITT-FALSE algorithm) is appended to the data field before RS encoding.

### Reed-Solomon FEC

An RS(255, 223) code is implemented.
* **Transmitter:** Encodes the 223 bytes of the CCSDS packet (Header + Data Field + CRC) with 32 bytes of parity information, creating a 255-byte block.
* **Receiver:** Uses the 32 parity bytes to detect and correct up to 16 byte errors within the received 255-byte block.

### LoRa Configuration

Successful communication **requires identical LoRa parameters** on both the transmitter and receiver:
* Carrier Frequency (`CARRIER_FREQ`)
* Bandwidth (`BANDWIDTH`)
* Spreading Factor (`SPREADING_FACTOR`)
* Coding Rate (`CODING_RATE`)
* Sync Word (`SYNC_WORD`)
Any mismatch will likely result in communication failure.

## Error Handling

* **Transmitter:** Checks for successful sensor initialization during setup. Includes a fatal error display routine (`sysFatal`).
* **Receiver:**
    * Checks for valid LoRa packets received via RadioLib.
    * Attempts RS-FEC decoding and reports failures.
    * Verifies the CRC-16 checksum and reports mismatches.
    * Includes basic data range checks (can be expanded).
    * Tracks counts for valid packets, RS errors, and CRC errors.
    * Includes a fatal error display routine (`sysFatal`).

## Contributing

Contributions to this project are welcome. If you find any issues or have suggestions for improvements, please feel free to open an issue or submit a pull request on the project's repository.

## License

The core code for this project (`Transmit_Interrupt.ino`, `Receive_Interrupt.ino`, and associated custom files authored by Tanay) is licensed under the MIT License. See the `LICENSE.md` file in the repository for the full text.

**Dependency Licenses:**

This project utilizes several third-party libraries. You must ensure compliance with their respective licenses:

* **RadioLib:** [Verify and Add License Type/Link - e.g., Apache 2.0 or MIT License]
* **Adafruit BME280 Library:** [MIT License (Verify link/details)]
* **U8g2 Library:** [2-Clause BSD License (Verify link/details)]
* **TinyGPS++ Library:** [Often Public Domain or MIT - Verify and Add License Type/Link]
* **SensorQMC6310 Library/Header:** [Verify and Add License Type/Link if from external source]
* **SensorQMI8658 Library/Header:** [Verify and Add License Type/Link if from external source]
* **RS-FEC Library/Header:** [Verify Source and Add License Type/Link]

Please review the license file included with each library.

## Acknowledgments

* The **RadioLib** library by JGromes ([Verify Author/Org]) for LoRa communication.
* The **Adafruit BME280 Library** by Adafruit Industries.
* The **U8g2lib** library by olikraus for the display.
* The **TinyGPS++** library by Mikal Hart for GPS parsing.
* The **SensorQMC6310** and **SensorQMI8658** libraries/headers ([Specify Source/Author if known]).
* The **RS-FEC** library/implementation ([Specify Source/Author if known]).
* The **Consultative Committee for Space Data Systems (CCSDS)** for the telemetry standards.
* **NASA/JPL-Caltech** for the **Deep Space Network** and **Voyager 1** mission context and inspiration.
