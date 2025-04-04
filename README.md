# Voyager 1 Spacecraft Telemetry System Simulation

## Author: Tanay

## Introduction

This project simulates a basic spacecraft telemetry system, demonstrating the fundamental principles of data acquisition, packetization, error correction, and wireless communication. It consists of two main components: a **transmitter** (code available at [https://github.com/Tanay7/Spacecraft-Telemetry/tree/main/Spacecraft%20Telemetry%20Transmitter](https://github.com/Tanay7/Spacecraft-Telemetry/tree/main/Spacecraft%20Telemetry%20Transmitter)) that simulates a spacecraft gathering sensor data and sending it to Earth, and a **receiver** (code available at [https://github.com/Tanay7/Spacecraft-Telemetry/tree/main/Spacecraft%20Telemetry%20Receiver](https://github.com/Tanay7/Spacecraft-Telemetry/tree/main/Spacecraft%20Telemetry%20Receiver)) that simulates a ground station receiving and processing this data. The system utilizes LoRa radio for communication, structures data into CCSDS (Consultative Committee for Space Data Systems) compliant packets, and employs Reed-Solomon Forward Error Correction (RS-FEC) for robust data transmission. **This project was built and tested using the LilyGO T-Beam Supreme module.**

## Voyager 1 Context

The design of this telemetry system draws inspiration from real-world spacecraft missions, most notably the Voyager 1 mission. Launched in 1977, Voyager 1 is one of humanity's most iconic deep space probes, currently exploring interstellar space billions of miles from Earth. Its communication system provides a compelling example of the challenges and solutions involved in transmitting data over vast distances.

![Voyager 1 Spacecraft](https://github.com/Tanay7/Spacecraft-Telemetry/raw/main/images/Voyager_1.jpg)
*A sample image of the Voyager 1 spacecraft.*

* **Deep Space Network (DSN):** Voyager 1 communicates with Earth using NASA's Deep Space Network, a network of large parabolic radio antennas located around the world. These antennas are highly sensitive to the faint signals received from deep space probes. Our simulation uses LoRa, a lower-power, shorter-range radio technology, but the concept of a dedicated network for receiving spacecraft data is analogous.
* **Data Rates and Distances:** Due to the immense distance, the data rate from Voyager 1 is extremely low, measured in bits per second (bps). As of 2020, the downlink rate was around 160 bps. While our LoRa simulation has a much higher potential data rate, the principle of balancing data rate with range and signal strength remains relevant.
* **Importance of Error Correction:** Over such vast distances, signals are significantly attenuated and corrupted by noise. Therefore, robust error correction codes are crucial for ensuring the integrity of the received scientific data. Voyager 1 has used various coding schemes throughout its mission, including convolutional codes and Reed-Solomon codes (though perhaps not exactly RS(255, 223) in all phases). Our use of RS-FEC mirrors this critical aspect of deep space communication.
* **CCSDS Standards:** Missions like Voyager 1 adhere to CCSDS standards for telemetry and telecommand to ensure interoperability and standardized data handling across different space agencies and missions. Our simulation's adoption of CCSDS packet structures reflects this industry-wide practice.
* **Sensor Data:** Voyager 1 carries a suite of sophisticated scientific instruments to study planetary environments, magnetic fields, and cosmic rays. The types of sensor data simulated in our project (environmental, magnetic, inertial, GPS, power management) are representative of the kinds of information collected by spacecraft, albeit in a simplified form.

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

* Arduino-compatible microcontroller
* SX1262 LoRa Transceiver module
* BME280 Environmental Sensor
* QMC6310 Magnetometer
* QMI8658 Inertial Measurement Unit (IMU)
* GPS Module (with serial output)
* PMU (Power Management Unit)
* (Optional) U8g2 compatible monochrome display

### Software Requirements

* Arduino IDE
* [RadioLib](https://github.com/jgromes/RadioLib)
* [Adafruit BME280 Library](https://github.com/adafruit/Adafruit_BME280_Library)
* [U8g2lib](https://github.com/olikraus/u8g2)
* [TinyGPS++](http://arduinogps.jjoe.org/)
* `LoRaBoards.h` (custom, board-specific definitions)
* `SensorQMC6310.hpp` (custom library for QMC6310)
* `SensorQMI8658.hpp` (custom library for QMI8658)
* `RS-FEC.h` (custom or library for Reed-Solomon FEC)

### Installation (Transmitter)

1.  Install the Arduino IDE and the required libraries as detailed in the individual `Transmit_Interrupt.ino` README.
2.  Place the custom header files (`LoRaBoards.h`, `SensorQMC6310.hpp`, `SensorQMI8658.hpp`, `RS-FEC.h`) in the same directory as the `Transmit_Interrupt.ino` sketch.
3.  Connect the hardware components to the microcontroller according to the pin definitions in `LoRaBoards.h` and the sensor documentation.
4.  Upload the `Transmit_Interrupt.ino` sketch to the Arduino board.

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

1.  Initialize the connected sensors.
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

* Arduino-compatible microcontroller
* SX1262 LoRa Transceiver module
* U8g2 compatible monochrome display

### Software Requirements

* Arduino IDE
* [RadioLib](https://github.com/jgromes/RadioLib)
* [U8g2lib](https://github.com/olikraus/u8g2)
* `RS-FEC.h` (custom or library for Reed-Solomon FEC)
* `LoRaBoards.h` (custom, board-specific definitions)

### Installation (Receiver)

1.  Install the Arduino IDE and the required libraries as detailed in the individual `Receive_Interrupt.ino` README.
2.  Place the custom header file (`LoRaBoards.h`, `RS-FEC.h`) in the same directory as the `Receive_Interrupt.ino` sketch.
3.  Connect the LoRa transceiver and the U8g2 display to the microcontroller according to the pin definitions in `LoRaBoards.h`.
4.  Upload the `Receive_Interrupt.ino` sketch to the Arduino board.

### Configuration (Receiver)

Key configuration parameters in `Receive_Interrupt.ino` include:

* LoRa radio parameters (`CARRIER_FREQ`, `BANDWIDTH`, `SPREADING_FACTOR`, `CODING_RATE`, `SYNC_WORD`). **These must perfectly match the transmitter's configuration.**
* Reed-Solomon FEC parameters (`rsMsgLen`, `ECC_LENGTH`). **These must match the transmitter's configuration.**
* RSSI threshold (`rssiThreshold`) for filtering weak signals.

### Usage (Receiver)

Once running, the receiver will:

1.  Initialize the LoRa radio and start listening for incoming packets.
2.  Upon receiving a packet, it will demodulate the LoRa signal.
3.  Decode the Reed-Solomon FEC.
4.  Verify the CRC-16 checksum.
5.  Extract the sensor data from the CCSDS packet.
6.  Display the received telemetry data and system status (RSSI, SNR, error counts) on the U8g2 display, cycling through different screens.
7.  Log detailed packet information to the Serial Monitor.

## Communication Protocol

### CCSDS Packet Structure

Both the transmitter and receiver are designed to work with CCSDS Application Packets (APs). The structure of these packets is defined by the `CCSDSPacket` struct in both codes and includes:

* **Primary Header:** Contains version, type, APID, sequence flags, sequence count, and packet length.
* **Data Field:** Contains the sensor readings in a specific order (temperature, pressure, humidity, altitude, magnetic field, inertial data, GPS data, PMU data, and a message string).
* **Packet Error Control:** A 16-bit CRC (CCITT-FALSE) is appended to the data field.

### Reed-Solomon FEC

Both the transmitter and receiver implement an RS(255, 223) Reed-Solomon code. The transmitter encodes the 223 bytes of CCSDS packet data with 32 bytes of parity information, resulting in a 255-byte block for transmission. The receiver uses these 32 parity bytes to correct up to 16 byte errors that may have occurred during transmission.

### LoRa Configuration

For successful communication, the LoRa radio parameters on both the transmitter and receiver **must be identical**. This includes the carrier frequency, bandwidth, spreading factor, coding rate, and sync word. Any mismatch in these parameters will prevent the receiver from correctly demodulating the signals from the transmitter.

## Error Handling

Both the transmitter and receiver include basic error handling mechanisms. The transmitter checks for successful sensor initialization, and both systems have a fatal error display routine for critical failures. The receiver also tracks CRC errors, Reed-Solomon decoding errors, and data range errors to provide insights into the quality of the communication link and the received data.

## Contributing

Contributions to this project are welcome. If you find any issues or have suggestions for improvements, please feel free to open an issue or submit a pull request on the project's repository (if hosted on a platform like GitHub).

## License

[Specify the license under which this code is released, e.g., MIT License, Apache License 2.0, etc.]

## Acknowledgments

* The **RadioLib** library by [mention author/organization] for LoRa communication.
* The **Adafruit BME280 Library** by Adafruit Industries for the BME280 sensor.
* The **U8g2lib** library by [mention author/organization, typically olikraus] for the display.
* The **SensorQMC6310** and **SensorQMI8658** libraries (source likely specific to the hardware).
* The **TinyGPS++** library by [mention author, typically Mikal Hart] for GPS parsing.
* The **RS-FEC** library (source needs to be specified based on where it was obtained).
* The CCSDS standards provided by the Consultative Committee for Space Data Systems.
* NASA's **Deep Space Network** and the **Voyager 1** mission for providing real-world context and inspiration.
