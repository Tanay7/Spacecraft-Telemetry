# Voyager 1 Spacecraft Telemetry System Simulation using T-Beam SUPREME LoRa development board

**Author: Tanay**

## Introduction

This project simulates a basic spacecraft telemetry system, demonstrating the fundamental principles of data acquisition, packetization, error correction, and wireless communication using the **LilyGO T-Beam SUPREME development board**. It consists of two main components running on separate **T-Beam SUPREME** boards: a **transmitter** that simulates a spacecraft gathering sensor data and sending it to Earth, and a **receiver** that simulates a ground station receiving and processing this data.

The system utilizes the integrated LoRa radio for communication, structures data into CCSDS (Consultative Committee for Space Data Systems) compliant packets, and employs Reed-Solomon Forward Error Correction (RS-FEC) for robust data transmission.

**This project was specifically built and tested using the LilyGO T-Beam Supreme module.**

## Voyager 1 Context

The design of this telemetry system draws inspiration from real-world spacecraft missions, most notably the Voyager 1 mission. Launched in 1977, Voyager 1 is one of humanity's most iconic deep space probes, currently exploring interstellar space billions of miles from Earth. Its communication system provides a compelling example of the challenges and solutions involved in transmitting data over vast distances.

![Voyager 1 Spacecraft](https://github.com/Tanay7/Spacecraft-Telemetry/raw/main/images/Voyager_1.jpg)

* **Deep Space Network (DSN):** Voyager 1 communicates with Earth using NASA's Deep Space Network. Our simulation uses the LoRa radio integrated into the **T-Beam SUPREME development board**, a lower-power, shorter-range technology, but the concept of dedicated transmission and reception is analogous.
* **Data Rates and Distances:** Voyager 1's data rate is extremely low (e.g., ~160 bps in 2020) due to distance. Our LoRa simulation on the **T-Beam SUPREME** has higher potential rates, but the principle of balancing data rate, range, and signal strength remains relevant.
* **Importance of Error Correction:** Robust error correction is crucial for deep space missions. Our use of RS-FEC mirrors this critical aspect, implemented within the constraints of the **T-Beam SUPREME**'s processing capabilities.
* **CCSDS Standards:** Adherence to CCSDS standards is common practice. Our simulation's adoption of CCSDS packet structures reflects this.
* **Sensor Data:** Voyager 1 has sophisticated instruments. The sensor data simulated here (environmental, magnetic, inertial, GPS, power management) uses sensors compatible with or integrated into the **T-Beam SUPREME development board**, representing typical spacecraft data types in a simplified form.

---

## Transmitter (`Transmit_Interrupt.ino`)

**(Full code available at: [https://github.com/Tanay7/Spacecraft-Telemetry/tree/main/Spacecraft%20Telemetry%20Transmitter](https://github.com/Tanay7/Spacecraft-Telemetry/tree/main/Spacecraft%20Telemetry%20Transmitter))**

### Overview

The transmitter code runs on a **T-Beam SUPREME development board** and simulates a spacecraft collecting data from various onboard and connected sensors, packaging this data into CCSDS compliant packets, encoding it with Reed-Solomon FEC, and transmitting it wirelessly using the board's integrated LoRa radio.

### Features

* Acquires data from sensors connected to or integrated with the **T-Beam SUPREME development board** (`BME280`, `QMC6310`, `QMI8658`, integrated `GPS`, integrated `PMU`).
* Generates CCSDS Application Packets with sensor data.
* Encodes packets using `RS(255, 223)` Reed-Solomon FEC.
* Transmits encoded data via the **T-Beam SUPREME**'s integrated `SX1262` LoRa radio at 433.0 MHz.
* Optionally displays sensor readings on a compatible display connected to the **T-Beam SUPREME**.
* Includes a packet sequence counter and CRC-16 checksum.
* Performs basic error handling for sensor initialization.

### Hardware Requirements

* **LilyGO T-Beam SUPREME development board** (includes ESP32-S3 microcontroller, SX1262 LoRa Transceiver, GPS Module, PMU (AXP2101)).
* `BME280` Environmental Sensor (connected externally).
* `QMC6310` Magnetometer (connected externally).
* `QMI8658` Inertial Measurement Unit (IMU) (connected externally).
* (Optional) U8g2 compatible monochrome display (connected externally or via integrated header).

### Software Requirements

* Arduino IDE (configured for ESP32-S3 / T-Beam SUPREME)
* [RadioLib Library](https://github.com/jgromes/RadioLib)
* [Adafruit BME280 Library](https://github.com/adafruit/Adafruit_BME280_Library)
* [U8g2 Library](https://github.com/olikraus/u8g2)
* [TinyGPS++ Library](http://arduinogps.jjoe.org/)
* `LoRaBoards.h` (custom header, containing **T-Beam SUPREME** specific pin definitions and configurations - *must be placed in sketch directory*)
* `SensorQMC6310.hpp` (custom library for QMC6310 - *must be placed in sketch directory*)
* `SensorQMI8658.hpp` (custom library for QMI8658 - *must be placed in sketch directory*)
* `RS-FEC.h` (custom or library for Reed-Solomon FEC - *must be placed in sketch directory*)

### Installation (Transmitter)

1.  Install the Arduino IDE, configure it for the ESP32-S3, and install the required libraries (RadioLib, Adafruit BME280, U8g2, TinyGPS++).
2.  Place the custom header files (`LoRaBoards.h`, `SensorQMC6310.hpp`, `SensorQMI8658.hpp`, `RS-FEC.h`) in the same directory as the `Transmit_Interrupt.ino` sketch.
3.  Connect the external hardware components (BME280, QMC6310, QMI8658, display if used) to the appropriate pins on the **T-Beam SUPREME development board**, as defined in `LoRaBoards.h`.
4.  Upload the `Transmit_Interrupt.ino` sketch to the **T-Beam SUPREME development board**.

### Configuration (Transmitter)

Key configuration parameters in `Transmit_Interrupt.ino` include:

* `SPACECRAFT_ID`: Identifier for the simulated spacecraft.
* CCSDS header parameters (`CCSDS_APID`, `CCSDS_VERSION`, etc.).
* Reed-Solomon FEC parameters (`rsMsgLen`, `ECC_LENGTH`).
* LoRa radio parameters (`CARRIER_FREQ`, `TX_POWER`, `BANDWIDTH`, `SPREADING_FACTOR`, `CODING_RATE`, `SYNC_WORD`) specific to the **T-Beam SUPREME**'s SX1262. **These must match the receiver's configuration.**
* Sensor update interval (`sensorUpdateInterval`).
* Display settings (`defaultScreenDelay`, `gpsScreenDelay`).
* Sea level pressure for altitude calculation (`SEALEVELPRESSURE_HPA`).

### Usage (Transmitter)

Once running on the **T-Beam SUPREME development board**, the transmitter will:

1.  Initialize integrated and connected sensors.
2.  Continuously read sensor data at the defined interval.
3.  Format the sensor data into CCSDS packets.
4.  Encode the packets using Reed-Solomon FEC.
5.  Transmit the encoded packets via the integrated LoRa radio.
6.  Optionally display sensor readings on the connected display, cycling through different screens.

---

## Receiver (`Receive_Interrupt.ino`)

**(Full code available at: [https://github.com/Tanay7/Spacecraft-Telemetry/tree/main/Spacecraft%20Telemetry%20Receiver](https://github.com/Tanay7/Spacecraft-Telemetry/tree/main/Spacecraft%20Telemetry%20Receiver))**

### Overview

The receiver code runs on a separate **T-Beam SUPREME development board** and simulates a ground station. It uses the integrated LoRa radio to listen for telemetry data, demodulates received signals, decodes the Reed-Solomon FEC, processes the CCSDS packets, and displays the extracted data and status.

### Features

* Demodulates LoRa signals using the **T-Beam SUPREME**'s `SX1262` radio via RadioLib.
* Decodes Reed-Solomon FEC (`RS(255, 223)`).
* Parses CCSDS Application Packets.
* Verifies CRC-16 checksums for data integrity.
* Displays telemetry data and system status on a compatible display connected to the **T-Beam SUPREME**.
* Monitors signal quality (`RSSI` and `SNR`) reported by the `SX1262`.
* Counts valid packets and various error types (CRC, RS-FEC, data range).
* Logs detailed packet information via serial output from the **T-Beam SUPREME**.

### Hardware Requirements

* **LilyGO T-Beam SUPREME development board** (includes ESP32-S3 microcontroller, SX1262 LoRa Transceiver).
* U8g2 compatible monochrome display (connected externally or via integrated header).

### Software Requirements

* Arduino IDE (configured for ESP32-S3 / T-Beam SUPREME)
* [RadioLib Library](https://github.com/jgromes/RadioLib)
* [U8g2 Library](https://github.com/olikraus/u8g2)
* `RS-FEC.h` (custom or library for Reed-Solomon FEC - *must be placed in sketch directory*)
* `LoRaBoards.h` (custom header, containing **T-Beam SUPREME** specific pin definitions and configurations - *must be placed in sketch directory*)

### Installation (Receiver)

1.  Install the Arduino IDE, configure it for the ESP32-S3, and install the required libraries (RadioLib, U8g2).
2.  Place the custom header files (`LoRaBoards.h`, `RS-FEC.h`) in the same directory as the `Receive_Interrupt.ino` sketch.
3.  Connect the U8g2 display to the appropriate pins on the **T-Beam SUPREME development board**, according to `LoRaBoards.h`.
4.  Upload the `Receive_Interrupt.ino` sketch to the **T-Beam SUPREME development board**.

### Configuration (Receiver)

Key configuration parameters in `Receive_Interrupt.ino` include:

* LoRa radio parameters (`CARRIER_FREQ`, `BANDWIDTH`, `SPREADING_FACTOR`, `CODING_RATE`, `SYNC_WORD`) for the **T-Beam SUPREME**'s SX1262. **These must perfectly match the transmitter's configuration.**
* Reed-Solomon FEC parameters (`rsMsgLen`, `ECC_LENGTH`). **These must match the transmitter's configuration.**
* RSSI threshold (`rssiThreshold`) for filtering weak signals.

### Usage (Receiver)

Once running on the **T-Beam SUPREME development board**, the receiver will:

1.  Initialize the integrated LoRa radio and start listening for incoming packets.
2.  Upon receiving a packet, it will demodulate the LoRa signal.
3.  Decode the Reed-Solomon FEC.
4.  Verify the CRC-16 checksum.
5.  Extract the sensor data from the CCSDS packet.
6.  Display the received telemetry data and system status (RSSI, SNR, error counts) on the connected display, cycling through different screens.
7.  Log detailed packet information to the Serial Monitor.

---

## Communication Protocol

### CCSDS Packet Structure

Both the transmitter and receiver are designed to work with CCSDS Application Packets (APs). The structure of these packets is defined by the `CCSDSPacket` struct in both codes and includes:

* **Primary Header:** Contains version, type, APID, sequence flags, sequence count, and packet length.
* **Data Field:** Contains the sensor readings in a specific order (temperature, pressure, humidity, altitude, magnetic field, inertial data, GPS data, PMU data, and a message string).
* **Packet Error Control:** A 16-bit CRC (CCITT-FALSE) is appended to the data field.

### Reed-Solomon FEC

Both the transmitter and receiver implement an `RS(255, 223)` Reed-Solomon code. The transmitter encodes the 223 bytes of CCSDS packet data with 32 bytes of parity information, resulting in a 255-byte block for transmission. The receiver uses these 32 parity bytes to correct up to 16 byte errors that may have occurred during transmission.

### LoRa Configuration

For successful communication, the LoRa radio parameters on both the transmitter and receiver **must be identical**. This includes the carrier frequency, bandwidth, spreading factor, coding rate, and sync word. Any mismatch in these parameters will prevent the receiver from correctly demodulating the signals from the transmitter.

---

## Error Handling

Both the transmitter and receiver include basic error handling mechanisms. The transmitter checks for successful sensor initialization, and both systems have a fatal error display routine for critical failures. The receiver also tracks CRC errors, Reed-Solomon decoding errors, and data range errors to provide insights into the quality of the communication link and the received data.

---

## Contributing

Contributions to this project are welcome. If you find any issues or have suggestions for improvements, please feel free to open an issue or submit a pull request on the project's repository (if hosted on a platform like GitHub).

---

## License

[Specify the license under which this code is released, e.g., MIT License, Apache License 2.0, etc.]

---

## Acknowledgments

* The **RadioLib** library by [mention author/organization] for LoRa communication.
* The **Adafruit BME280 Library** by Adafruit Industries for the BME280 sensor.
* The **U8g2lib** library by [mention author/organization, typically olikraus] for the display.
* The **SensorQMC6310** and **SensorQMI8658** libraries (source likely specific to the hardware).
* The **TinyGPS++** library by [mention author, typically Mikal Hart] for GPS parsing.
* The **RS-FEC** library (source needs to be specified based on where it was obtained).
* The CCSDS standards provided by the Consultative Committee for Space Data Systems.
* NASA's **Deep Space Network** and the **Voyager 1** mission for providing real-world context and inspiration.
