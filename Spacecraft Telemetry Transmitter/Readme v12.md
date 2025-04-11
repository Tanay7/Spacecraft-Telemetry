# 🚀 Spacecraft Telemetry Transmitter (LoRa/CCSDS/RS-FEC) 🛰️ - Comprehensive Guide

**(Project Status: Development / Educational Prototype)**

This Arduino project meticulously simulates the transmitter portion of a spacecraft telemetry system, drawing inspiration from real-world deep space communication principles like those employed by the Voyager probes. It serves as an educational tool to understand sensor integration, standardized data packetization (CCSDS), robust communication techniques (LoRa), and error correction (Reed-Solomon FEC).

The system autonomously gathers environmental, positional, inertial, magnetic, and power status data, encapsulates it within CCSDS-compliant Application Packets (AP), fortifies it with Reed-Solomon Forward Error Correction (RS-FEC), and transmits it periodically over a 433 MHz LoRa radio link using a powerful SX1262 transceiver module. An optional onboard display provides real-time visualization of system operations and sensor readings.

This document consolidates information from various stages of the project's documentation into a single, comprehensive guide.

---

## Table of Contents

* [✨ Core Features & Functionality](#-core-features--functionality)
* [🔌 Hardware Requirements & Connections](#-hardware-requirements--connections)
* [📚 Software & Dependencies](#-software--dependencies)
* [⚙️ Installation & Setup Guide](#️-installation--setup-guide)
* [🔧 Configuration Deep Dive](#-configuration-deep-dive)
* [▶️ Usage & Operation](#️-usage--operation)
* [📊 CCSDS Packet Structure (`CCSDSPacket`) Explained](#-ccsds-packet-structure-ccdspacket-explained)
* [🛡️ Reed-Solomon FEC (RS(255, 223)) Details](#️-reed-solomon-fec-rs255-223-details)
* [📺 Display Interface Details (Optional)](#-display-interface-details-optional)
* [❗ Error Handling (`displayFatalError`)](#-error-handling-displayfatalerror)
* [🧩 Code Structure Overview](#-code-structure-overview)
* [💡 Potential Improvements & Future Work](#-potential-improvements--future-work)
* [🤝 Contributing](#-contributing)
* [📜 License](#-license)
* [🎉 Acknowledgements](#-acknowledgements)

---

## ✨ Core Features & Functionality

* 📡 **Long-Range Communication (LoRa / SX1262):** Utilizes the SX1262 LoRa module and the versatile [RadioLib](https://github.com/jgromes/RadioLib) library for physical layer transmission. Configured for 433 MHz (check local regulations!), high spreading factor (SF12) for range, and specific coding rate/bandwidth for a balance between throughput and reliability. Uses interrupt-driven TX completion.
* 📦 **Standardized Data Formatting (CCSDS):** Adheres to the Consultative Committee for Space Data Systems (CCSDS) Application Packet (AP) structure (CCSDS 133.0-B-1) for telemetry data. Promotes interoperability and follows established space communication practices. Key header fields (Version, Type, APID, Sequence Count, etc.) are correctly populated using bitfield manipulation. `#pragma pack(push, 1)` ensures precise byte alignment.
* 🛡️ **Robust Error Correction (Reed-Solomon):** Implements RS(255, 223) Forward Error Correction over GF(2^8) using the `RS-FEC.h` library. Adds 32 bytes of parity data to every 223 bytes of telemetry data, allowing the receiver to detect and correct up to 16 byte errors within the 255-byte block, significantly enhancing data reliability.
* 🔬 **Comprehensive Sensor Suite Integration:** Gathers data from a diverse set of sensors:
    * 🌡️ **BME280 (I2C):** Barometric pressure (hPa), ambient temperature (°C), relative humidity (%), calculated altitude (m).
    * 🧭 **QMC6310 (I2C):** 3-axis magnetic field vector (uT) for orientation sensing. Includes calculation for magnetic heading (declination correction required).
    * <0xF0><0x9F><0xA7><0xBA> **QMI8658 (SPI):** 6-DOF Inertial Measurement Unit (IMU) providing 3-axis acceleration (g), 3-axis gyroscopic rate (dps), and internal die temperature (°C). Configured for specific ODR, Range, and LPF settings.
    * 🗺️ **GPS (UART/Serial):** Parses NMEA 0183 sentences via [TinyGPS++](http://arduinogps.jjoe.org/) to extract precise location (Latitude, Longitude), altitude (m), Coordinated Universal Time (UTC), date, number of visible satellites, and Horizontal Dilution of Precision (HDOP).
    * 🔋 **PMU (Board-Specific):** Monitors the Power Management Unit (if available on the target board) for critical power system parameters: battery voltage (V), estimated charge percentage (%), charging status (boolean), VBUS input voltage (V), and overall system voltage (V). Accessed via a board-support `PMU` object.
* 🔒 **Data Integrity Verification (CRC-16):** Calculates and appends a CRC-16 (CCITT-FALSE, polynomial `0x1021`) checksum to the CCSDS packet payload *before* RS-FEC encoding. Allows the receiver to verify data integrity after decoding. LoRa hardware CRC is disabled.
* 📟 **Real-time Monitoring (Optional Display):** Leverages the [U8g2 library](https://github.com/olikraus/u8g2) to drive monochrome OLED/LCD displays (I2C/SPI). Cycles through 11 informative screens showing sensor data, GPS status, LoRa parameters, and power status.
* ⚙️ **Structured & Modular Code:** Organized into functions for sensor reading, packet building, display management, and error handling, promoting readability and maintainability. Includes detailed comments.
* ❗ **Basic Error Handling:** Includes initialization checks for critical sensors and provides a `displayFatalError` routine that halts execution and provides visual feedback (serial, display, optional LED blink) upon critical failure.
* 🎓 **Educational Focus:** Designed to demonstrate core concepts of spacecraft telemetry systems in a practical, hands-on manner.

---

## 🔌 Hardware Requirements & Connections

* **Microcontroller Unit (MCU):** An Arduino-compatible board with sufficient Flash/RAM, hardware **SPI**, **I2C**, and at least one hardware **Serial (UART)** port. Processing power impacts encoding/sensor processing latency. The specific board choice dictates the required `LoRaBoards.h` configuration.
    * **Pin Requirements:** Dedicated SPI pins (MOSI, MISO, SCK), at least two SPI CS pins (Radio, IMU), I2C pins (SDA, SCL), UART pins (RX/TX for GPS), and Radio control lines (DIO1, BUSY, RST). Check pin availability.
* **LoRa Transceiver:** An **SX1262**-based module (e.g., Ebyte E22, Semtech reference designs).
    * **Interface:** SPI (MOSI, MISO, SCK, CS).
    * **Control Lines:** DIO1 (TxDone Interrupt), BUSY (Radio Busy Indicator), RST (Reset).
    * **Antenna:** A **50 Ohm impedance matched** antenna for the operating frequency (e.g., 433 MHz) is essential.
* **Sensors:**
    * **BME280 Module:** Interface: **I2C** (SDA, SCL). Requires VCC, GND. Check pull-up resistors.
    * **QMC6310 Module:** Interface: **I2C** (SDA, SCL). Requires VCC, GND. Check I2C address (default 0x2C).
    * **QMI8658 Module:** Interface: **SPI** (MOSI, MISO, SCK, dedicated CS). Requires VCC, GND. Check SPI mode compatibility.
    * **GPS Module:** Interface: **Serial/UART** (e.g., BN-220, BN-880). Requires VCC, GND, TX (GPS) -> RX (MCU), RX (GPS) -> TX (MCU). Ensure **logic level compatibility** (e.g., 3.3V). Connected to `SerialGPS` instance.
    * **PMU (Power Management Unit):** Assumed to be **integrated** onto the target MCU board (common on ESP32 LoRa boards). Accessed via a board-support-package provided `PMU` object (likely initialized within `setupBoards()`).
* **(Optional) Display:** A monochrome display compatible with the **U8g2 library** (e.g., SSD1306, SH1106).
    * **Interface:** **I2C** (SDA, SCL) or **SPI** (MOSI, SCK, CS, DC, RST - varies).
* **Power Source:** A stable power source (e.g., LiPo battery managed by the PMU, USB) capable of handling the peak current draw of the LoRa module during transmission (can exceed 100mA, limited by `radio.setCurrentLimit()`).

---

## 📚 Software & Dependencies

This project requires careful library management:

* **Core Communication:**
    * [**RadioLib**](https://github.com/jgromes/RadioLib) (`RadioLib.h`): For SX1262 LoRa interaction.
* **Board Abstraction & Configuration:**
    * **`LoRaBoards.h`**: **❗ CRITICAL CUSTOM FILE ❗** This header is *not* provided by a standard library. **You MUST create or adapt this file** for your specific hardware. It defines pin mappings, hardware peripheral instances (`SPI`, `Wire`, `SerialGPS`), and contains the `setupBoards()` function for board-specific initialization (e.g., PMU setup, U8g2 constructor).
* **Sensor Libraries:**
    * [**Adafruit BME280 Library**](https://github.com/adafruit/Adafruit_BME280_Library) (`Adafruit_BME280.h`): Interface for the BME280 sensor.
    * [**Adafruit Unified Sensor Driver**](https://github.com/adafruit/Adafruit_Sensor) (`Adafruit_Sensor.h`): Required dependency for the Adafruit BME280 library.
    * **`SensorQMC6310.hpp`**: Custom library/driver for the QMC6310 magnetometer. *(Source code required separately - link unavailable in source material).*
    * **`SensorQMI8658.hpp`**: Custom library/driver for the QMI8658 IMU. *(Source code required separately - link unavailable in source material).*
* **Data Processing & Utility:**
    * [**TinyGPS++**](http://arduinogps.jjoe.org/) (`TinyGPS++.h`): Robust library for parsing NMEA data streams from the GPS module.
    * **`RS-FEC.h`**: Reed-Solomon Forward Error Correction library for RS(255, 223). *(Source code required separately - link unavailable in source material).*
    * `math.h`: Standard C library for mathematical functions (`pow`, `atan2`, `M_PI`).
    * `stdio.h`: Standard C library for functions like `sprintf`.
* **Display:**
    * [**U8g2 Library**](https://github.com/olikraus/u8g2) (`U8g2lib.h`): Powerful library for driving monochrome displays (used if display is enabled).
* **Core Arduino:**
    * `Wire.h`: For I2C communication.
    * `SPI.h`: For SPI communication.
    * *(Other core headers included implicitly)*

---

## ⚙️ Installation & Setup Guide

1.  **Install Arduino IDE:** Download and install the latest version from the [official Arduino website](https://www.arduino.cc/en/software).
2.  **Install Board Support:** If using a non-standard board (like ESP32, STM32), install the appropriate board package via the IDE's Board Manager (`Tools` > `Board` > `Boards Manager...`).
3.  **Install Libraries:**
    * **Via Library Manager:** Open `Tools` > `Manage Libraries...` and install:
        * `RadioLib` by JGromes
        * `Adafruit BME280 Library` by Adafruit
        * `Adafruit Unified Sensor` by Adafruit
        * `U8g2` by olikraus
        * `TinyGPS++` by Mikal Hart
    * **Manually:**
        * Obtain the source code for the custom libraries: `SensorQMC6310.hpp`, `SensorQMI8658.hpp`, `RS-FEC.h`.
        * Create the board-specific `LoRaBoards.h` file based on your hardware.
        * Place these custom files/folders either directly into the Arduino sketch directory (e.g., `TelemetryTx/`) or into your Arduino libraries folder (usually `Documents/Arduino/libraries/`). Restart the IDE after adding libraries manually.
4.  **Clone or Download Project:** Get the `TelemetryTx.ino` sketch and any accompanying custom files.
5.  **❗ Configure `LoRaBoards.h` ❗:** This is the most critical step. Open `LoRaBoards.h` and meticulously define:
    * All pin assignments for Radio (CS, DIO1, BUSY, RST), Sensors (I2C SDA/SCL, SPI CS), GPS (Serial RX/TX), and Display (I2C/SPI pins as needed).
    * The correct `HardwareSerial` instance for `SerialGPS` (e.g., `Serial1`, `Serial2`).
    * Correct hardware peripheral instances (`Wire`, `SPI`) if non-default ones are used.
    * The correct U8g2 constructor for your specific display and connection (I2C/SPI).
    * Any necessary board-specific initializations within `setupBoards()` (e.g., PMU initialization, power rail enables, clock setup). **Refer to your specific board's documentation and schematics.**
6.  **Connect Hardware:** Carefully wire all components according to the pin definitions in `LoRaBoards.h`. Use short, reliable connections, especially for SPI. Ensure common ground. Verify antenna connection integrity. Double-check VCC and GND connections and voltage levels.
7.  **Configure Sketch Parameters:** Open `TelemetryTx.ino` and review the configuration constants near the top (LoRa settings, CCSDS ID, `SEALEVELPRESSURE_HPA`, timing intervals). Adjust if necessary. Crucially, update the magnetic `declination` value in `updateQMC6310Data()` for your geographical location (value in radians). See [Configuration Deep Dive](#-configuration-deep-dive) for details.
8.  **Select Board & Port:** In the Arduino IDE (`Tools` menu), select the correct board variant you are using and the corresponding COM port it enumerated on.
9.  **Compile & Upload:** Click the "Upload" button. Monitor the IDE output window for compilation messages and the Serial Monitor for runtime messages.

**Troubleshooting Tips:**
* **Serial Monitor:** Set baud rate to 115200 to view initialization messages, status, and potential errors. This is your primary debugging tool.
* **Incorrect `LoRaBoards.h`:** The most common cause of initialization failures for radio, sensors, or display. Triple-check every pin definition and peripheral setting against your board's schematic.
* **Wiring Errors:** Double-check every connection. Use a multimeter to check continuity and voltage levels. Ensure RX/TX are crossed for UART.
* **Library Conflicts/Missing:** Ensure all required libraries are installed correctly and there are no version conflicts. Verify custom libraries are accessible.
* **Power Issues:** Ensure your power source can supply sufficient peak current for the LoRa TX (~100-140mA). Unstable power can cause resets or erratic behavior.
* **Logic Analyzer/Oscilloscope:** Use these tools to debug I2C/SPI communication issues by observing signal timing, data, and integrity.
* **Isolation Testing:** Comment out sections of code (e.g., individual sensor initializations in `setup()`) to isolate the source of a problem.

---

## 🔧 Configuration Deep Dive

Key parameters adjustable in `TelemetryTx.ino` and their implications:

* **LoRa Parameters (`CARRIER_FREQ`, `TX_POWER`, `BANDWIDTH`, `SPREADING_FACTOR`, `CODING_RATE`, `SYNC_WORD`, `PREAMBLE_LENGTH`, `CURRENT_LIMIT`):**
    * `CARRIER_FREQ`: (e.g., `433.0` MHz) Must match the receiver and comply with local ISM band regulations (e.g., ETSI in Europe/Ireland - consider duty cycle limits).
    * `TX_POWER`: (e.g., `20` dBm) Transmit power. Higher power increases range but consumes more energy and may be legally restricted. Limited by `CURRENT_LIMIT`.
    * `BANDWIDTH`: (e.g., `125.0` kHz) Affects data rate and noise immunity. Lower bandwidth improves sensitivity but reduces data rate and increases Time-on-Air (ToA).
    * `SPREADING_FACTOR`: (7-12, e.g., `12`) Higher SF increases range (better sensitivity) but significantly decreases data rate and increases ToA. SF12 provides the longest range.
    * `CODING_RATE`: (5-8, representing 4/5 to 4/8, e.g., `6`) Higher rate adds more LoRa FEC bits, improving robustness but decreasing the effective data rate.
    * `SYNC_WORD`: (e.g., `0x35`) Differentiates LoRa networks. Must match between transmitter and receiver.
    * `PREAMBLE_LENGTH`: (e.g., `16` symbols) Longer preamble aids receiver sync in noisy conditions but adds to ToA.
    * `CURRENT_LIMIT`: (e.g., `140` mA) Protects the radio's Power Amplifier. Set based on SX1262/board specs.
    * **Trade-offs:** SF, BW, and CR settings determine the balance between range, data rate, robustness, and ToA. High SF / Low BW / High CR prioritizes range but results in very long ToA, impacting transmission frequency and power consumption. **Calculate ToA** to ensure regulatory compliance.
    * `radio.setCRC(false)`: Disables LoRa hardware CRC; relies on software CRC-16.
* **CCSDS Parameters (`SPACECRAFT_ID`, `CCSDS_APID`, Header Bits):**
    * `SPACECRAFT_ID`: (e.g., `"DSN-2025-001"`) A unique 12-byte name for your simulated craft.
    * `CCSDS_APID`: (e.g., `0x7FF`) Application Process Identifier (11 bits), used for routing. `0x7FF` is often used for testing/idle.
    * Header fields (`CCSDS_VERSION`, `CCSDS_TYPE`, `CCSDS_SEC_HDR_FLAG`, `CCSDS_SEQ_FLAGS`) are set according to CCSDS standards for basic unsegmented telemetry packets. Bitwise operations construct the header fields.
* **Sensor Calibration & Settings:**
    * `SEALEVELPRESSURE_HPA`: (e.g., `1013.25`) Critical for accurate altitude calculation from BME280 pressure. Calibrate using current local sea-level pressure for best results.
    * **Magnetic Declination:** The `declination` variable (in radians) in `updateQMC6310Data()` must be set for your specific geographic location to get accurate true heading from the magnetometer. The example `(-0.041)` is for Galway, Ireland (approx. -2.35 degrees). Use a reliable source like [NOAA's calculator](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml) to find your local value.
    * **Sensor ODR/Range/LPF:** Configured in `setup()` for QMI8658. Affect sample rate, measurement limits, and noise filtering. Adjust based on application needs.
* **Timing:**
    * `sensorUpdateInterval`: (e.g., `100` ms) How often the code attempts to read *new* sensor data. Actual read frequency depends on sensor readiness flags/methods.
    * `defaultScreenDelay`: (e.g., `2000` ms) Time each screen is shown on the optional display.

---

## ▶️ Usage & Operation

1.  **Power Up:** Apply power to the correctly wired and programmed system.
2.  **Initialization Sequence (`setup()`):**
    * Serial communication starts (115200 baud).
    * `setupBoards()` executes board-specific initialization.
    * Optional display (U8g2) initializes.
    * LoRa radio (RadioLib) initializes and configured with parameters. Current limit and TxDone ISR (`setFlag`) are set.
    * Sensors (BME280, QMC6310, QMI8658) are initialized and configured (ODR, Range, LPF). GPS serial port (`SerialGPS`) is started (9600 baud). PMU access is likely set up in `setupBoards()`.
    * **Critical Check:** If `bme.begin()`, `qmc.begin()`, or `qmi.begin()` fail, `displayFatalError()` is called, halting operation.
    * An initial sensor read, packet build (including CRC and RS-FEC), and the first non-blocking LoRa transmission (`radio.startTransmit()`) are triggered.
3.  **Main Operational Loop (`loop()`):** The code continuously performs these actions:
    * **Check Transmission Completion:** Checks if the `volatile bool transmittedFlag` is `true`. This flag is set to `true` by the `setFlag()` ISR when the previous LoRa transmission completes.
    * **If Transmission Completed:**
        * Reset the flag: `transmittedFlag = false`.
        * Increment the `packetCounter` (wraps around based on 14-bit CCSDS field).
        * Update Sensor Data: Call `updateSensorReadings()`, which polls individual sensors (`updateBME280Data`, etc.) and reads new values if available.
        * Build & Encode Packet: Call `buildTelemetryPacket()`:
            1.  Populate the `CCSDSPacket` struct (`tmPacket`) with the latest sensor data, timestamp (`millis()`), packet counter, etc.
            2.  Calculate the CRC-16 over the packet data (excluding the CRC field itself) and store it in `tmPacket.crc`.
            3.  Copy the `tmPacket` data into the `encodedData` buffer (or handle data input directly via the `rs.Encode` method depending on its implementation).
            4.  Call `rs.Encode()` to compute the 32 Reed-Solomon parity bytes based on the 223-byte data block (payload + padding) and place them appropriately to form the final 255-byte encoded block in `encodedData`.
        * Start Next Transmission: Call `radio.startTransmit(encodedData, sizeof(encodedData))` to begin asynchronously transmitting the 255-byte RS-encoded block.
    * **Update Display (If Enabled):** Call `updateDisplay()` to potentially cycle to the next screen based on `defaultScreenDelay`. Call `drawDisplay()` to render the current screen's content using U8g2 page buffer mode.
    * **Process GPS Data:** Continuously check `SerialGPS.available()` and feed incoming bytes to `gps.encode()` for the TinyGPS++ library to parse NMEA sentences.
    * **Brief Delay:** `delay(1)` yields processing time, preventing potential watchdog resets.
4.  **Transmission Interval:** The time between consecutive packet transmissions is **variable** and primarily determined by the **LoRa Time-on-Air (ToA)** for the 255-byte packet, which depends heavily on SF, BW, and CR. For SF12 / 125kHz, ToA can be over 1 second. Sensor read times, packet building, and RS encoding time also contribute but are typically much smaller than the ToA at high SF.
5.  **Receiver Requirements:** A corresponding LoRa receiver system must:
    * Use an SX1262 module configured with the **identical** LoRa parameters (Frequency, BW, SF, CR, Sync Word, Preamble Length). Hardware CRC should be disabled on the receiver as well.
    * Be capable of receiving LoRa packets (e.g., using RadioLib in receive mode).
    * Implement a compatible **RS(255, 223) decoder**.
    * Know the `CCSDSPacket` structure (147 bytes) to interpret the decoded data block.
    * Implement the **CRC-16-CCITT-FALSE** algorithm to calculate the checksum on the received, decoded payload (bytes 0 to 144) and compare it against the received CRC value (bytes 145-146) to verify data integrity.

---

## 📊 CCSDS Packet Structure (`CCSDSPacket`) Explained

The telemetry data is organized within the `CCSDSPacket` struct, ensuring byte alignment using `#pragma pack(push, 1)`.

| Field                  | Bytes | Type     | Description                                                                                             | Bits Detail (Primary Header)                               |
| :--------------------- | :---- | :------- | :------------------------------------------------------------------------------------------------------ | :--------------------------------------------------------- |
| `version_type_apid`    | 2     | `uint16_t` | CCSDS Primary Header: Packet Identification Field                                                       | Version (3b=`000`), Type (1b=`0`), Sec Hdr (1b=`1`), APID (11b) |
| `sequence_flags_count` | 2     | `uint16_t` | CCSDS Primary Header: Packet Sequence Control                                                           | Seq Flags (2b=`11`), Packet Seq Count (14b)                |
| `packet_length`        | 2     | `uint16_t` | CCSDS Primary Header: Packet Data Field Length - 1 (Value: 147 bytes total - 6 header bytes = 141 payload => Value=140) |                                                            |
| `spacecraft_id`        | 12    | `char[12]` | User-defined spacecraft identifier (e.g., "DSN-2025-001")                                                |                                                            |
| `mission_time`         | 4     | `uint32_t` | Milliseconds since MCU boot (Mission Elapsed Time)                                                      |                                                            |
| `temp_bme`             | 4     | `float`  | Temperature from BME280 (°C)                                                                            |                                                            |
| `pres_bme`             | 4     | `float`  | Pressure from BME280 (hPa)                                                                              |                                                            |
| `hum_bme`              | 4     | `float`  | Humidity from BME280 (%)                                                                                |                                                            |
| `alt_bme`              | 4     | `float`  | Altitude from BME280 (m), calculated from pressure                                                      |                                                            |
| `magX`, `magY`, `magZ` | 4 ea. | `float`  | Magnetic Field X, Y, Z from QMC6310 (uT)                                                                |                                                            |
| `accX`, `accY`, `accZ` | 4 ea. | `float`  | Acceleration X, Y, Z from QMI8658 (g)                                                                   |                                                            |
| `gyrX`, `gyrY`, `gyrZ` | 4 ea. | `float`  | Gyroscopic Rate X, Y, Z from QMI8658 (dps)                                                              |                                                            |
| `temp_qmi`             | 4     | `float`  | Temperature from QMI8658 (°C)                                                                           |                                                            |
| `gps_lat`, `gps_lon`   | 8 ea. | `double` | GPS Latitude, Longitude (degrees)                                                                       |                                                            |
| `gps_alt`              | 4     | `float`  | GPS Altitude (m)                                                                                        |                                                            |
| `gps_date_day`         | 1     | `uint8_t`| GPS Day (1-31)                                                                                          |                                                            |
| `gps_date_month`       | 1     | `uint8_t`| GPS Month (1-12)                                                                                          |                                                            |
| `gps_date_year`        | 2     | `uint16_t`| GPS Year                                                                                                |                                                            |
| `gps_time_hour`        | 1     | `uint8_t`| GPS Hour (0-23, UTC)                                                                                    |                                                            |
| `gps_time_minute`      | 1     | `uint8_t`| GPS Minute (0-59, UTC)                                                                                  |                                                            |
| `gps_time_second`      | 1     | `uint8_t`| GPS Second (0-59, UTC)                                                                                  |                                                            |
| `gps_satellites`       | 1     | `uint8_t`| Number of GPS Satellites in view                                                                        |                                                            |
| `gps_hdop`             | 4     | `float`  | GPS Horizontal Dilution of Precision                                                                    |                                                            |
| `battVoltage`          | 4     | `float`  | Battery Voltage (V) from PMU                                                                            |                                                            |
| `battPercent`          | 1     | `uint8_t`| Battery Percentage (%) from PMU                                                                         |                                                            |
| `isCharging`           | 1     | `bool`   | Battery Charging Status (1=Charging, 0=Not Charging) from PMU                                           |                                                            |
| `vbusVoltage`          | 4     | `float`  | VBUS Voltage (V) from PMU                                                                               |                                                            |
| `systemVoltage`        | 4     | `float`  | System Voltage (V) from PMU                                                                             |                                                            |
| `message`              | 16    | `char[16]`| Short status message string (e.g., "Packet: N")                                                         |                                                            |
| `crc`                  | 2     | `uint16_t`| CRC-16 (CCITT-FALSE) checksum over the preceding 145 bytes (from `version_type_apid` to `message`)      |                                                            |
| **Total (Raw Size)** | **147**|          | *(Size before RS-FEC encoding)* |                                                            |

* **Note:** The Secondary Header Flag is set (`1`), but this implementation uses the rest of the Data Field directly for payload without a formally defined Secondary Header structure. The `packet_length` field correctly reflects the length of the Data Field (Bytes 6 to 146 inclusive), which is `147 - 6 = 141` bytes. The value stored is `length - 1`, so `140`.

---

## 🛡️ Reed-Solomon FEC (RS(255, 223)) Details

Forward Error Correction (FEC) adds redundancy to data to allow the receiver to correct errors introduced during transmission over noisy channels.

* **Code:** RS(255, 223) using Galois Field GF(2^8).
    * `n = 255` bytes: Total size of the encoded block (codeword) transmitted.
    * `k = 223` bytes (`rsMsgLen`): Size of the original data payload block used by the encoder.
    * `ECC_LENGTH = n - k = 32` bytes: Number of parity/redundancy bytes added.
* **Error Correction Capability:** This code can correct up to `t = (n - k) / 2 = 16` byte errors occurring anywhere within the 255-byte received block. It can detect more errors than it can correct.
* **Implementation (`buildTelemetryPacket`):**
    1.  The 147-byte `tmPacket` struct (containing data + CRC) is populated.
    2.  The data from `tmPacket` is provided as input to the `rs.Encode()` function. *Note: The exact mechanism, including potential internal padding to 223 bytes and handling of the `memcpy` call preceding `rs.Encode` in the current code, depends on the specific `RS-FEC.h` library implementation and could be clarified for robustness.*
    3.  The `rs.Encode()` method calculates the 32 parity bytes based on the 223-byte data block (payload + padding) and writes the full 255-byte encoded block (padded data + parity bytes) into the `encodedData` buffer.
    4.  The entire 255-byte `encodedData` buffer is transmitted via LoRa.

---

## 📺 Display Interface Details (Optional)

If a U8g2 display is connected and initialized (`displayEnabled = true`):

* **Cycling Mechanism:** The `currentScreen` variable (0-10) tracks the active screen. `updateDisplay()` increments this index every `defaultScreenDelay` milliseconds (e.g., 2000ms), wrapping around. *Note: Ensure the modulo operator in `updateDisplay` is correct (e.g., `% 11`) to cycle through all 11 defined screens (0-10).*
* **Rendering (`drawDisplay`):** Uses U8g2's page buffer mode. A `switch` statement calls the appropriate drawing logic based on `currentScreen`. Each screen function clears the buffer (`u8g2->clearBuffer()`), draws formatted text/values using `u8g2->printf()` and the `u8g2_font_6x10_mr` font, and sends the buffer to the display (`u8g2->sendBuffer()`).
* **Screen Content:** (11 screens total, indexed 0-10)
    * **Screen 0:** QMI8658 Accel (X, Y, Z g), Temp (°C)
    * **Screen 1:** QMI8658 Gyro (X, Y, Z dps)
    * **Screen 2:** BME280 Env (Temp °C, Hum %, Pres hPa, Alt m)
    * **Screen 3:** QMC6310 Mag (X, Y, Z uT), Heading (deg)
    * **Screen 4:** QMI8658 Accel (Detailed)
    * **Screen 5:** QMI8658 Gyro (Detailed), Temp (°C)
    * **Screen 6:** GPS Date (DD/MM/YYYY) & Time (HH:MM:SS UTC) - *Requires valid GPS fix*
    * **Screen 7:** GPS Location (Lat/Lon deg, Alt m), Sats, HDOP - *Requires valid GPS fix*
    * **Screen 8:** LoRa Tx Params (Freq MHz, Power dBm, SF, BW kHz), Last Packet Msg
    * **Screen 9:** PMU Status 1 (Bat V, Bat %, Charging Y/N)
    * **Screen 10:** PMU Status 2 (VBUS V, System V)

---

## ❗ Error Handling (`displayFatalError`)

Basic robustness for critical initialization failures:

* **Checks:** `setup()` verifies the return status of `bme.begin()`, `qmc.begin()`, and `qmi.begin()`.
* **Routine:** If any check fails, `displayFatalError(const char* msg)` is invoked:
    1.  Prints a timestamped fatal error message to the `Serial` monitor (115200 baud).
    2.  If display is enabled, clears it and shows a large "SYSTEM FAILURE" message along with the specific error `msg`.
    3.  Enters an inescapable `while(1)` loop.
    4.  If `LED_BUILTIN` is defined for the board, it continuously blinks the onboard LED as a visual fault indicator. Otherwise, it simply halts CPU execution.

---

## 🧩 Code Structure Overview

* **`TelemetryTx.ino`:** Main Arduino sketch file containing:
    * **Global Scope:** Constant definitions (LoRa, CCSDS, Timing), sensor/radio object instances, `CCSDSPacket` struct definition, `encodedData` buffer, `volatile transmittedFlag`, variables for sensor data storage, display object pointer (`u8g2`).
    * **`setup()`:** System initialization (Serial, Board, Display, Radio, Sensors), configuration application, error checks, initial packet transmission trigger.
    * **`loop()`:** Core operational loop handling transmission completion checks, sensor updates, packet building/encoding, initiating new transmissions, display updates, and GPS data feeding.
    * **`buildTelemetryPacket()`:** Assembles `tmPacket`, calculates CRC-16, performs RS(255, 223) encoding into `encodedData`.
    * **`updateSensorReadings()`:** Orchestrates calls to individual sensor update functions (`updateBME280Data`, etc.).
    * **`update[Sensor]Data()` functions:** Contain logic to read data from specific sensors (BME280, QMC6310, QMI8658, PMU) and update global variables.
    * **`calculateCRC()`:** Implements the CRC-16-CCITT-FALSE algorithm.
    * **`setFlag()`:** Interrupt Service Routine (ISR) attached to LoRa TxDone interrupt; sets `transmittedFlag = true`.
    * **`updateDisplay()` / `drawDisplay()` / `drawScreenX()` functions:** Manage display screen cycling and rendering logic. *(Note: Contains some potentially redundant/unused functions like `drawPMUScreen1`/`2`).*
    * **`displayFatalError()`:** Handles critical initialization errors and halts the system.
* **`LoRaBoards.h` (External, User-Provided):** Defines hardware pin mappings, peripheral configurations, and the `setupBoards()` function. **Must be tailored to the specific hardware.**
* **Custom Sensor Libraries (External):** `SensorQMC6310.hpp`, `SensorQMI8658.hpp`. Encapsulate I2C/SPI communication and register logic for these sensors. *(Source code required separately).*
* **`RS-FEC.h` (External):** Provides the Reed-Solomon encoding function `rs.Encode()`. *(Source code required separately).*

---

## 💡 Potential Improvements & Future Work

* **Receiver Implementation:** Develop a companion Arduino sketch for a LoRa receiver capable of decoding RS(255, 223), parsing the CCSDS packet, verifying the CRC, and displaying/logging the received telemetry.
* **Data Logging:** Integrate an SD card module to log transmitted telemetry packets for later retrieval and analysis.
* **Two-Way Communication (Telecommand):** Implement functionality to receive and act upon simple commands sent via LoRa (requires modifying the loop for receive windows/modes).
* **Power Optimization:** Implement MCU sleep modes (e.g., deep sleep) between transmissions for extended battery life. Requires careful management of wake-up sources (timer, interrupts) and peripheral re-initialization.
* **Advanced Error Handling:** Implement more nuanced error reporting (e.g., non-fatal sensor read errors) and potential recovery strategies beyond halting on initialization failure.
* **Dynamic Configuration:** Allow modification of key parameters (e.g., LoRa settings, reporting intervals) via a serial command interface or other means without needing to recompile.
* **Sensor Fusion:** Implement algorithms (e.g., Madgwick or Mahony filter) using IMU (Accel/Gyro) and Magnetometer data to compute a more stable 3D orientation quaternion or Euler angles.
* **CCSDS Secondary Header:** Utilize the optional secondary header field for more detailed timestamping (e.g., GPS time) or other metadata.
* **Sensor Calibration:** Add routines or use libraries to perform runtime or stored calibration for sensors (especially IMU and Magnetometer) to improve accuracy.
* **Code Refinements:** Address minor issues identified in the current codebase, such as:
    * Correcting the display screen cycling logic in `updateDisplay` (change `% 10` to `% 11`) to include all 11 defined screens.
    * Removing unused/redundant functions (e.g., `drawPMUScreen1`, `drawPMUScreen2`) and variables (e.g., `sensorLastUpdateTime`).
    * Clarifying the interaction between `memcpy` and `rs.Encode` in `buildTelemetryPacket` for robustness and readability based on the specific `RS-FEC.h` library implementation.

---

## 🤝 Contributing

Contributions, issues, bug reports, and feature suggestions are welcome! Please feel free to check the [issues page](https://github.com/YOUR_USERNAME/YOUR_REPOSITORY/issues) of the repository where this code resides or submit a pull request. *(Note: Replace placeholder URL)*

---

## 📜 License

*(Specify the license under which this project is released here. E.g., MIT, Apache 2.0, GPL, etc. A common choice for open-source hardware/software projects is the MIT License.)*

**Example:**
Distributed under the MIT License. See the `LICENSE` or `LICENSE.txt` file distributed with this project for more information.

---

## 🎉 Acknowledgements

* This project heavily relies on the excellent work of the developers behind:
    * [RadioLib](https://github.com/jgromes/RadioLib) by JGromes
    * [Adafruit Libraries (BME280, Unified Sensor)](https://github.com/adafruit)
    * [U8g2](https://github.com/olikraus/u8g2) by olikraus
    * [TinyGPS++](http://arduinogps.jjoe.org/) by Mikal Hart
* Inspiration drawn from the CCSDS standards and the incredible engineering of real-world deep space missions like Voyager 1.
* *(Add acknowledgements for the specific sources/authors of `LoRaBoards.h`, `SensorQMC6310.hpp`, `SensorQMI8658.hpp`, and `RS-FEC.h` if they are based on existing works).*
* *(Add acknowledgement for the specific board support package used, if applicable).*
