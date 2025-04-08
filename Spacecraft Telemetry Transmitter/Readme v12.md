# 🛰️ LoRa Spacecraft Telemetry Transmitter (CCSDS & RS-FEC) 📡

This Arduino project implements a telemetry transmitter simulating a spacecraft subsystem. It collects data from various onboard sensors, formats it into CCSDS-compliant telemetry packets, enhances reliability using Reed-Solomon Forward Error Correction (RS-FEC), and transmits it over a LoRa radio link using the RadioLib library. The system is designed with concepts similar to those used in real space missions like Voyager 1 for data structuring and robustness.

An optional U8g2-compatible display provides real-time visualization of sensor readings and system status.

---

## 🌟 Key Features

* **Multi-Sensor Integration:** Collects data from:
    * 🌡️ **BME280:** Temperature, Pressure, Humidity, Altitude.
    * 🧭 **QMC6310:** 3-Axis Magnetometer.
    * ✈️ **QMI8658:** 6-Axis IMU (3-Axis Accelerometer, 3-Axis Gyroscope) + Temperature.
    * 🛰️ **GPS (TinyGPS++):** Latitude, Longitude, Altitude, Date, Time, Satellite Count, HDOP.
    * 🔋 **PMU (AXPxxx):** Battery Voltage, Charge Percentage, Charging Status, VBUS Voltage, System Voltage.
* **LoRa Communication:** Uses SX1262 LoRa module via RadioLib for long-range, low-power radio transmission (433 MHz band configured).
* **CCSDS Packet Formatting:** Structures telemetry data into CCSDS Application Packets (APs) for standardized space data handling. Includes:
    * Primary Header (Version, Type, Sec HDR Flag, APID, Seq Flags, Seq Count).
    * Packet Data Field containing timestamp, spacecraft ID, and all sensor readings.
* **Reed-Solomon Forward Error Correction (RS-FEC):** Implements RS(255, 223) coding to add redundancy, significantly improving data integrity over noisy radio channels. 🛡️
* **CRC Checksum:** Calculates CRC-16-CCITT-FALSE for end-to-end data validation within the packet. ✔️
* **Optional OLED Display:** Uses U8g2 library to display sensor data, GPS status, LoRa parameters, and PMU status on multiple cycling screens. 🖥️
* **Interrupt-Driven Transmission:** Uses LoRa module interrupts (`setFlag`) for efficient handling of transmission completion.

---

## 🛠️ Hardware Requirements

* **Microcontroller:** An Arduino-compatible board supported by the specified libraries and with sufficient I/O for all peripherals (e.g., ESP32, Heltec LoRa boards, TTGO LoRa boards). Board-specific pin definitions are expected in `LoRaBoards.h`.
* **LoRa Module:** SX1262 based LoRa transceiver module connected via SPI and relevant control pins (NSS, DIO1, NRESET, BUSY). TCXO control is optional.
* **Sensors:**
    * BME280 Sensor (I2C)
    * QMC6310 Magnetometer (I2C - *Note: Requires the specific `SensorQMC6310.hpp` library*)
    * QMI8658 IMU (SPI or I2C - *Note: Code uses SPI via `IMU_CS` pin. Requires the specific `SensorQMI8658.hpp` library*)
    * GPS Module (UART/Serial connection - `SerialGPS` defined in `LoRaBoards.h` likely)
    * PMU (Integrated on some boards, e.g., AXP192/AXP202, accessed via I2C implicitly through board support package/`LoRaBoards.h`).
* **Display (Optional):** U8g2 compatible OLED/LCD display (typically I2C).
* **Wiring:** Appropriate connections via I2C bus (SDA, SCL), SPI bus (MOSI, MISO, SCK, CS), UART (TX, RX), and power (3.3V/5V, GND).

---

## 📚 Software Dependencies (Libraries)

* **RadioLib:** For LoRa communication. ([https://github.com/jgromes/RadioLib](https://github.com/jgromes/RadioLib))
* **`LoRaBoards.h`:** **Crucial board-specific header.** This file *must* define pin assignments (`RADIO_CS_PIN`, `RADIO_DIO1_PIN`, etc.), potentially initialize the PMU (`PMU`), define `SerialGPS`, and set up the display instance (`u8g2`). This is **not** a standard library and needs to be provided/created for the specific target hardware.
* **Wire:** Arduino standard I2C library.
* **SPI:** Arduino standard SPI library.
* **Adafruit Unified Sensor:** Dependency for Adafruit BME280 library. ([https://github.com/adafruit/Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor))
* **Adafruit BME280 Library:** For interfacing with the BME280 sensor. ([https://github.com/adafruit/Adafruit_BME280_Library](https://github.com/adafruit/Adafruit_BME280_Library))
* **`SensorQMC6310.hpp`:** Custom library for the QMC6310 magnetometer. (User/Vendor provided - **Source URL needed**)
* **`SensorQMI8658.hpp`:** Custom library for the QMI8658 IMU. (User/Vendor provided - **Source URL needed**)
* **U8g2lib:** For monochrome OLED/LCD displays. ([https://github.com/olikraus/u8g2](https://github.com/olikraus/u8g2))
* **`RS-FEC.h`:** Reed-Solomon Forward Error Correction library. (Likely custom or specific library - **Source URL needed**)
* **TinyGPS++:** For parsing NMEA sentences from the GPS module. ([http://arduinogps.jjoe.org/](http://arduinogps.jjoe.org/))
* **Standard C Libraries:** `math.h`, `stdio.h` (Included with Arduino core).

---

## ⚙️ Configuration

Key parameters are defined using `#define` or constants near the beginning of the code:

* **`LoRaBoards.h`:** **Primary configuration point for hardware pins and board setup.**
* **`SPACECRAFT_ID`:** A string identifier for this telemetry source (Max 12 chars used in packet).
* **`CCSDS_*` Defines:** Configure CCSDS primary header fields (APID, Version, Type, etc.). Defaults are generally suitable for basic telemetry.
* **Reed-Solomon (`rsMsgLen`, `ECC_LENGTH`):** Defines the RS(255, 223) code parameters. Changing these requires corresponding changes on the receiver.
* **LoRa Parameters (`CARRIER_FREQ`, `TX_POWER`, `BANDWIDTH`, `SPREADING_FACTOR`, `CODING_RATE`, `SYNC_WORD`):** Must match the receiver configuration. Higher `SPREADING_FACTOR` and lower `BANDWIDTH` increase range but decrease data rate. Higher `CODING_RATE` increases robustness but decreases data rate.
* **Sensor Update (`sensorUpdateInterval`):** How often sensor data is polled (milliseconds).
* **Display Timing (`defaultScreenDelay`, `gpsScreenDelay`):** How long each screen is shown. (Note: The current `updateDisplay` function uses a fixed 2000ms delay, overriding these constants).
* **`SEALEVELPRESSURE_HPA`:** Used by BME280 for altitude calculation. Adjust for more accurate local readings if needed.
* **Magnetic Declination:** The `declination` variable inside `updateQMC6310Data()` is hardcoded for Galway, Ireland (`-0.041` radians). **Adjust this value based on your geographic location** for accurate compass heading.

---

## 🔬 Core Concepts Explained

### LoRa Communication (RadioLib) 📡

* **Modulation:** LoRa (Long Range) spread spectrum technique allowing long-distance communication with low power.
* **Library:** RadioLib provides a high-level interface to control the SX1262 transceiver.
* **Parameters:**
    * `Frequency (CARRIER_FREQ)`: 433.0 MHz (Ensure compliance with local regulations).
    * `Bandwidth (BANDWIDTH)`: 125.0 kHz.
    * `Spreading Factor (SPREADING_FACTOR)`: 12 (Higher = more robust, slower).
    * `Coding Rate (CODING_RATE)`: 4/6 (Explicitly 6/8 = 1.5). Higher = more FEC within LoRa itself.
    * `Sync Word (SYNC_WORD)`: 0x35. A private network identifier.
    * `Transmit Power (TX_POWER)`: 20 dBm. Maximum power for many modules.
    * `Preamble Length`: 16 symbols.
    * `CRC`: LoRa hardware CRC is disabled (`radio.setCRC(false)`) because a more robust packet-level CRC is implemented.
* **Operation:** The code transmits data asynchronously using `startTransmit()`. An Interrupt Service Routine (`setFlag`) signals transmission completion via `transmittedFlag`.

### CCSDS Packet Structure 📦📊

* **Standard:** Follows the Consultative Committee for Space Data Systems (CCSDS) recommendations for packet telemetry, enabling interoperability.
* **Primary Header (6 bytes total):**
    * `Version` (3 bits): `0b000` (CCSDS Version 1).
    * `Type` (1 bit): `0b0` (Telemetry).
    * `Secondary Header Flag` (1 bit): `0b1` (Indicates presence, though not used in data field).
    * `APID` (11 bits): `0x7FF` (Application Process ID - identifies the data source/type).
    * `Sequence Flags` (2 bits): `0b11` (Unsegmented data).
    * `Packet Sequence Count` (14 bits): Increments for each packet (`packetCounter`), wraps around.
    * `Packet Length` (16 bits): Length of the *packet data field* minus 1. Calculated as `sizeof(CCSDSPacket) - 7`.
* **Packet Data Field (`CCSDSPacket` Struct):** Contains the actual telemetry payload. Defined using `#pragma pack(push, 1)` to ensure no padding bytes are inserted by the compiler, crucial for byte-level transmission.

    | Field              | C Type          | Size (Bytes) | Description                                      |
    | :----------------- | :-------------- | :----------- | :----------------------------------------------- |
    | `version_type_apid`| `uint16_t`      | 2            | Combined Version, Type, Sec Hdr, APID          |
    | `sequence_flags_count` | `uint16_t`  | 2            | Combined Sequence Flags, Sequence Count          |
    | `packet_length`    | `uint16_t`      | 2            | Packet Data Field Length - 1                     |
    | `spacecraft_id`    | `char[12]`      | 12           | Spacecraft Identifier String                     |
    | `mission_time`     | `uint32_t`      | 4            | Milliseconds since microcontroller boot          |
    | `temp_bme`         | `float`         | 4            | BME280 Temperature (°C)                          |
    | `pres_bme`         | `float`         | 4            | BME280 Pressure (hPa)                            |
    | `hum_bme`          | `float`         | 4            | BME280 Humidity (%)                              |
    | `alt_bme`          | `float`         | 4            | BME280 Altitude (m)                              |
    | `magX`             | `float`         | 4            | QMC6310 Magnetometer X (uT)                      |
    | `magY`             | `float`         | 4            | QMC6310 Magnetometer Y (uT)                      |
    | `magZ`             | `float`         | 4            | QMC6310 Magnetometer Z (uT)                      |
    | `accX`             | `float`         | 4            | QMI8658 Accelerometer X (g)                      |
    | `accY`             | `float`         | 4            | QMI8658 Accelerometer Y (g)                      |
    | `accZ`             | `float`         | 4            | QMI8658 Accelerometer Z (g)                      |
    | `gyrX`             | `float`         | 4            | QMI8658 Gyroscope X (dps)                        |
    | `gyrY`             | `float`         | 4            | QMI8658 Gyroscope Y (dps)                        |
    | `gyrZ`             | `float`         | 4            | QMI8658 Gyroscope Z (dps)                        |
    | `temp_qmi`         | `float`         | 4            | QMI8658 Temperature (°C)                         |
    | `gps_lat`          | `double`        | 8            | GPS Latitude (Degrees)                           |
    | `gps_lon`          | `double`        | 8            | GPS Longitude (Degrees)                          |
    | `gps_alt`          | `float`         | 4            | GPS Altitude (m)                                 |
    | `gps_date_day`     | `uint8_t`       | 1            | GPS Day (1-31)                                   |
    | `gps_date_month`   | `uint8_t`       | 1            | GPS Month (1-12)                                 |
    | `gps_date_year`    | `uint16_t`      | 2            | GPS Year                                         |
    | `gps_time_hour`    | `uint8_t`       | 1            | GPS Hour (0-23 UTC)                              |
    | `gps_time_minute`  | `uint8_t`       | 1            | GPS Minute (0-59 UTC)                            |
    | `gps_time_second`  | `uint8_t`       | 1            | GPS Second (0-59 UTC)                            |
    | `gps_satellites`   | `uint8_t`       | 1            | GPS Satellites in View                           |
    | `gps_hdop`         | `float`         | 4            | GPS Horizontal Dilution of Precision             |
    | `battVoltage`      | `float`         | 4            | PMU Battery Voltage (V)                          |
    | `battPercent`      | `uint8_t`       | 1            | PMU Battery Percentage (%)                       |
    | `isCharging`       | `bool`          | 1            | PMU Charging Status (True/False)                 |
    | `vbusVoltage`      | `float`         | 4            | PMU VBUS Voltage (V)                             |
    | `systemVoltage`    | `float`         | 4            | PMU System Voltage (V)                           |
    | `message`          | `char[16]`      | 16           | Identification message string                    |
    | `crc`              | `uint16_t`      | 2            | CRC-16 Checksum (calculated over preceding data) |
    | **Total Size** |                 | **~173 bytes**| *(Approximate, check `sizeof(CCSDSPacket)`)* |

### Reed-Solomon FEC 🛡️📶

* **Purpose:** Adds redundancy to the data *before* LoRa transmission to correct errors introduced by the communication channel (noise, interference). Essential for reliable data transfer, especially over long distances or in noisy environments.
* **Code:** RS(255, 223) is used.
    * `n = 255`: Total block size after encoding (bytes).
    * `k = 223` (`rsMsgLen`): Size of the original data message (bytes).
    * `n - k = 32` (`ECC_LENGTH`): Number of parity/redundancy bytes added.
* **Capability:** This code can correct up to `(n - k) / 2 = 16` byte errors within each 255-byte block.
* **Implementation:** The `RS::ReedSolomon` class from `RS-FEC.h` is used. The `buildTelemetryPacket` function copies the first `rsMsgLen` (223) bytes of the `tmPacket` into `encodedData`, then calls `rs.Encode()` to calculate the 32 parity bytes and append them, filling the `encodedData` buffer (total size 255 bytes). The entire `encodedData` buffer is then transmitted via LoRa. *Note: The current `CCSDSPacket` size (~173 bytes) is less than `rsMsgLen` (223 bytes). The remaining bytes up to 223 in the `encodedData` buffer before encoding will contain undefined data unless explicitly padded/zeroed.*

### Sensor Integration & Updates 🌡️🧭✈️🛰️🔋

* Sensors are initialized in `setup()`. Failures halt execution via `displayFatalError()`.
* Accelerometer and Gyroscope ranges and data rates are configured for the QMI8658.
* Magnetometer mode, range, and data rate are configured for the QMC6310.
* `updateSensorReadings()` is called periodically in the loop (or after transmission). It checks the `sensorUpdateInterval` before calling individual update functions (`updateBME280Data`, `updateQMC6310Data`, `updateQMI8658Data`, `updatePMUData`).
* GPS data is processed continuously in `loop()` and `updateSensorReadings()` by feeding bytes from `SerialGPS` to `gps.encode()`.
* Magnetometer heading calculation includes a hardcoded magnetic declination value specific to Galway, Ireland. **This needs adjustment for other locations.**

### CRC Checksum ✔️🔒

* **Purpose:** Verifies data integrity *within* the CCSDS packet structure, detecting accidental corruption. This is done *before* RS-FEC encoding.
* **Algorithm:** CRC-16-CCITT-FALSE (Polynomial `0x1021`, Initial value `0xFFFF`, non-reflected).
* **Implementation:** The `calculateCRC()` function computes the checksum over the entire `CCSDSPacket` struct *except* for the final `crc` field itself. The result is stored in `tmPacket.crc`. The receiver must perform the same calculation on the received data (after RS-FEC decoding) and compare CRCs.

### Display (U8g2) 🖥️

* Uses the U8g2 library for flexibility with various monochrome displays.
* Initialized in `setup()`.
* `updateDisplay()` cycles through `currentScreen` index every 2 seconds (hardcoded).
* `drawDisplay()` uses a `switch` statement to call appropriate drawing routines based on `currentScreen`, showing data from different sensors, GPS, LoRa status, and PMU status. GPS screens have dedicated drawing functions (`drawGPSDateTimeScreen`, `drawGPSLocationScreen`).

---

## 🔄 Code Structure & Workflow

1.  **Includes & Globals:** Libraries are included, global constants, variables, sensor objects, radio object, packet structure, and RS encoder instance are defined.
2.  **`setup()`:**
    * Initializes Serial, GPS Serial, and the display (`u8g2`).
    * Calls `setupBoards()` (from `LoRaBoards.h`) for board-specific initialization (pins, PMU, etc.).
    * Initializes and configures the LoRa radio (frequency, power, SF, BW, CR, Sync Word, etc.). Sets `setFlag` as the TX completion ISR callback.
    * Initializes all sensors (BME280, QMC6310, QMI8658), halting on failure.
    * Configures IMU and Magnetometer parameters (range, data rate).
    * Performs an initial sensor reading (`updateSensorReadings`).
    * Builds the first telemetry packet (`buildTelemetryPacket`), including CCSDS header, sensor data, message, CRC calculation, and RS-FEC encoding into `encodedData`.
    * Starts the first LoRa transmission (`radio.startTransmit(encodedData, ...)`).
3.  **`loop()`:**
    * **Checks `transmittedFlag`:** If `true` (set by ISR upon TX completion):
        * Resets `transmittedFlag`.
        * Increments `packetCounter`.
        * Updates sensor readings (`updateSensorReadings`).
        * Builds the next telemetry packet (`buildTelemetryPacket`) with updated data and performs RS-FEC encoding.
        * Starts the next LoRa transmission.
    * **Updates Display:** Calls `updateDisplay()` to cycle the screen index and `drawDisplay()` to render the current screen content.
    * **Processes GPS:** Reads available bytes from `SerialGPS` and feeds them to `gps.encode()`.
    * Includes a small `delay(1)` for stability.
4.  **Helper Functions:**
    * `setFlag()`: ISR callback executed when LoRa transmission finishes. Sets `transmittedFlag`.
    * `calculateCRC()`: Computes the CRC-16 checksum.
    * `buildTelemetryPacket()`: Populates the `CCSDSPacket` struct, calculates CRC, and performs RS-FEC encoding into `encodedData`.
    * `updateSensorReadings()` / `update<Sensor>Data()`: Read data from respective sensors.
    * `updateDisplay()` / `drawDisplay()` / `draw<Screen>()`: Handle display logic.
    * `displayFatalError()`: Displays an error message and halts execution (implementation expected in `LoRaBoards.h` or similar).

---

## 🚀 Building and Running

1.  **Install Libraries:** Ensure all libraries listed under "Software Dependencies" are installed in your Arduino IDE or PlatformIO environment.
2.  **Configure `LoRaBoards.h`:** Create or modify this header file to match your specific microcontroller board's pin connections for the LoRa module (CS, DIO1, RST, BUSY, TCXO_ENABLE if used), GPS (`SerialGPS`), and display (SDA, SCL, RST if needed, and the `U8G2` constructor). It should also handle any board-specific setup like initializing the PMU (`PMU`).
3.  **Adjust Configuration:**
    * Modify the `declination` value in `updateQMC6310Data()` for your location.
    * Verify LoRa parameters match your receiver and local regulations.
    * Set the `SPACECRAFT_ID`.
4.  **Connect Hardware:** Wire up all sensors, the LoRa module, GPS, and display according to the pin definitions in `LoRaBoards.h`.
5.  **Build & Upload:** Compile and upload the sketch to your microcontroller board using the Arduino IDE or PlatformIO.
6.  **Monitor:** Open the Serial Monitor (115200 baud) for debug messages. Check the OLED display (if connected) for status information. A compatible LoRa receiver running corresponding decoding software is needed to receive and verify the telemetry data.

---

## ✨ Voyager 1 Context

This project draws inspiration from real spacecraft telemetry systems:

* **CCSDS:** The packet structure standard is widely used across space agencies (NASA, ESA) for interoperability, including missions like Voyager.
* **Telemetry Data:** The types of data collected (environmental, inertial, magnetic, position) are representative of housekeeping and basic science data sent by probes.
* **Error Correction:** RS-FEC is crucial for deep-space missions like Voyager, where signals are incredibly weak and prone to errors over vast distances. This code demonstrates the same principle over a shorter LoRa link.
* **DSN Analogy:** LoRa serves as the communication link, analogous to how Voyager uses the Deep Space Network (DSN) for its long-haul communication back to Earth.

---

## 💡 Potential Improvements & Future Work

* **Receiver Implementation:** Develop a corresponding LoRa receiver sketch to decode the RS-FEC data, verify the CRC, and parse the CCSDS packet.
* **Data Storage:** Implement logging of telemetry data to an SD card.
* **Power Optimization:** Add sleep modes between transmissions to conserve battery power.
* **Two-Way Communication:** Implement acknowledgments or telecommands from the receiver.
* **More Robust Error Handling:** Add more detailed checks for sensor read failures or communication issues.
* **Dynamic Configuration:** Allow configuration changes via serial commands or a simple interface.
* **Attitude Estimation:** Use IMU and Magnetometer data for basic attitude determination (e.g., roll, pitch, yaw).
* **Zero Padding:** Explicitly zero-pad the unused portion of the `tmPacket` data within the `rsMsgLen` boundary before RS encoding for cleaner data handling.

---

## 📄 License

(Specify your license here, e.g., MIT, GPL, Apache 2.0, or leave as is if unsure)
This project is provided as-is for educational purposes. No license specified.
