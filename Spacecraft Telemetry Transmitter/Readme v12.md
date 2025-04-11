# 🚀 Spacecraft Telemetry Transmitter (LoRa/CCSDS/RS-FEC) 🛰️

This Arduino project implements a telemetry transmitter simulating a spacecraft's communication system. It gathers data from various sensors, formats it into CCSDS-compliant packets (similar to standards used by missions like Voyager 1), applies Reed-Solomon Forward Error Correction (RS-FEC) for reliability, and transmits the data over a LoRa radio link using an SX1262 module.

An optional U8g2-compatible display can be used to monitor sensor readings and system status locally.

## ✨ Features

* 📡 **LoRa Transmission:** Sends telemetry data wirelessly using SX1262 LoRa module via the RadioLib library.
* 📦 **CCSDS Packet Standard:** Formats sensor data into CCSDS Application Packets (AP) for standardized space communication.
* 🛡️ **Forward Error Correction:** Implements Reed-Solomon RS(255, 223) encoding to improve data robustness against transmission errors.
* 🔬 **Multi-Sensor Integration:** Reads data from:
    * 🌡️ BME280: Temperature, Pressure, Humidity, Altitude.
    * 🧭 QMC6310: 3-Axis Magnetometer.
    * <0xF0><0x9F><0xA7><0xBA> QMI8658: 6-Axis IMU (Accelerometer, Gyroscope) & Temperature.
    * 🗺️ GPS: Location (Lat/Lon), Altitude, Date/Time, Satellite Count, HDOP.
    * 🔋 PMU: Battery Voltage, Charge Percentage, Charging Status, VBUS Voltage, System Voltage.
* 🔒 **Data Integrity:** Includes a CRC-16 (CCITT-FALSE) checksum within the telemetry packet.
* 📟 **Optional Display:** Supports U8g2 monochrome displays for visualizing real-time data across multiple screens.
* 🎓 **Educational Focus:** Designed to demonstrate core concepts of spacecraft telemetry systems.

## 🔌 Hardware Requirements

* **Microcontroller:** An Arduino-compatible board (Specific pinouts and board features are expected to be defined in `LoRaBoards.h`).
* **LoRa Module:** SX1262 based LoRa transceiver.
* **Sensors:**
    * Adafruit BME280 (or compatible I2C sensor).
    * QMC6310 Magnetometer (I2C).
    * QMI8658 IMU (SPI).
    * GPS Module (Serial connection, e.g., UART).
    * Integrated PMU (Power Management Unit - specific to the target board, accessed via `PMU` object).
* **(Optional) Display:** U8g2 compatible monochrome display (I2C or SPI).
* **Wiring:** Appropriate connections between the MCU, sensors, LoRa module, and display.

## 📚 Software & Dependencies

This project relies on the following libraries:

* [**RadioLib**](https://github.com/jgromes/RadioLib): For LoRa communication (`RadioLib.h`).
* **`LoRaBoards.h`**: **Custom/Board-Specific Header.** *You need to provide or configure this file based on your specific hardware setup.* It defines pin mappings (`RADIO_CS_PIN`, `RADIO_DIO1_PIN`, etc.) and board-specific initializations (`setupBoards()`).
* [**Adafruit BME280 Library**](https://github.com/adafruit/Adafruit_BME280_Library): For the BME280 sensor (`Adafruit_Sensor.h`, `Adafruit_BME280.h`). Requires the [Adafruit Unified Sensor Driver](https://github.com/adafruit/Adafruit_Sensor).
* **`SensorQMC6310.hpp`**: Custom library for the QMC6310 magnetometer. *(Link to source if available)*.
* **`SensorQMI8658.hpp`**: Custom library for the QMI8658 IMU. *(Link to source if available)*.
* [**U8g2 Library**](https://github.com/olikraus/u8g2): For the monochrome display (`U8g2lib.h`).
* **`RS-FEC.h`**: Reed-Solomon Forward Error Correction library. *(Provide source/link if custom or from a specific library)*.
* [**TinyGPS++**](http://arduinogps.jjoe.org/): For parsing NMEA sentences from the GPS module (`TinyGPS++.h`).
* **Standard Libraries:** `Wire.h`, `SPI.h`, `math.h`, `stdio.h` (usually included with the Arduino core).

## ⚙️ Installation & Setup

1.  **Install Arduino IDE:** Ensure you have the latest Arduino IDE installed.
2.  **Install Libraries:** Open the Arduino IDE Library Manager (`Sketch` > `Include Library` > `Manage Libraries...`) and install:
    * `RadioLib`
    * `Adafruit BME280 Library`
    * `Adafruit Unified Sensor`
    * `U8g2`
    * `TinyGPS++`
    * Manually add the custom libraries (`SensorQMC6310.hpp`, `SensorQMI8658.hpp`, `RS-FEC.h`) and the board-specific `LoRaBoards.h` to your Arduino sketch folder or libraries directory.
3.  **Clone Repository:** Download or clone this repository to your local machine.
4.  **Configure `LoRaBoards.h`:** **Crucially, ensure `LoRaBoards.h` matches your specific hardware connections** (SPI/I2C pins, Radio pins, GPS Serial port, etc.). This file likely contains pin definitions and the `setupBoards()` function.
5.  **Connect Hardware:** Wire the MCU, sensors, LoRa module, GPS, and display according to the pin definitions in your code and `LoRaBoards.h`. Ensure correct voltage levels are used.
6.  **Select Board & Port:** Choose the correct Arduino board and COM port in the Arduino IDE.
7.  **Upload:** Compile and upload the `TelemetryTx.ino` sketch to your board.

## 🔧 Configuration

Key parameters can be adjusted directly in the `TelemetryTx.ino` file:

* **LoRa Settings:**
    * `CARRIER_FREQ`: 433.0 MHz (Adjust based on local regulations and hardware).
    * `TX_POWER`: 20 dBm.
    * `BANDWIDTH`: 125.0 kHz.
    * `SPREADING_FACTOR`: 12 (Higher value = longer range, lower data rate).
    * `CODING_RATE`: 6 (Higher value = more error correction, lower data rate).
    * `SYNC_WORD`: 0x35 (Change to differentiate your network).
* **CCSDS Settings:**
    * `SPACECRAFT_ID`: "DSN-2025-001" (Customize as needed).
    * `CCSDS_APID`: 0x7FF (Example Application Process ID).
* **Sensor Settings:**
    * `SEALEVELPRESSURE_HPA`: 1013.25 (Calibrate for accurate altitude readings).
    * Magnetic Declination: Adjusted within `updateQMC6310Data()` for accurate compass heading (currently set for Galway, Ireland).
    * Sensor I2C addresses or SPI CS pins might be defined in `LoRaBoards.h` or the custom sensor libraries.
* **Timing:**
    * `sensorUpdateInterval`: 100 ms (How often sensor data is read).
    * `defaultScreenDelay`: 2000 ms (Time each screen is shown on the display).

## ▶️ Usage

1.  Power on the device.
2.  The system will initialize the radio, sensors, and display.
3.  It will continuously:
    * Read data from all connected sensors.
    * Read GPS data from the serial port.
    * Update PMU status.
    * Build a `CCSDSPacket` containing the latest data, timestamp, and packet counter.
    * Calculate a CRC-16 checksum.
    * Encode the packet using RS(255, 223).
    * Transmit the encoded packet via LoRa.
    * If a display is connected, it will cycle through different screens showing sensor data, GPS status, LoRa parameters, and PMU info.
4.  A corresponding LoRa receiver (not included in this code) is needed to decode and interpret the transmitted telemetry data. The receiver must:
    * Use the same LoRa parameters (Frequency, SF, BW, CR, Sync Word).
    * Be able to decode RS(255, 223) FEC.
    * Parse the `CCSDSPacket` structure.
    * Verify the CRC-16 checksum.

## 📊 CCSDS Packet Structure (`CCSDSPacket`)

The telemetry data is packed into the following structure (`#pragma pack(push, 1)` ensures byte alignment):

| Field                  | Bytes | Type      | Description                                                                                                |
| :--------------------- | :---- | :-------- | :--------------------------------------------------------------------------------------------------------- |
| `version_type_apid`    | 2     | `uint16_t`| CCSDS Primary Header: Version (3b), Type (1b), Sec Hdr Flag (1b), APID (11b)                               |
| `sequence_flags_count` | 2     | `uint16_t`| CCSDS Primary Header: Sequence Flags (2b), Packet Sequence Count (14b)                                     |
| `packet_length`        | 2     | `uint16_t`| CCSDS Primary Header: Packet Data Field Length - 1 (Size of payload + secondary header - 1)                 |
| `spacecraft_id`        | 12    | `char[12]`| User-defined spacecraft identifier                                                                         |
| `mission_time`         | 4     | `uint32_t`| Milliseconds since MCU boot                                                                                |
| `temp_bme`             | 4     | `float`   | Temperature from BME280 (°C)                                                                               |
| `pres_bme`             | 4     | `float`   | Pressure from BME280 (hPa)                                                                                 |
| `hum_bme`              | 4     | `float`   | Humidity from BME280 (%)                                                                                   |
| `alt_bme`              | 4     | `float`   | Altitude from BME280 (m)                                                                                   |
| `magX`, `magY`, `magZ` | 4 ea. | `float`   | Magnetic Field X, Y, Z from QMC6310 (uT)                                                                   |
| `accX`, `accY`, `accZ` | 4 ea. | `float`   | Acceleration X, Y, Z from QMI8658 (g)                                                                      |
| `gyrX`, `gyrY`, `gyrZ` | 4 ea. | `float`   | Gyroscopic Rate X, Y, Z from QMI8658 (dps)                                                                 |
| `temp_qmi`             | 4     | `float`   | Temperature from QMI8658 (°C)                                                                              |
| `gps_lat`, `gps_lon`   | 8 ea. | `double`  | GPS Latitude, Longitude (degrees)                                                                          |
| `gps_alt`              | 4     | `float`   | GPS Altitude (m)                                                                                           |
| `gps_date_day`         | 1     | `uint8_t` | GPS Day (1-31)                                                                                             |
| `gps_date_month`       | 1     | `uint8_t` | GPS Month (1-12)                                                                                           |
| `gps_date_year`        | 2     | `uint16_t`| GPS Year                                                                                                   |
| `gps_time_hour`        | 1     | `uint8_t` | GPS Hour (0-23, UTC)                                                                                       |
| `gps_time_minute`      | 1     | `uint8_t` | GPS Minute (0-59, UTC)                                                                                     |
| `gps_time_second`      | 1     | `uint8_t` | GPS Second (0-59, UTC)                                                                                     |
| `gps_satellites`       | 1     | `uint8_t` | Number of GPS Satellites in view                                                                           |
| `gps_hdop`             | 4     | `float`   | GPS Horizontal Dilution of Precision                                                                       |
| `battVoltage`          | 4     | `float`   | Battery Voltage (V)                                                                                        |
| `battPercent`          | 1     | `uint8_t` | Battery Percentage (%)                                                                                     |
| `isCharging`           | 1     | `bool`    | Battery Charging Status (1=Charging, 0=Not Charging)                                                       |
| `vbusVoltage`          | 4     | `float`   | VBUS Voltage (V)                                                                                           |
| `systemVoltage`        | 4     | `float`   | System Voltage (V)                                                                                         |
| `message`              | 16    | `char[16]`| Short message string (e.g., "Packet: N")                                                                   |
| `crc`                  | 2     | `uint16_t`| CRC-16 (CCITT-FALSE) checksum over the entire packet *before* this field                                   |
| **Total (Raw)** | **147**|           | *(Size before RS-FEC)* |

*Note: The actual transmitted packet size will be `rsMsgLen + ECC_LENGTH = 223 + 32 = 255` bytes after RS-FEC encoding. The `CCSDSPacket` structure (147 bytes) fits within the `rsMsgLen` (223 bytes), leaving padding space before encoding.*

## 📺 Display Interface (Optional)

If a U8g2 display is connected and initialized, the firmware will cycle through the following screens:

* **Screen 0:** QMI8658 Accelerometer (X, Y, Z) & Temperature.
* **Screen 1:** QMI8658 Gyroscope (X, Y, Z).
* **Screen 2:** BME280 Environmental Data (Temp, Hum, Pres, Alt).
* **Screen 3:** QMC6310 Magnetometer (X, Y, Z) & Calculated Heading.
* **Screen 4:** QMI8658 Accelerometer (Detailed).
* **Screen 5:** QMI8658 Gyroscope (Detailed) & Temperature.
* **Screen 6:** GPS Date & Time (UTC).
* **Screen 7:** GPS Location (Lat, Lon, Alt), Satellites, HDOP.
* **Screen 8:** LoRa Transmission Parameters (Freq, Power, SF, BW) & Last Packet Msg.
* **Screen 9:** PMU Status 1 (Battery Voltage, Percentage, Charging Status).
* **Screen 10:** PMU Status 2 (VBUS Voltage, System Voltage).

## 🤝 Contributing

Contributions, issues, and feature requests are welcome! Feel free to check the [issues page](https://github.com/YOUR_USERNAME/YOUR_REPOSITORY/issues).

## 📜 License

*(Specify your license here, e.g., MIT, Apache 2.0. If unsure, MIT is a common choice for open-source hardware projects.)*

Example: Distributed under the MIT License. See `LICENSE` file for more information.

## 🎉 Acknowledgements

* [RadioLib](https://github.com/jgromes/RadioLib) library developers.
* [Adafruit](https://github.com/adafruit) for their sensor libraries.
* [Olikraus](https://github.com/olikraus) for the U8g2 display library.
* [Mikal Hart](http://arduinogps.jjoe.org/) for the TinyGPS++ library.
* Inspiration from CCSDS standards and missions like Voyager 1.
* *(Add any other acknowledgements if necessary)*
