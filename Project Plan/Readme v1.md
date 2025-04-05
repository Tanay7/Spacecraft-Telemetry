# Spacecraft Telemetry Transmitter System Project Plan

## 1. Project Title

Spacecraft Telemetry Transmitter for LoRa Communication with Sensor Data

## 2. Project Goal

To develop a functional prototype of a spacecraft telemetry transmitter capable of collecting data from various sensors, formatting it into CCSDS packets, encoding it with Reed-Solomon FEC, and transmitting it over a LoRa radio link. The system will also include a basic display for visualizing sensor readings and system status.

## 3. Scope

### 3.1. Inclusions

* Implementation of data acquisition from the following sensors:
    * BME280: Environmental sensor (temperature, pressure, humidity, altitude).
    * QMC6310: Magnetometer (X, Y, Z axis magnetic field).
    * QMI8658: Inertial Measurement Unit (IMU) (accelerometer, gyroscope, temperature).
    * GPS: Global Positioning System module (location, altitude, date, time, satellites, HDOP).
    * PMU: Power Management Unit (battery voltage, percentage, charging status, VBUS voltage, system voltage).
* Formatting sensor data into CCSDS (Consultative Committee for Space Data Systems) Application Packets (APs).
* Implementation of Reed-Solomon (RS(255, 223)) Forward Error Correction (FEC) for enhanced data reliability.
* Transmission of telemetry packets using a SX1262 LoRa transceiver.
* Basic visualization of sensor data and system status on an optional U8g2 monochrome display.
* Calculation and inclusion of a CRC-16 checksum (CCITT-FALSE) for data integrity verification within the CCSDS packet.

### 3.2. Exclusions

* Development of the receiver system or ground station software.
* Detailed power budget analysis and optimization.
* Advanced error handling mechanisms beyond basic sensor initialization checks.
* Implementation of data logging or storage functionalities.
* Over-the-air firmware updates.
* Security considerations for the communication link.
* Physical enclosure design and fabrication.
* Flight testing or deployment of the system.
* Integration with any external systems beyond the LoRa radio link.
* Any aspects related to stakeholder identification, communication, or management.

## 4. Deliverables

* Functional Arduino code (`TelemetryTx.ino`) for the telemetry transmitter.
* CCSDS Application Packet structure definition within the code.
* Implementation of Reed-Solomon encoding using the `RS-FEC.h` library.
* Configuration of the SX1262 LoRa transceiver using the RadioLib library.
* Basic display functionality for sensor data and system status using the U8g2 library (if a display is connected).
* This project plan document in `README.md` format.

## 5. Technical Requirements

### 5.1. Hardware

* **Microcontroller:** Arduino-compatible board (specific board defined in `LoRaBoards.h`).
* **LoRa Transceiver:** SX1262 module with necessary antenna.
* **Environmental Sensor:** BME280 module.
* **Magnetometer:** QMC6310 module.
* **Inertial Measurement Unit (IMU):** QMI8658 module.
* **GPS Module:** Compatible with the `TinyGPS++.h` library, connected via serial.
* **Power Management Unit (PMU):** Integrated PMU on the microcontroller board (specific functionality depends on the board).
* **Display (Optional):** U8g2 compatible monochrome display (e.g., OLED 128x64 connected via I2C).
* **Wiring and Connectors:** Necessary wires, breadboard (for prototyping), and connectors to interface the components.
* **Power Supply:** Suitable power source for the microcontroller and sensors.

### 5.2. Software

* **Arduino IDE:** Integrated Development Environment for writing and uploading code to the microcontroller.
* **RadioLib:** Library for controlling the SX1262 LoRa transceiver ([https://github.com/jgromes/RadioLib](https://github.com/jgromes/RadioLib)).
* **LoRaBoards.h:** Board-specific definitions for pin assignments and potentially other hardware configurations (likely custom).
* **Wire.h:** Arduino library for I2C communication (required for BME280, QMC6310, and the U8g2 display).
* **SPI.h:** Arduino library for SPI communication (required for QMI8658 and potentially the LoRa module or SD card access within the QMI8658 library).
* **Adafruit_Sensor.h & Adafruit_BME280.h:** Libraries for interfacing with the BME280 sensor ([https://github.com/adafruit/Adafruit_BME280_Library](https://github.com/adafruit/Adafruit_BME280_Library)).
* **SensorQMC6310.hpp:** Custom library for the QMC6310 magnetometer (source code required).
* **SensorQMI8658.hpp:** Custom library for the QMI8658 IMU (source code required).
* **U8g2lib.h:** Library for controlling the U8g2 monochrome display ([https://github.com/olikraus/u8g2](https://github.com/olikraus/u8g2)).
* **math.h:** Standard C++ math library for mathematical functions.
* **RS-FEC.h:** Library for Reed-Solomon Forward Error Correction (source code or library file required).
* **TinyGPS++.h:** Library for parsing data from the GPS module ([http://arduinogps.jjoe.org/](http://arduinogps.jjoe.org/)).
* **stdio.h:** Standard C++ input/output library for functions like `sprintf`.

## 6. Development Plan

### 6.1. Phase 1: Setup and Initialization (Estimated Duration: 1 week)

* Set up the Arduino development environment and install all necessary libraries mentioned in Section 5.2.
* Configure the board-specific settings in `LoRaBoards.h` according to the chosen microcontroller board.
* Establish basic communication with the LoRa module using the RadioLib library, including setting the frequency and transmit power.
* Initialize the optional U8g2 display and display a basic welcome message.
* Verify the functionality of the serial communication for debugging and GPS data.

### 6.2. Phase 2: Sensor Integration (Estimated Duration: 2 weeks)

* Integrate and test the BME280 sensor using the Adafruit library to read temperature, pressure, humidity, and altitude.
* Integrate and test the QMC6310 magnetometer using the custom `SensorQMC6310.hpp` library to read magnetic field data in three axes. Implement heading calculation.
* Integrate and test the QMI8658 IMU using the custom `SensorQMI8658.hpp` library to read accelerometer, gyroscope, and temperature data. Configure the sensor parameters as defined in the code.
* Integrate and test the GPS module using the `TinyGPS++.h` library to parse location, altitude, date, time, and satellite information from the serial data stream.
* Integrate and test the PMU functionality (if available on the chosen board) to read battery voltage, charge percentage, charging status, VBUS voltage, and system voltage.

### 6.3. Phase 3: Telemetry Packet Construction (Estimated Duration: 1 week)

* Define the CCSDS Application Packet structure in the code, ensuring correct data types and sizes for each field.
* Implement the `buildTelemetryPacket()` function to read the latest sensor data and populate the CCSDS packet structure.
* Implement the logic to populate the CCSDS primary header fields (Version, Type, APID, Sequence Flags, Packet Sequence Count, Packet Length).
* Implement the CRC-16 checksum calculation function (`calculateCRC`) and integrate it into the `buildTelemetryPacket()` function to calculate the checksum over the packet data (excluding the CRC field itself).

### 6.4. Phase 4: Reed-Solomon FEC Implementation (Estimated Duration: 1 week)

* Integrate the `RS-FEC.h` library into the project.
* Instantiate the Reed-Solomon encoder with the specified parameters (RS(255, 223)).
* Modify the `buildTelemetryPacket()` function to encode the raw CCSDS packet data using the Reed-Solomon encoder and store the encoded data in the `encodedData` buffer.

### 6.5. Phase 5: LoRa Transmission (Estimated Duration: 1 week)

* Configure the LoRa radio parameters (carrier frequency, transmit power, bandwidth, spreading factor, coding rate, sync word, preamble length) using the RadioLib library as defined in the code.
* Implement the main loop logic to periodically trigger the transmission of the telemetry packet.
* Utilize the interrupt-driven transmission mechanism provided by RadioLib and the `setFlag` function to ensure non-blocking transmission.
* Increment the packet counter for each transmitted packet.

### 6.6. Phase 6: Display Implementation (Estimated Duration: 1 week)

* Implement the `updateDisplay()` function to cycle through different display screens at a defined interval.
* Implement the `drawDisplay()` function with a switch statement to display relevant sensor data and system status on each screen using the U8g2 library.
* Implement dedicated functions (`drawGPSDateTimeScreen`, `drawGPSLocationScreen`) to display GPS-specific information.

### 6.7. Phase 7: Testing and Debugging (Estimated Duration: Ongoing throughout the project)

* Conduct unit tests for individual sensor reading functions and the CRC calculation.
* Perform integration testing to ensure all components (sensors, packetization, FEC, LoRa transmission, display) work together correctly.
* Use serial output for debugging and monitoring sensor data and packet contents.
* Utilize appropriate tools (e.g., a LoRa receiver if available) to verify the transmitted data.

## 7. Timeline (Estimated)

* **Week 1:** Setup and Initialization
* **Week 2-3:** Sensor Integration
* **Week 4:** Telemetry Packet Construction
* **Week 5:** Reed-Solomon FEC Implementation
* **Week 6:** LoRa Transmission
* **Week 7:** Display Implementation
* **Ongoing:** Testing and Debugging

**Total Estimated Duration: 7 weeks** (This is a preliminary estimate and may be adjusted based on the complexity and challenges encountered during development.)

## 8. Resources

* **Hardware Components:** As listed in Section 5.1.
* **Software Libraries:** As listed in Section 5.2.
* **Development Tools:** Arduino IDE.
* **Documentation:** Datasheets for all hardware components, documentation for the RadioLib, U8g2, and TinyGPS++ libraries, documentation or source code for the custom sensor libraries (`SensorQMC6310.hpp`, `SensorQMI8658.hpp`), and documentation for the Reed-Solomon FEC library (`RS-FEC.h`).

## 9. Risk Management

| Risk ID | Risk Description                                      | Potential Impact                                                 | Mitigation Strategy                                                                                                   |
| :------ | :---------------------------------------------------- | :--------------------------------------------------------------- | :-------------------------------------------------------------------------------------------------------------------- |
| R01     | Failure to initialize one or more sensors.             | Loss of specific telemetry data. System functionality impacted.  | Implement robust error checking in the `setup()` function. Verify sensor wiring and I2C/SPI addresses.              |
| R02     | Issues with custom sensor libraries.                   | Difficulty in obtaining accurate sensor readings. Delays in development. | Thoroughly review and test the custom libraries. Consider alternative libraries or developing own if necessary. |
| R03     | Incorrect implementation of CCSDS packet structure.     | Telemetry data may not be interpreted correctly by a receiver.   | Carefully follow CCSDS standards. Implement thorough testing of the packet structure and data fields.                 |
| R04     | Errors in Reed-Solomon FEC implementation.            | Ineffective error correction, potentially leading to data loss. | Thoroughly test the FEC encoding process. Verify the correct usage of the `RS-FEC.h` library.                        |
| R05     | Problems with LoRa communication (transmission failures). | Inability to transmit telemetry data.                            | Verify LoRa module wiring and configuration. Test different LoRa parameters for optimal performance.                  |
| R06     | Bugs in the display implementation.                    | Incorrect or missing information displayed to the user.          | Test each display screen and functionality thoroughly.                                                              |
| R07     | Incompatibility between hardware components.           | System may not function as expected or components may be damaged. | Carefully review datasheets and ensure compatibility before connecting components.                                    |
| R08     | Power supply issues.                                  | System instability or failure to operate.                        | Ensure the power supply meets the requirements of all components.                                                   |

## 10. Quality Assurance

* **Code Reviews:** Regularly review the Arduino code for clarity, efficiency, and adherence to good coding practices.
* **Unit Testing:** Test individual functions and modules (e.g., sensor reading functions, CRC calculation) to ensure they operate correctly in isolation.
* **Integration Testing:** Test the interaction between different modules (e.g., sensor data acquisition and packetization, packetization and LoRa transmission) to ensure seamless data flow.
* **Functional Testing:** Verify that the telemetry transmitter meets all the functional requirements outlined in the project goal and scope.
* **Data Validation:** Ensure that the format and content of the transmitted CCSDS packets are correct and that the sensor data is accurate.

## 11. Metrics

* **Successful Sensor Initialization Rate:** Percentage of sensors that initialize correctly during system startup.
* **Telemetry Packet Transmission Rate:** Number of telemetry packets successfully transmitted per second or minute.
* **CRC Error Rate (if a receiver is available):** Percentage of received packets with CRC errors.
* **Display Functionality Coverage:** Percentage of planned display screens and information that are successfully implemented.
* **Code Complexity Metrics:** (Optional) Use static analysis tools to measure code complexity.

This project plan provides a high-level overview of the tasks, resources, and considerations for developing the spacecraft telemetry transmitter system. It will be reviewed and updated as the project progresses.
