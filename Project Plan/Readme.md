
# Project Plan: Simulated Voyager 1 CCSDS Telemetry System

**Version:** 1.1
**Date:** 2025-04-05
**Prepared By:** Tanay C

---

## 1. Project Overview & Executive Summary

This project aims to implement and validate a robust telemetry system simulating spacecraft data transmission using LoRa radio communication. The system consists of a Transmitter (Tx) unit, based on the provided `Transmit_Interrupt.ino` Arduino code, and a corresponding Receiver (Rx) unit to be developed. The Tx unit integrates multiple sensors (Environmental: BME280; Magnetic: QMC6310; Inertial/Temp: QMI8658; Positioning: GPS; Power: PMU) and formats the collected data into Consultative Committee for Space Data Systems (CCSDS) Application Packets (AP). Data integrity and reliability over the LoRa link are enhanced using Reed-Solomon Forward Error Correction (RS-FEC - specifically RS(255, 223)) and a CRC-16 checksum. The system operates on the 433 MHz ISM band using SX1262 LoRa transceivers. An optional U8g2 display on the Tx provides local status visualization. The Rx unit will be designed to receive the LoRa transmissions, perform RS-FEC decoding, validate data integrity via CRC, parse the CCSDS packet structure, and make the telemetry data available (e.g., via serial output for logging or further processing). The project emphasizes adherence to the specified protocols (CCSDS, RS-FEC) and thorough testing of all components and the integrated system. The context draws parallels to systems like Voyager 1 in terms of using standardized packet formats and error correction for reliable data transmission.

---

## 2. Project Goals & Objectives

* **Goal 1:** Develop a fully functional Transmitter (Tx) unit based on the provided `Transmit_Interrupt.ino` code, ensuring correct sensor integration, data acquisition, CCSDS packet formatting, RS-FEC encoding, and LoRa transmission.
    * *Objective 1.1:* Successfully initialize and read data periodically from BME280, QMC6310, QMI8658, GPS, and PMU sensors.
    * *Objective 1.2:* Correctly assemble sensor data and metadata into the defined `CCSDSPacket` structure, including primary header fields (Version, Type, Sec Hdr Flag, APID, Seq Flags, Seq Count, Packet Length), Spacecraft ID, mission time, sensor payloads, message string.
    * *Objective 1.3:* Implement and verify the CRC-16 (CCITT-FALSE) calculation over the packet data (excluding the CRC field itself).
    * *Objective 1.4:* Integrate and correctly utilize the `RS::ReedSolomon<223, 32>` library to encode the `CCSDSPacket` data (first 223 bytes) into a 255-byte encoded block.
    * *Objective 1.5:* Configure the SX1262 LoRa module using `RadioLib` with specified parameters (433.0 MHz, 20 dBm, 125.0 kHz BW, SF 12, CR 6, SyncWord 0x35, Preamble 16, HW CRC off) and successfully transmit the RS-encoded data.
    * *Objective 1.6:* Implement and verify the functionality of the U8g2 display for visualizing sensor data, GPS status, LoRa parameters, and PMU status across multiple cycling screens.
* **Goal 2:** Design, develop, and validate a corresponding Receiver (Rx) unit capable of reliably receiving, decoding, and parsing the transmitted telemetry data.
    * *Objective 2.1:* Configure an SX1262 LoRa module on the Rx hardware to match the Tx parameters and successfully receive LoRa packets.
    * *Objective 2.2:* Implement and utilize the `RS::ReedSolomon<223, 32>` library to decode the received 255-byte blocks, correcting potential errors introduced during transmission (up to the capability of the code - 16 byte errors).
    * *Objective 2.3:* Extract the decoded `CCSDSPacket` data (223 bytes).
    * *Objective 2.4:* Implement and verify the CRC-16 (CCITT-FALSE) calculation on the received packet data (excluding the CRC field) and compare it against the received CRC value for integrity checking.
    * *Objective 2.5:* Implement logic to parse the validated `CCSDSPacket` structure, extracting all header fields and sensor data payloads.
    * *Objective 2.6:* Provide a mechanism to output/log the received and parsed telemetry data (e.g., formatted serial output).
* **Goal 3:** Integrate and test the complete Tx-Rx system to ensure reliable end-to-end communication in a lab environment.
    * *Objective 3.1:* Demonstrate successful transmission and reception of telemetry packets over a short range (e.g., within the same room).
    * *Objective 3.2:* Verify the accuracy of the received sensor data against the transmitted data.
    * *Objective 3.3:* Test the system's robustness by simulating errors (if possible) or observing performance in varying RF conditions (within lab limits) and verifying RS-FEC/CRC effectiveness.
* **Goal 4:** Produce comprehensive documentation for the system.
    * *Objective 4.1:* Fully comment and document the Tx and Rx source code.
    * *Objective 4.2:* Create a technical document detailing the system architecture, hardware components, software design, communication protocol (CCSDS structure, RS-FEC usage), and testing procedures/results.
    * *Objective 4.3:* Develop a basic user guide for setting up and operating the Tx/Rx system.

---

## 3. Project Scope

### 3.1. Inclusions

* Refinement and validation of the provided `Transmit_Interrupt.ino` code.
* Hardware setup and configuration for both Tx and Rx units using Arduino-compatible platforms and SX1262 modules.
* Integration and testing of all specified sensors (BME280, QMC6310, QMI8658, GPS, PMU) on the Tx unit.
* Implementation and verification of CCSDS AP formatting on Tx.
* Implementation and verification of RS-FEC (255, 223) encoding on Tx and decoding on Rx.
* Implementation and verification of CRC-16 (CCITT-FALSE) generation on Tx and validation on Rx.
* Development of the complete software stack for the Rx unit (LoRa reception, RS decoding, CRC check, CCSDS parsing, data output).
* Configuration and testing of the LoRa communication link using `RadioLib`.
* Implementation and testing of the U8g2 display functionality on the Tx unit.
* Unit testing of individual hardware and software modules (sensors, LoRa, CCSDS, RS-FEC, CRC, display).
* Integration testing of the complete Tx unit.
* Integration testing of the complete Rx unit.
* End-to-end system testing in a controlled lab environment.
* Development of project documentation (code comments, technical specification, user guide).
* Configuration management for source code and documentation.

### 3.2. Exclusions

* Development of custom sensor hardware or drivers beyond integrating existing libraries specified or implied (`Adafruit_BME280`, `SensorQMC6310`, `SensorQMI8658`, `TinyGPS++`). Requires confirmation that `SensorQMC6310.hpp`, `SensorQMI8658.hpp`, `RS-FEC.h` and `LoRaBoards.h` are available and functional.
* Environmental hardening or testing (e.g., temperature extremes, vibration, vacuum).
* Long-range field testing beyond basic lab/office range verification.
* Development of a graphical user interface (GUI) for the receiver beyond serial output or a basic display (if added to Rx).
* Integration with actual Deep Space Network (DSN) or other professional ground station systems.
* Formal security analysis or implementation of encryption beyond the inherent properties of LoRa.
* Mass production optimization or manufacturing plan.
* Power consumption optimization beyond basic PMU monitoring.
* Development or procurement of specific antenna systems beyond standard LoRa module antennas.

### 3.3. Key Deliverables

* Functional Transmitter (Tx) Hardware Unit.
* Functional Receiver (Rx) Hardware Unit.
* Validated and documented Transmitter Firmware (`Transmit_Interrupt.ino` and dependencies).
* Validated and documented Receiver Firmware.
* Test Plan and Test Results Documentation.
* System Technical Specification Document.
* Basic User Guide.
* Final Project Report including lessons learned.

---

## 4. Project Management Approach & Methodology

A **Hybrid Approach** will be adopted, combining phases typical of a Waterfall or V-Model for structure with iterative development cycles within key phases (particularly Tx/Rx Development and Integration).

* **Phased Structure:** Clear phases for planning, hardware setup, development, integration, testing, and closure.
* **Iterative Development:** Within Phase 3 (Tx Dev) and Phase 4 (Rx Dev), development will proceed iteratively module by module (e.g., Sensor -> LoRa -> CCSDS -> RS-FEC -> Display). Each module will undergo unit testing before integration.
* **V-Model Emphasis:** Strong emphasis on testing at each stage. Unit tests verify individual components, integration tests verify module combinations, and system tests verify the end-to-end functionality against requirements.
* **Regular Check-ins:** Daily or frequent short meetings (stand-ups) during active development phases. Weekly progress reviews.
* **Configuration Management:** Use of Git or similar VCS for all source code and documentation.

---

## 5. Work Breakdown Structure (WBS) / Project Phases & Tasks

### 5.1. Phase 1: Project Initiation & Detailed Planning (Estimate: 1-2 weeks)

* 1.1. Define final project requirements & constraints.
* 1.2. Confirm availability & suitability of specified libraries (`SensorQMC6310`, `SensorQMI8658`, `RS-FEC.h`, `LoRaBoards.h`). Resolve any potential library conflicts or dependencies.
* 1.3. Finalize hardware selection (Microcontroller board, LoRa module type (confirm SX1262 compatibility with `LoRaBoards.h`), Sensors, Display, Power Source, Rx components).
* 1.4. Develop detailed WBS (this document).
* 1.5. Develop detailed schedule & resource allocation plan.
* 1.6. Develop Risk Management Plan (Section 8).
* 1.7. Develop Communication Plan (Section 9).
* 1.8. Develop Quality Management Plan (Section 10).
* 1.9. Project Kick-off Meeting.

### 5.2. Phase 2: Hardware Acquisition, Setup & Bring-up (Estimate: 1-3 weeks, depends on procurement)

* 2.1. Procure all required hardware components for Tx and Rx.
* 2.2. Assemble Tx Hardware Unit (MCU, LoRa, Sensors, Display, Power).
* 2.3. Assemble Rx Hardware Unit (MCU, LoRa, Power, PC Interface).
* 2.4. Install required development tools (Arduino IDE/PlatformIO, Libraries, Drivers).
* 2.5. Perform basic hardware bring-up tests (Power-on, MCU check, basic communication with components like I2C/SPI devices, Serial output).
* 2.6. Configure board-specific definitions (`LoRaBoards.h`). Verify pin assignments (RADIO_CS_PIN, DIO1, RST, BUSY, I2C, SPI, GPS Serial, IMU_CS).

### 5.3. Phase 3: Transmitter (Tx) Module Development & Unit Testing (Estimate: 4-6 weeks, iterative)

* 3.1. **Sensor Module Integration & Test:**
    * 3.1.1. BME280: Integrate `Adafruit_BME280` library, write test sketch to read Temp, Pressure, Humidity, Altitude. Verify readings.
    * 3.1.2. QMC6310: Integrate `SensorQMC6310.hpp`, configure (Continuous, 8G, 200Hz, OSR1, DSR1), write test sketch to read Mag X/Y/Z. Verify readings and heading calculation (note declination dependency - currently -0.041 rad for Galway).
    * 3.1.3. QMI8658: Integrate `SensorQMI8658.hpp`, configure Accel (4G, 1000Hz, LPF0) & Gyro (64DPS, 896.8Hz, LPF3), write test sketch to read Accel X/Y/Z, Gyro X/Y/Z, Temp. Verify readings.
    * 3.1.4. GPS: Integrate `TinyGPS++.h`, connect GPS module (`SerialGPS` @ 9600 baud), write test sketch to parse NMEA sentences, read Lat, Lon, Alt, Date, Time, Sats, HDOP. Test indoors/outdoors for fix acquisition.
    * 3.1.5. PMU: Integrate PMU access (assuming board-specific `PMU` object), write test sketch to read Batt Voltage/Percent, Charging status, VBUS/System Voltage. Verify readings.
* 3.2. **LoRa Tx Module Integration & Test:**
    * 3.2.1. Integrate `RadioLib` for SX1262.
    * 3.2.2. Configure LoRa parameters (Freq, Power, BW, SF, CR, SyncWord, Preamble, CurrentLimit, HW CRC Off). Handle TCXO if applicable.
    * 3.2.3. Implement basic packet transmission (`startTransmit`) using a simple payload.
    * 3.2.4. Implement `setFlag` ISR callback and `transmittedFlag` logic. Verify transmission completion detection.
* 3.3. **CCSDS Packet Module Implementation & Test:**
    * 3.3.1. Implement `CCSDSPacket` struct with `#pragma pack(push, 1)`.
    * 3.3.2. Implement `buildTelemetryPacket` function.
    * 3.3.3. Verify correct assignment of header fields (Version, Type, Sec Hdr, APID, Seq Flags, Seq Count, Length).
    * 3.3.4. Verify correct population of payload fields from sensor variables and metadata (Timestamp, Spacecraft ID). Handle invalid GPS data cases.
    * 3.3.5. Test `sprintf` usage for the message field.
* 3.4. **CRC-16 Module Implementation & Test:**
    * 3.4.1. Implement `calculateCRC` function (CCITT-FALSE).
    * 3.4.2. Unit test `calculateCRC` with known data patterns and expected CRC values.
    * 3.4.3. Integrate CRC calculation into `buildTelemetryPacket`, ensuring it covers the correct data range (`sizeof(CCSDSPacket) - 2`).
* 3.5. **RS-FEC Encoder Module Integration & Test:**
    * 3.5.1. Integrate `RS-FEC.h` library (`RS::ReedSolomon<223, 32> rs`).
    * 3.5.2. Implement RS encoding step in `buildTelemetryPacket`, copying the first `rsMsgLen` (223) bytes of the packet to the `encodedData` buffer and calling `rs.Encode`.
    * 3.5.3. Verify the size of the `CCSDSPacket` structure (must be >= 223 bytes for this encoding scheme). **Assumption:** Only the first 223 bytes are encoded as per `rsMsgLen`. The calculated CRC should ideally be within these first 223 bytes.
    * 3.5.4. Verify the final encoded data size is `rsMsgLen + ECC_LENGTH` (255 bytes).
* 3.6. **Display Module Integration & Test:**
    * 3.6.1. Integrate `U8g2lib.h`. Initialize display (`u8g2->begin()`).
    * 3.6.2. Implement `updateDisplay` and `drawDisplay` functions.
    * 3.6.3. Implement all display screens (Cases 0-10) as defined in the code.
    * 3.6.4. Implement `drawGPSDateTimeScreen` and `drawGPSLocationScreen`.
    * 3.6.5. Test screen cycling logic and display update rate. Verify data formatting (`printf`).
* 3.7. **Full Tx Integration:**
    * 3.7.1. Combine all modules into the main `Transmit_Interrupt.ino` structure.
    * 3.7.2. Implement main loop logic (`setup()`, `loop()`).
    * 3.7.3. Implement `updateSensorReadings` logic with `sensorUpdateInterval`.
    * 3.7.4. Verify correct sequencing: update sensors -> build packet (incl. CRC, RS) -> transmit -> wait for flag -> repeat.
    * 3.7.5. Implement `displayFatalError` function.

### 5.4. Phase 4: Receiver (Rx) Module Development & Unit Testing (Estimate: 3-5 weeks, iterative)

* 4.1. **LoRa Rx Module Integration & Test:**
    * 4.1.1. Integrate `RadioLib` for SX1262 on the Rx platform.
    * 4.1.2. Configure LoRa parameters identical to Tx (Freq, BW, SF, CR, SyncWord, Preamble, HW CRC Off).
    * 4.1.3. Implement basic packet reception (`startReceive`). Use DIO1 for RxDone interrupt if possible/required by `RadioLib` setup.
    * 4.1.4. Write test code to receive simple packets sent from Tx (initially without RS/CCSDS). Verify payload reception.
* 4.2. **RS-FEC Decoder Module Integration & Test:**
    * 4.2.1. Integrate `RS-FEC.h` library (`RS::ReedSolomon<223, 32> rs`) on Rx.
    * 4.2.2. Implement logic to receive the 255-byte encoded block.
    * 4.2.3. Implement call to `rs.Decode` on the received buffer.
    * 4.2.4. Test decoding with known encoded data (potentially generated offline or by Tx). Test error correction capability (e.g., manually introduce byte errors in received buffer before decoding). Check return value of `Decode` for success/failure.
* 4.3. **CRC-16 Validation Module Implementation & Test:**
    * 4.3.1. Implement `calculateCRC` function on Rx (identical to Tx).
    * 4.3.2. Extract the received `CCSDSPacket` data (first 223 bytes) after successful RS decoding.
    * 4.3.3. Extract the transmitted CRC value from the *end* of the 223-byte decoded message segment (assuming it fits there).
    * 4.3.4. Recalculate CRC on the received packet data bytes up to *before* the CRC field itself (`sizeof(CCSDSPacket) - 2`).
    * 4.3.5. Compare calculated CRC with received CRC. Implement logic to flag packets with CRC mismatch.
* 4.4. **CCSDS Parser Module Implementation & Test:**
    * 4.4.1. Define the `CCSDSPacket` struct on Rx (identical to Tx).
    * 4.4.2. After successful RS decode and CRC validation, cast the first 223 bytes of the decoded buffer to the `CCSDSPacket` struct pointer.
    * 4.4.3. Implement functions to extract and interpret header fields (APID, Seq Count, etc.).
    * 4.4.4. Implement functions to extract all sensor payload data points.
* 4.5. **Data Output/Logging Module Implementation & Test:**
    * 4.5.1. Implement formatted serial output of received and parsed telemetry data (Header info, sensor values). Use Baud 115200.
    * 4.5.2. Indicate packet status (Received OK, CRC Error, RS Decode Fail).
    * 4.5.3. (Optional) Implement logging to SD card if hardware supports it.
    * 4.5.4. (Optional) Add a basic display to the Rx unit.
* 4.6. **Full Rx Integration:**
    * 4.6.1. Combine all Rx modules into a main Receiver sketch (`Transmit_Interrupt.ino`).
    * 4.6.2. Implement main loop logic for continuous reception, decoding, validation, parsing, and output.
    * 4.6.3. Test overall Rx functionality and error handling.

### 5.5. Phase 5: System Integration & End-to-End Testing (Estimate: 2-3 weeks)

* 5.1. **Basic Connectivity Test:** Power on Tx and Rx. Verify Rx receives packets and logs basic reception status.
* 5.2. **End-to-End Data Verification:**
    * 5.2.1. Compare sensor values displayed on Tx screen (or serial debug) with values logged by Rx after full decoding/parsing. Verify accuracy for all fields.
    * 5.2.2. Verify packet sequence count increments correctly on both Tx and Rx logs.
    * 5.2.3. Verify timestamp/mission time consistency.
* 5.3. **RS-FEC & CRC Robustness Test:**
    * 5.3.1. Introduce intentional RF interference (if safe and possible in lab) or increase distance between Tx/Rx to induce errors.
    * 5.3.2. Monitor Rx logs for RS decode successes (potentially with corrections) and CRC failures. Verify correct behavior based on error levels.
* 5.4. **Functional Scenario Testing:**
    * 5.4.1. Test GPS data transmission/reception (requires GPS fix).
    * 5.4.2. Test PMU data reporting (e.g., connect/disconnect power source if applicable to see charging status change).
    * 5.4.3. Test sensor failure handling on Tx (verify `displayFatalError` works) and observe impact on transmission.
* 5.5. **Basic Range Test:** Determine approximate reliable communication range within the lab/office environment.
* 5.6. **Soak Test:** Run the system continuously for an extended period (e.g., 8-24 hours) to check for stability, memory leaks, or other issues.

### 5.6. Phase 6: Documentation, Refinement & Deployment Preparation (Estimate: 1-2 weeks)

* 6.1. Finalize code commenting and ensure code quality standards are met.
* 6.2. Write/update System Technical Specification Document.
* 6.3. Write/update User Guide.
* 6.4. Compile Test Results documentation.
* 6.5. Address any bugs or issues identified during system testing.
* 6.6. Prepare final firmware binaries for Tx and Rx.
* 6.7. Code freeze and final configuration management check-in.

### 5.7. Phase 7: Project Closure & Handover (Estimate: 1 week)

* 7.1. Final Project Review Meeting.
* 7.2. Obtain formal acceptance of deliverables.
* 7.3. Handover hardware, software, and documentation.
* 7.4. Archive project materials.
* 7.5. Conduct Lessons Learned session.
* 7.6. Conclude project activities.

---

## 6. High-Level Schedule & Milestones

* **(W0) Project Start**
* **(W1-2) Milestone 1:** Planning Complete, Hardware Specified, Initial Risks Assessed.
* **(W3-5) Milestone 2:** Hardware Acquired, Assembled, Basic Bring-up Tests Passed.
* **(W9-11) Milestone 3:** Transmitter Firmware Complete & Unit Tested. Tx Integration Complete.
* **(W14-16) Milestone 4:** Receiver Firmware Complete & Unit Tested. Rx Integration Complete.
* **(W16-18) Milestone 5:** System Integration Complete. End-to-End Testing Passed.
* **(W19-20) Milestone 6:** Documentation Complete. Final Firmware Binaries Ready.
* **(W21) Project End:** Project Closure, Handover Complete.

*(Note: Durations are estimates and heavily dependent on resource availability, hardware delivery times, and complexity encountered, especially with library integration and RF testing.)*

---

## 7. Resource Requirements

### 7.1. Hardware

* **Tx Unit:**
    * Arduino-compatible Microcontroller (specific board TBD, must match `LoRaBoards.h` & library reqs)
    * SX1262 LoRa Module (with antenna)
    * Adafruit BME280 Sensor
    * QMC6310 Sensor Module
    * QMI8658 Sensor Module
    * GPS Module (compatible with `TinyGPS++`, e.g., u-blox) + Antenna
    * PMU (likely integrated on the MCU board, e.g., AXP192/202)
    * U8g2 compatible Display (e.g., 128x64 OLED SSD1306 I2C)
    * Power Source (Battery + USB/DC Adapter)
    * Wiring, connectors, enclosure (optional)
* **Rx Unit:**
    * Arduino-compatible Microcontroller
    * SX1262 LoRa Module (with antenna)
    * Power Source
    * USB Cable for connection to PC
    * (Optional) Display, SD Card module
* **Development/Test Equipment:**
    * Development PC
    * USB Cables
    * Multimeter
    * Oscilloscope/Logic Analyzer (Recommended for debugging I2C/SPI/Serial)
    * Bench Power Supply
    * (Optional) RF Spectrum Analyzer (for advanced LoRa debugging)

### 7.2. Software & Tools

* Arduino IDE or PlatformIO IDE
* Required Libraries: `RadioLib`, `Adafruit_BME280` (+ dependencies like `Adafruit_Sensor`), `SensorQMC6310`, `SensorQMI8658`, `U8g2lib`, `TinyGPS++`, `RS-FEC` library.
* Serial Monitor software (e.g., Termite, PuTTY, IDE built-in)
* Git Client & Repository (e.g., GitHub, GitLab)
* Project Management Software (Optional)
* Documentation Tools (e.g., MS Office, Google Docs, Markdown editor)

### 7.3. Personnel Effort Areas

* Project Management & Coordination
* Embedded Software Engineering (Tx/Rx Firmware)
* Hardware Engineering & Technician Support (Assembly, Bring-up, Troubleshooting)
* Test Engineering (Planning, Execution, Reporting)

---

## 8. Risk Management Plan

| Risk ID | Risk Description                                                      | Likelihood (1-5) | Impact (1-5) | Risk Score | Mitigation Strategy                                                                                                                                | Responsibility Area |
| :------ | :-------------------------------------------------------------------- | :--------------- | :----------- | :--------- | :------------------------------------------------------------------------------------------------------------------------------------------------- | :------------------ |
| R01     | Hardware component failure or unavailability                          | 3                | 4            | 12         | Identify alternative suppliers/components early. Order spares for critical parts (MCU, LoRa). Perform thorough bring-up tests.                     | Hardware Eng.       |
| R02     | Library incompatibility or bugs (`RadioLib`, Sensors, RS-FEC)           | 3                | 4            | 12         | Verify library compatibility with target MCU & each other early. Check library issue trackers. Allocate time for debugging/patching/alternative finding. | Software Eng.       |
| R03     | Issues with custom/provided libraries (`SensorQMC/QMI`, `RS-FEC`, `LoRaBoards.h`) | 4                | 4            | 16         | **PRIORITY:** Validate these libraries thoroughly in Phase 1/2. If non-functional, requires significant re-planning/development effort. Seek source/support. | Software Eng./PM    |
| R04     | CCSDS packet structure definition error/ambiguity                     | 2                | 3            | 6          | Cross-reference `CCSDSPacket` struct with CCSDS standards (Blue Book). Verify field packing and byte order (endianness if crossing platforms).        | Software Eng.       |
| R05     | RS-FEC implementation error (encoding/decoding)                       | 3                | 5            | 15         | Use reference implementation/test vectors for RS library. Test rigorously with induced errors. Verify correct message/parity lengths are used.     | Software Eng.       |
| R06     | LoRa communication range/reliability issues                           | 3                | 3            | 9          | Optimize LoRa parameters (SF, Power - within limits). Ensure good antenna placement. Test in target lab environment early. Shielding/Interference check. | Hardware/Software Eng.|
| R07     | GPS signal acquisition difficulties (indoor testing)                  | 4                | 2            | 8          | Test GPS outdoors or near window. Implement robust handling for invalid GPS data in Tx/Rx logic. Use GPS simulator if available.                 | Software/Test Eng.  |
| R08     | Integration challenges between modules                                | 3                | 4            | 12         | Adopt iterative integration. Use stubs/mocks for unit testing. Perform integration tests frequently. Allocate buffer time for debugging.         | Software Eng.       |
| R09     | Power Management Unit (PMU) access issues                             | 2                | 3            | 6          | Verify PMU type and correct library/API usage for the specific board early.                                                                        | Hardware/Software Eng.|
| R10     | Scope Creep                                                           | 2                | 3            | 6          | Strictly adhere to defined scope (Section 3). Implement formal change request process.                                                             | Project Management  |

*(Likelihood: 1=Very Low, 5=Very High; Impact: 1=Very Low, 5=Very High; Risk Score = L \* I)*

---

## 9. Communication Plan

* **Team Meetings:**
    * Daily Stand-ups (15 mins) during active development/integration phases.
    * Weekly Progress Review (1 hour). Review progress against plan, discuss issues, plan next steps.
* **Reporting:**
    * Regular status updates on progress, issues, and risks.
    * Test reports after major test cycles.
* **Tools:**
    * Shared Task Board (e.g., Trello, Jira, Asana) for task tracking.
    * Version Control System (Git) for code and documentation commit history.
    * Email/Team Chat (e.g., Slack, Teams) for ad-hoc communication.
    * Shared Drive/Wiki (e.g., Google Drive, Confluence) for documentation storage.

---

## 10. Quality Management Plan

* **Code Quality:**
    * Adherence to defined coding style guidelines (e.g., Google C++ Style Guide adapted for Arduino).
    * Mandatory code reviews for all major feature implementations and bug fixes.
    * Extensive commenting within the source code.
* **Testing:**
    * Unit Tests for critical modules (CRC, RS-FEC, CCSDS parsing, individual sensor reads).
    * Integration Tests for combined modules (Sensor + Packetizing, LoRa Tx/Rx, RS + CRC).
    * System Tests for end-to-end functionality verification against requirements.
    * Use of a Test Plan detailing test cases, procedures, and expected results. Document all test executions and outcomes.
* **Standards Adherence:**
    * Verification of CCSDS packet structure against relevant standards (CCSDS 133.0-B-1 Packet Telemetry Service).
    * Verification of RS(255, 223) implementation parameters.
    * Verification of CRC-16-CCITT-FALSE algorithm.
* **Documentation:**
    * Ensure all deliverables (firmware, documents) are reviewed and approved before final release.
    * Maintain traceability between requirements, design, code, and tests.

---

## 11. Budget Considerations (Placeholder)

* **Hardware Costs:** Cost of MCUs, LoRa modules, all sensors, displays, GPS, power supplies, wiring, enclosures. Include spares.
* **Software Costs:** Primarily associated with development tools if commercial versions are used (unlikely for Arduino-based). Potential library licensing costs (check licenses for all dependencies).
* **Personnel Effort Costs:** Estimated effort (person-weeks/months) for required skill areas (SW Eng, HW Eng, Test Eng, PM) multiplied by applicable rates. This is typically the largest cost component.
* **Contingency:** Recommended budget reserve (e.g., 10-15%) to cover unforeseen issues or risks.

*(Detailed budget requires specific component pricing and personnel rates/effort allocation.)*

---

## 12. Success Criteria & Acceptance

The project will be considered successful upon meeting the following criteria:

1.  **Functional Tx Unit:** The transmitter unit successfully initializes all sensors, reads data, formats it into CCSDS packets, calculates CRC, encodes with RS(255, 223), and transmits via LoRa using the specified parameters. The display cycles through screens showing relevant data.
2.  **Functional Rx Unit:** The receiver unit successfully receives LoRa packets, decodes RS(255, 223) data (correcting errors where possible), validates CRC, parses the CCSDS packet structure, and outputs the correct telemetry data via the defined mechanism (e.g., Serial).
3.  **End-to-End Communication:** The integrated system demonstrates reliable transmission and reception of telemetry packets in a lab environment, with received data accurately matching transmitted data (within sensor tolerances). RS-FEC and CRC mechanisms demonstrably improve robustness or correctly flag errors.
4.  **Deliverables Complete:** All key deliverables (Hardware Units, Firmware, Documentation, Test Results) as listed in Section 3.3 are completed, reviewed, and approved.
5.  **Scope Adherence:** The project is completed within the defined scope (Section 3).
6.  **Formal Acceptance:** Formal sign-off based on a final demonstration and review of deliverables.

---
