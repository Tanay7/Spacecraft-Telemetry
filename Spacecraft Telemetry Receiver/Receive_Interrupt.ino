

/**
 * @file TelemetryRx.ino
 * @brief Spacecraft Telemetry Receiver for LoRa Communication with Sensor Data
 *
 * This code implements a telemetry receiver for a simulated spacecraft, receiving sensor data
 * transmitted over a LoRa radio link. The telemetry data is expected to be structured in CCSDS
 * (Consultative Committee for Space Data Systems) packets and encoded with Reed-Solomon Forward
 * Error Correction (RS-FEC).
 *
 * The receiver demodulates LoRa signals, decodes Reed-Solomon FEC, verifies CRC checksums,
 * and processes the received CCSDS telemetry packets. It monitors signal quality (RSSI and SNR),
 * counts valid packets and various error types (CRC errors, RS-FEC errors, data range errors),
 * and displays telemetry data and system status on a U8g2 display.
 *
 * Telemetry parameters received and displayed include:
 *   - Environmental data: Temperature, pressure, humidity, altitude from BME280 (received from transmitter).
 *   - Magnetic field data: X, Y, Z axis readings from QMC6310 magnetometer (received from transmitter).
 *   - Inertial data: Accelerometer (X, Y, Z), gyroscope (X, Y, Z), and temperature from QMI8658 IMU (received from transmitter).
 *   - GPS data: Latitude, longitude, altitude, date, time, satellites in view, HDOP (Horizontal Dilution of Precision) (received from transmitter).
 *   - Power Management Unit data: Battery voltage, battery percentage, charging status, VBUS voltage, system voltage (received from transmitter).
 *   - Packet sequence count and a message string for basic packet identification (received from transmitter).
 *   - Received Signal Strength Indication (RSSI) and Signal-to-Noise Ratio (SNR) of the LoRa signal.
 *   - Packet reception statistics: Valid packets, CRC errors, Reed-Solomon errors, data range errors.
 *
 * The code is designed to be compatible with SX1262 LoRa transceivers and utilizes RadioLib library
 * for LoRa demodulation and control. A U8g2 display is used to visualize received telemetry data
 * and system status. Serial output is used for detailed packet logging and debugging.
 *
 * Error handling includes CRC checksum verification, Reed-Solomon error correction capability
 * monitoring, and data range validation to ensure data integrity. A fatal error display routine
 * is included for critical system failures.
 *
 * Libraries Used:
 *   - RadioLib: For LoRa radio communication (https://github.com/jgromes/RadioLib)
 *   - LoRaBoards.h: Board-specific definitions for RadioLib (custom, likely board-dependent)
 *   - Wire.h: For I2C communication (required for U8g2 display)
 *   - SPI.h: For SPI communication (required for RadioLib might use SPI)
 *   - U8g2lib.h: For U8g2 monochrome display library (https://github.com/olikraus/u8g2)
 *   - RS-FEC.h: For Reed-Solomon Forward Error Correction (likely custom or from a library, details needed for URL)
 *   - stdio.h: For standard input/output functions (e.g., `sprintf`, `memset`)
 *
 * Target Platform: Arduino-compatible microcontroller (specific board defined in LoRaBoards.h)
 * LoRa Frequency: 433.0 MHz (matches transmitter configuration)
 *
 * CCSDS Packet Structure Notes:
 *   - Expects CCSDS Application Packet (AP) structure as defined in TelemetryTx.ino.
 *   - APID, Version, Type, Secondary Header Flag, and Sequence Flags are assumed to match
 *     the transmitter's configuration.
 *   - Packet Length Field is used to determine the size of the received packet data.
 *
 * Reed-Solomon FEC Notes:
 *   - RS(255, 223) decoding is performed to correct up to 16 byte errors per block, matching
 *     the RS(255, 223) encoding used in the transmitter (TelemetryTx.ino).
 *   - Error correction statistics (RS errors) are tracked to monitor link quality.
 *
 * Signal Quality Monitoring:
 *   - Received Signal Strength Indication (RSSI) is monitored to assess signal strength.
 *   - Signal-to-Noise Ratio (SNR) is monitored to assess signal quality relative to noise.
 *   - An RSSI threshold is defined to filter out packets received with very weak signals.
 */
#include <RadioLib.h>     // RadioLib library for LoRa communication
#include "LoRaBoards.h"     // Board-specific definitions for RadioLib and pin assignments
#include <RS-FEC.h>       // Reed-Solomon Forward Error Correction library
#include <U8g2lib.h>      // U8g2 library for monochrome displays
#ifdef USING_U8G2
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); // U8g2 display instance for SH1106 OLED
#endif
#include <stdio.h>        // Standard input/output library for sprintf and memset
// --- LoRa Radio Configuration ---
#define CARRIER_FREQ 433.0       // LoRa carrier frequency (MHz) - MUST MATCH TRANSMITTER
#define BANDWIDTH 125.0          // LoRa bandwidth (kHz) - MUST MATCH TRANSMITTER
#define SPREADING_FACTOR 12      // LoRa spreading factor (7-12) - MUST MATCH TRANSMITTER
#define CODING_RATE 6          // LoRa coding rate (5-8) - MUST MATCH TRANSMITTER
#define SYNC_WORD 0x35           // LoRa sync word (network ID) - MUST MATCH TRANSMITTER
#define TX_POWER 10              // LoRa transmit power (dBm) - NOT USED FOR RECEIVER, kept for consistency
#if defined(USING_SX1262)
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, // SX1262 radio module instance using RadioLib
                            RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif
// --- Reed-Solomon FEC Configuration ---
const int rsMsgLen = 223;          // Length of Reed-Solomon message (data bytes) - MUST MATCH TRANSMITTER
const uint8_t ECC_LENGTH = 32;     // Length of Reed-Solomon error correction code (parity bytes) - MUST MATCH TRANSMITTER
uint8_t encodedData[rsMsgLen + ECC_LENGTH]; // Buffer for received Reed-Solomon encoded data
RS::ReedSolomon<rsMsgLen, ECC_LENGTH> rs; // Reed-Solomon decoder instance
// --- CCSDS Packet Structure Definition ---
#pragma pack(push, 1) // Push current alignment and set to 1-byte alignment for CCSDS packet struct
struct CCSDSPacket {
  uint16_t version_type_apid;    // 2 bytes: Version, Type, APID (Application Process Identifier)
  uint16_t sequence_flags_count; // 2 bytes: Sequence Flags, Packet Sequence Count
  uint16_t packet_length;        // 2 bytes: Packet Length (total packet length - 7)
  char spacecraft_id[12];       // 12 bytes: Spacecraft identifier string
  uint32_t mission_time;         // 4 bytes: Mission time (milliseconds since epoch)
  float temp_bme;               // 4 bytes: Temperature from BME280 sensor (°C)
  float pres_bme;               // 4 bytes: Pressure from BME280 sensor (hPa)
  float hum_bme;                // 4 bytes: Humidity from BME280 sensor (%)
  float alt_bme;                // 4 bytes: Altitude from BME280 sensor (meters)
  float magX;                   // 4 bytes: Magnetic field X-axis from QMC6310 (uT)
  float magY;                   // 4 bytes: Magnetic field Y-axis from QMC6310 (uT)
  float magZ;                   // 4 bytes: Magnetic field Z-axis from QMC6310 (uT)
  float accX;                   // 4 bytes: Acceleration X-axis from QMI8658 (g)
  float accY;                   // 4 bytes: Acceleration Y-axis from QMI8658 (g)
  float accZ;                   // 4 bytes: Acceleration Z-axis from QMI8658 (g)
  float gyrX;                   // 4 bytes: Gyroscopic rate X-axis from QMI8658 (dps)
  float gyrY;                   // 4 bytes: Gyroscopic rate Y-axis from QMI8658 (dps)
  float gyrZ;                   // 4 bytes: Gyroscopic rate Z-axis from QMI8658 (dps)
  float temp_qmi;               // 4 bytes: Temperature from QMI8658 sensor (°C)
  double gps_lat;               // 8 bytes: GPS latitude (degrees)
  double gps_lon;               // 8 bytes: GPS longitude (degrees)
  float gps_alt;                // 4 bytes: GPS altitude (meters)
  uint8_t gps_date_day;         // 1 byte: GPS day of month (1-31)
  uint8_t gps_date_month;       // 1 byte: GPS month (1-12)
  uint16_t gps_date_year;        // 2 bytes: GPS year (e.g., 2025)
  uint8_t gps_time_hour;        // 1 byte: GPS hour (0-23)
  uint8_t gps_time_minute;      // 1 byte: GPS minute (0-59)
  uint8_t gps_time_second;      // 1 byte: GPS second (0-59)
  uint8_t gps_satellites;       // 1 byte: Number of GPS satellites in view
  float gps_hdop;               // 4 bytes: GPS HDOP (Horizontal Dilution of Precision)
  float battVoltage;            // 4 bytes: Battery voltage (V)
  uint8_t battPercent;          // 1 byte: Battery percentage (%)
  bool isCharging;            // 1 byte: Battery charging status (true/false)
  float vbusVoltage;            // 4 bytes: VBUS voltage (V)
  float systemVoltage;          // 4 bytes: System voltage (V)
  char message[16];             // 16 bytes: Message string for packet identification
  uint16_t crc;                 // 2 bytes: CRC-16 checksum (CCITT-FALSE)
};
#pragma pack(pop) // Restore default alignment
// --- Global Variables ---
static volatile bool receivedFlag = false; // Flag to indicate packet reception in ISR
CCSDSPacket rxPacket;                      // Structure to hold received telemetry packet data
uint16_t validPackets = 0;                // Counter for validly received packets
uint16_t crcErrors = 0;                   // Counter for CRC checksum errors
uint16_t rsErrors = 0;                    // Counter for Reed-Solomon decoding errors
uint16_t rangeErrors = 0;                 // Counter for data range validation errors
float rssi = -120.0;                      // Received Signal Strength Indication (dBm), initialized to a low value
float snr = -20.0;                       // Signal-to-Noise Ratio (dB), initialized to a low value
unsigned long lastDisplayUpdate = 0;      // Timestamp of the last display update
uint8_t currentScreen = 0;               // Current display screen index
const float rssiThreshold = -100.0;        // RSSI threshold (dBm) below which packets are discarded
// --- CRC-16 Calculation Function (CCITT-FALSE) ---
uint16_t calculateCRC(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF; // Initialize CRC value to 0xFFFF
  for (size_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8; // XOR CRC with data byte shifted left by 8 bits
    for (uint8_t j = 0; j < 8; ++j) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1); // CRC polynomial division (CCITT-FALSE)
    }
  }
  return crc; // Return calculated CRC value
}
// --- Radio Interrupt Service Routine (ISR) Function ---
void setFlag(void) {
  receivedFlag = true; // Set the received flag when a packet is received (called by RadioLib ISR)
}
// --- Reset Received Packet Data Function ---
void resetRxPacket() {
  memset(&rxPacket, 0, sizeof(rxPacket)); // Clear the received packet structure
  rssi = -120.0;                       // Reset RSSI value
  snr = -20.0;                        // Reset SNR value
}
// --- Setup Function (Runs once at startup) ---
void setup() {
  Serial.begin(115200);   // Initialize serial communication for debugging and packet logging
  setupBoards();          // Board-specific setup (from LoRaBoards.h, likely pin configurations)
  delay(500);             // Short delay for system initialization
  resetRxPacket();        // Initialize received packet data and signal quality readings
#ifdef USING_U8G2
  u8g2.begin();           // Initialize U8g2 display library
#endif
  u8g2->setFont(u8g2_font_5x7_tr); // Set default font for U8g2 display
#ifdef RADIO_TCXO_ENABLE
  pinMode(RADIO_TCXO_ENABLE, OUTPUT);    // Configure TCXO enable pin as output
  digitalWrite(RADIO_TCXO_ENABLE, HIGH); // Enable TCXO (Temperature Compensated Crystal Oscillator) for radio if defined
#endif
  int state = radio.begin(); // Initialize LoRa radio module
  if (state != RADIOLIB_ERR_NONE) { // Check for radio initialization errors
    displayFatalError("RF SUBSYS FAIL"); // Display fatal error message if radio initialization fails and halt
  }
  // --- Configure LoRa Radio Parameters (MUST MATCH TRANSMITTER) ---
  radio.setFrequency(CARRIER_FREQ);      // Set carrier frequency
  radio.setBandwidth(BANDWIDTH);         // Set bandwidth
  radio.setSpreadingFactor(SPREADING_FACTOR); // Set spreading factor
  radio.setCodingRate(CODING_RATE);         // Set coding rate
  radio.setSyncWord(SYNC_WORD);           // Set sync word (network ID)
  radio.setCurrentLimit(140);           // Set current limit for radio module (mA)
  radio.setPreambleLength(16);          // Set preamble length
  radio.setCRC(false);                   // Disable hardware CRC (CRC is implemented in software)
  radio.setPacketReceivedAction(setFlag); // Set callback function for packet received interrupt (setFlag)
  // --- Start LoRa Receiver ---
  if (radio.startReceive() != RADIOLIB_ERR_NONE) { // Start listening for LoRa packets in receive mode
    displayFatalError("NO CARRIER"); // Display fatal error if receiver fails to start and halt
  }
  drawDisplay(); // Initial display update to show startup screen
}
// --- Main Loop Function (Runs continuously) ---
void loop() {
  if (receivedFlag) {      // Check if a packet has been received (flag set by ISR)
    receivedFlag = false;   // Reset the received flag
    processPacket();        // Process the received LoRa packet
    radio.startReceive();   // Restart listening for the next packet
  }
  if (millis() - lastDisplayUpdate >= 2000) { // Check if display update interval has elapsed
    currentScreen = (currentScreen + 1) % 7; // Cycle to the next display screen (modulo 7 screens)
    drawDisplay();          // Update the display with the current screen
    lastDisplayUpdate = millis(); // Update the last display update timestamp
  }
}
// --- Process Received Packet Function ---
void processPacket() {
  int state = radio.readData(encodedData, sizeof(encodedData)); // Read received data from radio module into buffer
  if (state == RADIOLIB_ERR_NONE) { // Check if data read was successful
    rssi = radio.getRSSI();         // Get Received Signal Strength Indication (RSSI) in dBm
    snr = radio.getSNR();          // Get Signal-to-Noise Ratio (SNR) in dB
    if (rssi > rssiThreshold) { // Check if RSSI is above the defined threshold (minimum signal strength)
      uint8_t decodedData[rsMsgLen]; // Buffer for Reed-Solomon decoded data
      int corrections = rs.Decode(encodedData, decodedData); // Decode Reed-Solomon encoded data
      if (corrections >= 0) { // Check if Reed-Solomon decoding was successful (or corrected errors)
        memcpy(&rxPacket, decodedData, sizeof(CCSDSPacket)); // Copy decoded data to telemetry packet structure
        uint16_t calculatedCRC = calculateCRC( // Calculate CRC-16 checksum on received packet (excluding CRC field)
            reinterpret_cast<uint8_t*>(&rxPacket),
            sizeof(CCSDSPacket) - sizeof(rxPacket.crc)
        );
        if (calculatedCRC == rxPacket.crc) { // Verify CRC checksum - packet integrity check
          if (rxPacket.hum_bme >= 0.0 && rxPacket.hum_bme <= 100.0) { // Data range validation: Humidity (example)
            validPackets++;       // Increment valid packet counter
            logPacket();          // Log the received and processed packet data to serial
          } else {
            rangeErrors++;        // Increment range error counter if humidity is out of range
          }
        } else {
          crcErrors++;          // Increment CRC error counter if checksum verification fails
        }
      } else {
        rsErrors++;             // Increment Reed-Solomon error counter if decoding fails
      }
    } else {
      resetRxPacket(); // Reset received packet data if RSSI is below threshold (weak signal)
      Serial.println("[INFO] No signal detected (RSSI: " + String(rssi) + " dBm, Threshold: " + String(rssiThreshold) + " dBm)");
    }
  } else {
    Serial.println("[ERROR] radio.readData() error"); // Log error if radio data read fails
  }
}
// --- Log Received Packet Data to Serial Function ---
void logPacket() {
  Serial.print("[TM] SCID:");       Serial.print(rxPacket.spacecraft_id); // Spacecraft ID
  Serial.print(" MET:");        Serial.print(rxPacket.mission_time);    // Mission Elapsed Time
  Serial.print(" BME_T:");      Serial.print(rxPacket.temp_bme, 1);     // BME280 Temperature
  Serial.print("C BME_H:");     Serial.print(rxPacket.hum_bme, 1);      // BME280 Humidity
  Serial.print("% BME_P:");     Serial.print(rxPacket.pres_bme, 1);     // BME280 Pressure
  Serial.print("hPa BME_A:");    Serial.print(rxPacket.alt_bme, 1);     // BME280 Altitude
  Serial.print("m MAG:");        Serial.print(rxPacket.magX, 1);        // Magnetometer X
  Serial.print(",");            Serial.print(rxPacket.magY, 1);        // Magnetometer Y
  Serial.print(",");            Serial.print(rxPacket.magZ, 1);        // Magnetometer Z
  Serial.print("uT ACC:");       Serial.print(rxPacket.accX, 4);        // Accelerometer X
  Serial.print(",");            Serial.print(rxPacket.accY, 4);        // Accelerometer Y
  Serial.print(",");            Serial.print(rxPacket.accZ, 4);        // Accelerometer Z
  Serial.print(" GYR:");       Serial.print(rxPacket.gyrX, 4);        // Gyroscope X
  Serial.print(",");            Serial.print(rxPacket.gyrY, 4);        // Gyroscope Y
  Serial.print(",");            Serial.print(rxPacket.gyrZ, 4);        // Gyroscope Z
  Serial.print(" QMI_T:");      Serial.print(rxPacket.temp_qmi, 1);     // QMI8658 Temperature
  Serial.print("C GPS_LAT:");    Serial.print(rxPacket.gps_lat, 4);     // GPS Latitude
  Serial.print(" GPS_LON:");    Serial.print(rxPacket.gps_lon, 4);     // GPS Longitude
  Serial.print(" GPS_ALT:");    Serial.print(rxPacket.gps_alt, 1);     // GPS Altitude
  Serial.print("m GPS_SATS:");   Serial.print(rxPacket.gps_satellites);  // GPS Satellites in view
  Serial.print(" GPS_HDOP:");   Serial.print(rxPacket.gps_hdop);      // GPS HDOP
  Serial.print(" MSG:");        Serial.print(rxPacket.message);         // Message string
  Serial.print(" RSSI:");       Serial.print(rssi, 1);                 // RSSI
  Serial.print("dBm CRC:");      Serial.println(rxPacket.crc, HEX);      // CRC-16 checksum (hexadecimal)
}
// --- Draw Display Function (Updates U8g2 Display) ---
void drawDisplay() {
  if (!u8g2->getU8g2()) return; // Check if U8g2 display is initialized
  u8g2->clearBuffer();         // Clear the display buffer
  u8g2->drawRFrame(0, 0, 128, 64, 5); // Draw rounded rectangle frame around the display area
  u8g2->setFont(u8g2_font_5x7_tr); // Set font for display text
  switch (currentScreen) { // Switch based on currentScreen index to display different screens
    case 0: // Screen 0: Environmental Sensor Data (BME280)
      u8g2->setCursor(2, 10);  u8g2->printf("Env. Sensors");
      u8g2->setCursor(2, 20);  u8g2->printf("T:%.1fC H:%.1f%%", rxPacket.temp_bme, rxPacket.hum_bme);
      u8g2->setCursor(2, 30);  u8g2->printf("P:%.1fhPa", rxPacket.pres_bme);
      u8g2->setCursor(2, 40);  u8g2->printf("A:%.1fm", rxPacket.alt_bme);
      break;
    case 1: // Screen 1: Magnetometer Data (QMC6310)
      u8g2->setCursor(2, 10);  u8g2->printf("Magnetometer");
      u8g2->setCursor(2, 20);  u8g2->printf("MagX: %.1f uT", rxPacket.magX);
      u8g2->setCursor(2, 30);  u8g2->printf("MagY: %.1f uT", rxPacket.magY);
      u8g2->setCursor(2, 40);  u8g2->printf("MagZ: %.1f uT", rxPacket.magZ);
      break;
    case 2: // Screen 2: IMU Data (QMI8658 - Accelerometer, Gyroscope, Temperature)
      u8g2->setCursor(2, 10);  u8g2->printf("IMU - QMI8658");
      u8g2->setCursor(2, 20);  u8g2->printf("AccX: %.3f", rxPacket.accX);
      u8g2->setCursor(2, 30);  u8g2->printf("AccY: %.3f", rxPacket.accY);
      u8g2->setCursor(2, 40);  u8g2->printf("GyrZ: %.3f", rxPacket.gyrZ);
      u8g2->setCursor(2, 50);  u8g2->printf("QMI T:%.1fC", rxPacket.temp_qmi);
      break;
    case 3: // Screen 3: GPS Data (Location, Satellites, HDOP)
      u8g2->setCursor(2, 10);  u8g2->printf("GPS Data");
      u8g2->setCursor(2, 20);  u8g2->printf("Lat: %.4f", rxPacket.gps_lat);
      u8g2->setCursor(2, 30);  u8g2->printf("Lon: %.4f", rxPacket.gps_lon);
      u8g2->setCursor(2, 40);  u8g2->printf("Alt: %.1fm", rxPacket.gps_alt);
      u8g2->setCursor(2, 50);  u8g2->printf("Sats:%d HDOP:%.1f", rxPacket.gps_satellites, rxPacket.gps_hdop);
      break;
    case 4: // Screen 4: RF and Status Information (RSSI, SNR, Error Counts)
      u8g2->setCursor(2, 10);  u8g2->printf("RF & Status");
      u8g2->setCursor(2, 20);  u8g2->printf("RSSI:%.1fdB SNR:%.1fdB", rssi, snr);
      u8g2->setCursor(2, 30);  u8g2->printf("Valid:%d CRCerr:%d", validPackets, crcErrors);
      u8g2->setCursor(2, 40);  u8g2->printf("RangeErr:%d RSerr:%d", rangeErrors, rsErrors);
      u8g2->setCursor(2, 50);  u8g2->printf("Msg: %.16s", rxPacket.message);
      break;
    case 5: // Screen 5: PMU Status Screen 1 (Battery Voltage, Percentage, Charging)
      u8g2->setCursor(2, 10);  u8g2->printf("PMU Status 1");
      u8g2->setCursor(2, 20);  u8g2->printf("Bat: %.2fV", rxPacket.battVoltage);
      u8g2->setCursor(2, 30);  u8g2->printf("Percent: %d%%", rxPacket.battPercent);
      u8g2->setCursor(2, 40);  u8g2->printf("Charging: %s", rxPacket.isCharging ? "YES" : "NO");
      break;
    case 6: // Screen 6: PMU Status Screen 2 (VBUS, System Voltage)
      u8g2->setCursor(2, 10);  u8g2->printf("PMU Status 2");
      u8g2->setCursor(2, 20);  u8g2->printf("VBUS: %.2fV", rxPacket.vbusVoltage);
      u8g2->setCursor(2, 30);  u8g2->printf("System: %.2fV", rxPacket.systemVoltage);
      break;
  }
  u8g2->sendBuffer(); // Send buffer to display, updating the screen
}
// --- Display Fatal Error Function (Serial and Display Output, then Halt) ---
void displayFatalError(const char* msg) {
  Serial.printf("[CRITICAL][%lu] %s\n", millis(), msg); // Print error message to serial monitor with timestamp
  if (u8g2->getU8g2()) { // Check if U8g2 display is initialized
    u8g2->clearBuffer();         // Clear display buffer
    u8g2->setFont(u8g2_font_ncenB10_tr); // Set larger font for error title
    u8g2->drawFrame(0, 0, 128, 64); // Draw frame around display
    u8g2->setCursor(5, 15);     // Set cursor position for error title
    u8g2->print("SYSTEM FAILURE"); // Print "SYSTEM FAILURE" title
    u8g2->setFont(u8g2_font_5x7_tr); // Set smaller font for error message
    const uint8_t maxLineLength = 20; // Maximum characters per line for error message
    uint8_t line = 2;                  // Start line for error message
    char buffer[maxLineLength + 1];     // Buffer to hold formatted error message lines
    const char *ptr = msg;             // Pointer to the error message string
    // Loop to wrap error message text to multiple lines on the display
    while (*ptr && line <= 4) {
      uint8_t i = 0;
      while (*ptr && i < maxLineLength && *ptr != '\n') { // Copy characters until line limit, end of string, or newline
        buffer[i++] = *ptr++;
      }
      buffer[i] = '\0';                // Null-terminate the buffer
      u8g2->setCursor(5, 15 * line); // Set cursor position for current line
      u8g2->print(buffer);           // Print error message line
      line++;                        // Increment line number
      if (*ptr == '\n') ptr++;       // Skip newline character if present
    }
    u8g2->sendBuffer(); // Update display with error message
  }
#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT); // Configure built-in LED pin as output
  while (1) {                 // Infinite loop to blink built-in LED indicating fatal error
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED state
    delay(1500);                                          // Delay for blinking effect
  }
#else
  while (1); // Infinite loop to halt program execution if no built-in LED defined
#endif
}


