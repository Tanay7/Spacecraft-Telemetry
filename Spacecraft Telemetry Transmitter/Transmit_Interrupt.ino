

/**
 * @file TelemetryTx.ino
 * @brief Spacecraft Telemetry Transmitter for LoRa Communication with Sensor Data
 *
 * This code implements a telemetry transmitter for a simulated spacecraft, sending sensor data
 * over a LoRa radio link. The telemetry data is structured in CCSDS (Consultative Committee for Space Data Systems)
 * packets for compatibility with space communication standards, similar to those used by missions like Voyager 1.
 * Reed-Solomon Forward Error Correction (RS-FEC) is employed to enhance data reliability over the radio link.
 *
 * The system incorporates the following sensors:
 *   - BME280: Environmental sensor for temperature, pressure, humidity, and altitude.
 *   - QMC6310: Magnetometer for measuring magnetic field in three axes.
 *   - QMI8658: Inertial Measurement Unit (IMU) for accelerometer, gyroscope, and temperature readings.
 *   - GPS: Global Positioning System module for location, altitude, date, time, and satellite information.
 *   - PMU (Power Management Unit): Integrated PMU to monitor battery voltage, charge percentage, charging status, VBUS voltage, and system voltage.
 *
 * Data is packetized using a CCSDS Application Packet (AP) structure and transmitted via LoRa radio.
 * The code is designed to be compatible with SX1262 LoRa transceivers and utilizes RadioLib library for LoRa modulation and control.
 * An optional U8g2 display can be used to visualize sensor readings and system status.
 *
 * Telemetry parameters include:
 *   - Environmental data: Temperature, pressure, humidity, altitude from BME280.
 *   - Magnetic field data: X, Y, Z axis readings from QMC6310 magnetometer.
 *   - Inertial data: Accelerometer (X, Y, Z), gyroscope (X, Y, Z), and temperature from QMI8658 IMU.
 *   - GPS data: Latitude, longitude, altitude, date, time, satellites in view, HDOP (Horizontal Dilution of Precision).
 *   - Power Management Unit data: Battery voltage, battery percentage, charging status, VBUS voltage, system voltage.
 *   - Packet sequence count and a message string for basic packet identification.
 *   - CRC-16 checksum for data integrity verification.
 *
 * Error handling includes basic checks for sensor initialization and a fatal error display routine.
 * The code is intended for educational purposes and demonstration of basic spacecraft telemetry concepts.
 *
 * Libraries Used:
 *   - RadioLib: For LoRa radio communication (https://github.com/jgromes/RadioLib)
 *   - LoRaBoards.h: Board-specific definitions for RadioLib (custom, likely board-dependent)
 *   - Wire.h: For I2C communication (required for BME280, QMC6310, U8g2 display)
 *   - SPI.h: For SPI communication (required for QMI8658, SDCard if used in QMI8658 library, RadioLib might use SPI)
 *   - Adafruit_Sensor.h & Adafruit_BME280.h: For BME280 sensor (https://github.com/adafruit/Adafruit_BME280_Library)
 *   - SensorQMC6310.hpp & SensorQMI8658.hpp: Custom sensor libraries (likely user-defined or from a specific sensor vendor)
 *   - U8g2lib.h: For U8g2 monochrome display library (https://github.com/olikraus/u8g2)
 *   - math.h: For mathematical functions (e.g., `atan2`, `M_PI`, `pow`)
 *   - RS-FEC.h: For Reed-Solomon Forward Error Correction (likely custom or from a library, details needed for URL)
 *   - TinyGPS++.h: For GPS parsing (http://arduinogps.jjoe.org/)
 *   - stdio.h: For standard input/output functions (e.g., `sprintf`)
 *
 * Target Platform: Arduino-compatible microcontroller (specific board defined in LoRaBoards.h)
 * LoRa Frequency: 433.0 MHz
 *
 * CCSDS Packet Structure Notes:
 *   - Follows CCSDS Packet Utilization Standard for low-rate telemetry.
 *   - APID (Application Process Identifier) is set to 0x7FF (example value).
 *   - Packet Version Number is set to 0b000 (CCSDS version).
 *   - Type Indicator is set to 0b0 (Telemetry Packet).
 *   - Secondary Header Flag is set to 0b1 (Presence of Secondary Header - although not explicitly used in this basic example).
 *   - Sequence Flags are set to 0b11 (Continuation Segment - assuming packets are segmented if needed, though this example likely sends single packets).
 *   - Packet Length Field indicates the length of the packet data field minus one, in bytes.
 *
 * Reed-Solomon FEC Notes:
 *   - RS(255, 223) code is used, meaning 223 bytes of data and 32 bytes of parity (error correction) are used in each encoded block.
 *   - This adds redundancy to the transmitted data, improving robustness against channel errors.
 *
 * Voyager 1 Context:
 *   - Voyager 1, like this code, transmits telemetry data back to Earth.
 *   - Voyager uses deep space network (DSN) for communication, similar to how this code uses LoRa for shorter-range communication.
 *   - CCSDS standards are widely used in spacecraft communication, including Voyager, to ensure interoperability and standardized data handling.
 *   - Sensor data types in this code (environmental, magnetic, inertial, GPS) are representative of data collected by spacecraft like Voyager, although Voyager's sensor suite is far more complex.
 *   - Error correction is critical for deep space missions like Voyager due to signal attenuation and noise over vast distances. RS-FEC is a common technique used in space communication.
 */
#include "LoRaBoards.h" // Board-specific definitions for RadioLib and pin assignments
#include <RadioLib.h>    // RadioLib library for LoRa communication
#include <Wire.h>       // I2C library for BME280, QMC6310, and U8g2 display
#include <SPI.h>        // SPI library for QMI8658 and potentially RadioLib/SD card
#include <Adafruit_Sensor.h> // Adafruit Unified Sensor Library (dependency for BME280)
#include <Adafruit_BME280.h> // Adafruit BME280 sensor library
#include "SensorQMC6310.hpp" // Custom library for QMC6310 magnetometer
#include "SensorQMI8658.hpp" // Custom library for QMI8658 IMU
#include <U8g2lib.h>     // U8g2 library for monochrome displays
#include <math.h>        // Math library for calculations (atan2, PI, pow)
#include <RS-FEC.h>      // Reed-Solomon Forward Error Correction library
#include <TinyGPS++.h>   // TinyGPS++ library for GPS parsing
#include <stdio.h>       // Standard input/output library for sprintf
// Time variables for screen switching and sensor updates
unsigned long lastScreenSwitch = 0; // Timestamp of the last screen switch
uint8_t currentScreen = 0;         // Current screen index for display
const unsigned long sensorUpdateInterval = 100; // Interval for sensor data updates (milliseconds)
const unsigned long defaultScreenDelay = 2000; // Default delay between screen switches (milliseconds)
const unsigned long gpsScreenDelay = 5000;    // Longer delay for GPS screens (milliseconds)
// Spacecraft identification and CCSDS packet parameters
#define SPACECRAFT_ID "DSN-2025-001" // Spacecraft ID string
#define CCSDS_APID 0x7FF             // CCSDS Application Process Identifier (example value)
#define CCSDS_VERSION 0b000          // CCSDS Packet Version Number (CCSDS version)
#define CCSDS_TYPE 0b0             // CCSDS Type Indicator (Telemetry Packet)
#define CCSDS_SEC_HDR_FLAG 0b1      // CCSDS Secondary Header Flag (Presence of Secondary Header - not used in this example)
#define CCSDS_SEQ_FLAGS 0b11         // CCSDS Sequence Flags (Continuation Segment)
// Reed-Solomon FEC parameters
const int rsMsgLen = 223;          // Length of the Reed-Solomon message (data bytes)
const uint8_t ECC_LENGTH = 32;     // Length of Reed-Solomon error correction code (parity bytes)
uint8_t encodedData[rsMsgLen + ECC_LENGTH]; // Buffer to hold Reed-Solomon encoded data
RS::ReedSolomon<rsMsgLen, ECC_LENGTH> rs; // Reed-Solomon encoder/decoder instance
// LoRa radio configuration (SX1262 specific)
#if defined(USING_SX1262)
#define CARRIER_FREQ 433.0       // LoRa carrier frequency (MHz)
#define TX_POWER 20              // LoRa transmit power (dBm)
#define BANDWIDTH 125.0          // LoRa bandwidth (kHz)
#define SPREADING_FACTOR 12      // LoRa spreading factor (7-12, higher = longer range, lower data rate)
#define CODING_RATE 6          // LoRa coding rate (5-8, higher = better error correction, lower data rate)
#define SYNC_WORD 0x35           // LoRa sync word (network ID)
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, // SX1262 radio module instance using RadioLib
                            RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif
// CCSDS Packet Structure definition (packed to ensure byte alignment)
#pragma pack(push, 1) // Push current alignment and set to 1-byte alignment
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
#pragma pack(pop) // Restore previous alignment
// Sensor instances and GPS object
#define SEALEVELPRESSURE_HPA 1013.25 // Define sea level pressure for altitude calculation (hPa)
Adafruit_BME280 bme;                // BME280 environmental sensor instance
SensorQMC6310 qmc;                 // QMC6310 magnetometer instance
SensorQMI8658 qmi;                 // QMI8658 IMU instance
TinyGPSPlus gps;                   // TinyGPSPlus GPS parser instance
// Transmission flag and telemetry packet instance
static volatile bool transmittedFlag = false; // Flag set in radio ISR when packet transmission is complete
CCSDSPacket tmPacket;                      // Telemetry packet structure instance
uint16_t packetCounter = 0;                // Packet sequence counter
// Time variables for sensor updates
unsigned long lastSensorUpdate = 0;      // Timestamp of the last sensor data update
unsigned long sensorLastUpdateTime = 0;  // (Potentially unused, consider removing if redundant)
// Sensor data variables
float temp_bme, pres_bme, alt_bme, hum_bme; // BME280 sensor readings
float magX, magY, magZ;                     // QMC6310 magnetometer readings
float accX, accY, accZ;                     // QMI8658 accelerometer readings
float gyrX, gyrY, gyrZ;                     // QMI8658 gyroscope readings
float temp_qmi;                             // QMI8658 temperature reading
int angle = 0;                               // Calculated heading angle from magnetometer (degrees)
// PMU data variables
float battVoltage;                           // Battery voltage (V)
uint8_t battPercent;                         // Battery percentage (%)
bool isCharging;                             // Battery charging status (true/false)
float vbusVoltage;                           // VBUS voltage (V)
float systemVoltage;                         // System voltage (V)
const float txPowerW = pow(10, (TX_POWER - 30) / 10); // Calculate transmit power in Watts (not directly used in code, potentially for power budget calculations)
// Data structure for accelerometer and gyroscope readings
struct data_t {
  float x; // X-axis reading
  float y; // Y-axis reading
  float z; // Z-axis reading
};
data_t acc, gyr; // Instances to store accelerometer and gyroscope data
// Function to set the transmission flag in ISR (Interrupt Service Routine)
void setFlag(void) { transmittedFlag = true; }
// CRC-16 calculation function (CCITT-FALSE algorithm)
uint16_t calculateCRC(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF; // Initialize CRC value to 0xFFFF
  for (size_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8; // XOR CRC with data byte shifted left by 8 bits
    for (uint8_t j = 0; j < 8; ++j) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1); // CRC polynomial division
    }
  }
  return crc; // Return calculated CRC value
}
// Function to build the telemetry packet with sensor data
void buildTelemetryPacket() {
  // CCSDS Primary Header Construction
  tmPacket.version_type_apid = (CCSDS_VERSION << 13) |     // Packet version number (3 bits)
                                 (CCSDS_TYPE << 12) |        // Packet type (1 bit)
                                 (CCSDS_SEC_HDR_FLAG << 11) | // Secondary header flag (1 bit)
                                 CCSDS_APID;                // Application Process Identifier (11 bits)
  tmPacket.sequence_flags_count = (CCSDS_SEQ_FLAGS << 14) | // Sequence flags (2 bits)
                                    (packetCounter & 0x3FFF);  // Packet sequence count (14 bits, modulo 16384)
  tmPacket.packet_length = sizeof(CCSDSPacket) - 7; // Packet length (total packet length - 7, as per CCSDS standard)
  // Spacecraft ID
  memcpy(tmPacket.spacecraft_id, SPACECRAFT_ID, 12); // Copy spacecraft ID string
  // Mission Time (milliseconds since startup)
  tmPacket.mission_time = millis();
  // BME280 Sensor Data
  tmPacket.temp_bme = temp_bme;
  tmPacket.pres_bme = pres_bme;
  tmPacket.hum_bme = hum_bme;
  tmPacket.alt_bme = alt_bme;
  // QMC6310 Magnetometer Data
  tmPacket.magX = magX;
  tmPacket.magY = magY;
  tmPacket.magZ = magZ;
  // QMI8658 IMU Data
  tmPacket.accX = acc.x;
  tmPacket.accY = acc.y;
  tmPacket.accZ = acc.z;
  tmPacket.gyrX = gyr.x;
  tmPacket.gyrY = gyr.y;
  tmPacket.gyrZ = gyr.z;
  tmPacket.temp_qmi = temp_qmi;
  // GPS Data - Location
  if (gps.location.isValid()) {
    tmPacket.gps_lat = gps.location.lat();    // Latitude (degrees)
    tmPacket.gps_lon = gps.location.lng();    // Longitude (degrees)
    tmPacket.gps_alt = gps.altitude.meters(); // Altitude (meters)
  } else {
    tmPacket.gps_lat = 0.0;
    tmPacket.gps_lon = 0.0;
    tmPacket.gps_alt = 0.0;
  }
  // GPS Data - Date and Time
  if (gps.date.isValid() && gps.time.isValid()) {
    tmPacket.gps_date_day = gps.date.day();     // Day of month
    tmPacket.gps_date_month = gps.date.month();   // Month
    tmPacket.gps_date_year = gps.date.year();    // Year
    tmPacket.gps_time_hour = gps.time.hour();    // Hour (UTC)
    tmPacket.gps_time_minute = gps.time.minute(); // Minute (UTC)
    tmPacket.gps_time_second = gps.time.second(); // Second (UTC)
  } else {
    tmPacket.gps_date_day = 0;
    tmPacket.gps_date_month = 0;
    tmPacket.gps_date_year = 0;
    tmPacket.gps_time_hour = 0;
    tmPacket.gps_time_minute = 0;
    tmPacket.gps_time_second = 0;
  }
  // GPS Data - Status
  tmPacket.gps_satellites = gps.satellites.value(); // Number of satellites in view
  tmPacket.gps_hdop = gps.hdop.hdop();            // Horizontal Dilution of Precision
  // PMU Data
  tmPacket.battVoltage = battVoltage;
  tmPacket.battPercent = battPercent;
  tmPacket.isCharging = isCharging;
  tmPacket.vbusVoltage = vbusVoltage;
  tmPacket.systemVoltage = systemVoltage;
  // Message String
  sprintf(tmPacket.message, "Packet: %d", packetCounter); // Format packet counter into message
  // CRC-16 Calculation (excluding CRC field itself)
  tmPacket.crc = calculateCRC(reinterpret_cast<uint8_t*>(&tmPacket),
                              sizeof(CCSDSPacket) - 2);
  // Reed-Solomon Encoding
  memcpy(encodedData, &tmPacket, rsMsgLen); // Copy raw packet data to encoding buffer
  rs.Encode(reinterpret_cast<char*>(&tmPacket), // Encode the packet data
            reinterpret_cast<char*>(encodedData)); // Store encoded data in the buffer
}
// Setup function (runs once at startup)
void setup() {
  Serial.begin(115200);    // Initialize serial communication for debugging
  SerialGPS.begin(9600);   // Initialize serial communication for GPS module
  while (!Serial);         // Wait for serial port to connect (for debugging)
  setupBoards();           // Board-specific setup (from LoRaBoards.h, likely pin configurations)
  delay(1500);             // Delay for system initialization
  u8g2->begin();            // Initialize U8g2 display library
  u8g2->setFont(u8g2_font_5x7_tr); // Set default font for U8g2 display
#ifdef RADIO_TCXO_ENABLE
  pinMode(RADIO_TCXO_ENABLE, OUTPUT);    // Configure TCXO enable pin as output
  digitalWrite(RADIO_TCXO_ENABLE, HIGH); // Enable TCXO (Temperature Compensated Crystal Oscillator) for radio if defined
#endif
  radio.begin(); // Initialize LoRa radio module
  radio.setFrequency(CARRIER_FREQ);      // Set carrier frequency
  radio.setBandwidth(BANDWIDTH);         // Set bandwidth
  radio.setSpreadingFactor(SPREADING_FACTOR); // Set spreading factor
  radio.setCodingRate(CODING_RATE);         // Set coding rate
  radio.setSyncWord(SYNC_WORD);           // Set sync word (network ID)
  radio.setOutputPower(TX_POWER);          // Set transmit power
  radio.setCurrentLimit(140);           // Set current limit for radio module (mA)
  radio.setPreambleLength(16);          // Set preamble length
  radio.setCRC(false);                   // Disable hardware CRC (CRC is implemented in software)
  radio.setPacketSentAction(setFlag);     // Set callback function for packet sent interrupt (setFlag)
  // Initialize sensors and check for failures
  if (!bme.begin()) displayFatalError("BME280 sensor fail!"); // Initialize BME280 sensor, display fatal error if initialization fails
  if (!qmc.begin(Wire, QMC6310_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) // Initialize QMC6310 magnetometer, display fatal error if initialization fails
    displayFatalError("QMC6310 sensor fail!");
  if (!qmi.begin(IMU_CS, -1, -1, -1, SDCardSPI)) { // Initialize QMI8658 IMU, display fatal error if initialization fails
    displayFatalError("QMI8658 sensor fail!");
  }
  // Configure QMI8658 Accelerometer
  qmi.configAccelerometer(
      SensorQMI8658::ACC_RANGE_4G,       // Accelerometer range: +/- 4g
      SensorQMI8658::ACC_ODR_1000Hz,    // Accelerometer output data rate: 1000 Hz
      SensorQMI8658::LPF_MODE_0,        // Accelerometer low-pass filter mode: Mode 0
      true);                           // Enable accelerometer
  // Configure QMI8658 Gyroscope
  qmi.configGyroscope(
      SensorQMI8658::GYR_RANGE_64DPS,     // Gyroscope range: +/- 64 degrees per second
      SensorQMI8658::GYR_ODR_896_8Hz,   // Gyroscope output data rate: 896.8 Hz
      SensorQMI8658::LPF_MODE_3,        // Gyroscope low-pass filter mode: Mode 3
      true);                           // Enable gyroscope
  qmi.enableGyroscope();     // Enable gyroscope measurements
  qmi.enableAccelerometer(); // Enable accelerometer measurements
  // Configure QMC6310 Magnetometer
  qmc.configMagnetometer(
      SensorQMC6310::MODE_CONTINUOUS, // Magnetometer mode: Continuous measurement
      SensorQMC6310::RANGE_8G,       // Magnetometer range: +/- 8 Gauss
      SensorQMC6310::DATARATE_200HZ,  // Magnetometer data rate: 200 Hz
      SensorQMC6310::OSR_1,           // Magnetometer oversampling rate: 1
      SensorQMC6310::DSR_1);           // Magnetometer data sample rate: 1
  updateSensorReadings();     // Initial sensor data update
  buildTelemetryPacket();     // Build the first telemetry packet
  radio.startTransmit(encodedData, sizeof(encodedData)); // Start LoRa transmission of the encoded telemetry packet
}
// Main loop function (runs continuously)
void loop() {
  if (transmittedFlag) {      // Check if packet transmission is complete (flag set by ISR)
    transmittedFlag = false;   // Reset transmission flag
    packetCounter++;           // Increment packet counter
    updateSensorReadings();    // Update sensor readings
    buildTelemetryPacket();    // Build the next telemetry packet
    radio.startTransmit(encodedData, sizeof(encodedData)); // Start transmission of the next packet
  }
  updateDisplay();         // Update display screen index (for cycling through screens)
  drawDisplay();           // Draw the current display screen
  delay(1);                // Small delay to prevent excessive CPU usage
  while (SerialGPS.available() > 0) // Process incoming GPS data from serial port
    gps.encode(SerialGPS.read());    // Pass each byte to the GPS parser
}
// Function to update all sensor readings
void updateSensorReadings() {
  if (millis() - lastSensorUpdate >= sensorUpdateInterval) { // Check if sensor update interval has elapsed
    updateBME280Data();    // Update BME280 sensor data
    updateQMC6310Data();    // Update QMC6310 sensor data
    updateQMI8658Data();    // Update QMI8658 sensor data
    updatePMUData();        // Update PMU data (battery voltage, etc.)
    lastSensorUpdate = millis(); // Update last sensor update timestamp
  }
  while (SerialGPS.available() > 0) // Continuously process GPS data in case data arrives between intervals
    gps.encode(SerialGPS.read());
}
// Function to update BME280 sensor data
void updateBME280Data() {
  temp_bme = bme.readTemperature();        // Read temperature in °C
  pres_bme = bme.readPressure() / 100.0F; // Read pressure in hPa (divide by 100 to convert from Pa)
  alt_bme = bme.readAltitude(SEALEVELPRESSURE_HPA); // Read altitude in meters using sea level pressure reference
  hum_bme = bme.readHumidity();             // Read humidity in % relative humidity
}
// Function to update QMC6310 magnetometer data
void updateQMC6310Data() {
  if (qmc.isDataReady()) { // Check if new magnetometer data is available
    qmc.readData();       // Read magnetometer data registers
    magX = qmc.getX();    // Read magnetic field X-axis (uT)
    magY = qmc.getY();    // Read magnetic field Y-axis (uT)
    magZ = qmc.getZ();    // Read magnetic field Z-axis (uT)
    // Calculate heading angle (compass heading)
    float heading = atan2(qmc.getY(), qmc.getX()); // Calculate heading in radians
    float declination = -0.041;                      // Magnetic declination for Galway, Ireland (adjust for location if needed)
    heading = heading + declination;                 // Apply magnetic declination correction
    // Wrap heading angle to 0-2PI range
    if (heading < 0) heading += 2 * PI;
    if (heading > 2 * PI) heading -= 2 * PI;
    angle = int(heading * 180 / M_PI); // Convert heading from radians to degrees and cast to integer
  }
}
// Function to update QMI8658 IMU data
void updateQMI8658Data() {
  if (qmi.getDataReady()) { // Check if new IMU data is available
    if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) { // Read accelerometer data (g)
      accX = acc.x;
      accY = acc.y;
      accZ = acc.z;
    }
    if (qmi.getGyroscope(gyr.x, gyr.y, gyr.z)) { // Read gyroscope data (dps)
      gyrX = gyr.x;
      gyrY = gyr.y;
      gyrZ = gyr.z;
    }
    temp_qmi = qmi.getTemperature_C(); // Read temperature from IMU (°C)
  }
}
// Function to update PMU (Power Management Unit) data
void updatePMUData() {
  battVoltage = PMU->getBattVoltage() / 1000.0f;   // Read battery voltage (V)
  battPercent = PMU->getBatteryPercent();          // Read battery percentage (%)
  isCharging = PMU->isCharging();                 // Check if battery is charging (true/false)
  vbusVoltage = PMU->getVbusVoltage() / 1000.0f;   // Read VBUS voltage (V)
  systemVoltage = PMU->getSystemVoltage() / 1000.0f; // Read system voltage (V)
}
// Function to update display screen index for cycling through screens
void updateDisplay() {
  if (millis() - lastScreenSwitch >= 2000) { // Check if screen switch delay has elapsed
    currentScreen = (currentScreen + 1) % 10; // Cycle to the next screen (modulo 10 screens)
    lastScreenSwitch = millis();              // Update last screen switch timestamp
  }
}
// Function to draw the current display screen based on currentScreen index
void drawDisplay() {
  if (!u8g2->getU8g2()) return; // Check if U8g2 display is initialized
  u8g2->clearBuffer();         // Clear the display buffer
  u8g2->setFont(u8g2_font_6x10_mr); // Set font for display text
//  u8g2->drawRFrame(0, 0, 128, 64, 5); // Draw rounded rectangle frame around the display area
  // Switch statement to draw different screens based on currentScreen value
  switch (currentScreen) {
    case 0: // Screen 0: Accelerometer and Temperature from QMI8658
      u8g2->setCursor(2, 10);
      u8g2->printf("ACCEL: X=%.2f", acc.x);
      u8g2->setCursor(2, 20);
      u8g2->printf("Y=%.2f", acc.y);
      u8g2->setCursor(2, 30);
      u8g2->printf("Z=%.2f", acc.z);
      u8g2->setCursor(2, 40);
      u8g2->printf("Temp: %.1fC", qmi.getTemperature_C());
      break;
    case 1: // Screen 1: Gyroscope data from QMI8658
      u8g2->setCursor(2, 10);
      u8g2->printf("GYRO: X=%.2f", gyr.x);
      u8g2->setCursor(2, 20);
      u8g2->printf("Y=%.2f", gyr.y);
      u8g2->setCursor(2, 30);
      u8g2->printf("Z=%.2f", gyr.z);
      break;
    case 2: // Screen 2: BME280 Environmental Sensor Data
      u8g2->setCursor(2, 10); u8g2->printf("BME280 Sensor");
      u8g2->setCursor(2, 20); u8g2->printf("Temp: %.2fC", temp_bme);
      u8g2->setCursor(2, 30); u8g2->printf("Hum: %.2f%%", hum_bme);
      u8g2->setCursor(2, 40); u8g2->printf("Pres: %.2fhPa", pres_bme);
      u8g2->setCursor(2, 50); u8g2->printf("Alt: %.2fm", alt_bme);
      break;
    case 3: // Screen 3: QMC6310 Magnetometer Data
      u8g2->setCursor(2, 10); u8g2->printf("QMC6310 Mag");
      u8g2->setCursor(2, 20); u8g2->printf("MagX: %.2f uT", magX);
      u8g2->setCursor(2, 30); u8g2->printf("MagY: %.2f uT", magY);
      u8g2->setCursor(2, 40); u8g2->printf("MagZ: %.2f uT", magZ);
      u8g2->setCursor(2, 50); u8g2->printf("Heading: %d deg", angle);
      break;
    case 4: // Screen 4: QMI8658 Accelerometer (Detailed)
      u8g2->setCursor(2, 10); u8g2->printf("QMI8658 ACCEL");
      u8g2->setCursor(2, 20); u8g2->printf("AccX: %.4f", accX);
      u8g2->setCursor(2, 30); u8g2->printf("AccY: %.4f", accY);
      u8g2->setCursor(2, 40); u8g2->printf("AccZ: %.4f", accZ);
      break;
    case 5: // Screen 5: QMI8658 Gyroscope and Temperature (Detailed)
      u8g2->setCursor(2, 10); u8g2->printf("QMI8658 GYRO");
      u8g2->setCursor(2, 20); u8g2->printf("GyrX: %.4f", gyrX);
      u8g2->setCursor(2, 30); u8g2->printf("GyrY: %.4f", gyrY);
      u8g2->setCursor(2, 40); u8g2->printf("GyrZ: %.4f", gyrZ);
      u8g2->setCursor(2, 50); u8g2->printf("Temp: %.2fC", temp_qmi);
      break;
    case 6: drawGPSDateTimeScreen(); break; // Screen 6: GPS Date and Time (calls dedicated function)
    case 7: drawGPSLocationScreen(); break; // Screen 7: GPS Location Data (calls dedicated function)
    case 8: // Screen 8: LoRa Transmission Parameters
      u8g2->setCursor(2, 10); u8g2->printf("LoRa Transmit");
      u8g2->setCursor(2, 20); u8g2->printf("Freq: %.1fMHz", CARRIER_FREQ);
      u8g2->setCursor(2, 30); u8g2->printf("Power: %ddBm", TX_POWER);
      u8g2->setCursor(2, 40); u8g2->printf("SF: %d BW: %.0f", SPREADING_FACTOR, BANDWIDTH);
      u8g2->setCursor(2, 50); u8g2->printf("Msg: %s", tmPacket.message);
      break;
    case 9: // Screen 9: PMU Status Screen 1 (Battery Voltage, Percentage, Charging)
      u8g2->setCursor(2, 10); u8g2->printf("PMU Status 1");
      u8g2->setCursor(2, 20); u8g2->printf("Bat: %.2fV", battVoltage);
      u8g2->setCursor(2, 30); u8g2->printf("Percent: %d%%", battPercent);
      u8g2->setCursor(2, 40); u8g2->printf("Charging: %s", isCharging ? "YES" : "NO");
      break;
    case 10: // Screen 10: PMU Status Screen 2 (VBUS, System Voltage)
      u8g2->setCursor(2, 10); u8g2->printf("PMU Status 2");
      u8g2->setCursor(2, 20); u8g2->printf("VBUS: %.2fV", vbusVoltage);
      u8g2->setCursor(2, 30); u8g2->printf("System: %.2fV", systemVoltage);
      break;
  }
  u8g2->sendBuffer(); // Send buffer to display to update the screen
}
// Function to draw GPS Date and Time screen
void drawGPSDateTimeScreen() {
  if (!u8g2->getU8g2()) return; // Check if U8g2 display is initialized
  u8g2->setCursor(2, 10);     // Set cursor position for text
  u8g2->printf("GPS Date & Time"); // Print screen title
  u8g2->setCursor(2, 20);     // Set cursor position for date
  if (gps.date.isValid() && gps.time.isValid()) { // Check if GPS date and time data is valid
    u8g2->printf("Date: %02d/%02d/%04d", gps.date.day(), gps.date.month(), gps.date.year()); // Print GPS date (DD/MM/YYYY)
    u8g2->setCursor(2, 30);     // Set cursor position for time
    u8g2->printf("Time: %02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second()); // Print GPS time (HH:MM:SS UTC)
  } else {
    u8g2->printf("Invalid GPS Data"); // Print message if GPS data is invalid
    u8g2->setCursor(2, 30);     // Set cursor position for "Waiting for Fix" message
    u8g2->printf("Waiting for Fix"); // Print "Waiting for Fix" message
  }
}
// Function to draw GPS Location screen
void drawGPSLocationScreen() {
  if (!u8g2->getU8g2()) return; // Check if U8g2 display is initialized
  u8g2->setCursor(2, 10);     // Set cursor position for text
  u8g2->printf("GPS Location");  // Print screen title
  u8g2->setCursor(2, 20);     // Set cursor position for Latitude
  if (gps.location.isValid()) { // Check if GPS location data is valid
    u8g2->printf("Lat: %.4f", gps.location.lat()); // Print GPS latitude (decimal degrees, 4 decimal places)
    u8g2->setCursor(2, 30);     // Set cursor position for Longitude
    u8g2->printf("Lon: %.4f", gps.location.lng()); // Print GPS longitude (decimal degrees, 4 decimal places)
    u8g2->setCursor(2, 40);     // Set cursor position for Altitude
    u8g2->printf("Alt: %.1fm", gps.altitude.meters()); // Print GPS altitude (meters, 1 decimal place)
    u8g2->setCursor(2, 50);     // Set cursor position for Satellites and HDOP
    u8g2->printf("Sats:%d HDOP:%.1f", gps.satellites.value(), gps.hdop.hdop()); // Print GPS satellites and HDOP
  } else {
    u8g2->printf("Invalid Location"); // Print message if GPS location is invalid
    u8g2->setCursor(2, 30);     // Set cursor position for "Waiting for Fix" message
    u8g2->printf("Waiting for Fix"); // Print "Waiting for Fix" message
    u8g2->setCursor(2, 50);     // Set cursor position for Satellites and HDOP (display even if location is invalid)
    u8g2->printf("Sats:%d HDOP:%.1f", gps.satellites.value(), gps.hdop.hdop()); // Print GPS satellites and HDOP
  }
}
// Function to display a fatal error message on serial and display, then halt execution
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
    delay(500);                                          // Delay for blinking effect
  }
#else
  while (1); // Infinite loop to halt program execution if no built-in LED defined
#endif
}
// Function to draw PMU Status Screen 1 (Battery Voltage, Percentage, Charging) - (Redundant, already in drawDisplay switch case)
void drawPMUScreen1() {
  u8g2->setCursor(2, 10);
  u8g2->printf("PMU Status 1");
  u8g2->setCursor(2, 20);
  u8g2->printf("Bat: %.2fV", battVoltage);
  u8g2->setCursor(2, 30);
  u8g2->printf("Percent: %d%%", battPercent);
  u8g2->setCursor(2, 40);
  u8g2->printf("Charging: %s", isCharging ? "YES" : "NO");
}
// Function to draw PMU Status Screen 2 (VBUS, System Voltage) - (Redundant, already in drawDisplay switch case)
void drawPMUScreen2() {
  u8g2->setCursor(2, 10);
  u8g2->printf("PMU Status 2");
  u8g2->setCursor(2, 20);
  u8g2->printf("VBUS: %.2fV", vbusVoltage);
  u8g2->setCursor(2, 30);
  u8g2->printf("System: %.2fV", systemVoltage);
}
