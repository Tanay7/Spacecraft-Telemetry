


/**
 * @file TelemetryTx_Revised_FixedBME.ino
 * @brief Spacecraft Telemetry Transmitter (LoRa) with Sensors, CCSDS-like structure, CRC, and RS-FEC.
 * @version 2.1.1 (Comment Enhancement)
 * @date 2025-04-11 (Based on user request date, comments updated 2025-04-11)
 *
 * @description
 * This Arduino sketch implements a telemetry transmitter simulating a spacecraft's data downlink.
 * It collects data from various sensors (environmental, inertial, magnetic, GPS, power),
 * formats it into a CCSDS-like packet structure, adds data integrity measures (CRC-16, RS-FEC),
 * and transmits the resulting data block via a LoRa radio module (specifically targeting SX1262).
 *
 * **Version 2.1 specifically fixed an issue with BME280 initialization, reverting to the
 * parameterless `bme.begin()` method based on user feedback indicating it was the working configuration.**
 *
 * Key Features & Revisions:
 * - **CCSDS-like Packet Structure:** Employs structures (`CCSDSPacketBuffer`, `CCSDSPayload`, `CcsdsTime`)
 * using fixed-width integer types (`stdint.h`). Adheres to Big-Endian (Network Byte Order)
 * convention for multi-byte fields within the packet using helper functions (`htonX_custom`).
 * Includes standard CCSDS primary header fields (Version, Type, Sec Hdr Flag=0, APID, Seq Flags, Seq Count, Length).
 * - **Data Integrity:**
 * - **CRC-16:** Calculates a CRC-16 CCITT-FALSE checksum over the entire packet (Header + Payload, excluding CRC field itself)
 * *before* Reed-Solomon encoding. The CRC value is included in the packet payload (Big-Endian).
 * - **Reed-Solomon FEC:** Encodes the CRC-protected packet data using RS(255, 223) Forward Error Correction,
 * adding 32 parity bytes to create a 255-byte block for transmission, enhancing robustness.
 * - **Modularity & Configuration:** Configuration parameters (#defines, consts) are grouped for easier management.
 * Redundant functions from previous versions are removed. Magic numbers are replaced with named constants.
 * - **Timekeeping:** Includes a `CcsdsTime` structure for mission time (seconds since boot + fractional seconds).
 * If a valid GPS time fix is available, the code *could* be adapted to use GPS time as the primary source,
 * but currently uses `millis()` based time. GPS date/time fields are populated directly if valid.
 * - **Data Management:** Focuses on transmitting a binary payload. Human-readable strings are removed from the
 * transmitted packet itself, requiring an external Interface Control Document (ICD) or Data Dictionary
 * on the receiving side to interpret the binary fields correctly.
 * - **Endianness:** Provides `htonX_custom` helper functions to convert data from the microcontroller's
 * native Host Byte Order (typically Little-Endian) to Network Byte Order (Big-Endian) before packing into the buffer.
 * - **Error Handling:** Includes initialization checks for sensors and the radio. Basic `isnan()` checks
 * are implemented for floating-point sensor readings before packing to handle potential sensor read errors.
 * Fatal errors halt execution and display messages.
 * - **Readability:** Enhanced comments, consistent formatting (4-space indent), and use of descriptive names.
 * - **Display Logic:** Uses U8g2 library to show sensor data, GPS status, LoRa config, and system info across multiple screens.
 * Screen cycling logic is implemented.
 * - **Magnetic Declination:** Includes a placeholder for magnetic declination correction (`MAGNETIC_DECLINATION`).
 * **CRITICAL NOTE:** This is currently hardcoded and inaccurate. A proper implementation requires dynamic lookup
 * or calculation based on the current GPS location using a magnetic field model library (e.g., WMM).
 * - **Sensor Integration:** Interfaces with BME280, QMC6310, QMI8658 sensors, GPS (via TinyGPS++), and a PMU (Power Management Unit, assumed accessible via `LoRaBoards.h`).
 *
 * @dependencies Libraries Used:
 * - RadioLib: For LoRa communication control (TX configuration, sending).
 * - LoRaBoards.h: Custom header for board-specific pins, hardware setup (`setupBoards`), potentially defines `u8g2` display object, `PMU` object, `SerialGPS`. **MUST BE PROVIDED.**
 * - Wire.h: Arduino I2C library (for sensors like BME280, QMC6310).
 * - SPI.h: Arduino SPI library (for LoRa radio module like SX1262, potentially QMI8658).
 * - Adafruit_Sensor.h: Adafruit's unified sensor abstraction layer (base for BME280).
 * - Adafruit_BME280.h: Library for the BME280 environmental sensor.
 * - SensorQMC6310.hpp: Custom library/header for the QMC6310 magnetometer. **MUST BE PROVIDED.**
 * - SensorQMI8658.hpp: Custom library/header for the QMI8658 IMU. **MUST BE PROVIDED.**
 * - U8g2lib.h: Library for controlling monochrome displays.
 * - math.h: Standard C math library (used for `pow`, `atan2` in heading calc, `isnan`).
 * - RS-FEC.h: Custom header for the Reed-Solomon FEC library implementation. **MUST BE PROVIDED.**
 * - TinyGPS++.h: Library for parsing NMEA sentences from a GPS module.
 * - stdio.h: Standard C I/O, used here only for `sprintf`/`printf` for display formatting.
 * - stdint.h: Provides standard integer types (e.g., `uint16_t`, `uint32_t`) for precise data sizing.
 * - string.h: Provides memory manipulation functions (`memcpy`, `memset`) and string functions (`strncpy`).
 *
 * Target Platform: Arduino-compatible microcontroller (specific board details expected in LoRaBoards.h).
 * LoRa Frequency: 433.0 MHz (Configurable via `LORA_CARRIER_FREQ`).
 *
 * @note Requires an external **Interface Control Document (ICD) / Data Dictionary** to define the
 * exact meaning, format, units, and encoding of each field within the `CCSDSPayload`.
 * @note Magnetic declination calculation (`MAGNETIC_DECLINATION`) is currently a placeholder and **MUST** be replaced
 * with a dynamic calculation based on real-time GPS location for accurate True North heading.
 * @note Assumes the RS-FEC library's `Encode` function correctly handles the input data. If the size of
 * `tmPacketBuffer` is less than `RS_MSG_LEN`, the library might need to pad the input. If `sizeof(tmPacketBuffer)`
 * is greater than `RS_MSG_LEN`, data truncation *will* occur during encoding, leading to corrupted data on the receiver side.
 * **It is CRITICAL that `sizeof(CCSDSPacketBuffer)` <= `RS_MSG_LEN`.**
 */

//==============================================================================
// Includes - Required library headers
//==============================================================================
#include "LoRaBoards.h"      // Board-specific definitions (pins, peripherals like display/PMU). **MUST BE PROVIDED SEPARATELY**.
#include <RadioLib.h>        // Core library for LoRa radio interactions (sending, configuration).
#include <Wire.h>            // Arduino library for I2C communication (used by BME280, QMC6310).
#include <SPI.h>             // Arduino library for SPI communication (used by SX1262 radio, possibly QMI8658).
#include <Adafruit_Sensor.h> // Adafruit Unified Sensor Driver library (base class for BME280).
#include <Adafruit_BME280.h> // Adafruit library for the BME280 sensor.
#include "SensorQMC6310.hpp" // Custom library/header for the QMC6310 magnetometer. **MUST BE PROVIDED SEPARATELY**.
#include "SensorQMI8658.hpp" // Custom library/header for the QMI8658 IMU. **MUST BE PROVIDED SEPARATELY**.
#include <U8g2lib.h>         // Library for controlling monochrome graphical displays.
#include <math.h>            // Standard C math library (used for atan2, isnan, M_PI, pow).
#include <RS-FEC.h>          // Reed-Solomon Forward Error Correction library. **MUST BE PROVIDED SEPARATELY**.
#include <TinyGPS++.h>       // Library for parsing NMEA sentences from GPS modules.
#include <stdio.h>           // Standard C Input/Output library, used only for `sprintf`/`printf` for formatting strings for the display.
#include <stdint.h>          // Provides standard integer types (uint8_t, int16_t, uint32_t etc.) for precise data sizing.
#include <string.h>          // Provides memory functions like `memcpy` (for copying data blocks), `memset` (for clearing buffers), and `strncpy`.

//==============================================================================
// Configuration Parameters - Define operational settings. (Consider moving to a separate "config.h")
//==============================================================================

// --- Telemetry & CCSDS Configuration ---
#define SPACECRAFT_ID "DSN-25-001"      // Spacecraft Identifier string. Max 11 chars + null recommended for the struct field below.
#define ICD_REFERENCE "DSN-ICD-XYZ-RevA"// Placeholder: Reference to the document defining this packet structure.

// CCSDS Primary Header Field Values (Customize APID as needed)
#define CCSDS_VERSION_NUMBER        0b000 // Packet Version Number (Bits 15-13). Standard is 0.
#define CCSDS_TYPE_INDICATOR        0b0   // Packet Type Indicator (Bit 12). 0 = Telemetry.
#define CCSDS_SECONDARY_HEADER_FLAG 0b0   // Secondary Header Flag (Bit 11). 0 = Absent. Change to 1 if secondary header is added.
#define CCSDS_APID_TELEMETRY        0x100 // Application Process ID (Bits 10-0). Example ID for this sensor stream.
#define CCSDS_SEQUENCE_FLAGS        0b11  // Sequence Flags (Bits 15-14). 0b11 = Unsegmented user data.
// Packet Sequence Count (Bits 13-0) is handled by the `packetCounter` variable.

// --- Reed-Solomon FEC Configuration ---
// Parameters for RS(255, 223) code. MUST MATCH RECEIVER.
const int RS_MSG_LEN     = 223;           // Length (bytes) of the original data message *before* RS encoding (Header + Payload + CRC). MUST MATCH RX.
const uint8_t RS_ECC_LEN = 32;            // Length (bytes) of the error correction code (parity) added by RS encoding. MUST MATCH RX.
#define RS_BLOCK_LEN (RS_MSG_LEN + RS_ECC_LEN) // Total length (255 bytes) of one encoded RS block transmitted over LoRa. MUST MATCH RX.
// ** CRITICAL: Ensure sizeof(CCSDSPacketBuffer) <= RS_MSG_LEN **

// --- LoRa Radio Configuration (Specific settings for the SX1262 module) ---
// Ensure LoRaBoards.h defines USING_SX1262 if using this chip.
#if defined(USING_SX1262)
#define LORA_CARRIER_FREQ       433.0   // LoRa carrier frequency in MHz. MUST MATCH RX.
#define LORA_TX_POWER           20      // LoRa transmit power in dBm (check local regulations!).
#define LORA_BANDWIDTH          125.0   // LoRa signal bandwidth in kHz. MUST MATCH RX.
#define LORA_SPREADING_FACTOR   12      // LoRa spreading factor (7-12). Higher SF = longer range, lower data rate. MUST MATCH RX.
#define LORA_CODING_RATE        6       // LoRa forward error correction coding rate (5=4/5, 6=4/6, 7=4/7, 8=4/8). MUST MATCH RX.
#define LORA_SYNC_WORD          0x35    // LoRa synchronization word (byte value). Acts as a network identifier. MUST MATCH RX.
#define LORA_PREAMBLE_LENGTH    16      // Length of the LoRa preamble (symbols). MUST MATCH RX expectation.
#define LORA_CURRENT_LIMIT      140     // Hardware current limit setting for the SX126x radio module PA in mA.
#else
// If no supported radio is defined, trigger a compile-time error.
#error "Unsupported or undefined radio module type! Check LoRaBoards.h and platform."
#endif // USING_SX1262

// --- Sensor Configuration ---
#define BME280_I2C_ADDRESS      (0x76)  // Default BME280 I2C address (can sometimes be 0x77). Used for info/check only, library handles it.
#define QMC6310_ADDRESS         (QMC6310_SLAVE_ADDRESS) // Use I2C address defined in Sensor lib or LoRaBoards.h.
#define SEALEVELPRESSURE_HPA    (1013.25f) // Standard sea level pressure (hPa) used for BME280 altitude calculation.

// QMI8658 IMU Configuration (Adjust ranges/rates as needed for application)
#define QMI_ACCEL_RANGE         SensorQMI8658::ACC_RANGE_4G    // Accelerometer range (e.g., 2G, 4G, 8G, 16G).
#define QMI_ACCEL_ODR           SensorQMI8658::ACC_ODR_1000Hz  // Accelerometer Output Data Rate (Hz).
#define QMI_ACCEL_LPF           SensorQMI8658::LPF_MODE_0      // Accelerometer Low Pass Filter mode.
#define QMI_GYRO_RANGE          SensorQMI8658::GYR_RANGE_64DPS // Gyroscope range (e.g., 16, 32, 64, 128, 256, 512, 1024, 2048 dps).
#define QMI_GYRO_ODR            SensorQMI8658::GYR_ODR_896_8Hz // Gyroscope Output Data Rate (Hz).
#define QMI_GYRO_LPF            SensorQMI8658::LPF_MODE_3      // Gyroscope Low Pass Filter mode.

// QMC6310 Magnetometer Configuration (Adjust range/rate as needed)
#define QMC_MODE                SensorQMC6310::MODE_CONTINUOUS // Operating mode (e.g., STANDBY, CONTINUOUS).
#define QMC_RANGE               SensorQMC6310::RANGE_8G        // Magnetic field measurement range (Gauss).
#define QMC_DATARATE            SensorQMC6310::DATARATE_200HZ  // Output Data Rate (Hz).
#define QMC_OSR                 SensorQMC6310::OSR_1           // Over Sample Ratio.
#define QMC_DSR                 SensorQMC6310::DSR_1           // Down Sample Ratio.

// Magnetic Declination - ** PLACEHOLDER ONLY **
// Value for Galway, Ireland (approx -2.35 degrees West as of early 2025, sign convention varies).
// Needs to be calculated dynamically based on GPS location using a World Magnetic Model (WMM) library for accuracy.
#define MAGNETIC_DECLINATION    (-2.35f) // DEGREES - ** VERY APPROXIMATE PLACEHOLDER - MUST BE UPDATED DYNAMICALLY **

// --- Timing Configuration ---
const unsigned long SENSOR_UPDATE_INTERVAL_MS = 100;  // Interval (ms) between polling sensors. Affects data freshness.
const unsigned long DISPLAY_SCREEN_INTERVAL_MS = 2000;// Interval (ms) to show each screen on the display before cycling.
const uint8_t NUM_DISPLAY_SCREENS = 11;               // Total number of distinct information screens to cycle through.

// --- CRC Configuration ---
// Parameters for CRC-16 CCITT-FALSE calculation. MUST MATCH RECEIVER.
#define CRC16_CCITT_FALSE_POLY 0x1021 // Generator polynomial for CRC-16 CCITT-FALSE. MUST MATCH RX.
#define CRC16_CCITT_FALSE_INIT 0xFFFF // Initial value for the CRC-16 calculation. MUST MATCH RX.

//==============================================================================
// Endian Swap Helper Functions (Host to Network - hton)
// Converts data types from the microcontroller's native byte order (Host - usually Little-Endian)
// to Big-Endian (Network Byte Order) for transmission according to CCSDS conventions.
//==============================================================================

/**
 * @brief Converts a uint16_t from Host Byte Order to Network Byte Order (Big-Endian).
 * @param val The 16-bit unsigned integer in Host Byte Order.
 * @return The 16-bit unsigned integer in Network Byte Order.
 * @details Performs a byte swap: (byte0 byte1) -> (byte1 byte0).
 */
uint16_t htons_custom(uint16_t val) {
    // Shift the low byte to the high position, shift the high byte to the low position, OR them together.
    return (val << 8) | (val >> 8);
}

/**
 * @brief Converts a uint32_t from Host Byte Order to Network Byte Order (Big-Endian).
 * @param val The 32-bit unsigned integer in Host Byte Order.
 * @return The 32-bit unsigned integer in Network Byte Order.
 * @details Performs a byte swap: (byte0 byte1 byte2 byte3) -> (byte3 byte2 byte1 byte0).
 */
uint32_t htonl_custom(uint32_t val) {
    // Shifts and masks each byte to its new position in the 32-bit word.
    return ((val << 24) & 0xFF000000) | // Move byte 3 to byte 0
           ((val << 8)  & 0x00FF0000) | // Move byte 2 to byte 1
           ((val >> 8)  & 0x0000FF00) | // Move byte 1 to byte 2
           ((val >> 24) & 0x000000FF);  // Move byte 0 to byte 3
}

/**
 * @brief Converts a float from Host Byte Order representation to its Network Byte Order (Big-Endian) binary representation.
 * @param val The float value in Host Byte Order.
 * @return The float value whose underlying bytes are arranged in Network Byte Order.
 * @note Assumes IEEE 754 standard representation for floating-point numbers.
 * This function reinterprets the float's memory as a uint32_t, swaps the bytes
 * using htonl_custom, and then reinterprets the swapped bytes back into a float's memory space.
 */
float htonf_custom(float val) {
    uint32_t temp;
    memcpy(&temp, &val, sizeof(temp)); // Copy float bytes into an integer container
    temp = htonl_custom(temp);         // Swap the bytes of the integer representation
    memcpy(&val, &temp, sizeof(val));  // Copy the swapped bytes back into the float variable
    return val;
}

/**
 * @brief Converts a double from Host Byte Order representation to its Network Byte Order (Big-Endian) binary representation.
 * @param val The double value in Host Byte Order.
 * @return The double value whose underlying bytes are arranged in Network Byte Order.
 * @note Assumes IEEE 754 standard representation for double-precision floating-point numbers.
 * Similar to htonf_custom, but handles 64 bits by swapping two 32-bit halves.
 */
double htond_custom(double val) {
    uint64_t temp;
    memcpy(&temp, &val, sizeof(temp)); // Copy double bytes into a 64-bit integer

    // Split the 64-bit integer into two 32-bit halves
    uint32_t low = (uint32_t)(temp & 0xFFFFFFFF);
    uint32_t high = (uint32_t)(temp >> 32);

    // Swap the bytes of each 32-bit half using htonl_custom
    low = htonl_custom(low);
    high = htonl_custom(high);

    // Reassemble the 64-bit integer with the swapped halves in reversed order for Big-Endian representation
    // (The original low part's bytes become the new high part, original high part's bytes become the new low part)
    temp = ((uint64_t)low << 32) | high;

    memcpy(&val, &temp, sizeof(val)); // Copy the fully swapped bytes back into the double variable
    return val;
}

//==============================================================================
// Time Structure (Basic CCSDS CUC Unsegmented Time Code - Like)
// Used within the telemetry payload to timestamp the data.
//==============================================================================
#pragma pack(push, 1) // Instruct compiler to pack struct members tightly without padding bytes.
struct CcsdsTime {
    uint32_t seconds;       // 4 bytes: Seconds elapsed since a defined epoch (e.g., mission start or GPS epoch if synced). Stored Big-Endian in packet.
    uint16_t subseconds;    // 2 bytes: Fractional part of the second (e.g., units of 1/2^16 seconds). Stored Big-Endian in packet.
    // NOTE: The epoch (time zero point) must be defined in the ICD. Here, it's implicitly derived from millis() startup time.
};
#pragma pack(pop) // Restore the compiler's default packing alignment settings.

//==============================================================================
// CCSDS Telemetry Packet Structure Definition - Payload Part
// Defines the structure of the data field *following* the 6-byte primary header.
// All multi-byte fields should be converted to Big-Endian before being placed here.
//==============================================================================
#pragma pack(push, 1) // Ensure tight packing for predictable size and byte alignment.
struct CCSDSPayload {
    // --- Metadata (18 Bytes) ---
    char spacecraft_id[12]; // 12 bytes: Spacecraft identifier string. Fixed length; ensure null termination if ID is shorter, or handle fixed length on Rx.
    CcsdsTime missionTime;  // 6 bytes: Mission elapsed time structure (`seconds` + `subseconds`). Stored Big-Endian.

    // --- BME280 Environmental Sensor Data (16 Bytes) ---
    float temp_bme;         // 4 bytes: Temperature reading from BME280 sensor (°C). Stored Big-Endian. NaN if invalid.
    float pres_bme;         // 4 bytes: Atmospheric pressure reading from BME280 sensor (hPa). Stored Big-Endian. NaN if invalid.
    float hum_bme;          // 4 bytes: Relative humidity reading from BME280 sensor (%). Stored Big-Endian. NaN if invalid.
    float alt_bme;          // 4 bytes: Calculated altitude based on pressure (meters). Stored Big-Endian. NaN if invalid.

    // --- QMC6310 Magnetometer Data (16 Bytes) ---
    float magX;             // 4 bytes: Magnetic field strength, X-axis (microTesla, uT). Stored Big-Endian. NaN if invalid.
    float magY;             // 4 bytes: Magnetic field strength, Y-axis (uT). Stored Big-Endian. NaN if invalid.
    float magZ;             // 4 bytes: Magnetic field strength, Z-axis (uT). Stored Big-Endian. NaN if invalid.
    float heading_deg;      // 4 bytes: Calculated compass heading (Degrees). Ideally corrected for declination (True North), currently likely Magnetic North based on placeholder. Stored Big-Endian. NaN if invalid.

    // --- QMI8658 Inertial Measurement Unit (IMU) Data (28 Bytes) ---
    float accX;             // 4 bytes: Acceleration, X-axis (g, standard gravity). Stored Big-Endian. NaN if invalid.
    float accY;             // 4 bytes: Acceleration, Y-axis (g). Stored Big-Endian. NaN if invalid.
    float accZ;             // 4 bytes: Acceleration, Z-axis (g). Stored Big-Endian. NaN if invalid.
    float gyrX;             // 4 bytes: Angular velocity (gyroscopic rate), X-axis (degrees per second, dps). Stored Big-Endian. NaN if invalid.
    float gyrY;             // 4 bytes: Angular velocity, Y-axis (dps). Stored Big-Endian. NaN if invalid.
    float gyrZ;             // 4 bytes: Angular velocity, Z-axis (dps). Stored Big-Endian. NaN if invalid.
    float temp_qmi;         // 4 bytes: Temperature reading from the IMU sensor (°C). Stored Big-Endian. NaN if invalid.

    // --- GPS Data (36 Bytes) ---
    uint8_t gps_fix_valid;  // 1 byte: GPS fix status flags (Bit 0: Time valid, Bit 1: Location valid. 0=None, 1=Time, 2=Loc, 3=Both).
    double gps_lat;         // 8 bytes: Latitude (degrees, double precision). Stored Big-Endian (0.0 or NAN if invalid).
    double gps_lon;         // 8 bytes: Longitude (degrees, double precision). Stored Big-Endian (0.0 or NAN if invalid).
    float gps_alt;          // 4 bytes: Altitude above sea level (meters). Stored Big-Endian (0.0f or NAN if invalid).
    uint8_t gps_date_day;   // 1 byte: Day of the month (1-31). (0 if invalid).
    uint8_t gps_date_month; // 1 byte: Month of the year (1-12). (0 if invalid).
    uint16_t gps_date_year; // 2 bytes: Year (e.g., 2025). Stored Big-Endian (0 if invalid).
    uint8_t gps_time_hour;  // 1 byte: Hour (0-23, UTC). (0 if invalid).
    uint8_t gps_time_minute;// 1 byte: Minute (0-59, UTC). (0 if invalid).
    uint8_t gps_time_second;// 1 byte: Second (0-59, UTC). (0 if invalid).
    uint8_t gps_satellites; // 1 byte: Number of GPS satellites used in the fix.
    float gps_hdop;         // 4 bytes: Horizontal Dilution of Precision. Stored Big-Endian (High value like 99.9 or NAN if invalid).

    // --- Power Management Unit (PMU) Data (14 Bytes) ---
    float battVoltage;      // 4 bytes: Measured battery terminal voltage (V). Stored Big-Endian. NaN if invalid.
    uint8_t battPercent;    // 1 byte: Estimated battery charge percentage (%).
    uint8_t isCharging;     // 1 byte: Battery charging status (0 = Not Charging, 1 = Charging).
    float vbusVoltage;      // 4 bytes: Voltage measured on the VBUS input (e.g., USB 5V) (V). Stored Big-Endian. NaN if invalid.
    float systemVoltage;    // 4 bytes: Main regulated system voltage (V). Stored Big-Endian. NaN if invalid.

    // --- CRC Checksum (2 Bytes) ---
    // CRC is calculated over Header + Payload (excluding CRC field itself).
    uint16_t crc;           // 2 bytes: CRC-16 CCITT-FALSE checksum value. Stored Big-Endian.
};
#pragma pack(pop) // Restore default packing alignment.


// --- Structure for the Complete In-Memory Packet Buffer ---
// This structure defines the layout of the entire packet (Header + Payload)
// as it exists in memory *before* Reed-Solomon encoding.
// All multi-byte fields within this structure (Header and Payload) should be
// converted to Big-Endian format before CRC calculation and RS encoding.
#pragma pack(push, 1) // Ensure tight packing.
struct CCSDSPacketBuffer {
    // --- CCSDS Primary Header Fields (6 bytes total) ---
    // These fields will be populated directly in the buffer before transmission.
    uint16_t version_type_sec_apid; // 2 bytes: Combined field: Ver(3b)|Type(1b)|SecHdr(1b)|APID(11b). Stored Big-Endian.
    uint16_t sequence_flags_count;  // 2 bytes: Combined field: SeqFlags(2b)|PktSeqCount(14b). Stored Big-Endian.
    uint16_t packet_data_length;    // 2 bytes: Length of the packet data field (CCSDSPayload size in bytes minus 1). Stored Big-Endian.

    // --- CCSDS Payload ---
    // Contains the actual telemetry data defined in the structure above.
    CCSDSPayload payload;           // Size defined by CCSDSPayload struct.
};
#pragma pack(pop) // Restore default packing.

// ** VALIDATION CHECK (Informative - requires manual check or static assert if available) **
// Serial.println(sizeof(CCSDSPacketBuffer)); // Check this value against RS_MSG_LEN
// static_assert(sizeof(CCSDSPacketBuffer) <= RS_MSG_LEN, "CCSDSPacketBuffer size exceeds RS_MSG_LEN!");

//==============================================================================
// Global Variables - Program state and data storage accessible throughout the sketch.
//==============================================================================

// --- Radio & Communication ---
#if defined(USING_SX1262)
// Create an instance of the RadioLib SX1262 class using pins defined in LoRaBoards.h.
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif
// Flag set by the radio's interrupt service routine (ISR) when a packet transmission is complete.
// `volatile` ensures the compiler doesn't optimize away accesses, as it can change unexpectedly.
static volatile bool transmittedFlag = false;
// Buffer to hold the final Reed-Solomon encoded data block (message + parity) ready for transmission.
// Size must match the full RS block length (255 bytes for RS(255, 223)).
uint8_t encodedData[RS_BLOCK_LEN];
// Counter for the Packet Sequence Count field in the CCSDS header. Incremented after each transmission.
// Wraps around based on its 14-bit field size (0 to 16383).
uint16_t packetCounter = 0;

// --- Reed-Solomon Encoder ---
// Instance of the Reed-Solomon encoder class template from the RS-FEC library.
// Templated with the message length (data part) and ECC length (parity part).
RS::ReedSolomon<RS_MSG_LEN, RS_ECC_LEN> rs;

// --- Packet Buffer ---
// In-memory buffer instance where the complete telemetry packet (Header + Payload)
// is assembled before CRC calculation and RS encoding.
CCSDSPacketBuffer tmPacketBuffer;

// --- Sensors & GPS Instances ---
Adafruit_BME280 bme;       // Instance of the BME280 library class.
SensorQMC6310 qmc;         // Instance of the custom QMC6310 library class.
SensorQMI8658 qmi;         // Instance of the custom QMI8658 library class.
TinyGPSPlus gps;           // Instance of the TinyGPS++ library class for parsing GPS data.

// --- Sensor Data Storage (Local Cache) ---
// Variables to hold the latest readings from sensors between packet builds.
// This avoids reading sensors directly inside the time-critical packet build function.
float temp_bme_local, pres_bme_local, alt_bme_local, hum_bme_local; // BME280 readings
float magX_local, magY_local, magZ_local, heading_local_deg;       // QMC6310 readings + calculated heading
float accX_local, accY_local, accZ_local;                          // QMI8658 Accelerometer readings
float gyrX_local, gyrY_local, gyrZ_local;                          // QMI8658 Gyroscope readings
float temp_qmi_local;                                              // QMI8658 Temperature reading
// GPS data is read directly from the 'gps' object during packet build (`buildTelemetryPacket`).

// --- PMU Data Storage (Local Cache) ---
// Variables to hold the latest readings from the Power Management Unit.
// Assumes `updatePMUData()` function populates these.
float battVoltage_local;
uint8_t battPercent_local;
bool isCharging_local;
float vbusVoltage_local;
float systemVoltage_local;

// --- Timing ---
unsigned long lastSensorUpdate = 0;  // Timestamp (`millis()`) of the last sensor polling cycle.
unsigned long lastScreenSwitch = 0;  // Timestamp (`millis()`) of the last display screen change.
uint8_t currentScreen = 0;           // Index (0 to NUM_DISPLAY_SCREENS - 1) of the screen currently shown.

// --- Display ---
// The U8g2 display object pointer ('u8g2') is expected to be created and managed
// within LoRaBoards.h or its associated setup code (`setupBoards`).
// Example instantiation (might be in LoRaBoards.h):
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C *u8g2 = new U8G2_SSD1306_128X64_NONAME_F_HW_I2C(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
// This code assumes 'u8g2' is a valid pointer after `setupBoards()` is called.

//==============================================================================
// Function Prototypes (Forward declarations for clarity and good practice)
//==============================================================================
void setFlag(void);                      // ISR callback function for RadioLib transmission completion.
uint16_t calculateCRC(const uint8_t* data, size_t length); // Calculates CRC-16 CCITT-FALSE.
void buildTelemetryPacket();             // Assembles the telemetry packet in `tmPacketBuffer`.
void setup();                            // Arduino standard setup function for initialization.
void loop();                             // Arduino standard main loop function.
void updateSensorReadings();             // Calls individual sensor update functions based on interval.
void updateBME280Data();                 // Reads BME280 data and updates local cache.
void updateQMC6310Data();                // Reads QMC6310 data, calculates heading, updates cache.
void updateQMI8658Data();                // Reads QMI8658 data and updates local cache.
void updatePMUData();                    // Reads PMU data (via assumed PMU object) and updates cache.
void updateDisplay();                    // Checks timing and cycles the `currentScreen` index.
void drawDisplay();                      // Renders the content for the `currentScreen` onto the display buffer.
void drawGPSDateTimeScreen();            // Helper function to draw GPS Date/Time screen content.
void drawGPSLocationScreen();            // Helper function to draw GPS Location screen content.
void displayFatalError(const char* msg); // Displays a critical error message and halts execution.

//==============================================================================
// ISR Callback Function - Executed upon hardware interrupt post-transmission.
//==============================================================================
/**
 * @brief Interrupt Service Routine (ISR) callback triggered by RadioLib when
 * the LoRa packet transmission process has finished.
 * @warning ISRs should be minimal. This just sets a flag for the main loop.
 */
void setFlag(void) {
    // Set the global flag to indicate transmission completion, allowing the main loop
    // to prepare and start the next transmission.
    transmittedFlag = true;
}

//==============================================================================
// CRC Calculation Function - Data integrity check.
//==============================================================================
/**
 * @brief Calculates the CRC-16 CCITT-FALSE checksum for a given data buffer.
 * @param data Pointer to the byte array containing the data over which to calculate the CRC.
 * @param length The number of bytes in the `data` buffer to include in the calculation.
 * @return The calculated 16-bit CRC value (in the host's native endianness).
 * @note This implementation MUST precisely match the CRC algorithm, polynomial (`CRC16_CCITT_FALSE_POLY`),
 * and initial value (`CRC16_CCITT_FALSE_INIT`) used by the receiver.
 */
uint16_t calculateCRC(const uint8_t* data, size_t length) {
    uint16_t crc = CRC16_CCITT_FALSE_INIT; // Start with the predefined initial value.
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8; // XOR the next data byte (shifted to high byte position) into CRC register.
        for (uint8_t j = 0; j < 8; ++j) { // Process each bit of the data byte.
            // Check if the most significant bit (MSB) of the CRC register is set.
            if (crc & 0x8000) {
                // If MSB is 1, shift left by 1 and XOR with the polynomial.
                crc = (crc << 1) ^ CRC16_CCITT_FALSE_POLY;
            } else {
                // If MSB is 0, just shift left by 1.
                crc = crc << 1;
            }
        }
    }
    return crc; // Return the final calculated CRC value.
}

//==============================================================================
// Telemetry Packet Building Function - Assembles data into the buffer.
//==============================================================================
/**
 * @brief Gathers the latest sensor data (from local cache and GPS object),
 * populates the `tmPacketBuffer` structure (Header and Payload), performs
 * necessary Endian conversions (Host to Network/Big-Endian) for multi-byte fields,
 * calculates the CRC-16 over the assembled packet (Header + Payload up to CRC field),
 * and stores the Big-Endian CRC value in the packet buffer.
 * @note This function *does not* perform Reed-Solomon encoding; that step happens
 * separately just before transmission using the contents of `tmPacketBuffer`.
 * @warning Assumes `_local` sensor variables have been updated recently via `updateSensorReadings()`.
 */
void buildTelemetryPacket() {
    // --- 1. Populate Payload Structure ---
    // Fill the payload part of the global `tmPacketBuffer`.

    // --- Metadata ---
    // Clear the ID field first to ensure null termination if ID is short.
    memset(tmPacketBuffer.payload.spacecraft_id, 0, sizeof(tmPacketBuffer.payload.spacecraft_id));
    // Copy the defined spacecraft ID, ensuring not to overflow the buffer. strncpy handles this.
    strncpy(tmPacketBuffer.payload.spacecraft_id, SPACECRAFT_ID, sizeof(tmPacketBuffer.payload.spacecraft_id) - 1);

    // Mission Time (based on millis() since boot)
    uint32_t currentMillis = millis(); // Get current uptime in milliseconds.
    uint32_t missionSec = currentMillis / 1000; // Calculate whole seconds.
    // Calculate fractional seconds: scale the remainder (0-999 ms) to the full uint16_t range (0-65535).
    // Use 65536UL for calculation to avoid potential overflow with standard int multiplication.
    uint16_t missionSubSec = (uint16_t)(((currentMillis % 1000) * 65536UL) / 1000UL);
    // Convert time fields to Big-Endian and store in the packet structure.
    tmPacketBuffer.payload.missionTime.seconds = htonl_custom(missionSec);
    tmPacketBuffer.payload.missionTime.subseconds = htons_custom(missionSubSec);

    // --- Sensor Data (using local cache variables) ---
    // Copy cached sensor data into the packet buffer, converting to Big-Endian.
    // Include checks for NaN (Not a Number) to handle potential sensor read errors.
    // If NaN, store NaN representation (requires receiver handling); otherwise, store the value.

    // BME280 Data
    tmPacketBuffer.payload.temp_bme = isnan(temp_bme_local) ? htonf_custom(NAN) : htonf_custom(temp_bme_local);
    tmPacketBuffer.payload.pres_bme = isnan(pres_bme_local) ? htonf_custom(NAN) : htonf_custom(pres_bme_local);
    tmPacketBuffer.payload.hum_bme  = isnan(hum_bme_local)  ? htonf_custom(NAN) : htonf_custom(hum_bme_local);
    tmPacketBuffer.payload.alt_bme  = isnan(alt_bme_local)  ? htonf_custom(NAN) : htonf_custom(alt_bme_local);

    // QMC6310 Magnetometer Data
    tmPacketBuffer.payload.magX = isnan(magX_local) ? htonf_custom(NAN) : htonf_custom(magX_local);
    tmPacketBuffer.payload.magY = isnan(magY_local) ? htonf_custom(NAN) : htonf_custom(magY_local);
    tmPacketBuffer.payload.magZ = isnan(magZ_local) ? htonf_custom(NAN) : htonf_custom(magZ_local);
    tmPacketBuffer.payload.heading_deg = isnan(heading_local_deg) ? htonf_custom(NAN) : htonf_custom(heading_local_deg);

    // QMI8658 IMU Data
    tmPacketBuffer.payload.accX = isnan(accX_local) ? htonf_custom(NAN) : htonf_custom(accX_local);
    tmPacketBuffer.payload.accY = isnan(accY_local) ? htonf_custom(NAN) : htonf_custom(accY_local);
    tmPacketBuffer.payload.accZ = isnan(accZ_local) ? htonf_custom(NAN) : htonf_custom(accZ_local);
    tmPacketBuffer.payload.gyrX = isnan(gyrX_local) ? htonf_custom(NAN) : htonf_custom(gyrX_local);
    tmPacketBuffer.payload.gyrY = isnan(gyrY_local) ? htonf_custom(NAN) : htonf_custom(gyrY_local);
    tmPacketBuffer.payload.gyrZ = isnan(gyrZ_local) ? htonf_custom(NAN) : htonf_custom(gyrZ_local);
    tmPacketBuffer.payload.temp_qmi = isnan(temp_qmi_local) ? htonf_custom(NAN) : htonf_custom(temp_qmi_local);

    // --- GPS Data (Read directly from TinyGPS++ object) ---
    // Check validity flags provided by TinyGPS++.
    bool timeValid = gps.time.isValid() && gps.date.isValid();
    // Check location validity AND if it has been updated since last check. Also check altitude validity.
    bool locValid  = gps.location.isValid() && gps.location.isUpdated() && gps.altitude.isValid();
    // Combine flags into the gps_fix_valid byte (Bit 0: Time Valid, Bit 1: Location Valid).
    tmPacketBuffer.payload.gps_fix_valid = (locValid << 1) | timeValid; // Results in 0, 1, 2, or 3.

    // Populate location fields only if the location fix is valid.
    if (locValid) {
        tmPacketBuffer.payload.gps_lat  = htond_custom(gps.location.lat());
        tmPacketBuffer.payload.gps_lon  = htond_custom(gps.location.lng());
        tmPacketBuffer.payload.gps_alt  = htonf_custom(gps.altitude.meters());
        tmPacketBuffer.payload.gps_hdop = htonf_custom(gps.hdop.hdop());
    } else {
        // If location is invalid, store default values (e.g., 0 or NaN). Receiver must handle these.
        tmPacketBuffer.payload.gps_lat  = htond_custom(0.0); // Using 0.0 as default invalid indicator.
        tmPacketBuffer.payload.gps_lon  = htond_custom(0.0);
        tmPacketBuffer.payload.gps_alt  = htonf_custom(0.0f);
        tmPacketBuffer.payload.gps_hdop = htonf_custom(99.9f); // Using a high value to indicate invalid HDOP.
    }

    // Populate time/date fields only if the time fix is valid.
    if (timeValid) {
        tmPacketBuffer.payload.gps_date_day    = gps.date.day();   // Single byte, no endian swap needed.
        tmPacketBuffer.payload.gps_date_month  = gps.date.month(); // Single byte.
        tmPacketBuffer.payload.gps_date_year   = htons_custom(gps.date.year()); // uint16_t, needs swap.
        tmPacketBuffer.payload.gps_time_hour   = gps.time.hour();    // Single byte.
        tmPacketBuffer.payload.gps_time_minute = gps.time.minute();  // Single byte.
        tmPacketBuffer.payload.gps_time_second = gps.time.second();  // Single byte.
    } else {
        // If time is invalid, store default zero values.
        tmPacketBuffer.payload.gps_date_day    = 0;
        tmPacketBuffer.payload.gps_date_month  = 0;
        tmPacketBuffer.payload.gps_date_year   = htons_custom(0);
        tmPacketBuffer.payload.gps_time_hour   = 0;
        tmPacketBuffer.payload.gps_time_minute = 0;
        tmPacketBuffer.payload.gps_time_second = 0;
    }
    // Satellites count is usually available even without a full fix.
    tmPacketBuffer.payload.gps_satellites = gps.satellites.value(); // Single byte.

    // --- PMU Data (using local cache variables) ---
    tmPacketBuffer.payload.battVoltage   = isnan(battVoltage_local)   ? htonf_custom(NAN) : htonf_custom(battVoltage_local);
    tmPacketBuffer.payload.battPercent   = battPercent_local;         // Single byte. Assume valid uint8 if PMU read succeeded.
    tmPacketBuffer.payload.isCharging    = isCharging_local ? 1 : 0;  // Single byte (0 or 1).
    tmPacketBuffer.payload.vbusVoltage   = isnan(vbusVoltage_local)   ? htonf_custom(NAN) : htonf_custom(vbusVoltage_local);
    tmPacketBuffer.payload.systemVoltage = isnan(systemVoltage_local) ? htonf_custom(NAN) : htonf_custom(systemVoltage_local);


    // --- 2. Populate Primary Header ---
    // Assemble the 6-byte CCSDS Primary Header fields directly into the `tmPacketBuffer`.

    // Field 1: Version (3b), Type (1b), Secondary Header Flag (1b), APID (11b)
    // Combine these using bitwise shifts and OR operations based on defined constants.
    tmPacketBuffer.version_type_sec_apid =
        (CCSDS_VERSION_NUMBER << 13) |          // Shift Version number to bits 15-13.
        (CCSDS_TYPE_INDICATOR << 12) |          // Shift Type indicator to bit 12.
        (CCSDS_SECONDARY_HEADER_FLAG << 11) |   // Shift Secondary Header flag to bit 11.
        (CCSDS_APID_TELEMETRY & 0x7FF);         // Mask APID to ensure it fits in lower 11 bits (0-10).

    // Field 2: Sequence Flags (2b), Packet Sequence Count (14b)
    // Combine sequence flags and the current packet counter.
    tmPacketBuffer.sequence_flags_count =
        (CCSDS_SEQUENCE_FLAGS << 14) |          // Shift Sequence flags to bits 15-14.
        (packetCounter & 0x3FFF);               // Mask Packet Counter to ensure it fits in lower 14 bits (0-13).

    // Field 3: Packet Data Length
    // Defined by CCSDS as: (Total number of octets in the Packet Data Field) - 1.
    // The Packet Data Field starts *after* the primary header, so it's our CCSDSPayload.
    // Check size calculation: Is CCSDSPayload *just* the payload, or does it include CRC field?
    // Assuming CCSDSPayload *includes* the CRC field based on its definition.
    // If CRC is considered part of the "Packet Data Field", then this is correct.
    tmPacketBuffer.packet_data_length = sizeof(CCSDSPayload) - 1;

    // Convert the assembled header fields to Big-Endian format.
    tmPacketBuffer.version_type_sec_apid = htons_custom(tmPacketBuffer.version_type_sec_apid);
    tmPacketBuffer.sequence_flags_count  = htons_custom(tmPacketBuffer.sequence_flags_count);
    tmPacketBuffer.packet_data_length    = htons_custom(tmPacketBuffer.packet_data_length);


    // --- 3. Calculate and Store CRC ---
    // Calculate the CRC-16 over the entire packet buffer *up to*, but not including,
    // the `crc` field itself within the payload structure.
    size_t crcDataLength = sizeof(CCSDSPacketBuffer) - sizeof(tmPacketBuffer.payload.crc);
    // Calculate CRC using the helper function. The result is in Host Byte Order.
    uint16_t crcCalculated = calculateCRC(reinterpret_cast<uint8_t*>(&tmPacketBuffer), crcDataLength);
    // Convert the calculated CRC to Big-Endian format and store it in the designated field within the payload.
    tmPacketBuffer.payload.crc = htons_custom(crcCalculated);


    // --- 4. Reed-Solomon Encoding (Deferred) ---
    // At this point, `tmPacketBuffer` contains the complete, Big-Endian formatted,
    // CRC-protected packet ready for the next step: Reed-Solomon encoding.
    // The RS encoding will be performed just before transmission in the main loop or setup.
}

//==============================================================================
// Setup Function - Initialization code executed once at startup.
//==============================================================================
/**
 * @brief Initializes hardware components (Serial ports, board specifics, display, radio, sensors),
 * libraries, sets up initial variable states, performs initial sensor read and packet build,
 * performs the first RS encoding, and starts the first LoRa transmission.
 */
void setup() {
    // --- Initialize Serial Communication ---
    Serial.begin(115200);    // Start primary serial port (USB) for debugging/logging output.
    // Initialize the Serial port connected to the GPS module. Baud rate must match GPS config.
    // Assumes LoRaBoards.h defines SerialGPS (e.g., `#define SerialGPS Serial1`)
    SerialGPS.begin(9600);

    // // Optional: Uncomment to pause execution until Serial Monitor is opened (useful for boards like Leonardo/Micro).
    // while (!Serial);

    delay(1500); // Generous delay to allow power supplies and external components (like GPS) to stabilize after power-on.

    Serial.println(F("--- Spacecraft Telemetry Transmitter (Revised + BME Fix) v2.1.1 ---"));
    Serial.print(F("ICD Reference: ")); Serial.println(ICD_REFERENCE); // Display ICD reference from config.

    // --- Initialize Board-Specific Hardware ---
    // Calls a function (expected in LoRaBoards.h/cpp) that handles microcontroller pin setup,
    // power management, and potentially initializes shared peripherals like the display ('u8g2') or PMU ('PMU').
    setupBoards();

    // --- Initialize Display (if available) ---
    // Check if the 'u8g2' object pointer provided by setupBoards() is valid.
    if (u8g2) { // Using `u8g2` directly checks if the pointer is non-null.
        u8g2->begin();                   // Initialize the display controller.
        u8g2->setFont(u8g2_font_6x10_mr); // Set the default text font.
        u8g2->clearBuffer();             // Clear the display's internal buffer.
        u8g2->setCursor(0, 10);          // Set cursor position (x, y).
        u8g2->print(F("System Booting..."));// Show initial message.
        u8g2->sendBuffer();              // Transfer buffer to the physical display.
    } else {
        // Log a warning if the display object wasn't initialized (e.g., not defined/enabled in LoRaBoards.h).
        Serial.println(F("[WARNING] U8g2 display object not initialized or accessible!"));
    }

    // --- Initialize LoRa Radio Module ---
    Serial.print(F("Initializing LoRa Radio (SX1262)... "));

    // Optional: Enable Temperature Controlled Crystal Oscillator (TCXO) for frequency stability, if available.
    #ifdef RADIO_TCXO_ENABLE // Check if TCXO control pin is defined (likely in LoRaBoards.h).
    pinMode(RADIO_TCXO_ENABLE, OUTPUT);    // Configure the TCXO control pin as output.
    digitalWrite(RADIO_TCXO_ENABLE, HIGH); // Set pin HIGH to enable TCXO (adjust logic if needed).
    delay(10); // Short delay for TCXO to stabilize.
    #endif // RADIO_TCXO_ENABLE

    // Initialize the radio module using RadioLib. Checks hardware connection and basic setup.
    int radioState = radio.begin();
    if (radioState == RADIOLIB_ERR_NONE) {
        Serial.println(F("Success!")); // Initialization successful.
    } else {
        // Initialization failed. Print error code and halt execution.
        Serial.print(F("Failed, code ")); Serial.println(radioState);
        displayFatalError("Radio Init Fail!"); // Display fatal error on screen and halt.
    }

    // --- Configure Radio Parameters ---
    // Set LoRa parameters. MUST match receiver settings where applicable.
    radio.setFrequency(LORA_CARRIER_FREQ);         // Set carrier frequency (MHz).
    radio.setBandwidth(LORA_BANDWIDTH);           // Set bandwidth (kHz).
    radio.setSpreadingFactor(LORA_SPREADING_FACTOR);// Set spreading factor (7-12).
    radio.setCodingRate(LORA_CODING_RATE);         // Set coding rate (5-8).
    radio.setSyncWord(LORA_SYNC_WORD);             // Set sync word (network ID).
    radio.setOutputPower(LORA_TX_POWER);           // Set transmit power (dBm).
    radio.setCurrentLimit(LORA_CURRENT_LIMIT);     // Set SX126x PA current limit (mA).
    radio.setPreambleLength(LORA_PREAMBLE_LENGTH); // Set preamble length.

    // IMPORTANT: Disable RadioLib's built-in hardware CRC checking.
    // We implement our own software CRC within the CCSDS packet structure, protected by RS FEC.
    radio.setCRC(false);

    // --- Setup Interrupt Handling for Transmission ---
    // Configure RadioLib to call the `setFlag()` function automatically (via ISR)
    // *after* the radio hardware finishes transmitting a packet.
    radio.setPacketSentAction(setFlag);

    // --- Initialize Sensors ---
    Serial.print(F("Initializing BME280... "));
    // *** BME Init Fix V2.1: Use parameterless begin() as confirmed working. ***
    // Assumes BME280 is connected via I2C and library finds it at default addresses.
    if (!bme.begin()) {
        // BME280 initialization failed.
        Serial.println(F("Fail!"));
        displayFatalError("BME280 Sensor Fail!"); // Halt execution.
    }
    Serial.println(F("OK")); // BME280 initialized successfully.

    Serial.print(F("Initializing QMC6310... "));
    // Initialize QMC6310 magnetometer. Assumes I2C connection.
    // The specific `begin` arguments might depend on the custom library used. Adjust if necessary.
    // Assumes Wire (I2C), address, and potentially pin definitions from LoRaBoards.h
    if (!qmc.begin(Wire, QMC6310_ADDRESS, I2C_SDA, I2C_SCL)) { // Example using Wire and defined pins
         Serial.println(F("Fail!"));
         displayFatalError("QMC6310 Sensor Fail!"); // Halt execution.
    }
    Serial.println(F("OK")); // QMC6310 initialized successfully.

    Serial.print(F("Initializing QMI8658... "));
    // Initialize QMI8658 IMU. Can be I2C or SPI. This example assumes SPI.
    // Assumes CS pin `IMU_CS` and SPI instance `SDCardSPI` are defined in LoRaBoards.h. Adjust if needed.
    if (!qmi.begin(IMU_CS, -1, -1, -1, SDCardSPI)) { // Example SPI init, -1 for unused pins
         Serial.println(F("Fail!"));
         displayFatalError("QMI8658 Sensor Fail!"); // Halt execution.
    }
      Serial.println(F("OK")); // QMI8658 initialized successfully.

    // --- Configure Sensors ---
    // Configure operating parameters (range, data rate, filters) for IMU and Magnetometer.
    qmi.configAccelerometer(QMI_ACCEL_RANGE, QMI_ACCEL_ODR, QMI_ACCEL_LPF, true); // Configure Accel
    qmi.configGyroscope(QMI_GYRO_RANGE, QMI_GYRO_ODR, QMI_GYRO_LPF, true);       // Configure Gyro
    qmi.enableAccelerometer(); // Enable Accelerometer readings
    qmi.enableGyroscope();     // Enable Gyroscope readings

    qmc.configMagnetometer(QMC_MODE, QMC_RANGE, QMC_DATARATE, QMC_OSR, QMC_DSR); // Configure Magnetometer

    Serial.println(F("Sensors Initialized and Configured."));

    // --- Initial Actions ---
    // Perform the first sensor readings and packet assembly immediately after setup.
    updateSensorReadings();      // Poll all sensors to get initial data.
    buildTelemetryPacket();      // Assemble the first packet structure in `tmPacketBuffer`.

    // --- Perform First Reed-Solomon Encoding ---
    // Copy the assembled packet data from `tmPacketBuffer` into the input area for RS encoding.
    // `memcpy` copies the raw bytes. Assumes tmPacketBuffer fits within RS_MSG_LEN.
    // If tmPacketBuffer is smaller, RS lib must handle padding. If larger, truncation occurs here!
    memcpy(encodedData, &tmPacketBuffer, RS_MSG_LEN); // Copy message part for encoding input.

    // Encode the data. The `rs.Encode` function reads the message data (first argument, cast to char*)
    // and writes the full encoded block (message + parity bytes) into the output buffer
    // (`encodedData`, second argument, cast to char*).
    rs.Encode(reinterpret_cast<char*>(&tmPacketBuffer), reinterpret_cast<char*>(encodedData));

    // --- Start First LoRa Transmission ---
    Serial.print(F("Starting first LoRa transmission ("));
    Serial.print(RS_BLOCK_LEN); Serial.println(F(" bytes encoded)..."));
    // Initiate the transmission of the RS-encoded data block (`encodedData`).
    // `startTransmit` is non-blocking; it starts the TX process and returns immediately.
    // The `setFlag` ISR will be called upon completion.
    int txState = radio.startTransmit(encodedData, RS_BLOCK_LEN);
    if (txState != RADIOLIB_ERR_NONE) {
        // Handle error if transmission could not be started.
        Serial.print(F("First transmit failed to start, code ")); Serial.println(txState);
        // Consider retrying or entering a fatal error state depending on requirements.
        displayFatalError("Initial TX Fail!");
    }

    Serial.println(F("Setup Complete. Entering main loop."));
    lastScreenSwitch = millis(); // Initialize the display screen switch timer.
}

//==============================================================================
// Main Loop - Continuously executed after setup().
//==============================================================================
/**
 * @brief The main execution loop. It checks if the previous transmission completed,
 * if so, updates sensors, builds the next packet, performs RS encoding, starts the
 * next transmission, and increments the packet counter. It also handles display updates
 * and continuously processes incoming GPS data.
 */
void loop() {

    // --- Handle Completed Transmission ---
    // Check if the `transmittedFlag` was set by the ISR, indicating the previous TX finished.
    if (transmittedFlag) {
        transmittedFlag = false; // Reset the flag immediately to detect the *next* completion.

        packetCounter++;         // Increment the packet sequence counter for the next packet header. (Wraps at 14 bits).

        // Optional: Short delay or radio status check might be useful here in some scenarios,
        // e.g., to ensure radio is ready or to rate-limit transmissions slightly.
        // delay(10);

        // --- Prepare for Next Transmission ---
        updateSensorReadings(); // Poll sensors to get fresh data for the *next* packet.
        buildTelemetryPacket(); // Assemble the new data into `tmPacketBuffer`, including new header fields and CRC.

        // --- Perform Reed-Solomon Encoding ---
        // Copy the newly assembled packet data (message part) for encoding input.
        // CRITICAL: Assumes sizeof(tmPacketBuffer) <= RS_MSG_LEN. Truncation occurs if larger.
        memcpy(encodedData, &tmPacketBuffer, RS_MSG_LEN);
        // Encode the data from `tmPacketBuffer` into the `encodedData` buffer (message + parity).
        rs.Encode(reinterpret_cast<char*>(&tmPacketBuffer), reinterpret_cast<char*>(encodedData));

        // --- Start Next Transmission ---
        // Initiate the transmission of the *newly encoded* data block. This is non-blocking.
        int txState = radio.startTransmit(encodedData, RS_BLOCK_LEN);
        if (txState != RADIOLIB_ERR_NONE) {
            // Log an error if starting the transmission failed.
            Serial.print(F("[ERR] LoRa Transmit failed to start, code: ")); Serial.println(txState);
            // Consider adding retry logic, logging to persistent storage, or other error handling.
        }
    } // End of transmittedFlag handling

    // --- Update Display ---
    updateDisplay(); // Check if it's time to switch to the next display screen.
    drawDisplay();   // Redraw the display with current data / screen content.

    // --- Process GPS Data Continuously ---
    // Keep feeding characters from the GPS serial port to the TinyGPS++ object
    // whenever they are available. TinyGPS++ parses the NMEA sentences internally.
    while (SerialGPS.available() > 0) {
        gps.encode(SerialGPS.read());
    }

    // --- Yield ---
    // Small delay to prevent the loop from consuming 100% CPU, allowing time for
    // background tasks, ISRs (like Serial input), and potentially improving stability.
    delay(1);

} // End of loop()

//==============================================================================
// Sensor Data Update Functions - Read sensors and store in local cache.
//==============================================================================

/**
 * @brief Master function to update sensor readings. Checks if the update interval
 * has passed and calls individual sensor update functions. Also processes GPS data.
 */
void updateSensorReadings() {
    // Check if enough time has passed since the last sensor polling cycle.
    if (millis() - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL_MS) {
        // Call functions to read each sensor and update its corresponding `_local` cache variables.
        updateBME280Data();
        updateQMC6310Data();
        updateQMI8658Data();
        updatePMUData(); // Update Power Management Unit data.
        lastSensorUpdate = millis(); // Reset the timer for the next interval.
    }

    // Process GPS data continuously, even between main sensor update intervals,
    // as GPS messages arrive asynchronously.
    while (SerialGPS.available() > 0) {
        gps.encode(SerialGPS.read());
    }
}

/**
 * @brief Reads data from the BME280 sensor (Temperature, Pressure, Humidity, Altitude)
 * and updates the corresponding `_local` global variables.
 */
void updateBME280Data() {
    // Read sensor values using the Adafruit BME280 library functions.
    temp_bme_local = bme.readTemperature();       // Reads temperature in °C.
    pres_bme_local = bme.readPressure() / 100.0F; // Reads pressure in Pascals (Pa), converts to hectopascals (hPa).
    alt_bme_local  = bme.readAltitude(SEALEVELPRESSURE_HPA); // Calculates altitude in meters based on current pressure and defined sea level pressure.
    hum_bme_local  = bme.readHumidity();          // Reads relative humidity in %.
    // Basic check for validity (optional, as buildPacket checks isnan)
    // if (isnan(temp_bme_local) || isnan(pres_bme_local) || ...) { Serial.println("[WARN] BME Read Error"); }
}

/**
 * @brief Reads data from the QMC6310 magnetometer (X, Y, Z axes), calculates
 * a heading (relative to magnetic north, potentially corrected by declination),
 * and updates the corresponding `_local` global variables.
 */
void updateQMC6310Data() {
    // Check if the sensor has new data available (specific to the SensorQMC6310 library implementation).
    if (qmc.isDataReady()) {
        qmc.readData(); // Instruct the library to read the latest values from the sensor registers.
        // Get magnetometer readings in microTesla (uT) from the library.
        magX_local = qmc.getX();
        magY_local = qmc.getY();
        magZ_local = qmc.getZ();

        // --- Calculate Heading ---
        // Calculate the heading angle in radians using atan2 on the X and Y components.
        // atan2 handles quadrants correctly. Typically, Y is used first for standard compass heading (North=0, East=90).
        float heading_rad = atan2(magY_local, magX_local);

        // --- Apply Magnetic Declination Correction (Approximate) ---
        // Convert the declination angle (degrees) to radians.
        float declination_rad = MAGNETIC_DECLINATION * M_PI / 180.0f;
        // Add the declination to the magnetic heading to approximate True North heading.
        // ** NOTE: THIS IS A MAJOR SIMPLIFICATION. Accurate declination varies significantly
        // ** with location and time and requires a proper magnetic model (e.g., WMM).
        heading_rad += declination_rad;

        // --- Normalize Heading ---
        // Ensure the heading angle stays within the 0 to 2*PI range (0 to 360 degrees).
        if (heading_rad < 0)    heading_rad += 2 * M_PI;
        if (heading_rad > 2*M_PI) heading_rad -= 2 * M_PI;

        // Convert the final heading from radians back to degrees for storage.
        heading_local_deg = heading_rad * 180.0f / M_PI;

    }
    // else: Data wasn't ready. Keep the previous values in the `_local` variables,
    // or optionally set them to NaN to indicate stale data if required by the system design.
    // magX_local = magY_local = magZ_local = heading_local_deg = NAN;
}

/**
 * @brief Reads data from the QMI8658 IMU (Accelerometer, Gyroscope, Temperature)
 * and updates the corresponding `_local` global variables.
 */
void updateQMI8658Data() {
    // Temporary structure to hold readings retrieved from the library function, if needed by the library's API.
    struct data_t { float x, y, z; };
    data_t acc_temp, gyr_temp;

    // Check if the IMU indicates new data is available (specific to SensorQMI8658 library).
    if (qmi.getDataReady()) {
        // Attempt to read accelerometer data (units typically 'g').
        if (qmi.getAccelerometer(acc_temp.x, acc_temp.y, acc_temp.z)) {
            // Store valid readings in local cache.
            accX_local = acc_temp.x;
            accY_local = acc_temp.y;
            accZ_local = acc_temp.z;
        } else {
            // If reading failed, set local cache variables to NaN (Not a Number).
            accX_local = accY_local = accZ_local = NAN;
            // Serial.println("[WARN] QMI Accel Read Fail"); // Optional warning
        }

        // Attempt to read gyroscope data (units typically 'dps' - degrees per second).
        if (qmi.getGyroscope(gyr_temp.x, gyr_temp.y, gyr_temp.z)) {
            // Store valid readings in local cache.
            gyrX_local = gyr_temp.x;
            gyrY_local = gyr_temp.y;
            gyrZ_local = gyr_temp.z;
        } else {
            // If reading failed, set local cache variables to NaN.
             gyrX_local = gyrY_local = gyrZ_local = NAN;
             // Serial.println("[WARN] QMI Gyro Read Fail"); // Optional warning
        }

        // Read the IMU's internal temperature sensor reading (units typically °C).
        temp_qmi_local = qmi.getTemperature_C();
        // Could add isnan check here too if the library might return NaN on temp error.
        // if (isnan(temp_qmi_local)) { Serial.println("[WARN] QMI Temp Read Fail"); }

    }
    // else: Data wasn't ready. Keep previous values or set to NaN as needed.
}

/**
 * @brief Reads data from the Power Management Unit (PMU) - Battery Voltage,
 * Percentage, Charging Status, VBUS Voltage, System Voltage - and updates
 * the corresponding `_local` global variables.
 * @note Assumes `LoRaBoards.h` or `setupBoards()` provides a global object pointer
 * named `PMU` to interact with the specific PMU hardware/library on the board.
 */
void updatePMUData() {
    // Check if the PMU object pointer is valid (non-null).
    if (PMU) {
        // Read values using the assumed PMU object's methods.
        // Convert units if necessary (e.g., library returns mV, we need V).
        battVoltage_local = PMU->getBattVoltage() / 1000.0f; // Assume getBattVoltage returns mV -> V.
        battPercent_local = PMU->getBatteryPercent();        // Assume returns uint8 percentage.
        isCharging_local  = PMU->isCharging();             // Assume returns boolean.
        vbusVoltage_local = PMU->getVbusVoltage() / 1000.0f; // Assume getVbusVoltage returns mV -> V.
        systemVoltage_local = PMU->getSystemVoltage() / 1000.0f; // Assume getSystemVoltage returns mV -> V.
        // Add isnan checks if the PMU library might return NaN on error.
        // if (isnan(battVoltage_local)) { /* Handle error */ }
    } else {
        // If PMU object doesn't exist (e.g., board doesn't have one or wasn't initialized),
        // set local cache variables to default/invalid values (e.g., NaN or 0).
        battVoltage_local = vbusVoltage_local = systemVoltage_local = NAN; // Use NaN for floats.
        battPercent_local = 0; // Use 0 for percentage.
        isCharging_local  = false; // Default to not charging.
        // Log warning only once if PMU is expected but not found?
        // static bool pmuWarned = false; if (!pmuWarned) { Serial.println("[WARN] PMU object not available!"); pmuWarned = true; }
    }
}

//==============================================================================
// Display Functions - Control the OLED/LCD screen output.
//==============================================================================

/**
 * @brief Checks if the display interval (`DISPLAY_SCREEN_INTERVAL_MS`) has elapsed
 * and increments the `currentScreen` index to cycle through the available
 * information screens. Wraps around to the first screen after the last one.
 */
void updateDisplay() {
    // Check if the time since the last screen switch exceeds the defined interval.
    if (millis() - lastScreenSwitch >= DISPLAY_SCREEN_INTERVAL_MS) {
        // Increment the screen index, using the modulo operator (%) to wrap around
        // back to 0 after reaching the total number of screens.
        currentScreen = (currentScreen + 1) % NUM_DISPLAY_SCREENS;
        // Reset the timer for the next interval.
        lastScreenSwitch = millis();
    }
}

/**
 * @brief Clears the display buffer and calls the appropriate drawing function
 * or draws content directly based on the `currentScreen` index. Uses data
 * primarily from the `_local` sensor cache variables and the `gps` object.
 * Finally, sends the completed buffer to the physical display.
 * @note Uses `u8g2->printf` for formatted output, relying on `stdio.h`.
 * Assumes `u8g2` pointer is valid.
 */
void drawDisplay() {
    // Pre-check: Bail out if the display object pointer is invalid.
    if (!u8g2 /* || !u8g2->getU8g2() */ ) return; // Basic check is usually sufficient.

    // Clear the display's internal memory buffer before drawing new content.
    u8g2->clearBuffer();
    // Ensure the correct font is selected for this drawing pass.
    u8g2->setFont(u8g2_font_6x10_mr); // Consistent font from setup.

    // Use a switch statement to render the specific content for the `currentScreen` index.
    switch (currentScreen) {
        case 0: // Screen 0: BME280 Environmental Sensor Data
            u8g2->setCursor(2, 10); u8g2->print(F("BME280 Sensor")); // Title
            // Display Temp, Humidity, Pressure, Altitude using printf formatting from local cache.
            u8g2->setCursor(2, 22); u8g2->printf("T:%.1fC H:%.1f%%", temp_bme_local, hum_bme_local);
            u8g2->setCursor(2, 34); u8g2->printf("P:%.1fhPa", pres_bme_local);
            u8g2->setCursor(2, 46); u8g2->printf("Alt:%.1fm", alt_bme_local);
             u8g2->setCursor(2, 58); u8g2->printf("Pkt:%u", packetCounter); // Show current packet number
            break;

        case 1: // Screen 1: QMC6310 Magnetometer Data
            u8g2->setCursor(2, 10); u8g2->print(F("QMC6310 Mag")); // Title
            // Display Mag X, Y, Z, and calculated Heading from local cache.
            u8g2->setCursor(2, 22); u8g2->printf("X:%.1f uT", magX_local);
            u8g2->setCursor(2, 34); u8g2->printf("Y:%.1f uT", magY_local);
            u8g2->setCursor(2, 46); u8g2->printf("Z:%.1f uT", magZ_local);
            u8g2->setCursor(2, 58); u8g2->printf("Head:%.1f deg?", heading_local_deg); // Add '?' due to declination uncertainty
            break;

        case 2: // Screen 2: QMI8658 Accelerometer Data
            u8g2->setCursor(2, 10); u8g2->print(F("QMI8658 ACCEL (g)")); // Title
            // Display Accel X, Y, Z from local cache. Use more precision if needed.
            u8g2->setCursor(2, 22); u8g2->printf("X: %.3f", accX_local);
            u8g2->setCursor(2, 34); u8g2->printf("Y: %.3f", accY_local);
            u8g2->setCursor(2, 46); u8g2->printf("Z: %.3f", accZ_local);
             u8g2->setCursor(2, 58); u8g2->printf("Pkt:%u", packetCounter);
            break;

        case 3: // Screen 3: QMI8658 Gyroscope Data
            u8g2->setCursor(2, 10); u8g2->print(F("QMI8658 GYRO(dps)")); // Title
            // Display Gyro X, Y, Z from local cache.
            u8g2->setCursor(2, 22); u8g2->printf("X: %.2f", gyrX_local);
            u8g2->setCursor(2, 34); u8g2->printf("Y: %.2f", gyrY_local);
            u8g2->setCursor(2, 46); u8g2->printf("Z: %.2f", gyrZ_local);
             u8g2->setCursor(2, 58); u8g2->printf("Pkt:%u", packetCounter);
            break;

        case 4: // Screen 4: QMI8658 IMU Temperature
              u8g2->setCursor(2, 10); u8g2->print(F("QMI8658 Temp")); // Title
              // Display IMU temperature from local cache.
              u8g2->setCursor(2, 25); u8g2->printf("Temp: %.1f C", temp_qmi_local);
              u8g2->setCursor(2, 58); u8g2->printf("Pkt:%u", packetCounter);
            break;

        case 5: // Screen 5: GPS Date and Time (uses helper function)
            drawGPSDateTimeScreen();
            break;

        case 6: // Screen 6: GPS Location Data (uses helper function)
            drawGPSLocationScreen();
            break;

        case 7: // Screen 7: LoRa Transmission Parameters (from defines)
            u8g2->setCursor(2, 10); u8g2->print(F("LoRa Transmit")); // Title
            // Display configured LoRa parameters.
            u8g2->setCursor(2, 22); u8g2->printf("Freq: %.1fMHz", LORA_CARRIER_FREQ);
            u8g2->setCursor(2, 34); u8g2->printf("Pwr:%ddBm SF:%d", LORA_TX_POWER, LORA_SPREADING_FACTOR);
            // Show Coding Rate as 4/X format for clarity (CR value is 5-8).
            u8g2->setCursor(2, 46); u8g2->printf("BW:%.0fkHz CR:4/%d", LORA_BANDWIDTH, LORA_CODING_RATE + 4);
            // Show total encoded packet size being transmitted and packet counter.
            u8g2->setCursor(2, 58); u8g2->printf("Pkt:%u (%dB)", packetCounter, RS_BLOCK_LEN);
            break;

        case 8: // Screen 8: PMU Status Screen 1 (Battery)
            u8g2->setCursor(2, 10); u8g2->print(F("PMU Status 1")); // Title
            // Display Battery Voltage, Percentage, Charging Status from local cache.
            u8g2->setCursor(2, 22); u8g2->printf("Batt: %.2f V", battVoltage_local);
            u8g2->setCursor(2, 34); u8g2->printf("Percent: %d %%", battPercent_local);
            u8g2->setCursor(2, 46); u8g2->printf("Charging: %s", isCharging_local ? "YES" : "NO");
            u8g2->setCursor(2, 58); u8g2->printf("Pkt:%u", packetCounter);
            break;

        case 9: // Screen 9: PMU Status Screen 2 (Voltages)
            u8g2->setCursor(2, 10); u8g2->print(F("PMU Status 2")); // Title
            // Display VBUS and System Voltage from local cache.
            u8g2->setCursor(2, 22); u8g2->printf("VBUS: %.2f V", vbusVoltage_local);
            u8g2->setCursor(2, 34); u8g2->printf("System: %.2f V", systemVoltage_local);
            u8g2->setCursor(2, 58); u8g2->printf("Pkt:%u", packetCounter);
            break;

         case 10: // Screen 10: System Information
            u8g2->setCursor(2, 10); u8g2->print(F("System Info")); // Title
            // Display system uptime calculated from millis().
            u8g2->setCursor(2, 22); u8g2->printf("Uptime: %lu s", millis() / 1000);
            // Display the configured Spacecraft ID.
            u8g2->setCursor(2, 34); u8g2->print(F("ID: ")); u8g2->print(SPACECRAFT_ID);
            // Optional: Add Free RAM display if a reliable function `freeMemory()` is available for the platform.
            // u8g2->setCursor(2, 46); u8g2->printf("Free RAM: %d B", freeMemory());
            u8g2->setCursor(2, 58); u8g2->printf("Pkt:%u", packetCounter);
            break;

        default: // Fallback case for unexpected screen index
            u8g2->setCursor(2, 10); u8g2->print(F("Unknown Screen"));
            // Log an error if this occurs, indicates a bug in screen cycling or NUM_DISPLAY_SCREENS.
            // Serial.print(F("[ERR] Invalid currentScreen index: ")); Serial.println(currentScreen);
            currentScreen = 0; // Reset to screen 0 to recover.
            break;
    } // End switch(currentScreen)

    // After drawing all elements for the current screen into the buffer,
    // transfer the entire buffer to the physical display hardware.
    u8g2->sendBuffer();
}

/**
 * @brief Helper function specifically for drawing the GPS Date and Time screen content.
 * Checks GPS fix validity before displaying data.
 */
void drawGPSDateTimeScreen() {
    // Pre-check for valid display object.
    if (!u8g2 /* || !u8g2->getU8g2() */ ) return;

    u8g2->setCursor(2, 10); u8g2->print(F("GPS Date & Time")); // Title
    // Check if both date and time components from GPS are marked as valid by TinyGPS++.
    if (gps.date.isValid() && gps.time.isValid()) {
        // Display formatted date (YYYY-MM-DD). Use leading zeros for month and day.
        u8g2->setCursor(2, 25);
        u8g2->printf("%04d-%02d-%02d", gps.date.year(), gps.date.month(), gps.date.day());
        // Display formatted time (HH:MM:SS UTC). Use leading zeros.
        u8g2->setCursor(2, 40);
        u8g2->printf("%02d:%02d:%02d UTC", gps.time.hour(), gps.time.minute(), gps.time.second());
    } else {
        // If date/time data is not valid, display a waiting message.
        u8g2->setCursor(2, 30); u8g2->print(F("Waiting for Time Fix"));
    }
    // Display the number of satellites currently tracked, regardless of fix validity.
    u8g2->setCursor(2, 55); u8g2->printf("Sats: %d Pkt:%u", gps.satellites.value(), packetCounter);
}

/**
 * @brief Helper function specifically for drawing the GPS Location screen content.
 * Checks GPS fix validity and update status before displaying data.
 */
void drawGPSLocationScreen() {
     // Pre-check for valid display object.
    if (!u8g2 /* || !u8g2->getU8g2() */ ) return;

    u8g2->setCursor(2, 10); u8g2->print(F("GPS Location")); // Title
    // Check if location is valid AND has been updated since the last query.
    if (gps.location.isValid() && gps.location.isUpdated()) {
        // Display Latitude, Longitude, Altitude, and HDOP with appropriate precision.
        u8g2->setCursor(2, 22); u8g2->printf("Lat: %.4f", gps.location.lat()); // Use 4 decimal places for Lat/Lon.
        u8g2->setCursor(2, 34); u8g2->printf("Lon: %.4f", gps.location.lng());
        u8g2->setCursor(2, 46); u8g2->printf("Alt: %.1fm", gps.altitude.meters()); // Altitude with 1 decimal place.
        u8g2->setCursor(2, 58); u8g2->printf("HDOP:%.1f Pk:%u", gps.hdop.hdop(), packetCounter); // HDOP with 1 decimal place.
    } else {
        // If location data is not valid or hasn't updated, display a waiting message.
        u8g2->setCursor(2, 30); u8g2->print(F("Waiting for Location Fix"));
        // Still display satellite count and HDOP as they might be available sooner than a full fix.
        u8g2->setCursor(2, 58); u8g2->printf("Sats:%d HDOP:%.1f", gps.satellites.value(), gps.hdop.hdop());
    }
}

//==============================================================================
// Fatal Error Handler - Stop execution on critical failure.
//==============================================================================
/**
 * @brief Handles critical, unrecoverable errors. Logs the error message to Serial,
 * displays a "SYSTEM HALTED" message on the screen (if available), and
 * enters an infinite loop (optionally blinking an LED) to stop further execution.
 * @param msg A C-string containing the error message to be displayed.
 */
void displayFatalError(const char* msg) {
    // --- Log to Serial Monitor ---
    Serial.printf("[FATAL][%lu] %s\n", millis(), msg); // Print error + timestamp.

    // --- Display on Screen (if available) ---
    if (u8g2 /* && u8g2->getU8g2() */ ) { // Check if display object is valid.
        u8g2->clearBuffer(); // Clear previous content.

        // Use a larger font for the main title.
        u8g2->setFont(u8g2_font_ncenB10_tr); // Example bold font.
        u8g2->drawFrame(0, 0, u8g2->getDisplayWidth(), u8g2->getDisplayHeight()); // Draw border frame.
        u8g2->setCursor(5, 15); // Position cursor for title.
        u8g2->print(F("SYSTEM HALTED")); // Print title.

        // --- Display Error Message with Basic Word Wrap ---
        u8g2->setFont(u8g2_font_5x7_tr); // Switch to smaller font for message.
        uint8_t maxLineChars = u8g2->getDisplayWidth() / 6; // Estimate chars per line based on font width (~6px).
        uint8_t currentLine = 3; // Start printing message lines below the title.
        const char* start = msg; // Pointer to the beginning of the message string.
        const char* end = msg + strlen(msg); // Pointer to the end of the message string.

        // Loop to print the message, wrapping text onto multiple lines.
        while (start < end && currentLine < 8) { // Limit lines to fit display height (adjust 8 if needed).
            const char* line_end = start + maxLineChars; // Calculate potential end of the line.
            if (line_end > end) line_end = end; // Don't go past the actual end of the string.

            const char* wrap_point = line_end; // Start searching for wrap point from potential end.
            // If not at the end of the string, try to find a space or newline to wrap at.
            if (line_end < end) {
                while (wrap_point > start && *wrap_point != ' ' && *wrap_point != '\n') {
                    wrap_point--; // Scan backwards for space/newline.
                }
                // If no suitable wrap point found in the segment, force break at maxLineChars.
                if (wrap_point == start) {
                    wrap_point = line_end;
                }
                // If we found a space/newline, use that as the end of the current line.
                else {
                    line_end = wrap_point; // Wrap *before* the space/newline.
                }
            }

            // --- Print the Line ---
            char lineBuf[maxLineChars + 1]; // Temporary buffer for the current line.
            size_t len = line_end - start; // Calculate length of the segment.
            if (len > maxLineChars) len = maxLineChars; // Safety clamp length.
            strncpy(lineBuf, start, len); // Copy the segment.
            lineBuf[len] = '\0'; // Null-terminate the buffer.

            // Set cursor and print the line. Adjust Y position based on font height + spacing.
            u8g2->setCursor(5, currentLine * 8 + 5); // Example Y positioning for 5x7 font.
            u8g2->print(lineBuf);

            // --- Advance Pointers for Next Line ---
            start = line_end; // Move start pointer to the end of the printed segment.
            // Skip any leading spaces or newlines at the beginning of the *next* segment.
            while(start < end && (*start == ' ' || *start == '\n')) {
                 start++;
            }
            currentLine++; // Move to the next display line number.
        } // End while loop (word wrapping)

        u8g2->sendBuffer(); // Send the complete fatal error message buffer to the display.
    } // End if (u8g2)

    // --- Halt Execution ---
    // Enter an infinite loop to stop the program.
    #ifdef LED_BUILTIN
    // If a built-in LED is defined, blink it as a visual indicator of the halted state.
    pinMode(LED_BUILTIN, OUTPUT);
    while (1) {
        digitalWrite(LED_BUILTIN, HIGH); delay(300);
        digitalWrite(LED_BUILTIN, LOW); delay(300);
    }
    #else
    // If no built-in LED is defined, just loop indefinitely.
    while (1) {
        delay(1000); // Prevent tight loop from potentially causing issues, yield time.
    }
    #endif // LED_BUILTIN
}

