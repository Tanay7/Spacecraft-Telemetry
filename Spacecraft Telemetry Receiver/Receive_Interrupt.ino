

/**
 * @file TelemetryRx_Consistent_Timeout.ino
 * @brief Spacecraft Telemetry Receiver (LoRa) with style consistency, timeout, RS-FEC, and CRC.
 * @version 2.4.1 (Comment Enhancement)
 * @date 2025-04-11 (Based on user request date, comments updated 2025-04-11)
 *
 * @description
 * This Arduino sketch implements a telemetry receiver designed to decode data packets
 * simulating spacecraft telemetry, received via a LoRa radio module (specifically
 * targeting SX1262). It incorporates several key features:
 *
 * 1.  **Stylistic Consistency:** Adopts commenting, spacing, indentation (4 spaces),
 * and display font conventions based on a related Transmitter v2.1 sketch.
 * 2.  **Signal Timeout:** Implements a mechanism (`NO_SIGNAL_TIMEOUT_MS`) to detect
 * loss of signal. If no valid packets (passing RS decoding and CRC checks)
 * are received within the specified timeout period, the displayed data is
 * cleared to indicate a stale state.
 * 3.  **Reed-Solomon FEC:** Utilizes a Reed-Solomon Forward Error Correction library
 * (`RS-FEC.h`) to decode RS(255, 223) encoded data, enhancing robustness
 * against transmission errors.
 * 4.  **CRC Check:** Performs a CRC-16 CCITT-FALSE checksum validation on the
 * decoded data payload to ensure data integrity, matching the transmitter's CRC.
 * 5.  **CCSDS-like Structure:** Defines C-style structures (`CCSDSPacketBuffer`,
 * `CCSDSPayload`, `CcsdsTime`) mimicking elements of CCSDS packet formats,
 * including handling of Big-Endian (Network Byte Order) data received over the air.
 * 6.  **Endian Conversion:** Includes helper functions (`ntohs_custom`, `ntohl_custom`,
 * `ntohf_custom`, `ntohd_custom`) to convert multi-byte fields from Network
 * Byte Order to the microcontroller's native Host Byte Order after validation.
 * 7.  **Display Integration:** Uses the U8g2 library to display received telemetry
 * data and status information across multiple screens on a monochrome display
 * (e.g., SH1106 OLED).
 * 8.  **Error Handling:** Includes basic error checking for radio initialization,
 * packet reception, RS decoding, and CRC validation, logging errors to Serial
 * and potentially halting on fatal errors.
 *
 * @note CRITICAL CONFIGURATION: The size of the primary data structure
 * `CCSDSPacketBuffer` *must* be less than or equal to `RS_MSG_LEN` (223 bytes).
 * If `sizeof(CCSDSPacketBuffer)` exceeds `RS_MSG_LEN`, the transmitter's RS encoding
 * step WILL truncate the data. This receiver code *assumes* the `RS_MSG_LEN` bytes
 * it decodes correspond directly to the beginning of the `CCSDSPacketBuffer` layout.
 * A mismatch will lead to incorrect data interpretation and likely CRC failures.
 * The code includes a check and logs a fatal configuration error if this condition occurs.
 *
 * @dependencies Libraries Used:
 * - RadioLib: For LoRa communication control.
 * - LoRaBoards.h: Custom header file expected to contain board-specific pin definitions,
 * hardware setup functions (`setupBoards`), and potentially the `u8g2` object definition/pointer.
 * *** THIS FILE MUST BE PROVIDED AND CORRECTLY CONFIGURED FOR THE TARGET HARDWARE. ***
 * - RS-FEC.h: Custom header file expected to contain the Reed-Solomon FEC library implementation.
 * *** THIS FILE MUST BE PROVIDED. ***
 * - U8g2lib.h: For controlling monochrome displays (like OLEDs).
 * - stdio.h: Standard C I/O, used here primarily for `sprintf`/`printf` (for display formatting).
 * - stdint.h: Provides standard integer types (e.g., `uint16_t`, `uint32_t`).
 * - string.h: Provides memory manipulation functions (`memcpy`, `memset`) and string functions (`strncpy`).
 * - math.h: Used for `isnan` check (data validation example) and potentially by sensor libs indirectly.
 * - Wire.h: Arduino I2C communication library (required by U8g2 for I2C displays).
 * - SPI.h: Arduino SPI communication library (required by RadioLib for SPI-connected radios).
 */

//==============================================================================
// Includes - Library headers required for functionality
//==============================================================================
#include <RadioLib.h>       // Core library for LoRa radio interactions (sending, receiving, configuration).
#include "LoRaBoards.h"     // Board-specific hardware definitions (pins, display type, etc.). **MUST BE PROVIDED SEPARATELY**.
#include <RS-FEC.h>         // Reed-Solomon Forward Error Correction library. **MUST BE PROVIDED SEPARATELY**.
#include <U8g2lib.h>        // Library for controlling monochrome graphical displays.

// Conditional compilation for U8g2 display initialization.
// Assumes LoRaBoards.h might define USING_U8G2 and potentially the 'u8g2' object.
#ifdef USING_U8G2
// Define the specific U8g2 display driver constructor.
// This example uses SH1106 via I2C. Modify according to your specific display hardware.
// Note: LoRaBoards.h or setupBoards() might instantiate 'u8g2' as a pointer,
// which is why '->' is used later for accessing its methods.
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE); // Example I2C pins - adjust if needed or rely on LoRaBoards.h
#endif // USING_U8G2

#include <stdio.h>          // Standard C Input/Output library, used for `sprintf`/`printf` for formatting strings for the display.
#include <stdint.h>         // Provides standard integer types (uint8_t, int16_t, uint32_t etc.) for precise data sizing.
#include <string.h>         // Provides memory functions like `memcpy` (for copying data blocks) and `memset` (for clearing buffers), and `strncpy`.
#include <math.h>           // Provides mathematical functions, specifically `isnan` used here for basic data validation.
#include <Wire.h>           // Arduino library for I2C communication, essential for I2C-based displays (like the SH1106 example).
#include <SPI.h>            // Arduino library for SPI communication, essential for SPI-based LoRa modules (like the SX1262).

//==============================================================================
// Configuration Parameters - Define operational settings for LoRa, RS, CRC, Display, etc.
//==============================================================================

// --- LoRa Radio Configuration (Specific settings for the SX1262 module) ---
// These parameters MUST exactly match the settings used on the transmitting device.
#define LORA_CARRIER_FREQ       433.0       // LoRa carrier frequency in MHz (e.g., 433.0, 868.0, 915.0). MUST MATCH TX.
#define LORA_BANDWIDTH          125.0       // LoRa signal bandwidth in kHz (e.g., 125.0, 250.0, 500.0). Affects data rate and range. MUST MATCH TX.
#define LORA_SPREADING_FACTOR   12          // LoRa spreading factor (integer 7-12). Higher SF increases range but decreases data rate. MUST MATCH TX.
#define LORA_CODING_RATE        6           // LoRa forward error correction coding rate (integer 5-8, representing 4/5 to 4/8). MUST MATCH TX.
#define LORA_SYNC_WORD          0x35        // LoRa synchronization word (byte value). Acts as a network identifier. MUST MATCH TX.
#define LORA_PREAMBLE_LENGTH    16          // Length of the LoRa preamble (symbols). MUST MATCH TX configuration for reliable reception.
#define LORA_CURRENT_LIMIT      140         // Hardware current limit setting for the SX126x radio module in mA. (Retained from original Rx).

// --- Radio Module Instantiation ---
// Conditional instantiation based on the radio type defined (likely in LoRaBoards.h).
// This example assumes an SX1262 connected via SPI.
#if defined(USING_SX1262)
// Create an instance of the RadioLib SX1262 class.
// Module constructor expects pins: CS (Chip Select), DIO1 (Interrupt), RST (Reset), BUSY.
// These pin definitions (RADIO_CS_PIN, etc.) MUST be provided in LoRaBoards.h.
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#else
// If no supported radio is defined, trigger a compile-time error.
#error "Unsupported or undefined radio module type! Check LoRaBoards.h and platform."
#endif // USING_SX1262

// --- Reed-Solomon FEC Configuration ---
// Parameters for the RS(255, 223) code. MUST MATCH TX.
const int RS_MSG_LEN = 223;                 // Length (in bytes) of the original data message block *before* RS encoding. MUST MATCH TX.
const uint8_t RS_ECC_LEN = 32;              // Length (in bytes) of the error correction code (parity) added by RS encoding. MUST MATCH TX.
#define RS_BLOCK_LEN (RS_MSG_LEN + RS_ECC_LEN) // Total length (255 bytes) of one encoded RS block (message + ECC). MUST MATCH TX.

// --- CRC Configuration ---
// Parameters for CRC-16 CCITT-FALSE calculation. MUST MATCH TX.
#define CRC16_CCITT_FALSE_POLY 0x1021       // Generator polynomial for CRC-16 CCITT-FALSE. MUST MATCH TX.
#define CRC16_CCITT_FALSE_INIT 0xFFFF       // Initial value for the CRC-16 calculation. MUST MATCH TX.

// --- Display Configuration ---
#define DISPLAY_SCREEN_INTERVAL_MS 2000     // Time interval (in milliseconds) to show each screen on the display before cycling.
#define NUM_DISPLAY_SCREENS 9               // Total number of distinct information screens to cycle through on the display.

// --- Signal Quality Threshold ---
const float RSSI_THRESHOLD = -100.0;        // Minimum Received Signal Strength Indicator (RSSI) in dBm required to attempt processing a packet. Packets weaker than this are ignored.

// --- Timeout Configuration ---
#define NO_SIGNAL_TIMEOUT_MS 5000           // Duration (in milliseconds) without receiving a *valid* packet before data is considered stale and cleared from the display.

//==============================================================================
// Endian Swap Helper Functions (Network to Host - ntoh)
// Converts data types from Big-Endian (Network Byte Order) as received
// to the native byte order of the microcontroller (Host Byte Order).
//==============================================================================

/**
 * @brief Converts a uint16_t from Network Byte Order (Big-Endian) to Host Byte Order.
 * @param val The 16-bit unsigned integer in Network Byte Order.
 * @return The 16-bit unsigned integer in Host Byte Order.
 * @details Performs a byte swap: (byte1 byte0) -> (byte0 byte1).
 */
uint16_t ntohs_custom(uint16_t val) {
    // Shift the high byte to the low position, shift the low byte to the high position, OR them together.
    return (val << 8) | (val >> 8);
}

/**
 * @brief Converts a uint32_t from Network Byte Order (Big-Endian) to Host Byte Order.
 * @param val The 32-bit unsigned integer in Network Byte Order.
 * @return The 32-bit unsigned integer in Host Byte Order.
 * @details Performs a byte swap: (byte3 byte2 byte1 byte0) -> (byte0 byte1 byte2 byte3).
 */
uint32_t ntohl_custom(uint32_t val) {
    // Shifts and masks each byte to its new position in the 32-bit word.
    return ((val << 24) & 0xFF000000) | // Move byte 0 to byte 3
           ((val << 8)  & 0x00FF0000) | // Move byte 1 to byte 2
           ((val >> 8)  & 0x0000FF00) | // Move byte 2 to byte 1
           ((val >> 24) & 0x000000FF);  // Move byte 3 to byte 0
}

/**
 * @brief Converts a float from its Network Byte Order (Big-Endian) binary representation to a Host float.
 * @param val The float value whose underlying bytes are currently in Network Byte Order.
 * @return The float value correctly interpreted in the Host's byte order.
 * @note Assumes IEEE 754 standard representation for floating-point numbers.
 * This function reinterprets the float's memory as a uint32_t, swaps the bytes
 * using ntohl_custom, and then reinterprets the swapped bytes back into a float.
 */
float ntohf_custom(float val) {
    uint32_t temp;
    memcpy(&temp, &val, sizeof(temp)); // Copy float bytes into an integer container
    temp = ntohl_custom(temp);         // Swap the bytes of the integer representation
    memcpy(&val, &temp, sizeof(val));  // Copy the swapped bytes back into the float variable
    return val;
}

/**
 * @brief Converts a double from its Network Byte Order (Big-Endian) binary representation to a Host double.
 * @param val The double value whose underlying bytes are currently in Network Byte Order.
 * @return The double value correctly interpreted in the Host's byte order.
 * @note Assumes IEEE 754 standard representation for double-precision floating-point numbers.
 * Similar to ntohf_custom, but handles 64 bits by swapping two 32-bit halves.
 */
double ntohd_custom(double val) {
    uint64_t temp;
    memcpy(&temp, &val, sizeof(temp)); // Copy double bytes into a 64-bit integer

    // Split the 64-bit integer into two 32-bit halves
    uint32_t low = (uint32_t)(temp & 0xFFFFFFFF);
    uint32_t high = (uint32_t)(temp >> 32);

    // Swap the bytes of each 32-bit half using ntohl_custom
    low = ntohl_custom(low);
    high = ntohl_custom(high);

    // Reassemble the 64-bit integer with the swapped halves in reversed order
    // (The original low part becomes the new high part, and vice versa)
    temp = ((uint64_t)low << 32) | high;

    memcpy(&val, &temp, sizeof(val)); // Copy the fully swapped bytes back into the double variable
    return val;
}

//==============================================================================
// Time & Packet Structure Definitions (Matching Transmitter v2.1 Layout)
// These structures define how the received telemetry data is organized in memory.
// Fields are assumed to be received in Big-Endian (Network Byte Order).
//==============================================================================

// --- Time Structure (Basic CCSDS CUC Unsegmented Time Code - Like) ---
#pragma pack(push, 1) // Instruct compiler to pack struct members tightly without padding bytes. Crucial for correct data mapping.
struct CcsdsTime {
    uint32_t seconds;       // 4 bytes: Seconds elapsed since a defined epoch (e.g., mission start). Received Big-Endian.
    uint16_t subseconds;    // 2 bytes: Fractional part of the second (e.g., units of 1/2^16 seconds). Received Big-Endian.
};
#pragma pack(pop) // Restore the compiler's default packing alignment settings.

// --- CCSDS Telemetry Payload Structure ---
// Defines the data fields within the main payload section of the packet.
// All multi-byte fields are received in Big-Endian format.
#pragma pack(push, 1) // Ensure tight packing.
struct CCSDSPayload {
    // --- Metadata (18 Bytes) ---
    char spacecraft_id[12]; // 12 bytes: Null-termination is NOT guaranteed by protocol; treat as fixed-size byte array.
    CcsdsTime missionTime;  // 6 bytes: Mission elapsed time structure (`seconds` + `subseconds`).

    // --- BME280 Environmental Sensor Data (16 Bytes) ---
    float temp_bme;         // 4 bytes: Temperature reading from BME280 sensor (°C).
    float pres_bme;         // 4 bytes: Atmospheric pressure reading from BME280 sensor (hPa).
    float hum_bme;          // 4 bytes: Relative humidity reading from BME280 sensor (%).
    float alt_bme;          // 4 bytes: Calculated altitude based on pressure (meters).

    // --- QMC6310 Magnetometer Data (16 Bytes) ---
    float magX;             // 4 bytes: Magnetic field strength, X-axis (microTesla, uT).
    float magY;             // 4 bytes: Magnetic field strength, Y-axis (uT).
    float magZ;             // 4 bytes: Magnetic field strength, Z-axis (uT).
    float heading_deg;      // 4 bytes: Calculated compass heading based on magnetometer readings (Degrees).

    // --- QMI8658 Inertial Measurement Unit (IMU) Data (28 Bytes) ---
    float accX;             // 4 bytes: Acceleration, X-axis (g, standard gravity).
    float accY;             // 4 bytes: Acceleration, Y-axis (g).
    float accZ;             // 4 bytes: Acceleration, Z-axis (g).
    float gyrX;             // 4 bytes: Angular velocity (gyroscopic rate), X-axis (degrees per second, dps).
    float gyrY;             // 4 bytes: Angular velocity, Y-axis (dps).
    float gyrZ;             // 4 bytes: Angular velocity, Z-axis (dps).
    float temp_qmi;         // 4 bytes: Temperature reading from the IMU sensor (°C).

    // --- GPS Data (36 Bytes) ---
    uint8_t gps_fix_valid;  // 1 byte: GPS fix status flags (Bit 0: Time valid, Bit 1: Location valid. 0=None, 1=Time, 2=Loc, 3=Both).
    double gps_lat;         // 8 bytes: Latitude (degrees, double precision).
    double gps_lon;         // 8 bytes: Longitude (degrees, double precision).
    float gps_alt;          // 4 bytes: Altitude above sea level (meters).
    uint8_t gps_date_day;   // 1 byte: Day of the month (1-31).
    uint8_t gps_date_month; // 1 byte: Month of the year (1-12).
    uint16_t gps_date_year; // 2 bytes: Year (e.g., 2025). Received Big-Endian.
    uint8_t gps_time_hour;  // 1 byte: Hour (0-23, UTC).
    uint8_t gps_time_minute;// 1 byte: Minute (0-59, UTC).
    uint8_t gps_time_second;// 1 byte: Second (0-59, UTC).
    uint8_t gps_satellites; // 1 byte: Number of GPS satellites used in the current fix.
    float gps_hdop;         // 4 bytes: Horizontal Dilution of Precision (lower is better).

    // --- Power Management Unit (PMU) Data (14 Bytes) ---
    float battVoltage;      // 4 bytes: Measured battery terminal voltage (V).
    uint8_t battPercent;    // 1 byte: Estimated battery charge percentage (%).
    uint8_t isCharging;     // 1 byte: Battery charging status (0 = Not Charging, 1 = Charging).
    float vbusVoltage;      // 4 bytes: Voltage measured on the VBUS input (e.g., USB 5V) (V).
    float systemVoltage;    // 4 bytes: Main regulated system voltage (V).

    // --- CRC Checksum (2 Bytes) ---
    uint16_t crc;           // 2 bytes: CRC-16 CCITT-FALSE checksum calculated over the *entire* preceding `CCSDSPacketBuffer` data (Header + Payload excluding CRC field itself). Received Big-Endian.
};
#pragma pack(pop) // Restore default packing.

// --- Structure for the complete CCSDS-like Packet Buffer ---
// Represents the entire structure expected within the RS-decoded message data.
// This structure is used to map the `decodedData` buffer after RS decoding.
#pragma pack(push, 1) // Ensure tight packing.
struct CCSDSPacketBuffer {
    // --- CCSDS Primary Header Fields (6 bytes total) ---
    // These fields are received in Big-Endian format.
    uint16_t version_type_sec_apid; // 2 bytes: Combined field: Packet Version (3b), Type (1b), Secondary Header Flag (1b), Application Process ID (APID) (11b).
    uint16_t sequence_flags_count;  // 2 bytes: Combined field: Sequence Flags (2b), Packet Sequence Count (14b).
    uint16_t packet_data_length;    // 2 bytes: Length of the packet data field (Payload size in bytes minus 1).

    // --- CCSDS Payload ---
    // Contains the actual telemetry data defined in the structure above.
    CCSDSPayload payload;           // Size depends on CCSDSPayload definition.
};
#pragma pack(pop) // Restore default packing.


//==============================================================================
// Global Variables - Program state and data storage accessible throughout the sketch.
//==============================================================================

// --- Radio & Communication ---
// Flag set by the radio's interrupt service routine (ISR) when a packet reception event occurs (e.g., preamble detected or full packet received).
// `volatile` keyword ensures the compiler doesn't optimize away accesses, as it can change unexpectedly outside the main program flow.
static volatile bool receivedFlag = false;

// --- Buffers ---
// Buffer to store the raw, potentially error-containing data received directly from the LoRa radio module.
// This data includes the Reed-Solomon ECC parity bytes. Size must match the full RS block length.
uint8_t encodedData[RS_BLOCK_LEN];
// Buffer to store the data *after* it has been processed by the Reed-Solomon decoder.
// This should contain the original message data (Header + Payload + CRC) if decoding is successful. Size must match the RS message length.
uint8_t decodedData[RS_MSG_LEN];

// --- Packet Data Storage ---
// Structure instance to hold the *latest valid* telemetry packet data *after* it has passed
// RS decoding, CRC check, and Endian conversion. This is the primary source for display/logging.
CCSDSPacketBuffer rxPacket;

// --- Statistics Counters ---
uint16_t validPackets = 0;          // Counter for packets that successfully passed both RS decoding and CRC validation.
uint16_t crcErrors = 0;             // Counter for packets that passed RS decoding but failed the CRC checksum validation (indicating data corruption).
uint16_t rsErrors = 0;              // Counter for packets where the Reed-Solomon decoding process failed (too many errors to correct).
uint16_t rangeErrors = 0;           // Counter for packets that passed CRC but contained data fields outside expected ranges (example validation).
uint16_t sizeMismatchErrors = 0;    // Counter for critical configuration errors where `sizeof(CCSDSPacketBuffer)` > `RS_MSG_LEN`.
float rssi = -120.0;                // Received Signal Strength Indicator (RSSI) in dBm of the last received packet. Initialized to a low value.
float snr = -20.0;                  // Signal-to-Noise Ratio (SNR) in dB of the last received packet. Initialized to a low value.

// --- Reed-Solomon Decoder ---
// Instance of the Reed-Solomon decoder class template from the RS-FEC library.
// Templated with the message length (data part) and ECC length (parity part).
RS::ReedSolomon<RS_MSG_LEN, RS_ECC_LEN> rs;

// --- Timing & Display Control ---
unsigned long lastDisplayUpdate = 0; // Timestamp (`millis()`) of the last time the display screen was changed. Used for screen cycling.
uint8_t currentScreen = 0;           // Index (0 to NUM_DISPLAY_SCREENS - 1) of the data screen currently being shown on the display.

// --- Timeout Tracking ---
unsigned long lastValidPacketTime = 0;// Timestamp (`millis()`) when the *last fully validated* (RS decoded, CRC OK) packet was received.
bool isSignalLost = true;            // Flag indicating whether the receiver currently considers the signal lost due to timeout. Starts `true` until the first valid packet arrives.

// --- Display Object ---
// The `u8g2` display object/pointer itself is declared/created globally in the Includes section,
// potentially relying on definitions from LoRaBoards.h. Initialization happens in `setup()`.

//==============================================================================
// Function Prototypes (Declarations) - Forward declaration of functions defined later in the file.
//==============================================================================
void setFlag(void);                     // ISR callback function for RadioLib packet reception events.
uint16_t calculateCRC(const uint8_t* data, size_t length); // Calculates CRC-16 CCITT-FALSE.
void resetRxPacket();                   // Clears the global `rxPacket` struct and resets signal indicators.
void processPacket();                   // Main handler for processing a newly received packet.
void convertPacketEndianness();         // Converts fields in `rxPacket` from Network to Host byte order.
void logPacket();                       // Prints the content of `rxPacket` to the Serial monitor.
void updateDisplay();                   // Checks timing and cycles the `currentScreen` index.
void drawDisplay();                     // Renders the content for the `currentScreen` onto the display buffer.
void displayFatalError(const char* msg);// Displays a critical error message and halts execution.
void setup();                           // Arduino standard setup function for initialization.
void loop();                            // Arduino standard main loop function.

//==============================================================================
// ISR Callback Function - Function executed upon hardware interrupt.
//==============================================================================
/**
 * @brief Interrupt Service Routine (ISR) callback triggered by RadioLib upon packet reception completion.
 * @warning ISRs should be extremely short and fast. Avoid complex calculations, delays,
 * or Serial prints within an ISR. Typically, just set a flag for the main loop to handle.
 */
void setFlag(void) {
    // Set the global flag to indicate a packet reception event needs processing in the main loop.
    receivedFlag = true;
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
 * and initial value (`CRC16_CCITT_FALSE_INIT`) used by the transmitter.
 */
uint16_t calculateCRC(const uint8_t* data, size_t length) {
    uint16_t crc = CRC16_CCITT_FALSE_INIT; // Start with the predefined initial value.
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8; // XOR the next data byte (shifted to the high byte of CRC) into the CRC register.
        for (uint8_t j = 0; j < 8; ++j) { // Perform 8 shifts/XORs for each bit of the data byte.
            // Check if the most significant bit (MSB) of the CRC register is set.
            // If it is, shift left by 1 and XOR with the polynomial.
            // Otherwise, just shift left by 1.
            crc = (crc & 0x8000) ? (crc << 1) ^ CRC16_CCITT_FALSE_POLY : (crc << 1);
        }
    }
    return crc; // Return the final calculated CRC value.
}

//==============================================================================
// Reset Received Packet Data Function - Clear stale/invalid data.
//==============================================================================
/**
 * @brief Resets the global `rxPacket` structure to all zeros/defaults and resets
 * signal quality indicators (RSSI, SNR).
 * @details This function is called when:
 * 1. A packet fails RS decoding.
 * 2. A packet fails the CRC check.
 * 3. The signal timeout period (`NO_SIGNAL_TIMEOUT_MS`) expires.
 * 4. During initial setup.
 * Clearing the packet ensures that stale or invalid data is not displayed or logged.
 */
void resetRxPacket() {
    // Use memset to efficiently fill the entire rxPacket structure with zero bytes.
    memset(&rxPacket, 0, sizeof(rxPacket));

    // Reset signal strength and quality indicators to default 'poor signal' or initial values.
    rssi = -120.0; // Reset RSSI to a low value.
    snr = -20.0;   // Reset SNR to a low value.

    // Note: All fields within rxPacket, including mission time, spacecraft ID, sensor readings, etc.,
    // will be zeroed or set to their default state (e.g., null characters for the ID string) by memset.
    // The display function should handle these zero/default values appropriately (e.g., show "---" or "Waiting").
}

//==============================================================================
// Setup Function - Initialization code executed once at startup.
//==============================================================================
/**
 * @brief Initializes hardware components (Serial, board specifics, display, radio),
 * libraries, sets up initial variable states, and starts LoRa reception mode.
 */
void setup() {
    // --- Initialize Serial Communication ---
    Serial.begin(115200); // Start serial port at 115200 baud for debugging and logging output.
    // // Optional: Uncomment to pause execution until Serial Monitor is opened. Useful for seeing early boot messages.
    // while (!Serial);
    // --- Initialize Board-Specific Hardware ---
    // Calls a function (expected to be defined in LoRaBoards.h or its associated .cpp file)
    // that handles microcontroller pin configurations, power settings, and potentially
    // initializes the 'u8g2' display object/pointer if required by the board.
    setupBoards();

    delay(500); // Short delay to allow hardware and power supplies to stabilize after initialization.

    Serial.println(F("--- Spacecraft Telemetry Receiver (Style OK + Timeout) v2.4.1 ---")); // Startup message

    // --- Initialize Packet State ---
    resetRxPacket();             // Ensure the packet buffer starts in a clean, zeroed state.
    lastValidPacketTime = millis(); // Initialize the timeout timer baseline relative to startup time.
    isSignalLost = true;         // Start assuming the signal is lost until the first *valid* packet arrives.

    // --- Initialize Display (if enabled) ---
    #ifdef USING_U8G2
    // Check if the u8g2 object/pointer seems valid (it might be initialized in setupBoards).
    // Using `u8g2->` implies u8g2 is likely treated as a pointer in this configuration.
    if (u8g2 /* && u8g2->getU8g2() */) { // Added basic check; getU8g2() might be safer if available/needed
        u8g2->begin(); // Initialize the display controller (sends configuration commands).
        // Set the default text font for the display, matching the Transmitter v2.1 style.
        u8g2->setFont(u8g2_font_6x10_mr);
        u8g2->clearBuffer(); // Clear the display's internal memory buffer.
        u8g2->setCursor(0, 10); // Set cursor position (x, y).
        u8g2->print(F("Receiver Booting...")); // Print booting message.
        u8g2->sendBuffer(); // Transfer the buffer content to the physical display.
    } else {
        // Log a warning if the display object appears uninitialized. Functionality might be limited.
        Serial.println(F("[WARNING] U8g2 display object not initialized or accessible!"));
    }
    #endif // USING_U8G2

    // --- Initialize LoRa Radio Module ---
    Serial.print(F("Initializing LoRa Radio (SX1262)... "));

    // Optional: Enable Temperature Controlled Crystal Oscillator (TCXO) if the board supports it
    // and RADIO_TCXO_ENABLE pin is defined in LoRaBoards.h. Improves frequency stability.
    #ifdef RADIO_TCXO_ENABLE
    pinMode(RADIO_TCXO_ENABLE, OUTPUT);    // Configure the TCXO control pin as output.
    digitalWrite(RADIO_TCXO_ENABLE, HIGH); // Set the pin HIGH to enable the TCXO (adjust logic if needed).
    delay(10); // Short delay to allow the TCXO to stabilize.
    #endif // RADIO_TCXO_ENABLE

    // Initialize the radio module using RadioLib. This performs hardware checks and basic setup.
    int radioState = radio.begin();
    if (radioState == RADIOLIB_ERR_NONE) {
        Serial.println(F("Success!")); // Initialization successful.
    } else {
        // Initialization failed. Print error code and halt.
        Serial.print(F("Failed, code ")); Serial.println(radioState);
        displayFatalError("RF SUBSYS FAIL"); // Display fatal error on screen and halt execution.
    }

    // --- Configure Radio Parameters ---
    // Set LoRa parameters. These MUST match the transmitter's settings exactly.
    radio.setFrequency(LORA_CARRIER_FREQ);         // Set carrier frequency (MHz).
    radio.setBandwidth(LORA_BANDWIDTH);           // Set bandwidth (kHz).
    radio.setSpreadingFactor(LORA_SPREADING_FACTOR);// Set spreading factor (7-12).
    radio.setCodingRate(LORA_CODING_RATE);         // Set coding rate (5-8).
    radio.setSyncWord(LORA_SYNC_WORD);             // Set sync word (network ID).
    radio.setCurrentLimit(LORA_CURRENT_LIMIT);     // Set SX126x PA current limit (mA).
    radio.setPreambleLength(LORA_PREAMBLE_LENGTH); // Set preamble length.

    // IMPORTANT: Disable RadioLib's built-in hardware CRC checking.
    // We are implementing our own software CRC check on the payload *after* RS decoding.
    // The CRC field is part of the data protected by RS FEC and the software CRC.
    radio.setCRC(false);

    // --- Setup Interrupt Handling ---
    // Configure RadioLib to call the `setFlag()` function automatically (via ISR)
    // whenever a complete LoRa packet has been successfully received by the radio hardware.
    radio.setPacketReceivedAction(setFlag);

    // --- Start LoRa Reception ---
    Serial.print(F("Starting LoRa reception mode... "));
    // Put the radio into continuous receive mode.
    int rxState = radio.startReceive();
    if (rxState == RADIOLIB_ERR_NONE) {
        Serial.println(F("OK. Waiting for packets.")); // Reception started successfully.
    } else {
        // Failed to enter receive mode. Print error code and halt.
        Serial.print(F("Failed, code ")); Serial.println(rxState);
        displayFatalError("NO CARRIER"); // Display fatal error and halt. (Using "NO CARRIER" as placeholder error)
    }

    Serial.println(F("Receiver Setup Complete."));

    // --- Final Setup Steps ---
    lastDisplayUpdate = millis(); // Initialize the display update timer.
    drawDisplay();                // Draw the initial display screen (will show zeros/defaults/waiting messages).
}

//==============================================================================
// Main Loop Function - Continuously executed after setup().
//==============================================================================
/**
 * @brief The main execution loop of the receiver. It continuously checks for
 * packet reception flags, handles signal timeouts, and updates the display.
 */
void loop() {

    // --- Handle Received Packet ---
    // Check if the ISR (`setFlag`) has indicated that a packet reception event occurred.
    if (receivedFlag) {
        receivedFlag = false; // Immediately reset the flag to be ready for the next interrupt.

        // Call the function to handle all steps of processing the received data.
        // This includes reading from radio, RS decoding, CRC check, endian conversion, logging, etc.
        processPacket();

        // --- Crucially, restart listening for the next packet ---
        // After processing (or attempting to process) the current packet,
        // immediately put the radio back into receive mode.
        int rxState = radio.startReceive();
        if (rxState != RADIOLIB_ERR_NONE) {
            // Log an error if restarting reception fails. This might indicate a radio hardware issue.
            Serial.print(F("[ERR] Failed to restart LoRa RX mode after packet processing, code: "));
            Serial.println(rxState);
            // Consider adding more robust error handling here, like attempting reset/reinit or halting.
        }
    } // End of receivedFlag handling

    // --- Check for Signal Timeout ---
    // Check if enough time has passed since the last *valid* packet was received.
    // The check `!isSignalLost` prevents this block from running repeatedly if the signal is already marked as lost.
    // We check `millis() - lastValidPacketTime` against the timeout threshold.
    if (!isSignalLost && (millis() - lastValidPacketTime > NO_SIGNAL_TIMEOUT_MS)) {
        Serial.println(F("[WARN] Signal Lost (Timeout threshold exceeded). Clearing stale data."));
        resetRxPacket();        // Clear the global packet buffer (`rxPacket`) and reset signal indicators.
        isSignalLost = true;    // Set the flag to indicate signal is currently considered lost. This prevents repeated resets until a new valid packet arrives.
    } // End of timeout check

    // --- Update Display Periodically ---
    updateDisplay(); // Check if it's time to switch to the next display screen and update `currentScreen`.
    drawDisplay();   // Redraw the display content based on the `currentScreen` and the current data in `rxPacket` (or zeros if signal is lost).

    // --- Yield (Optional) ---
    // A small delay can sometimes be beneficial in simple Arduino loops, especially if using FreeRTOS or
    // needing to ensure background tasks/other ISRs get processing time, but often unnecessary here.
    // delay(1);

} // End of loop()

//==============================================================================
// Update Display Screen Function - Manages screen cycling.
//==============================================================================
/**
 * @brief Checks if the display interval (`DISPLAY_SCREEN_INTERVAL_MS`) has elapsed
 * and increments the `currentScreen` index to cycle through the available
 * information screens. Wraps around to the first screen after the last one.
 */
void updateDisplay() {
    // Check if the time since the last display update exceeds the defined interval.
    if (millis() - lastDisplayUpdate >= DISPLAY_SCREEN_INTERVAL_MS) {
        // Increment the screen index, using the modulo operator (%) to wrap around
        // back to 0 after reaching the total number of screens.
        currentScreen = (currentScreen + 1) % NUM_DISPLAY_SCREENS;
        // Reset the timer for the next interval.
        lastDisplayUpdate = millis();
    }
}

//==============================================================================
// Process Received Packet Function - Core logic for handling incoming data.
//==============================================================================
/**
 * @brief Orchestrates the processing steps for a packet indicated by `receivedFlag`.
 * Steps include: reading raw data, checking signal quality, Reed-Solomon decoding,
 * validating packet structure size, performing CRC check, converting endianness
 * if valid, performing optional data range validation, and logging valid data.
 * Updates timeout tracking variables (`lastValidPacketTime`, `isSignalLost`) upon
 * successful validation (RS decode + CRC OK). Resets packet data on errors.
 */
void processPacket() {
    // --- 1. Get Packet Length from RadioLib ---
    // Ask RadioLib how many bytes were actually received in the last packet event.
    int receivedBytes = radio.getPacketLength();

    // --- 2. Basic Length Check ---
    // Verify if the received number of bytes matches the expected full Reed-Solomon block length.
    // If lengths don't match, the packet is malformed or incomplete; discard it silently.
    if (receivedBytes != RS_BLOCK_LEN) {
        // Optional: Log discarded packet due to length mismatch for debugging.
        // Serial.print(F("[WARN] RX length mismatch. Exp:")); Serial.print(RS_BLOCK_LEN);
        // Serial.print(F(", Got:")); Serial.println(receivedBytes);
        return; // Exit processing for this packet.
    }

    // --- 3. Read Raw Data into Buffer ---
    // Read the actual raw byte data from the radio's internal buffer into our `encodedData` buffer.
    int state = radio.readData(encodedData, RS_BLOCK_LEN);

    // --- 4. Check Radio Read Status and Signal Quality ---
    if (state == RADIOLIB_ERR_NONE) {
        // Successfully read data from the radio module.
        // Get signal quality metrics for the received packet.
        rssi = radio.getRSSI(); // Get Received Signal Strength Indicator (dBm).
        snr = radio.getSNR();   // Get Signal-to-Noise Ratio (dB).

        // Check if the signal strength meets the minimum required threshold.
        if (rssi <= RSSI_THRESHOLD) {
             // Signal is too weak; likely corrupted. Discard silently.
             // Optional: Could increment a 'weak signal discard' counter here.
            return; // Exit processing.
        }

        // --- 5. Attempt Reed-Solomon Decoding ---
        // Try to decode the raw `encodedData` (which includes message + ECC)
        // into the `decodedData` buffer (which should contain only the message part).
        // The `Decode` function returns the number of corrected errors, or -1 if decoding failed (too many errors).
        int corrections = rs.Decode(encodedData, decodedData);

        if (corrections >= 0) {
            // --- RS Decoding Succeeded (or had 0 errors) ---
            // `decodedData` now holds the potentially corrected message (Header + Payload + CRC).
            // Optional: Log number of corrected bytes: Serial.print(F("RS Corrected: ")); Serial.println(corrections);

            // --- 6. CRITICAL: Verify Packet Structure Size vs. Decoded Data Length ---
            // Ensure the defined C structure (`CCSDSPacketBuffer`) isn't larger than what RS decoding provides (`RS_MSG_LEN`).
            size_t expectedPacketSize = sizeof(CCSDSPacketBuffer);
            if (expectedPacketSize > RS_MSG_LEN) {
                // This is a FATAL CONFIGURATION ERROR. The transmitter is likely truncating data
                // because the structure being encoded is larger than the RS message capacity.
                if (sizeMismatchErrors == 0) { // Log this critical error only once to avoid spamming.
                    Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
                    Serial.println(F("[FATAL CONFIG ERR] sizeof(CCSDSPacketBuffer) > RS_MSG_LEN!"));
                    Serial.print(F("  Structure Size: ")); Serial.print(expectedPacketSize);
                    Serial.print(F(", RS Message Length: ")); Serial.println(RS_MSG_LEN);
                    Serial.println(F("  Transmitter is likely truncating data during RS encoding."));
                    Serial.println(F("  Receiver cannot correctly interpret or CRC check the full packet."));
                    Serial.println(F("  FIX: Reduce CCSDSPacketBuffer size or adjust RS_MSG_LEN on BOTH Tx and Rx."));
                    Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
                }
                sizeMismatchErrors++;
                // Even though RS decode succeeded, the data is fundamentally mismatched.
                // CRC check below will likely fail, but we must not proceed assuming the structure is valid.
                // We still proceed to memcpy and CRC check, which *should* fail if the CRC was part of the truncated data.
                // If by chance CRC passes, the data beyond RS_MSG_LEN is garbage.
                // Consider adding `resetRxPacket(); return;` here for immediate discard. Let CRC fail for now.
            }

            // --- 7. Copy Decoded Data into Packet Structure ---
            // Copy the bytes from the `decodedData` buffer into the `rxPacket` structure.
            // Use `min()` to prevent buffer overflow if the structure size *is* larger than RS_MSG_LEN
            // (though this indicates the fatal config error above). This copies only the available bytes.
            memcpy(&rxPacket, decodedData, min((size_t)RS_MSG_LEN, expectedPacketSize));

            // --- 8. Verify CRC Checksum ---
            // Calculate the CRC over the received data *before* the CRC field itself.
            size_t crcDataLength = expectedPacketSize - sizeof(rxPacket.payload.crc);

            // Handle the fatal size mismatch case: If the struct is too big, the CRC field itself
            // might not even be present in the `decodedData` buffer. We cannot reliably check it.
            if (expectedPacketSize > RS_MSG_LEN) {
                 if (RS_MSG_LEN <= crcDataLength) {
                    // We *might* have enough data to calculate *something*, but it's based on truncated data.
                    // It's safer to assume failure.
                     Serial.println(F("[ERR] Cannot reliably verify CRC: Packet structure > RS Msg Len. CRC check skipped, marking as error."));
                     crcErrors++;
                     resetRxPacket(); // Data is incomplete/corrupt due to config mismatch.
                     return;
                 } else {
                    // Not even enough decoded data to reach where the CRC *should* be.
                    Serial.println(F("[ERR] Cannot verify CRC: Packet structure > RS Msg Len AND CRC field likely truncated."));
                    crcErrors++;
                    resetRxPacket(); // Data is definitely incomplete.
                    return;
                 }
            }

            // Calculate CRC on the data portion (Header + Payload, excluding the received CRC field).
            // We cast `&rxPacket` to a byte pointer for the `calculateCRC` function.
            uint16_t calculatedCRC = calculateCRC(reinterpret_cast<uint8_t*>(&rxPacket), crcDataLength);

            // Extract the received CRC value from the packet structure.
            // IMPORTANT: The received CRC is still in Network Byte Order (Big-Endian) at this point.
            // Convert it to Host Byte Order *before* comparing with the calculated CRC (which is native Host order).
            uint16_t receivedCRC_native = ntohs_custom(rxPacket.payload.crc);

            // Compare calculated CRC with the received (and endian-converted) CRC.
            if (calculatedCRC == receivedCRC_native) {
                // --- CRC Match: Packet is considered valid! ---
                validPackets++; // Increment valid packet counter.

                // --- Update Timeout Tracking ---
                lastValidPacketTime = millis(); // Record the timestamp of this valid packet.
                isSignalLost = false;           // Reset the signal lost flag; we have good data.

                // --- 9. Convert Packet Fields Endianness ---
                // Now that the packet structure and integrity are validated, convert all
                // multi-byte fields from Network Byte Order (Big-Endian) to Host Byte Order.
                convertPacketEndianness();

                // --- 10. Data Validation (Optional Sanity Checks) ---
                // Perform basic checks on data values to see if they are within expected ranges.
                // This is an example; add more checks as needed for critical parameters.
                bool dataValid = true; // Assume valid initially.
                // Example: Check humidity reading. isnan() checks for "Not a Number".
                if (isnan(rxPacket.payload.hum_bme) || rxPacket.payload.hum_bme < -5.0f || rxPacket.payload.hum_bme > 110.0f) {
                    rangeErrors++; // Increment range error counter.
                    // Decide whether to invalidate the entire packet based on this range error.
                    // dataValid = false; // Uncomment to discard packets failing validation.
                    // Serial.println(F("[WARN] Humidity data out of range.")); // Optional log.
                }
                // Add more checks here for temperature, pressure, voltages, GPS coordinates, etc.

                // --- 11. Log Valid Packet Data ---
                if (dataValid) {
                    // If data passed optional validation (or if validation is skipped), log the packet contents.
                    logPacket();
                } else {
                    // Optional: Handle packets that failed validation checks.
                    // Serial.println(F("[WARN] Packet failed validation checks. Discarding/Logging as invalid."));
                    // logPacket(); // Still log it?
                    // resetRxPacket(); // Or discard the data entirely? Depends on requirements.
                }

            } else {
                // --- CRC Mismatch ---
                crcErrors++; // Increment CRC error counter.
                 // Optional: Log CRC failure details for debugging.
                // Serial.print(F("[ERR] CRC Mismatch! Calc: 0x")); Serial.print(calculatedCRC, HEX);
                // Serial.print(F(", Rcvd (BE): 0x")); Serial.print(rxPacket.payload.crc, HEX); // Show original Big-Endian
                // Serial.print(F(", Rcvd (Host): 0x")); Serial.println(receivedCRC_native, HEX); // Show Host Endian
                resetRxPacket(); // Discard the corrupted data by clearing the global packet buffer.
            } // End CRC check

        } else {
            // --- RS Decoding Failed ---
            rsErrors++; // Increment RS error counter.
            // Serial.println(F("[ERR] Reed-Solomon decoding failed (too many errors).")); // Optional log.
            resetRxPacket(); // Discard the unrecoverable data.
        } // End RS decode check

    } else {
        // --- Error Reading Data from Radio Module ---
        // `radio.readData()` returned an error. This might indicate an issue with SPI/I2C communication
        // or an internal radio module fault.
        Serial.print(F("[ERR] RadioLib readData() failed, code: ")); Serial.println(state);
        resetRxPacket(); // Clear any potentially partial/garbage data in the buffer.
    } // End radio read check
}


//==============================================================================
// Endian Conversion Function - Byte order correction.
//==============================================================================
/**
 * @brief Converts multi-byte fields within the globally stored `rxPacket` structure
 * from Network Byte Order (Big-Endian) to the microcontroller's native
 * Host Byte Order. This is called *only after* the packet has passed the
 * CRC check, ensuring we operate on validated data structure content.
 */
void convertPacketEndianness() {
    // --- CCSDS Primary Header Fields ---
    rxPacket.version_type_sec_apid = ntohs_custom(rxPacket.version_type_sec_apid); // uint16_t
    rxPacket.sequence_flags_count = ntohs_custom(rxPacket.sequence_flags_count);   // uint16_t
    rxPacket.packet_data_length = ntohs_custom(rxPacket.packet_data_length);     // uint16_t

    // --- Payload Fields ---
    // Metadata
    rxPacket.payload.missionTime.seconds = ntohl_custom(rxPacket.payload.missionTime.seconds);     // uint32_t
    rxPacket.payload.missionTime.subseconds = ntohs_custom(rxPacket.payload.missionTime.subseconds); // uint16_t
    // BME280
    rxPacket.payload.temp_bme = ntohf_custom(rxPacket.payload.temp_bme);         // float
    rxPacket.payload.pres_bme = ntohf_custom(rxPacket.payload.pres_bme);         // float
    rxPacket.payload.hum_bme = ntohf_custom(rxPacket.payload.hum_bme);           // float
    rxPacket.payload.alt_bme = ntohf_custom(rxPacket.payload.alt_bme);           // float
    // QMC6310
    rxPacket.payload.magX = ntohf_custom(rxPacket.payload.magX);                 // float
    rxPacket.payload.magY = ntohf_custom(rxPacket.payload.magY);                 // float
    rxPacket.payload.magZ = ntohf_custom(rxPacket.payload.magZ);                 // float
    rxPacket.payload.heading_deg = ntohf_custom(rxPacket.payload.heading_deg);   // float
    // QMI8658
    rxPacket.payload.accX = ntohf_custom(rxPacket.payload.accX);                 // float
    rxPacket.payload.accY = ntohf_custom(rxPacket.payload.accY);                 // float
    rxPacket.payload.accZ = ntohf_custom(rxPacket.payload.accZ);                 // float
    rxPacket.payload.gyrX = ntohf_custom(rxPacket.payload.gyrX);                 // float
    rxPacket.payload.gyrY = ntohf_custom(rxPacket.payload.gyrY);                 // float
    rxPacket.payload.gyrZ = ntohf_custom(rxPacket.payload.gyrZ);                 // float
    rxPacket.payload.temp_qmi = ntohf_custom(rxPacket.payload.temp_qmi);         // float
    // GPS
    rxPacket.payload.gps_lat = ntohd_custom(rxPacket.payload.gps_lat);           // double
    rxPacket.payload.gps_lon = ntohd_custom(rxPacket.payload.gps_lon);           // double
    rxPacket.payload.gps_alt = ntohf_custom(rxPacket.payload.gps_alt);           // float
    rxPacket.payload.gps_hdop = ntohf_custom(rxPacket.payload.gps_hdop);         // float
    rxPacket.payload.gps_date_year = ntohs_custom(rxPacket.payload.gps_date_year); // uint16_t
    // PMU
    rxPacket.payload.battVoltage = ntohf_custom(rxPacket.payload.battVoltage);   // float
    rxPacket.payload.vbusVoltage = ntohf_custom(rxPacket.payload.vbusVoltage);   // float
    rxPacket.payload.systemVoltage = ntohf_custom(rxPacket.payload.systemVoltage);// float

    // Note: Single-byte fields (like spacecraft_id chars, gps_fix_valid, gps date/time bytes, battPercent, isCharging)
    // do not require endian conversion.
    // Note: The CRC field (`rxPacket.payload.crc`) was already converted using ntohs_custom()
    // *before* the CRC check in `processPacket()`, so it's not converted again here.
}

//==============================================================================
// Log Received Packet Data to Serial Function - Debugging output.
//==============================================================================
/**
 * @brief Prints the contents of the globally stored, validated, and endian-converted
 * `rxPacket` structure to the Serial monitor in a structured, human-readable format.
 * Also includes the latest signal quality (RSSI, SNR) information.
 */
void logPacket() {
    // Extract Sequence Count (lower 14 bits) and APID (lower 11 bits) from header fields
    // (These fields are now in Host Byte Order after convertPacketEndianness()).
    uint16_t seqCount = rxPacket.sequence_flags_count & 0x3FFF; // Mask for 14 bits
    uint16_t apid = rxPacket.version_type_sec_apid & 0x07FF;    // Mask for 11 bits

    Serial.print("[TM] Pkt#"); Serial.print(seqCount); // Log packet sequence number
    Serial.print(" APID:0x"); Serial.print(apid, HEX); // Log Application Process ID in Hex

    // Safely handle the spacecraft ID string. Copy to a temporary buffer and ensure null termination
    // as the original might not be null-terminated in the fixed-size array.
    char scid_safe[sizeof(rxPacket.payload.spacecraft_id) + 1]; // Temp buffer +1 for null terminator
    strncpy(scid_safe, rxPacket.payload.spacecraft_id, sizeof(rxPacket.payload.spacecraft_id)); // Copy max N chars
    scid_safe[sizeof(rxPacket.payload.spacecraft_id)] = '\0'; // Ensure null termination
    Serial.print(" SCID:"); Serial.print(scid_safe); // Log spacecraft ID

    // Log Mission Elapsed Time
    Serial.print(" MET:"); Serial.print(rxPacket.payload.missionTime.seconds);
    Serial.print("."); Serial.print(rxPacket.payload.missionTime.subseconds);

    // Log BME280 Sensor Data
    Serial.print(" BME:");
    Serial.print(rxPacket.payload.temp_bme, 1); Serial.print("C,"); // Temp (1 decimal place)
    Serial.print(rxPacket.payload.hum_bme, 1); Serial.print("%,");  // Humidity (1 decimal place)
    Serial.print(rxPacket.payload.pres_bme, 1); Serial.print("hPa,");// Pressure (1 decimal place)
    Serial.print(rxPacket.payload.alt_bme, 1); Serial.print("m");    // Altitude (1 decimal place)

    // Log Magnetometer Data
    Serial.print(" MAG:");
    Serial.print(rxPacket.payload.magX, 1); Serial.print(","); // Mag X (1 dec pl)
    Serial.print(rxPacket.payload.magY, 1); Serial.print(","); // Mag Y (1 dec pl)
    Serial.print(rxPacket.payload.magZ, 1); Serial.print("uT,H:"); // Mag Z (1 dec pl)
    Serial.print(rxPacket.payload.heading_deg, 1); Serial.print("deg"); // Heading (1 dec pl)

    // Log IMU Accelerometer Data
    Serial.print(" ACC:");
    Serial.print(rxPacket.payload.accX, 3); Serial.print(","); // Accel X (3 dec pl)
    Serial.print(rxPacket.payload.accY, 3); Serial.print(","); // Accel Y (3 dec pl)
    Serial.print(rxPacket.payload.accZ, 3); Serial.print("g");   // Accel Z (3 dec pl)

    // Log IMU Gyroscope Data and IMU Temperature
    Serial.print(" GYR:");
    Serial.print(rxPacket.payload.gyrX, 3); Serial.print(","); // Gyro X (3 dec pl)
    Serial.print(rxPacket.payload.gyrY, 3); Serial.print(","); // Gyro Y (3 dec pl)
    Serial.print(rxPacket.payload.gyrZ, 3); Serial.print("dps"); // Gyro Z (3 dec pl)
    Serial.print(", T(IMU):"); Serial.print(rxPacket.payload.temp_qmi, 1); Serial.print("C"); // IMU Temp (1 dec pl)

    // Log GPS Data
    Serial.print(" GPS Fix("); Serial.print(rxPacket.payload.gps_fix_valid); Serial.print("):"); // Fix validity flags
    Serial.print(rxPacket.payload.gps_lat, 6); Serial.print(","); // Latitude (6 dec pl)
    Serial.print(rxPacket.payload.gps_lon, 6); Serial.print(","); // Longitude (6 dec pl)
    Serial.print(rxPacket.payload.gps_alt, 1); Serial.print("m, "); // Altitude (1 dec pl)
    // Log Date/Time only if the 'Time Valid' flag (bit 0) is set in gps_fix_valid
    if (rxPacket.payload.gps_fix_valid & 1) {
         Serial.print(rxPacket.payload.gps_date_year); Serial.print("-");
         Serial.print(rxPacket.payload.gps_date_month); Serial.print("-");
         Serial.print(rxPacket.payload.gps_date_day); Serial.print(" ");
         Serial.print(rxPacket.payload.gps_time_hour); Serial.print(":");
         Serial.print(rxPacket.payload.gps_time_minute); Serial.print(":");
         Serial.print(rxPacket.payload.gps_time_second); Serial.print("UTC, ");
    } else {
         Serial.print("NoTimeFix, "); // Indicate if time data is not valid
    }
    Serial.print(rxPacket.payload.gps_satellites); Serial.print("sats, "); // Number of satellites
    Serial.print(rxPacket.payload.gps_hdop, 1); Serial.print("hdop"); // HDOP (1 dec pl)

    // Log PMU/Battery Data
    Serial.print(" BATT:");
    Serial.print(rxPacket.payload.battVoltage, 2); Serial.print("V, "); // Battery Voltage (2 dec pl)
    Serial.print(rxPacket.payload.battPercent); Serial.print("%, Charge:"); // Battery Percentage
    Serial.print(rxPacket.payload.isCharging ? "Y" : "N"); // Charging status (Yes/No)

    // Log Power System Voltages
    Serial.print(" PWR:");
    Serial.print(rxPacket.payload.vbusVoltage, 2); Serial.print("V(VBUS), "); // VBUS Voltage (2 dec pl)
    Serial.print(rxPacket.payload.systemVoltage, 2); Serial.print("V(SYS)");  // System Voltage (2 dec pl)

    // Log Signal Quality Metrics from the last packet read
    Serial.print(" SIG:");
    Serial.print(rssi, 1); Serial.print("dBm, "); // RSSI (1 dec pl)
    Serial.print(snr, 1); Serial.print("dB");     // SNR (1 dec pl)

    Serial.println(); // End the log line
}


//==============================================================================
// Draw Display Function - Renders information to the screen.
//==============================================================================
/**
 * @brief Clears the display buffer and draws the content for the currently
 * selected screen (`currentScreen`) using data from the global `rxPacket`.
 * Handles displaying default/waiting messages if `isSignalLost` is true or
 * if GPS data is invalid. Finally, sends the buffer to the physical display.
 * @note Uses `u8g2->printf` for formatted output, which relies on `stdio.h`. Ensure
 * sufficient stack/memory is available for `printf` operations.
 * Accesses `u8g2` via the pointer (`->`) operator.
 */
void drawDisplay() {
    // --- Pre-checks ---
    // Check if the u8g2 object/pointer is valid before attempting to use it. Bail out if not.
    if (!u8g2 /* || !u8g2->getU8g2() */ ) return; // Basic check is usually sufficient if initialized properly.

    // --- Clear and Setup ---
    u8g2->clearBuffer();             // Clear the internal display buffer first.
    u8g2->setFont(u8g2_font_6x10_mr); // Ensure the correct (consistent) font is selected for this drawing pass.

    // --- Extract Sequence Count ---
    // Get sequence count from the (already endian-converted) rxPacket header field.
    // If signal is lost, rxPacket was reset, so seqCount will be 0.
    uint16_t seqCount = rxPacket.sequence_flags_count & 0x3FFF;

    // --- Optional: Visual Indicator for Lost Signal ---
    // Display "NO SIG" in the corner if the timeout has occurred.
    // Only show this message *after* at least one packet was successfully received previously (`validPackets > 0`).
    if (isSignalLost && validPackets > 0) {
        u8g2->setCursor(u8g2->getDisplayWidth() - 40, 10); // Position near top right. Adjust as needed.
        u8g2->print(F("NO SIG")); // Display the indicator text.
    }

    // --- Draw Content Based on Current Screen ---
    // Use a switch statement to render the specific content for the `currentScreen` index.
    switch (currentScreen) {
        case 0: // Screen 0: Environmental Sensors (BME280)
            u8g2->setCursor(2, 10); u8g2->print(F("Env. Sensors")); // Title
            // Display Temp, Humidity, Pressure, Altitude using printf formatting.
            u8g2->setCursor(2, 22); u8g2->printf("T:%.1fC H:%.1f%%", rxPacket.payload.temp_bme, rxPacket.payload.hum_bme);
            u8g2->setCursor(2, 34); u8g2->printf("P:%.1fhPa", rxPacket.payload.pres_bme);
            u8g2->setCursor(2, 46); u8g2->printf("A:%.1fm", rxPacket.payload.alt_bme);
            u8g2->setCursor(2, 58); u8g2->printf("Pkt:%u", seqCount); // Show last packet number
            break;

        case 1: // Screen 1: Magnetometer (QMC6310)
            u8g2->setCursor(2, 10); u8g2->print(F("Magnetometer")); // Title
            // Display Mag X, Y, Z, and Heading.
            u8g2->setCursor(2, 22); u8g2->printf("X: %.1f Y: %.1f", rxPacket.payload.magX, rxPacket.payload.magY);
            u8g2->setCursor(2, 34); u8g2->printf("Z: %.1f uT", rxPacket.payload.magZ);
            u8g2->setCursor(2, 46); u8g2->printf("Head: %.1f deg", rxPacket.payload.heading_deg);
            u8g2->setCursor(2, 58); u8g2->printf("Pkt:%u", seqCount);
            break;

        case 2: // Screen 2: IMU Accelerometer (QMI8658)
            u8g2->setCursor(2, 10); u8g2->print(F("IMU - Accel (g)")); // Title
            // Display Accel X, Y, Z.
            u8g2->setCursor(2, 22); u8g2->printf("X: %.3f", rxPacket.payload.accX);
            u8g2->setCursor(2, 34); u8g2->printf("Y: %.3f", rxPacket.payload.accY);
            u8g2->setCursor(2, 46); u8g2->printf("Z: %.3f", rxPacket.payload.accZ);
            u8g2->setCursor(2, 58); u8g2->printf("Pkt:%u", seqCount);
            break;

        case 3: // Screen 3: IMU Gyroscope & Temperature (QMI8658)
            u8g2->setCursor(2, 10); u8g2->print(F("IMU - Gyro(dps)/T")); // Title
            // Display Gyro X, Y, Z, and IMU temperature.
            u8g2->setCursor(2, 22); u8g2->printf("X: %.2f", rxPacket.payload.gyrX);
            u8g2->setCursor(2, 34); u8g2->printf("Y: %.2f", rxPacket.payload.gyrY);
            u8g2->setCursor(2, 46); u8g2->printf("Z: %.2f", rxPacket.payload.gyrZ);
            u8g2->setCursor(2, 58); u8g2->printf("T:%.1fC Pkt:%u", rxPacket.payload.temp_qmi, seqCount);
            break;

        case 4: // Screen 4: GPS Time/Date
            u8g2->setCursor(2, 10); u8g2->print(F("GPS Date & Time")); // Title
            // Check BOTH signal status AND the GPS time valid flag (bit 0) before displaying date/time.
            if (!isSignalLost && (rxPacket.payload.gps_fix_valid & 1)) {
                // Display formatted date and time. Use leading zeros for month, day, h, m, s.
                u8g2->setCursor(2, 22); u8g2->printf("%04d-%02d-%02d", rxPacket.payload.gps_date_year, rxPacket.payload.gps_date_month, rxPacket.payload.gps_date_day);
                u8g2->setCursor(2, 34); u8g2->printf("%02d:%02d:%02d UTC", rxPacket.payload.gps_time_hour, rxPacket.payload.gps_time_minute, rxPacket.payload.gps_time_second);
            } else {
                // If signal lost or time fix not valid, show an appropriate status message.
                 u8g2->setCursor(2, 30); u8g2->print(isSignalLost ? F("-- Signal Lost --") : F("Waiting for Time Fix"));
            }
            // Display satellite count regardless of fix status (if packet received).
            u8g2->setCursor(2, 46); u8g2->printf("Sats: %d", rxPacket.payload.gps_satellites);
            u8g2->setCursor(2, 58); u8g2->printf("Fix:%d Pkt:%u", rxPacket.payload.gps_fix_valid, seqCount); // Show fix flags value
            break;

       case 5: // Screen 5: GPS Location
            u8g2->setCursor(2, 10); u8g2->print(F("GPS Location")); // Title
            // Check BOTH signal status AND the GPS location valid flag (bit 1) before displaying Lat/Lon/Alt.
            if (!isSignalLost && (rxPacket.payload.gps_fix_valid & 2)) {
                // Display formatted Lat, Lon, Alt, HDOP.
                u8g2->setCursor(2, 22); u8g2->printf("Lat: %.4f", rxPacket.payload.gps_lat); // 4 decimal places for Lat/Lon
                u8g2->setCursor(2, 34); u8g2->printf("Lon: %.4f", rxPacket.payload.gps_lon);
                u8g2->setCursor(2, 46); u8g2->printf("Alt: %.1fm HDOP:%.1f", rxPacket.payload.gps_alt, rxPacket.payload.gps_hdop);
            } else {
                 // If signal lost or location fix not valid, show status message.
                 u8g2->setCursor(2, 30); u8g2->print(isSignalLost ? F("-- Signal Lost --") : F("Waiting for Loc Fix"));
                 // Show HDOP even without a location fix, as it might still be reported.
                 u8g2->setCursor(2, 46); u8g2->printf("HDOP: %.1f", rxPacket.payload.gps_hdop);
            }
            u8g2->setCursor(2, 58); u8g2->printf("Fix:%d Pkt:%u", rxPacket.payload.gps_fix_valid, seqCount); // Show fix flags value
            break;

       case 6: // Screen 6: Power Management Unit (PMU) - Battery Status
            u8g2->setCursor(2, 10); u8g2->print(F("PMU Status 1")); // Title
            // Display Battery Voltage, Percentage, Charging Status.
            u8g2->setCursor(2, 22); u8g2->printf("Batt: %.2fV", rxPacket.payload.battVoltage);
            u8g2->setCursor(2, 34); u8g2->printf("Percent: %d%%", rxPacket.payload.battPercent);
            // Show charging status as YES/NO, or "---" if signal is lost (as status might be unknown).
            u8g2->setCursor(2, 46); u8g2->printf("Charging: %s", rxPacket.payload.isCharging ? "YES" : (isSignalLost ? "---" : "NO"));
            u8g2->setCursor(2, 58); u8g2->printf("Pkt:%u", seqCount);
            break;

        case 7: // Screen 7: PMU - Voltages & Mission Time
            u8g2->setCursor(2, 10); u8g2->print(F("PMU Status 2 / MET")); // Title
            // Display VBUS voltage, System voltage, and Mission Elapsed Time.
            u8g2->setCursor(2, 22); u8g2->printf("VBUS: %.2fV", rxPacket.payload.vbusVoltage);
            u8g2->setCursor(2, 34); u8g2->printf("System: %.2fV", rxPacket.payload.systemVoltage);
            // Display Mission Time (seconds.subseconds). Note: %lu for unsigned long seconds.
            u8g2->setCursor(2, 46); u8g2->printf("MET: %lu.%us", rxPacket.payload.missionTime.seconds, rxPacket.payload.missionTime.subseconds);
            u8g2->setCursor(2, 58); u8g2->printf("Pkt:%u", seqCount);
            break;

        case 8: // Screen 8: RF Signal Quality & Status Counters
            u8g2->setCursor(2, 10);  u8g2->print(F("RF & Status")); // Title
            // Display last received RSSI and SNR. Note: These are reset to defaults by resetRxPacket() if signal is lost.
            u8g2->setCursor(2, 22);  u8g2->printf("RSSI:%.1f SNR:%.1f", rssi, snr);
            // Display packet counters: Valid (OK), CRC Errors, RS Errors.
            u8g2->setCursor(2, 34);  u8g2->printf("OK:%u CRC:%u RS:%u", validPackets, crcErrors, rsErrors);
            // Display other error counters: Range validation errors, Size mismatch errors.
            u8g2->setCursor(2, 46);  u8g2->printf("Rng:%u Size:%u", rangeErrors, sizeMismatchErrors);
            // Display the sequence number of the last *processed* packet (will be 0 if signal currently lost).
            u8g2->setCursor(2, 58); u8g2->printf("Last Pkt:%u", seqCount);
            break;

        default: // Fallback case for unexpected screen index
            u8g2->setCursor(2, 10); u8g2->print(F("Unknown Screen"));
            // Log an error if this happens, indicates a bug in screen cycling logic.
            // Serial.print(F("[ERR] Invalid currentScreen index: ")); Serial.println(currentScreen);
            currentScreen = 0; // Reset to screen 0 to recover.
            break;
    } // End switch(currentScreen)

    // --- Send Buffer to Display ---
    // After drawing all elements for the current screen into the buffer,
    // transfer the entire buffer to the physical display hardware.
    u8g2->sendBuffer();
}

//==============================================================================
// Display Fatal Error Function - Halts execution on critical failure.
//==============================================================================
/**
 * @brief Handles critical, unrecoverable errors. Logs the error message to Serial,
 * displays a "SYSTEM HALTED" message on the screen (if available), and
 * enters an infinite loop (optionally blinking an LED) to stop further execution.
 * @param msg A C-string containing the error message to be displayed.
 * @note Matches the style and behavior of the Transmitter's fatal error handler for consistency.
 */
void displayFatalError(const char* msg) {
    // --- Log to Serial Monitor ---
    // Print the error message along with a timestamp to the Serial port.
    Serial.printf("[FATAL][%lu] %s\n", millis(), msg);

    // --- Display on Screen (if available) ---
    // Check if the u8g2 object/pointer is valid before attempting to use it.
    if (u8g2 /* && u8g2->getU8g2() */ ) {
        u8g2->clearBuffer(); // Clear any previous content.

        // Use a larger font for the main "SYSTEM HALTED" title.
        u8g2->setFont(u8g2_font_ncenB10_tr); // Example bold font
        u8g2->drawFrame(0, 0, u8g2->getDisplayWidth(), u8g2->getDisplayHeight()); // Draw a border frame.
        u8g2->setCursor(5, 15); // Position cursor for title.
        u8g2->print(F("SYSTEM HALTED")); // Print the title.

        // Switch to a smaller font for the detailed error message.
        u8g2->setFont(u8g2_font_5x7_tr); // Example small font
        uint8_t maxLineChars = u8g2->getDisplayWidth() / 6; // Estimate chars per line based on font width (approx 6px)
        uint8_t currentLine = 3; // Start printing message lines below the title.
        const char* start = msg; // Pointer to the beginning of the message string.
        const char* end = msg + strlen(msg); // Pointer to the end of the message string.

        // --- Word Wrapping Logic ---
        // Loop to print the message, wrapping text onto multiple lines if necessary.
        while (start < end && currentLine < 8) { // Limit lines to fit display height.
            const char* line_end = start + maxLineChars; // Calculate potential end of the line.
            if (line_end > end) line_end = end; // Don't go past the actual end of the string.

            const char* wrap_point = line_end; // Start searching for wrap point from potential end.
            // If not at the end of the string, try to find a space to wrap at.
            if (line_end < end) {
                while (wrap_point > start && *wrap_point != ' ' && *wrap_point != '\n') { // Scan backwards for space or newline
                    wrap_point--;
                }
                // If no space found in the segment, force break at maxLineChars (prevents infinite loop on long words).
                if (wrap_point == start) {
                   wrap_point = line_end;
                }
                 // If we found a space/newline, use that as the end of the current line.
                else {
                    line_end = wrap_point;
                }
            }

            // --- Print the Line ---
            // Copy the segment for the current line into a temporary buffer.
            char lineBuf[maxLineChars + 1];
            size_t len = line_end - start;
            if (len > maxLineChars) len = maxLineChars; // Clamp length just in case.
            strncpy(lineBuf, start, len);
            lineBuf[len] = '\0'; // Null-terminate the line buffer.

            // Set cursor and print the line.
            u8g2->setCursor(5, currentLine * 8 + 5); // Position Y based on line number (adjust spacing if needed)
            u8g2->print(lineBuf);

            // --- Advance Pointers ---
            start = line_end; // Move start pointer to the end of the printed segment.
            // Skip any leading spaces at the beginning of the next line.
            while(start < end && (*start == ' ' || *start == '\n')) {
                 start++;
            }
            currentLine++; // Move to the next display line.
        } // End while loop (word wrapping)

        u8g2->sendBuffer(); // Send the complete fatal error message buffer to the display.
    } // End if (u8g2)

    // --- Halt Execution ---
    // Enter an infinite loop to stop the program.
    #ifdef LED_BUILTIN
    // If a built-in LED is defined for the board, blink it as a visual indicator of the halted state.
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

