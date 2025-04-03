

#define T_BEAM_S3_SUPREME_SX1262
#ifndef T_BEAM_S3_SUPREME
#define T_BEAM_S3_SUPREME
#endif
#ifndef USING_SX1262
#define USING_SX1262
#endif
#define I2C_SDA                     (17)
#define I2C_SCL                     (18)
#define I2C1_SDA                    (42)
#define I2C1_SCL                    (41)
#define PMU_IRQ                     (40)
#define GPS_RX_PIN                  (9)
#define GPS_TX_PIN                  (8)
#define GPS_WAKEUP_PIN              (7)
#define GPS_PPS_PIN                 (6)
#define BUTTON_PIN                  (0)
#define BUTTON_PIN_MASK             (GPIO_SEL_0)
#define BUTTON_COUNT                (1)
#define BUTTON_ARRAY                {BUTTON_PIN}
#define RADIO_SCLK_PIN              (12)
#define RADIO_MISO_PIN              (13)
#define RADIO_MOSI_PIN              (11)
#define RADIO_CS_PIN                (10)
#define RADIO_RST_PIN               (5)
#define RADIO_DIO1_PIN              (1)
#define RADIO_BUSY_PIN              (4)
#define SPI_MOSI                    (35)
#define SPI_SCK                     (36)
#define SPI_MISO                    (37)
#define SPI_CS                      (47)
#define IMU_CS                      (34)
#define IMU_INT                     (33)
#define SDCARD_MOSI                 SPI_MOSI
#define SDCARD_MISO                 SPI_MISO
#define SDCARD_SCLK                 SPI_SCK
#define SDCARD_CS                   SPI_CS
#define RTC_INT                     (14)
#define GPS_BAUD_RATE               (9600)
#define HAS_SDCARD
#define HAS_GPS
#define HAS_DISPLAY
#define HAS_PMU
#define __HAS_SPI1__
#define HAS_SENSOR
#define PMU_WIRE_PORT               Wire1
#define DISPLAY_MODEL               U8G2_SH1106_128X64_NONAME_F_HW_I2C
#define DISPLAY_MODEL_SSD_LIB       SH1106Wire
#define BOARD_VARIANT_NAME          "T-Beam S3"
