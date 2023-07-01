#ifndef __AQW_H__
#define __AQW_H__

#define VERSION "v1.0"
#define BANNER "Particle Argon/Air Quality Wing -- " VERSION


#define SPI_INTERFACES_COUNT 1
#include "AirQualityWing.h"
#include "MQTT.h"
//#include <SPI.h>
#include <Wire.h>
#undef ARDUINO
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TelnetLogger.h>

//
// AQW Hardware related def's
//
#define I2C_SDA_PIN     D0
#define I2C_SCL_PIN     D1
#define HPMA1150_EN_PIN D5
#define CCS811_ADDRESS  0x5a
#define SSD1306_ADDRESS 0x3c
// I2C clock speed (used by AirQualityWing HW)
#define I2C_CLK_SPEED   100000

//
// OLED FeatherWing buttons map to different
// pins depending on board:
//
#if defined(PARTICLE)
  #define BUTTON_A D4
  #define BUTTON_B D3
  #define BUTTON_C D2
#elif defined(ESP8266)
  #define BUTTON_A  0
  #define BUTTON_B 16
  #define BUTTON_C  2
#elif defined(ESP32)
  #define BUTTON_A 15
  #define BUTTON_B 32
  #define BUTTON_C 14
#elif defined(ARDUINO_STM32_FEATHER)
  #define BUTTON_A PA15
  #define BUTTON_B PC7
  #define BUTTON_C PC5
#elif defined(TEENSYDUINO)
  #define BUTTON_A  4
  #define BUTTON_B  3
  #define BUTTON_C  8
#elif defined(ARDUINO_FEATHER52832)
  #define BUTTON_A 31
  #define BUTTON_B 30
  #define BUTTON_C 27
#else // 32u4, M0, M4, nrf52840 and 328p
  #define BUTTON_A  9
  #define BUTTON_B  6
  #define BUTTON_C  5
#endif


/////////////////////////////////////////////////////////////////////////
//
//  Timer queue support
//
/////////////////////////////////////////////////////////////////////////

/*
 * Display is 21 x 4 for font size 1
 * Display is 10 x 2 for font size 2
 * Display is  7 x 1 for font size 3
 */

typedef enum
{
  TQ_EMPTY = 0,
  TQ_CLEAR,
  TQ_SHOW
} TQCMD;

typedef struct
{
  TQCMD cmd;
  unsigned long timeout;
  unsigned long expiration;
  unsigned short textSize;
  char text[84];    // Display is 21 x 4 (cols x rows) ... see above
} TQENTRY;


/////////////////////////////////////////////////////////////////////////
//
//  Error Stats struct
//
/////////////////////////////////////////////////////////////////////////
//
// Track error statistics
//
typedef struct {
  unsigned int hardware;
  unsigned int checksum;
  unsigned int data_range;
  unsigned int mqtt;
  unsigned int timeout;
  unsigned int wifi;
  unsigned int internal;
} ERRORS;

/////////////////////////////////////////////////////////////////////////
//
//  EEPROM support
//
/////////////////////////////////////////////////////////////////////////
typedef struct {
  char node_name[16];
  char ssid[16];
  char psk[16];
  char mqtt_broker[40];
  char mqtt_user[8];
  char mqtt_pass[8];
} EEPROM_DATA;


// Cloud function for setting interval
int set_interval (String period);

// Forward decl for timer queue
void tqEmptyQueue();
void tqRemove();
void checkTimeQueue();

// Forward declaration of event handler
void AirQualityWingEvent();
void tqAdd(TQCMD cmd, unsigned long timeout, const char *text, unsigned short textSize, bool immediate);

#endif