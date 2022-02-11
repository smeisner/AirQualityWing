/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021,2022 Steve Meisner (steve@meisners.net)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/* 
 * Title: Air Quality Wing with HUZZAH32 uController
 * Author: Steve Meisner (plus many others; See docs below)
 * 
 * Description
 * 
 * The HUZZAH32 board is one of the few that has the Fetaher footprint and
 * supplies onboard wifi. The wifi allows for MQTT publishing to a broker.
 * 
 * The Adafruit OLED Feather Wing also provides a convienent display for the AQW.
 * This display is used to display air quality info and some minimal statisitcs.
 * The display is not required for operation and is dynamically detected
 * during setup().
 *
 * A telnet client is also embedded in the code to provide status and diagnostic
 * info.
 * 
 * See the README.md file for additional details.
 *
 * I2C Addresses:
 *  - SHTC3 0x70
 *  - SGP40 0x59
 *  - OLED  0x3c
 *  
 */

// Next line enables code to support the OLED Feather Wing
#define AQW_OLED

#include <WiFi.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include "uptime_formatter.h"
#include "Adafruit_SGP40.h"
#include "Adafruit_SHTC3.h"
#ifdef AQW_OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif
// OTA
#include "OTA.h"
#include <TelnetStream.h>
#include <Logger.h>
// Secret credentials
#include "arduino_secrets.h"

// Network credentials (set in arduino_secrets.h)
const char ssid[] = SECRET_SSID;
const char password[] = SECRET_PASS;
const char mqtt_server[] = SECRET_MQTT_BROKER;

#define VERSION "v2.0"
#define BANNER "HUZZAH32 Feather/Air Quality Wing -- " VERSION

// Global device structs
WiFiClient aqwClient;
PubSubClient mqttClient(aqwClient);
Adafruit_SGP40 sgp;
Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();
HardwareSerial HPMA115S0(1);  // Use the builtin UART

#ifdef AQW_OLED
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

// OLED FeatherWing buttons map to different pins depending on board:
#if defined(ESP8266)
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
  char text[84];    // Display is 21 x 4 (cols x rows)
} TQENTRY;

/*
 * Display is 21 x 4 for font size 1
 * Display is 10 x 2 for font size 2
 * Display is  7 x 1 for font size 3
 */

#define TQSIZE 4
TQENTRY PROGMEM tq[TQSIZE];

unsigned short PROGMEM tqTail;
unsigned short PROGMEM tqHead;

bool PROGMEM AqwOled = false;
#endif // #ifdef AQW_OLED

#define INTERVAL 60           // Interval to take readings (seconds)
#define LOGMSG_LEN 92         // Size of log message scratch buffer
char PROGMEM logMsg[LOGMSG_LEN];      // General use buffer for log messages
#define MQTT_PUB_LEN 32       // Max size of an MQTT payload
#define MQTT_TOPIC_LEN 40     // Max size of MQTT topic, including hostname
char PROGMEM mqtt_pub[MQTT_PUB_LEN];  // Buffer for MQTT msg published
char PROGMEM mqtt_topic[MQTT_TOPIC_LEN];

#define SGP_SERIAL_LEN 16
char PROGMEM SgpSerial[SGP_SERIAL_LEN];

//
// Battery support definitions
//
const PROGMEM int MAX_ANALOG_VAL = 4095;
const PROGMEM float MAX_BATTERY_VOLTAGE = 4.2; // Max LiPoly voltage of a 3.7 battery is 4.2
float PROGMEM batteryVoltage = 0.0;
float PROGMEM batteryPercentage = 0.0;

/*
 *
 * ESP32     HUZZAH     AQW Schematic (Particle board)
 *   13     A12 / D13      D8      //
 *   12     A11 / D12      D7      //
 *   27     A10 / D27      D6      //
 *   33      A9 / D33      D5      // HPMA 5V power enable, CS811 RESET (active low)
 *   15      A8 / D15      D4      // Button 1, Sensors VCC
 *   32      A7 / D32      D3      // Button 3
 *   14      A6 / D14      D2      // Button 2
 *
*/

//
// Air Quality Wing hardware related details
//
#define AQW_5V_EN_PIN A9     // Enable pin for builtin AQW power supply (which provides 5V)
// HPM Sensor definitions
#define RXD2 16              // Pin assigments for onboard UART
#define TXD2 17
//#define CS811_WAKE_PIN A8    // Wake pin for CS811 sensor
#define MAX_AQW_WAIT 5       // Maximum time to wait for HPM sensor to respond

//
// Most recent readings...
// and variable to manipulate data
//
int32_t PROGMEM voc_index;
sensors_event_t PROGMEM humidity;
sensors_event_t PROGMEM temp;
int PROGMEM pm25;
int PROGMEM pm10;

//
// Historical Data tracking used to validqate reading received.
// This will filter out erroneous readings.
//
typedef enum {
  PM25,
  PM10,
  TEMPERATURE,
  HUMIDITY,
  VOC_INDEX
} DataType;

// Keeping the last 2 readings to average
typedef struct {
  unsigned long pm25[2];
  unsigned long pm10[2];
  unsigned long temperature[2]; // Converted to int to simplify code
  unsigned long humidity[2];    //      "         "       "
  unsigned long voc_index[2];
} HISTORICAL_DATA;

HISTORICAL_DATA PROGMEM HistoricalData;
int PROGMEM ValidationFactor = 20;

//
// Support for event log buffer recall
//
#define RECALL_BUFFER_LEN 1020
int PROGMEM RecallPosition = 0;
// Allocate 4 extra bytes to allow for a safety buffer
char PROGMEM RecallBuffer[RECALL_BUFFER_LEN+4];


//////////////////////////////////////////////////
///                                            ///
///          Arduino Code Start                ///
///                                            ///
//////////////////////////////////////////////////

void setup()
{
  // Turn on LED during setup()
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(100);

  // Toggle LED to show a pattern in case the OTA/wifi setup doesn't work
  digitalWrite(BUILTIN_LED, LOW);
  setupOTA(AQW_HOSTNAME, ssid, password);
  digitalWrite(BUILTIN_LED, HIGH);
  TelnetStream.begin();

  // Enable all logging during startup
  Logger::setLogLevel(Logger::VERBOSE);
  Logger::setOutputFunction(localLogger);

  // Delay to allow connection via Telnet to get started
  delay(5000);

  Logger::notice(BANNER);

#ifdef AQW_OLED
  if (AqwOled = OledPresent(0x3c))
  {
    Logger::notice("OLED dislay found at address 0x3c");

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32

    // Show image buffer on the display hardware.
    // Since the buffer is intialized with an Adafruit splashscreen
    // internally, this will display the splashscreen.
    display.display();
    delay(750);

    // Clear the buffer.
//   display.clearDisplay();
//   display.display();

    pinMode(BUTTON_A, INPUT_PULLUP);
    pinMode(BUTTON_B, INPUT_PULLUP);
    pinMode(BUTTON_C, INPUT_PULLUP);

    tqInit();
    tqAdd(TQ_SHOW, 2500, BANNER, 1, false);
    checkTimeQueue();
  }
  else
  {
    Logger::notice("** No OLED dislay found **");
  }

#endif

  IPAddress ip = WiFi.localIP();
  Logger::notice("Connected to WiFi network. Connect with Telnet client to: ");
  snprintf (logMsg, LOGMSG_LEN, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  Logger::notice(logMsg);

  Logger::verbose("Connecting to MQTT broker");
  mqttClient.setServer(mqtt_server, 1883);

  // Read LiPo battery values
  updateBatteryVoltage();

  // Clear all historical data before any readings are taken
  memset (&HistoricalData, sizeof(HISTORICAL_DATA), 0);

  Logger::verbose("Enabling AQW 5V supply...");
  pinMode(AQW_5V_EN_PIN, OUTPUT);
  digitalWrite(AQW_5V_EN_PIN, HIGH);

//  Logger::verbose("Waking CS811 sensor...");
//  pinMode(CS811_WAKE_PIN, OUTPUT);
//  digitalWrite(CS811_WAKE_PIN, HIGH);

  Logger::verbose("Connecting to i2C devices");
  sgp40_Start();
  shtc3_Start();
  // Do an initial reading to populate the validation database.
  sgp40_Read(true);

  Logger::verbose("Connecting to HPM sensor");
  hpma_Start();
  // Do an initial reading to populate the validation database.
  hpma_Read(true);

  disconnectMqtt();

  // Turn off AQW 5V supply
//  digitalWrite(CS811_WAKE_PIN, LOW);
  digitalWrite(AQW_5V_EN_PIN, LOW);

  Logger::notice("Setup() done!");
  digitalWrite(BUILTIN_LED, LOW);

// Test code
//  test(Logger::FATAL, "Module", "This is a test twelve = %d --> \\n\n", 12);
//  test(Logger::WARNING, "TestModule", "Warning...no args...new line\n");
//  test(Logger::VERBOSE, "", "No module... verbose. Value of BUILTIN_LED = %d <no eol>", BUILTIN_LED);
//  test(Logger::NOTICE, "WIFI", "wifi ssid = %s psk = %s<eol>\n", ssid, password);


  Logger::notice("Setting loglevel to ERROR");
  Logger::notice("---------------------------------------------");
  Logger::setLogLevel(Logger::ERROR);
}

void loop()
{
  long now = millis();        // Grab timestamp on every entry
  static long lastMsg = 0;    // Timestamp of last loop() pass

  messagePump();

  // Publish data every INTERVAL seconds
  if (now - lastMsg > INTERVAL*1000)
  {
    Logger::notice("-------   Starting sensor read cycle  -------");
    // Turn on LED to indicate we're taking a reading
    digitalWrite(BUILTIN_LED, HIGH);
    // Turn on AQW 5V supply & wake CS811
    digitalWrite(AQW_5V_EN_PIN, HIGH);
//    digitalWrite(CS811_WAKE_PIN, HIGH);
    // Delay 6s to let the HPMA115 wake up
    //delay(6000);
    for (int n=0; n < 600; n++) { if ((n % 10) == 0) messagePump(); delay (10); }
    // Bring up MQTT broker connection
    connectMqtt();
    // Save the timestamp
    lastMsg = now;
    // Receive the particle data
    hpma_Read(false);
    // Read temp, humidity and TVOC/CO2
    sgp40_Read(false);
    // Read LiPo battery values
    updateBatteryVoltage();
    // Drop MQTT connection
    disconnectMqtt();
    // Turn off AQW 5V supply
//    digitalWrite(CS811_WAKE_PIN, LOW);
    digitalWrite(AQW_5V_EN_PIN, LOW);
    // Done...turn off the LED
    digitalWrite(BUILTIN_LED, LOW);
    Logger::notice("----------------  Sleeping  -----------------");
  }
}

boolean OledPresent(short address)
{
  Wire.begin();
  Wire.beginTransmission (address);
  if (Wire.endTransmission() == 0)
    return true;
  else
    return false;
}

//////////////////////////////////////////////////
///                                            ///
///               Logging                      ///
///                                            ///
//////////////////////////////////////////////////

/*
 * Move on to the next log level and return the string to indicate new level.
 * Note of the available log levels, Silent is skipped.
 * 
 * From logging header file:
 *    VERBOSE = 0,
 *    NOTICE,
 *    WARNING,
 *    ERROR,
 *    FATAL,
 *    SILENT
 *    
 *  If no valid log level is set, we default to Notice level.
 * 
 */
char *cycleLogLevel()
{
  Logger::Level logLevel = Logger::getLogLevel();
  switch (logLevel)
  {
    case Logger::VERBOSE: Logger::setLogLevel(Logger::NOTICE); return (char *)"Notice"; break;
    case Logger::NOTICE: Logger::setLogLevel(Logger::WARNING); return (char *)"Warning"; break;
    case Logger::WARNING: Logger::setLogLevel(Logger::ERROR); return (char *)"Error"; break;
    case Logger::ERROR: Logger::setLogLevel(Logger::FATAL); return (char *)"Fatal"; break;
    case Logger::FATAL: Logger::setLogLevel(Logger::VERBOSE); return (char *)"Verbose"; break;
    default: Logger::setLogLevel(Logger::NOTICE); return (char *)"Notice"; break;
  }
}

// Test code
/*
 * try to implement a printf-like debug logging
 */
void test(Logger::Level level, const char* module, const char *format, ...)
{
  va_list va;
  va_start(va, format);
  vsnprintf(logMsg, LOGMSG_LEN, format, va);
  localLogger(level, module, logMsg);
  va_end(va);
}

void localLogger(Logger::Level level, const char* module, const char* message)
{
  Serial.print(F("AQW Logger: ["));
  Serial.print(Logger::asString(level));
  Serial.print(F("] "));

  if (strlen(module) > 0)
  {
      Serial.print(F(": "));
      Serial.print(module);
      Serial.print(F(" "));
  }

  Serial.println(message);

// Unreliable. Need a better way to detect if someone is telnet'd in
//  if (TelnetStream.available())
//  {
    TelnetStream.print(Logger::asString(level));
    TelnetStream.print(F(" : "));
    TelnetStream.println(message);
//  }


//////////////////////////////////////////////////
///       Recall Buffer Management             ///
//////////////////////////////////////////////////
  if (strlen(message) > RECALL_BUFFER_LEN)
  {
    Serial.println(F("*** RECALL BUFFER TOO SMALL **"));
    TelnetStream.println(F("*** RECALL BUFFER TOO SMALL **"));
    return;
  }
  // +2 for '\r' and '\n'
  else if (strlen(message) + 2 + RecallPosition < RECALL_BUFFER_LEN)
  {
    // No wrapping
    strcpy (&RecallBuffer[RecallPosition], message);
    RecallPosition += strlen(message) + 2;
  }
  else
  {
    int max_chars;
//    Serial.print("RecallPosition="); Serial.print(RecallPosition);
//    Serial.print(" strlen(message)="); Serial.print(strlen(message));
//    Serial.print(" RECALL_BUFFER_LEN - RecallPosition="); Serial.print(RECALL_BUFFER_LEN - RecallPosition);

    if (RecallPosition + strlen(message) + 2 <= RECALL_BUFFER_LEN)
      max_chars = strlen(message);
    else if (RecallPosition + strlen(message) + 1 <= RECALL_BUFFER_LEN)
      // If we align with the end of the recall buffer, we just need space for a '\0'
      max_chars = strlen(message) - 1;
    else
      max_chars = RECALL_BUFFER_LEN - RecallPosition;

//    Serial.print(" max_chars="); Serial.println(max_chars);

    memcpy (&RecallBuffer[RecallPosition], message, max_chars);
    memcpy (RecallBuffer, &message[max_chars], strlen(message) - max_chars);

    RecallPosition = strlen(message) - max_chars + 2;

    // We allocated 1 extra byte, so this is OK it's not zero-based
    RecallBuffer[RECALL_BUFFER_LEN] = '\0';

//    Serial.print(" new RecallPosition="); Serial.println(RecallPosition);
  }
  // Add necessary extra characters to make it print pretty
  RecallBuffer[RecallPosition - 2] = '\r';
  RecallBuffer[RecallPosition - 1] = '\n';
  RecallBuffer[RecallPosition] = '\0';
}

void RecallBufferDump()
{
  if (RecallPosition == 0)
  {
    TelnetStream.println (F("Recall Buffer Empty!"));
  }
  else
  {
    TelnetStream.println(F("=================================================="));
    TelnetStream.println(F("===     Begin Event Log Buffer Dump            ==="));
    TelnetStream.println(F("=================================================="));
    TelnetStream.print(&RecallBuffer[RecallPosition+2]);
    TelnetStream.print(RecallBuffer);
    TelnetStream.println(F("=================================================="));
    TelnetStream.println(F("===     End of Event Log Buffer Dump           ==="));
    TelnetStream.println(F("=================================================="));
  }
}

uint16_t rssiToPercent(int rssi_i)
{
  float rssi = (float)rssi_i;
  rssi = isnan(rssi) ? -100.0 : rssi;
  rssi = min(max(2 * (rssi + 100.0), 0.0), 100.0);
  return (uint16_t)rssi;
}

void AQWStatus()
{
  TelnetStream.println(F("=================================================="));
  TelnetStream.print(F("Hostname:       ")); TelnetStream.println(WiFi.getHostname());
  TelnetStream.print(F("IP Address:     ")); TelnetStream.println(WiFi.localIP());
  TelnetStream.print(F("MAC Address:    ")); TelnetStream.println(WiFi.macAddress());
  TelnetStream.print(F("Signal quality: ")); TelnetStream.print(rssiToPercent(WiFi.RSSI())); TelnetStream.print(F("%    RSSI: ")); TelnetStream.println(WiFi.RSSI());
  TelnetStream.print(F("FW Version:     ")); TelnetStream.println(VERSION);
  TelnetStream.print(F("OLED Display:   ")); if (AqwOled) TelnetStream.println(F("Detected")); else TelnetStream.println(F("Not Detected"));
  TelnetStream.print(F("Log level:      ")); TelnetStream.println(Logger::asString(Logger::getLogLevel()));
  TelnetStream.print(F("Uptime:         ")); TelnetStream.println(uptime_formatter::getUptime());
  TelnetStream.println(F("Last readings:"));
  TelnetStream.print(F("  PM 2.5:       ")); TelnetStream.print(pm25); TelnetStream.println(F(" ug/m3"));
  TelnetStream.print(F("  PM 10:        ")); TelnetStream.print(pm10); TelnetStream.println(F(" ug/m3"));
  TelnetStream.print(F("  Temperature:  ")); TelnetStream.print(temp.temperature); TelnetStream.println(F("\xC2\xB0 C"));
  TelnetStream.print(F("  Humidity:     ")); TelnetStream.print(humidity.relative_humidity); TelnetStream.println(F("% rH"));
  TelnetStream.print(F("  VOC Index:    ")); TelnetStream.println(voc_index);
  TelnetStream.print(F("  LiPo Battery: ")); TelnetStream.print(batteryVoltage); TelnetStream.println(F("V"));
  TelnetStream.print(F("  LiPo Battery: ")); TelnetStream.print(batteryPercentage); TelnetStream.println(F("%"));
  TelnetStream.println(F("=================================================="));
}

//////////////////////////////////////////////////
///                                            ///
///               Wifi & MQTT                  ///
///                                            ///
//////////////////////////////////////////////////

void CheckTelnet()
{
  char ch = TelnetStream.read();
  switch (ch)
  {
    char *newLog;
    case 'R':
    case 'r':
      TelnetStream.println(F("Restarting ESP32..."));
      TelnetStream.stop();
      delay(100);
      ESP.restart();
      break;
    case 'L':
    case 'l':
      TelnetStream.print(F("Changing log level to: "));
      Serial.print(F("Changing log level to: "));
      newLog = cycleLogLevel();
      TelnetStream.println(newLog);
      Serial.println(newLog);
      break;
    case 'V':
    case 'v':
      Logger::setLogLevel(Logger::VERBOSE);
      TelnetStream.println(F("Changed log level to: Verbose"));
      Serial.println(F("Changed log level to: Verbose"));
      break;
    case 'N':
    case 'n':
      Logger::setLogLevel(Logger::NOTICE);
      TelnetStream.println(F("Changed log level to: Notice"));
      Serial.println(F("Changed log level to: Notice"));
      break;
    case 'W':
    case 'w':
      Logger::setLogLevel(Logger::WARNING);
      TelnetStream.println(F("Changed log level to: Warning"));
      Serial.println(F("Changed log level to: Warning"));
      break;
    case 'E':
    case 'e':
      Logger::setLogLevel(Logger::ERROR);
      TelnetStream.println(F("Changed log level to: Error"));
      Serial.println(F("Changed log level to: Error"));
      break;
    case 'F':
    case 'f':
      Logger::setLogLevel(Logger::FATAL);
      TelnetStream.println(F("Changed log level to: Fatal"));
      Serial.println(F("Changed log level to: Fatal"));
      break;
    case 'D':
    case 'd':
      TelnetStream.println(F("Dump log history: "));
      RecallBufferDump();
      break;
    case 'S':
    case 's':
      TelnetStream.println(F("Most recent status: "));
      AQWStatus();
      break;
    case 'H':
    case 'h':
    case '?':
      TelnetStream.println(F("Commands:"));
      TelnetStream.println(F("    R - Reboot"));
      TelnetStream.println(F("    L - Cycle log level"));
      TelnetStream.println(F("    V, N, W, E, F - Change to log level"));
      TelnetStream.println(F("      (Verbose, Notice, Warning, Error, Fatal)"));
      TelnetStream.println(F("    D - Dump most recent log"));
      TelnetStream.println(F("    S - Status"));
      break;
  }
//@@@  if (ch != NULL)
//    TelnetStream.begin();
}

void setup_wifi()
{
  delay(10);
  WiFi.begin(ssid, password);
  Logger::warning("Wifi connecting.");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Logger::warning(".");
    // No need to call ArduinoOTA.handle() since we're not connected to wifi
  }
  Logger::notice("Wifi connected.");
}

void connectMqtt()
{
  if (mqttClient.connected())
  {
    Logger::warning("MQTT already connected: Reestablishing");
    disconnectMqtt();
  }

  // Loop until we're reconnected
  Logger::verbose("MQTT connecting.");
  while (!mqttClient.connected())
  {
    // Attempt to connect
    if (mqttClient.connect("ESP32-AQW-Client"))
    {
      digitalWrite(BUILTIN_LED, HIGH);
      Logger::verbose("MQTT connected.");
    }
    else
    {
      // Wait 5 seconds before retrying
      //delay(5000);
      for (int n=0; n < 500; n++) { if ((n % 10) == 0) messagePump(); delay (10); }
      digitalWrite(BUILTIN_LED, LOW);
      Logger::error("MQTT cannot connect.");
    }
    messagePump();
  }
}

void disconnectMqtt()
{
    mqttClient.disconnect();
}

void reconnectMqtt()
{
  if (mqttClient.connected())
  {
    Logger::warning("Reestablishing MQTT connection");
    mqttClient.disconnect();
  }

  // Loop until we're reconnected
  Logger::notice("MQTT connecting.");
  while (!mqttClient.connected())
  {
    // Attempt to connect
    if (mqttClient.connect("ESP32-AQW-Client"))
    {
      digitalWrite(BUILTIN_LED, HIGH);
      Logger::notice("MQTT connected.");
    }
    else
    {
      // Wait 5 seconds before retrying
      //delay(5000);
      for (int n=0; n < 500; n++) { if ((n % 10) == 0) messagePump(); delay (10); }
      digitalWrite(BUILTIN_LED, LOW);
      Logger::error("MQTT cannot connect.");
    }
    messagePump();
  }
}


//////////////////////////////////////////////////
///                                            ///
///               Message Pump                 ///
///                                            ///
//////////////////////////////////////////////////

#ifdef AQW_OLED
/*
 * Look for any button presses on the OLED display
 */
void checkOledButtons()
{
  if (!digitalRead(BUTTON_A))
  {
    snprintf(logMsg, LOGMSG_LEN, "PM2.5  %d\nPM10   %d", pm25, pm10);
    tqAdd(TQ_SHOW, 5000, logMsg, 2, true);
    // Debounce switches
    delay(200);
  }
  if (!digitalRead(BUTTON_B))
  {
    snprintf(logMsg, LOGMSG_LEN, "VOC  %d", voc_index);
    tqAdd(TQ_SHOW, 5000, logMsg, 2, true);
    snprintf(logMsg, LOGMSG_LEN, "Hum  %.0f%%\nTemp %.0f%c", humidity.relative_humidity, temp.temperature, (char)247);
    tqAdd(TQ_SHOW, 5000, logMsg, 2, false);
    // Debounce switches
    delay(200);
  }
  if (!digitalRead(BUTTON_C))
  {
    IPAddress ip = WiFi.localIP();
    snprintf(logMsg, LOGMSG_LEN, "Battery: %.0f%%\nVersion: %s\nWifi:    %d%%\nIP:   %d.%d.%d.%d",
      batteryPercentage, VERSION, rssiToPercent(WiFi.RSSI()), ip[0], ip[1], ip[2], ip[3]);
    tqAdd(TQ_SHOW, 20000, logMsg, 1, true);
    // Debounce switches
    delay(200);
  }
}

/*
 * When adding entries:
 * - If timeout != 0, text will be displayed for that long (CLEAR event automatically queued)
 * - If timeout == 0, text will be displayed after a CLEAR event
 */
void tqAdd(TQCMD cmd, unsigned long timeout, const char *text, unsigned short textSize, bool immediate)
{
  int nextPos;

  if ((tqHead != tqTail) && (immediate))
  {
    tqEmptyQueue();
  }

  nextPos = tqHead + 1;
  if (nextPos >= TQSIZE)
    nextPos = 0;

  if (nextPos != tqTail)
  {
    if (timeout != 0)
      tq[tqHead].timeout = timeout;
    else
      // Default to 1 second
      tq[tqHead].timeout = 1000;
    tq[tqHead].cmd = cmd;
    tq[tqHead].textSize = textSize;
    strcpy(tq[tqHead].text, text);
    tqHead = nextPos;
  }
  else
  {
    Logger::warning("Timer queue full!!");
  }
}

void tqEmptyQueue()
{
  if (tqTail != tqHead)
  {
    display.clearDisplay();
    display.display();
    while (tqTail != tqHead) tqRemove();
  }
}

void tqRemove()
{
  int nextPos = tqTail + 1;
  if (nextPos >= TQSIZE)
    nextPos = 0;
  tq[tqTail].cmd = TQ_EMPTY;
  tqTail = nextPos;
}


/*
 * Check queue for any time-based updates to OLED display
 */
void checkTimeQueue()
{
  if (tqTail != tqHead)
  {
    switch (tq[tqTail].cmd)
    {
      case TQ_CLEAR:
        if (millis() > tq[tqTail].expiration)
        {
          display.clearDisplay();
          display.display();
          tqRemove();
        }
        break;
      case TQ_SHOW:
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(tq[tqTail].textSize);
        display.print(tq[tqTail].text);
        display.display();
        tq[tqTail].expiration = millis() + tq[tqTail].timeout;
        tq[tqTail].cmd = TQ_CLEAR;
        break;
      default:
        tqRemove();
        break;
    }
  }
}

void tqInit()
{
  tqTail = 0;
  tqHead = 0;
  memset (tq, sizeof(TQENTRY) * TQSIZE, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.display();
}
#endif  // #ifdef AQW_OLED

/*
 * This function takes care of checking for async activity such as
 * someone typing in the telnet session and for OTA commands.
 */
void messagePump()
{
  ArduinoOTA.handle();
  CheckTelnet();
#ifdef AQW_OLED
  if (AqwOled)
  {
    checkOledButtons();
    checkTimeQueue();
  }
#endif
}

//////////////////////////////////////////////////
///                                            ///
///             Data Validation                ///
///                                            ///
//////////////////////////////////////////////////

/* 
 *  This function takes a pointer to a value and a type. Basedon the type,
 *  the value will get cast and used. The last two readings are kept and
 *  the avergae is used to determine if the next reading is legit. A factor
 *  is used to allow for sudden, realistic spikes.
 *  
 *  All console logging is done in this routing. Either true or false is
 *  returned to the caller.
 */

#define PM25_MAX 900
#define PM10_MAX 1000
#define VOC_MAX 2000
#define TEMPERATURE_MAX 150
#define HUMIDITY_MAX 125

bool validateReading(void *data_r, DataType type)
{
  unsigned long max, data, avg;
  String str;

  //
  // Check type param is valid
  //
  if (type != PM25 && type != PM10 && type != VOC_INDEX &&
      type != TEMPERATURE && type != HUMIDITY)
  {
    Logger::error("Data Valididater: Invalid type specified");
    return false;
  }

  //
  // Based on the type, collect the right data
  //
  switch (type)
  {
    case PM25:
      max = PM25_MAX;
      str = "PM2.5";
      data = *(unsigned long *)data_r;
      avg = (HistoricalData.pm25[1] + HistoricalData.pm25[0]) / 2;
      break;
    case PM10:
      max = PM10_MAX;
      str = "PM10";
      data = *(unsigned long *)data_r;
      avg = (HistoricalData.pm10[1] + HistoricalData.pm10[0]) / 2;
      break;
    case VOC_INDEX:
      max = VOC_MAX;
      str = "VOC Index";
      data = *(unsigned long *)data_r;
      avg = (HistoricalData.voc_index[1] + HistoricalData.voc_index[0]) / 2;
      break;
    case TEMPERATURE:
      max = TEMPERATURE_MAX;
      str = "Temperature";
      data = (unsigned long)(*(float *)data_r);
      avg = (HistoricalData.temperature[1] + HistoricalData.temperature[0]) / 2;
      break;
    case HUMIDITY:
      max = HUMIDITY_MAX;
      str = "Humidity";
      data = (unsigned long)(*(float *)data_r);
      avg = (HistoricalData.humidity[1] + HistoricalData.humidity[0]) / 2;
      break;
  }

  //
  // Basic sanity test to see if the value is just plain nuts!
  //
  if (data > max)
  {
    test(Logger::ERROR, "", "Initial %s value read is beyond max", str.c_str());
    return false;
  }

  //
  // Be sure the avg value is greater than 0
  //
  if (avg == 0) avg = 1;

  //
  // Using value calculated above, test for legit value. Notice the
  // test is only for invalid rising readings. Invalid decreases are
  // not caught.
  //
  if (data > (avg * ValidationFactor))
  {
    test(Logger::ERROR, "", "%s data value read out of bounds", str.c_str());
    return false;
  }

  //
  // Rotate old data and insert new. Only add the new value if it
  // appears to be within a normal range (passes above tests).
  //
  switch (type)
  {
    case PM25:
      HistoricalData.pm25[0] = HistoricalData.pm25[1];
      HistoricalData.pm25[1] = data;
      break;
    case PM10:
      HistoricalData.pm10[0] = HistoricalData.pm10[1];
      HistoricalData.pm10[1] = data;
      break;
    case VOC_INDEX:
      HistoricalData.voc_index[0] = HistoricalData.voc_index[1];
      HistoricalData.voc_index[1] = data;
      break;
    case TEMPERATURE:
      HistoricalData.temperature[0] = HistoricalData.temperature[1];
      HistoricalData.temperature[1] = data;
      break;
    case HUMIDITY:
      HistoricalData.humidity[0] = HistoricalData.humidity[1];
      HistoricalData.humidity[1] = data;
      break;
  }

  return true;
}

//////////////////////////////////////////////////
///                                            ///
///       Battery Voltage Reading              ///
///                                            ///
//////////////////////////////////////////////////

void updateBatteryVoltage()
{
  // A13 pin is not exposed on Huzzah32 board because it's tied to
  // measuring voltage level of battery. Note: you must
  // multiply the analogRead value by 2x to get the true battery
  // level. See: 
  // https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/esp32-faq
  float rawValue = analogRead(A13);

  // Reference voltage on ESP32 is 1.1V
  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html#adc-calibration
  // See also: https://bit.ly/2zFzfMT
  batteryVoltage = (rawValue / 4095.0) * 2.0 * 1.1 * 3.3; // calculate voltage level
  batteryPercentage = batteryVoltage / MAX_BATTERY_VOLTAGE * 100.0;
  if (batteryPercentage > 100.0) batteryPercentage = 100.0;

  snprintf(logMsg, LOGMSG_LEN, "Raw battery reading:   %.0f", rawValue);
  Logger::notice(logMsg);
  snprintf(logMsg, LOGMSG_LEN, "Battery voltage:       %.1f (%.1f%%)", batteryVoltage, batteryPercentage);
  Logger::notice(logMsg);

  snprintf (mqtt_pub, MQTT_PUB_LEN, "%.1f", AQW_HOSTNAME, batteryVoltage);
  snprintf (mqtt_topic, MQTT_TOPIC_LEN, "%s/battery/voltage", AQW_HOSTNAME);
  mqttClient.publish(mqtt_topic, mqtt_pub);
  snprintf (mqtt_pub, MQTT_PUB_LEN, "%.1f", batteryPercentage);
  snprintf (mqtt_topic, MQTT_TOPIC_LEN, "%s/battery/percentage", AQW_HOSTNAME);
  mqttClient.publish(mqtt_topic, mqtt_pub);
}

//////////////////////////////////////////////////
///                                            ///
///               SHTC3 Sensor                 ///
///                                            ///
//////////////////////////////////////////////////

void shtc3_Start(void)
{
  Logger::verbose("SHTC3 test");
  if (! shtc3.begin())
  {
    Logger::error("Couldn't find SHTC3");
#ifdef AQW_OLED
    if (AqwOled)
    {
      display.setCursor(0,0);
      display.print(F("SHTC3 Sensor Failure"));
      display.display();
    }
#endif
    int cnt=0;
    while (1)
    {
      delay(10);
      cnt++;
      if (cnt > 100)    // 1 second
      {
        cnt = 0;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      }
      messagePump();
    }
  }
  Logger::notice("Found SHTC3 sensor");
}


//////////////////////////////////////////////////
///                                            ///
///               SGP40 Sensor                 ///
///                                            ///
//////////////////////////////////////////////////

void sgp40_Start(void)
{
  Logger::verbose("SGP40 test with SHTC3 compensation");

  if (! sgp.begin())
  {
    int cnt=0;
    Logger::error("SGP40 sensor not found :(");
#ifdef AQW_OLED
    if (AqwOled)
    {
      display.setCursor(0,0);
      display.print(F("SGP40 Sensor Failure"));
      display.display();
    }
#endif
    while (1)
    {
      delay(10);
      cnt++;
      if (cnt > 200)    // 2 seconds
      {
        cnt = 0;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      }
      messagePump();
    }
  }

  memset (SgpSerial, SGP_SERIAL_LEN, 0);
  snprintf (SgpSerial, SGP_SERIAL_LEN - 1, "%02x%02x%02x",
    sgp.serialnumber[0], sgp.serialnumber[1], sgp.serialnumber[2]);
  snprintf(logMsg, LOGMSG_LEN, "Found SGP40 serial # %s", SgpSerial);
  Logger::notice(logMsg);
}

void sgp40_Read(bool FirstRun)
{
  uint16_t sraw;
  bool err = false;

  shtc3.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data

  snprintf(logMsg, LOGMSG_LEN, "Temperature:           %.2f degrees C", temp.temperature);
  Logger::notice(logMsg);
  snprintf(logMsg, LOGMSG_LEN, "Humidity:              %.2f%% rH", humidity.relative_humidity);
  Logger::notice(logMsg);

  sraw = sgp.measureRaw(temp.temperature, humidity.relative_humidity);
  snprintf(logMsg, LOGMSG_LEN, "Raw SGP40 measurement: %d", sraw);
  Logger::notice(logMsg);

  voc_index = sgp.measureVocIndex(temp.temperature, humidity.relative_humidity);
  snprintf(logMsg, LOGMSG_LEN, "VOC Index:             %d", voc_index);
  Logger::notice(logMsg);

  // Turn off the SGP40 heater to save current
  sgp.heaterOff();

  if (FirstRun)
  {
    HistoricalData.temperature[0] = (unsigned long)(temp.temperature);
    HistoricalData.temperature[1] = (unsigned long)(temp.temperature);
    HistoricalData.humidity[0] = (unsigned long)(humidity.relative_humidity);
    HistoricalData.humidity[1] = (unsigned long)(humidity.relative_humidity);
    HistoricalData.voc_index[0] = voc_index;
    HistoricalData.voc_index[1] = voc_index;
    return;
  }

  snprintf (mqtt_pub, MQTT_PUB_LEN, "%.2f", temp.temperature);
  snprintf (mqtt_topic, MQTT_TOPIC_LEN, "%s/temperature", AQW_HOSTNAME);
  if (mqttClient.publish(mqtt_topic, mqtt_pub) == false)
    err = true;

  snprintf (mqtt_pub, MQTT_PUB_LEN, "%.2f", humidity.relative_humidity);
  snprintf (mqtt_topic, MQTT_TOPIC_LEN, "%s/humidity", AQW_HOSTNAME);
  if (mqttClient.publish(mqtt_topic, mqtt_pub) == false)
    err = true;

  snprintf (mqtt_pub, MQTT_PUB_LEN, "%d", voc_index);
  snprintf (mqtt_topic, MQTT_TOPIC_LEN, "%s/voc_index", AQW_HOSTNAME);
  if (mqttClient.publish(mqtt_topic, mqtt_pub) == false)
    err = true;

  if (err == true)
    Logger::error("Error while publishing SHTC3 & SGP40 data via MQTT");
  else
    Logger::verbose("Published SHTC3 & SGP40 data via MQTT");
}

//////////////////////////////////////////////////
///                                            ///
///               HPMA115S0 Sensor             ///
///                                            ///
//////////////////////////////////////////////////

void hpma_Start(void)
{
  long start = millis();
  HPMA115S0.begin(9600, SERIAL_8N1, RXD2, TXD2);
  while ((!HPMA115S0) && (millis() - start < MAX_AQW_WAIT * 1000)) messagePump();
  // If there's any problem from the line above, the start_autosend() fn will call it out.
  Logger::notice("Pausing 6s for particle sensor...");
  delay(6000);
  Logger::verbose("Starting HPM autosend");
  start_autosend();
}

void hpma_Read(bool FirstRun)
{
  bool err = false;

  if (!receive_measurement(&pm25, &pm10))
  {
    digitalWrite(BUILTIN_LED, 0);
    Logger::warning("Cannot receive data from HPMA115S0!");
    return;
  }

  snprintf(logMsg, LOGMSG_LEN, "PM 2.5:                %d ug/m3", pm25);
  Logger::notice(logMsg);
  snprintf(logMsg, LOGMSG_LEN, "PM 10:                 %d ug/m3", pm10);
  Logger::notice(logMsg);

  if (FirstRun)
  {
    HistoricalData.pm25[0] = pm25;
    HistoricalData.pm25[1] = pm25;
    HistoricalData.pm10[0] = pm10;
    HistoricalData.pm10[1] = pm10;
    return;
  }

  if (validateReading(&pm25, PM25))
  {
    snprintf (mqtt_pub, MQTT_PUB_LEN, "%d", pm25);
    snprintf (mqtt_topic, MQTT_TOPIC_LEN, "%s/PM2.5", AQW_HOSTNAME);
    if (mqttClient.publish(mqtt_topic, mqtt_pub) == false)
      err = true;
  }
  if (validateReading(&pm10, PM10))
  {
    snprintf (mqtt_pub, MQTT_PUB_LEN, "%d", pm10);
    snprintf (mqtt_topic, MQTT_TOPIC_LEN, "%s/PM10", AQW_HOSTNAME);
    if (mqttClient.publish(mqtt_topic, mqtt_pub) == false)
      err = true;
  }

  if (err == true)
    Logger::error("Error while publishing HPMA115S0 data via MQTT");
  else
    Logger::verbose("Published HPMA115S0 data via MQTT");

}

bool receive_measurement (int *pm25, int *pm10)
{
  long start = millis();
  while ((HPMA115S0.available() < 32) && (millis() - start < MAX_AQW_WAIT * 1000 + 6000)) messagePump();
  if (HPMA115S0.available() < 32)
  {
    Logger::warning("Timeout reading from HPMA1150");
    return false;
  }
  byte HEAD0 = HPMA115S0.read();
  byte HEAD1 = HPMA115S0.read();
  start = millis();
  while ((HEAD0 != 0x42) && (millis() - start < MAX_AQW_WAIT * 1000))
  {
    if (HEAD1 == 0x42)
    {
      HEAD0 = HEAD1;
      HEAD1 = HPMA115S0.read();
    }
    else
    {
      HEAD0 = HPMA115S0.read();
      HEAD1 = HPMA115S0.read();
    }
    messagePump();
  }
  if (HEAD0 != 0x42)
  {
    Logger::warning("Timeout or Incorrect data read from HPMA1150 sensor");
    return false;
  }
  if (HEAD0 == 0x42 && HEAD1 == 0x4D)
  {
    byte LENH = HPMA115S0.read();
    byte LENL = HPMA115S0.read();
    byte Data0H = HPMA115S0.read();
    byte Data0L = HPMA115S0.read();
    byte Data1H = HPMA115S0.read();
    byte Data1L = HPMA115S0.read();
    byte Data2H = HPMA115S0.read();
    byte Data2L = HPMA115S0.read();
    byte Data3H = HPMA115S0.read();
    byte Data3L = HPMA115S0.read();
    byte Data4H = HPMA115S0.read();
    byte Data4L = HPMA115S0.read();
    byte Data5H = HPMA115S0.read();
    byte Data5L = HPMA115S0.read();
    byte Data6H = HPMA115S0.read();
    byte Data6L = HPMA115S0.read();
    byte Data7H = HPMA115S0.read();
    byte Data7L = HPMA115S0.read();
    byte Data8H = HPMA115S0.read();
    byte Data8L = HPMA115S0.read();
    byte Data9H = HPMA115S0.read();
    byte Data9L = HPMA115S0.read();
    byte Data10H = HPMA115S0.read();
    byte Data10L = HPMA115S0.read();
    byte Data11H = HPMA115S0.read();
    byte Data11L = HPMA115S0.read();
    byte Data12H = HPMA115S0.read();
    byte Data12L = HPMA115S0.read();
    byte CheckSumH = HPMA115S0.read();
    byte CheckSumL = HPMA115S0.read();
    if (((HEAD0 + HEAD1 + LENH + LENL + Data0H + Data0L + Data1H + Data1L + Data2H + Data2L + Data3H + Data3L + Data4H + Data4L + Data5H + Data5L + Data6H + Data6L + Data7H + Data7L + Data8H + Data8L + Data9H + Data9L + Data10H + Data10L + Data11H + Data11L + Data12H + Data12L) % 256) != CheckSumL)
    {
      Logger::warning("HPMA115S0 Checksum fail");
      return false;
    }
    *pm25 = (Data1H * 256) + Data1L;
    *pm10 = (Data2H * 256) + Data2L;
    return true;
  }
  return false;
}
 
bool start_autosend(void)
{
  // Start auto send
  byte start_autosend[] = {0x68, 0x01, 0x40, 0x57 };
  long start = millis();

  HPMA115S0.write(start_autosend, sizeof(start_autosend));
  HPMA115S0.flush();
  delay(500);

  // Then we wait for the response
  while ((HPMA115S0.available() < 2)  && (millis() - start < MAX_AQW_WAIT * 1000 + 10000)) messagePump();

  if (HPMA115S0.available() < 2)
  {
    int cnt=0;
    Logger::fatal("Cannot communicate with the HPMA115S0!");
#ifdef AQW_OLED
    if (AqwOled)
    {
      display.setCursor(0,0);
      display.print(F("HPMA115S0 Sensor Failure"));
      display.display();
    }
#endif
    while (1)
    {
      delay(10);
      cnt++;
      if (cnt > 50)   // 1/2 sec
      {
        cnt = 0;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      }
      messagePump();
    }
  }

  byte read1 = HPMA115S0.read();
  byte read2 = HPMA115S0.read();

  // Test the response
  if ((read1 == 0xA5) && (read2 == 0xA5))
  {
    // ACK
    return 1;
  }
  else if ((read1 == 0x96) && (read2 == 0x96))
  {
    // NACK
    return 0;
  }
  else
    return 0;
}
