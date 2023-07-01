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
 * Title: Air Quality Wing with Particle Argon uController
 * Author: Steve Meisner (plus many others; See docs below)
 * email: steve@meisners.net
 * 
 * Description
 * 
 * The Particle Argon board is one of the few that has the Fetaher footprint and
 * supplies onboard wifi. The wifi allows for MQTT publishing to a broker. It
 * also provides a telnet server to connect to.
 * 
 * The Adafruit OLED Feather Wing also provides a convienent display for the AQW.
 * This display is used to display air quality info and some minimal statisitcs.
 * The display is not required for operation and is dynamically detected
 * during setup().
 *
 * A telnet client is also embedded in the code to provide config, status
 * and diagnostic info.
 * 
 * See the README.md file for additional details.
 *
 * I2C Addresses:
 *  - SHTC3 0x70
 *  - SGP40 0x59
 *  - OLED  0x3c
 *  
 */

///////////////////////////////////////////////////////////////////////////
//
// From VSCode Particle UI:
// -- Configure for Device; deviceOS@5.1.0, argon
// From VSCode, command palette;
// Particle: install library xxxxx
//  -- MQTT
//  -- Adafruit_SSD1306_RK
//  -- TelnetLogger
//
//  To install AirQualityWing library, go to the folowing URL:
//    https://github.com/circuitdojo/air-quality-wing-library/archive/refs/tags/1.1.0.zip
//  The downloaded archive should be expanded into the ./lib folder as:
//    airqualitywing-particle/lib/air-quality-wing-library-master
//  Then add the following line to project.properties:
//    dependencies.air-quality-wing-library-master=1.0.2  <-- Yes, it's the wrong version
//
// Local compile and local flash via the Particle UI
//
///////////////////////////////////////////////////////////////////////////
//
// To bring up a new Particle device:
//
//   1. Connect via USB
//   2. issue: screen /dev/ttyACM0 -s 115200
//   3. Type 'w' to set wifi credentials & cipher settings
//
// After restart, it should connect and using the same 'screen' command,
// output can be monitored.
//
///////////////////////////////////////////////////////////////////////////
//
// To connect via USB serial:
// At Linux bash terminal: screen /dev/ttyACM0 -s 115200
//
///////////////////////////////////////////////////////////////////////////
//
// During startup, if wifi credentials stored in EEPROM do not succeed
// in connecting, the serial console will be enabled. The user will be
// prompted for an SSID and pre-shared key. Upon entering these, they 
// will be written to EEPROM.
//
// A telnet server will ne enabled to allow control and log display.
// Typing '?' will bring up help menu.
//
// All commands are 1 character, except to modify the EEPROM. This is
// indicated by starting the command line with "ee".
//
// ee <field> <value>
// ee clear
//
///////////////////////////////////////////////////////////////////////////

// To enable PARTICLE_BENCH, be sure Argon is registered with Particle
//#define PARTICLE_BENCH

#include "aqw.h"

STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY)); //Allows the system to retain variables for quick startup
#ifdef PARTICLE_BENCH
SYSTEM_MODE(SEMI_AUTOMATIC);
#else
SYSTEM_MODE(MANUAL);
#endif

/////////////////////////////////////////////////////////////////////////
//
//  EEPROM support
//
/////////////////////////////////////////////////////////////////////////
EEPROM_DATA eeprom;
uint16_t eeprom_crc;   // CRC for entire data struct

/////////////////////////////////////////////////////////////////////////
//
// Battery support
//
/////////////////////////////////////////////////////////////////////////
bool onBatteryPower = false;
float Battery_Voltage;
float Battery_Percent;

/////////////////////////////////////////////////////////////////////////
//
//  OLED display support
//
/////////////////////////////////////////////////////////////////////////
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
bool PROGMEM AqwOled = false;

/////////////////////////////////////////////////////////////////////////
//
// AirQualityWing board support
//
/////////////////////////////////////////////////////////////////////////
// AirQualityWing object
AirQualityWing AirQual = AirQualityWing();

/////////////////////////////////////////////////////////////////////////
//
//  Error Stats struct
//
/////////////////////////////////////////////////////////////////////////
ERRORS PROGMEM ErrorStats;

/////////////////////////////////////////////////////////////////////////
//
//  Timer queue support
//
/////////////////////////////////////////////////////////////////////////
#define TQSIZE 4
TQENTRY PROGMEM tq[TQSIZE];
unsigned short PROGMEM tqTail;
unsigned short PROGMEM tqHead;

/////////////////////////////////////////////////////////////////////////
//
//  MQTT support
//
/////////////////////////////////////////////////////////////////////////
#define MQTT_TOPIC_SIZE 32
#define MQTT_PAYLOAD_SIZE 32
static char mqtt_topic[MQTT_TOPIC_SIZE+1];
static char mqtt_payload[MQTT_PAYLOAD_SIZE+1];
MQTT MqttClient("", 1883, NULL);

/////////////////////////////////////////////////////////////////////////
//
//  Logging support
//
/////////////////////////////////////////////////////////////////////////
#define LOGMSG_LEN 92               // Size of log message scratch buffer
char PROGMEM logMsg[LOGMSG_LEN];    // General use buffer for log messages

// Logger
SerialLogHandler logHandler(115200, LOG_LEVEL_ERROR, {
   { "app", LOG_LEVEL_TRACE }, // enable all app messages
});
//SerialLogHandler logHandler(115200, LOG_LEVEL_INFO);

/////////////////////////////////////////////////////////////////////////
//
// Telnet support
//
/////////////////////////////////////////////////////////////////////////
#define TELNET_APP_NAME "aqw"
// telnet defaults to port 23
TCPServer telnetServer = TCPServer(23);
TCPClient telnetClient;
#define TELNET_BUFF_LEN 80
char telnetCommand[TELNET_BUFF_LEN];
// Install our logger
TelnetLogger *telnetLogger;


/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
//
//  Code Start
//
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////
//
//  Utility code
//
////////////////////////////////////////////////////////
uint16_t _crc16_update(uint16_t crc, uint8_t a)
{
  int i;
  crc ^= a;
  for (i = 0; i < 8; ++i)
  {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001;
    else
      crc = (crc >> 1);
  }
  return crc;
}

uint16_t calcCRC(char* str, int len)
{
  uint16_t crc=0; // starting value as you like, must be the same before each calculation
  for (int i=0; i < len; i++) // for each byte in the stuct
  {
    crc= _crc16_update (crc, str[i]); // update the crc value
  }
  return crc;
}

////////////////////////////////////////////////////////
//
//  EEPROM support
//
////////////////////////////////////////////////////////
boolean read_eeprom()
{
  uint16_t crc=0;

  EEPROM.get(0x0000, eeprom);
  EEPROM.get(sizeof(eeprom), eeprom_crc);

  crc = calcCRC((char *)&eeprom, sizeof(eeprom));
  if (crc != eeprom_crc)
  {
    Log.warn("EEPROM corrupt or unintialized");
    Log.info("CRC read:      %u", eeprom_crc);
    Log.info("CRC expected:  %u", crc);
    memset (&eeprom, 0x00, sizeof(eeprom));
    return false;
  }

  // Mask wifi PSK
  char psk[sizeof(eeprom.psk)+1];
  memset (psk, '*', strlen(eeprom.psk));
  psk[0] = eeprom.psk[0]; psk[1] = eeprom.psk[1]; psk[2] = eeprom.psk[2];
  psk[strlen(eeprom.psk)-2] = eeprom.psk[strlen(eeprom.psk)-2];
  psk[strlen(eeprom.psk)-1] = eeprom.psk[strlen(eeprom.psk)-1];
  psk[strlen(eeprom.psk)] = '\0';

  Log.info("Read from EEPROM:");
  Log.info("  Name:        %s", eeprom.node_name);
  Log.info("  SSID:        %s", eeprom.ssid);
  Log.info("  PSK:         %s", psk);
  Log.info("  mqtt Broker: %s", eeprom.mqtt_broker);
  Log.info("  mqtt User:   %s", eeprom.mqtt_user);
  Log.info("  mqtt Pass:   %s", eeprom.mqtt_pass);

  return true;
}

void clear_eeprom()
{
  uint16_t eeprom_crc=0;
  memset (&eeprom, 0, sizeof(eeprom));
  EEPROM.put(0x0000, eeprom);
  EEPROM.put(sizeof(eeprom), eeprom_crc);
}

void write_eeprom()
{
  Log.warn("Writing to EEPROM:");
  Log.info("  Name:        %s", eeprom.node_name);
  Log.info("  SSID:        %s", eeprom.ssid);
  Log.info("  PSK:         %s", eeprom.psk);
  Log.info("  mqtt Broker: %s", eeprom.mqtt_broker);
  Log.info("  mqtt User:   %s", eeprom.mqtt_user);
  Log.info("  mqtt Pass:   %s", eeprom.mqtt_pass);
  Log.info("  Size of eeprom: %d", sizeof(eeprom));

  eeprom_crc = calcCRC((char *)&eeprom, sizeof(eeprom));
  Log.info("CRC of eeprom:  %u", eeprom_crc);

  EEPROM.put(0x0000, eeprom);
  EEPROM.put(sizeof(eeprom), eeprom_crc);
}

bool eepromModifyValue(char *field, char *value)
{
  if (strcmp(field, "node_name") == 0)
    strncpy(eeprom.node_name, value, 16);
  else if (strcmp(field, "ssid") == 0)
    strncpy(eeprom.ssid, value, 16);
  else if (strcmp(field, "psk") == 0)
    strncpy(eeprom.psk, value, 16);
  else if (strcmp(field, "mqtt_broker") == 0)
    strncpy(eeprom.mqtt_broker, value, 40);
  else if (strcmp(field, "mqtt_user") == 0)
    strncpy(eeprom.mqtt_user, value, 8);
  else if (strcmp(field, "mqtt_pass") == 0)
    strncpy(eeprom.mqtt_pass, value, 8);
  else
    return false;
  write_eeprom();
  return true;
}

////////////////////////////////////////////////////////
//
//  Wi-Fi support
//
////////////////////////////////////////////////////////
uint16_t rssiToPercent(int rssi_i)
{
  float rssi = (float)rssi_i;
  rssi = isnan(rssi) ? -100.0 : rssi;
  rssi = min(max(2 * (rssi + 100.0), 0.0), 100.0);
  return (uint16_t)rssi;
}

void setupWifi()
{
  WiFi.disconnect();
  WiFi.off();
  WiFi.clearCredentials();
  delay (100);
  WiFi.on();
  WiFi.setCredentials(eeprom.ssid, eeprom.psk, WPA2, WLAN_CIPHER_AES);
  WiFi.connect();
  waitFor(WiFi.ready, 15000);
  if (WiFi.ready())
  {
#ifdef PARTICLE_BENCH
    Particle.connect();
#endif
    Log.info("ip address: %s", WiFi.localIP().toString().c_str());
  } else {
    tqAdd(TQ_SHOW, 5000, "Wifi Conn\r\nFailed", 2, true);
    Log.error("WiFi failed to connect");
    ErrorStats.wifi++;
  }
}

////////////////////////////////////////////////////////
//
//  MQTT support
//
////////////////////////////////////////////////////////
bool mqttConnect()
{
  MqttClient.disconnect();
  MqttClient.setBroker(eeprom.mqtt_broker, 1883);
  // connect to the server(unique id by Time.now())
  MqttClient.connect("aqwclient_" + String(Time.now()), eeprom.mqtt_user, eeprom.mqtt_pass);

  if (!MqttClient.isConnected())
    return false;
  else
    return true;
}

bool mqttCheckConnection()
{
  if ( !(MqttClient.isConnected()) )
  {
    tqAdd(TQ_SHOW, (AirQual.getInterval() - 10000) / 1000, "MQTT Conn\r\nFailed", 2, true);
    Log.error("MQTT Connect Failed");
    ErrorStats.mqtt++;
#ifdef PARTICLE_BENCH
    Particle.publish("err", "MQTT connect FAILED!!", PRIVATE, NO_ACK);
#endif
    return false;
  }
  return true;
}

void _sendMqttMessage (const char *mqtt_topic, const char *mqtt_payload)
{
  if ( (WiFi.ready()) && ( ! MqttClient.isConnected() ) )
  {
    Log.warn ("Lost MQTT connection");
    mqttConnect();
  }
  if (mqttCheckConnection())
  {
    if ( ! MqttClient.publish(mqtt_topic, mqtt_payload) )
    {
      Log.error("MQTT Publish failed");
      ErrorStats.mqtt++;
      tqAdd (TQ_SHOW, (AirQual.getInterval() - 10000) / 1000, "MQTT Pub\r\nFailed", 2, true);
    }
  }
  else
  {
    Log.error("Cannot establish MQTT connection - MQTT Publish failed");
    ErrorStats.mqtt++;
  }
}

void SendMqttMessage (const char *node, const char *topic, float value)
{
  snprintf (mqtt_topic, MQTT_TOPIC_SIZE, "%s/%s", node, topic);
  snprintf (mqtt_payload, MQTT_PAYLOAD_SIZE, "%.1f", value);
  _sendMqttMessage(mqtt_topic, mqtt_payload);
}

void SendMqttMessage (const char *node, const char *topic, int32_t value)
{
  snprintf (mqtt_topic, MQTT_TOPIC_SIZE, "%s/%s", node, topic);
  snprintf (mqtt_payload, MQTT_PAYLOAD_SIZE, "%d", (int)value);
  _sendMqttMessage(mqtt_topic, mqtt_payload);
}

void SendMqttMessage (const char *node, const char *topic, const char *value)
{
  snprintf (mqtt_topic, MQTT_TOPIC_SIZE, "%s/%s", node, topic);
  snprintf (mqtt_payload, MQTT_PAYLOAD_SIZE, "%s", value);
  _sendMqttMessage(mqtt_topic, mqtt_payload);
}

////////////////////////////////////////////////////////
//
//  Battery support
//
////////////////////////////////////////////////////////
//
// LiPO battery will charge up to 4.3v
// "Full" is considered 4.2v or more
// Nominal is 3.7v, but not relied on
// Argon charge indicator turns off at 4.3v
// Note:
//    Called from AirQualityWingEvent() handler, so
//    we already have verified wifi & connected to MQTT
//    broker.
//
void SendBatteryInfo()
{
  // When no battery is connected, CHG = 1, PWR = 1
  // and voltage is ~4.41v (> 4.3)
  Battery_Voltage = analogRead(BATT) * 0.0011224;

  // PWR: 0=no USB power, 1=USB powered
	// CHG: 0=charging, 1=not charging
  int chargeState = int(digitalRead(CHG));
  int powerState = int(digitalRead(PWR));
  static int lastPowerState = -1; // -1 forces battery state update

  if (lastPowerState != powerState)
  {
    Log.info ("power: USB Power state change");
    if (powerState)
    {
      set_interval("60000");
      onBatteryPower = false;
      Log.info ("Power: On USB power");
      RGB.control(true);
      RGB.color(0, 100, 200);
      RGB.brightness(200, true);
      RGB.control(false);
    }
    else
    {
      set_interval("120000");
      onBatteryPower = true;
      Log.info ("Power: On battery power");
      RGB.control(true);
      RGB.color(0, 0, 0);
      RGB.brightness(0, true);
      //RGB.control(false);
    }
    lastPowerState = powerState;
  }

  //
  // Make the assumption if the battery is not being
  // charged and we are on USB power, then the effective
  // battery percentage is 100%
  //
  if (chargeState == 1 && powerState == 1)
    Battery_Percent = 100.0;
  else if (Battery_Voltage >= 4.18)
    Battery_Percent = 100.0;
  else
    Battery_Percent = float((Battery_Voltage - 3.3) / 0.88 * 100.0);

  SendMqttMessage (eeprom.node_name, "battery/voltage", Battery_Voltage);
  SendMqttMessage (eeprom.node_name, "battery/percentage", (int32_t)Battery_Percent);

  Log.info ("Battery: Voltage: %.2fV, Percent: %.0f%% (PWR: %d, CHG: %d)",
    Battery_Voltage, Battery_Percent, powerState, chargeState);

}

////////////////////////////////////////////////////////
//
//  Air Quality Wing support
//
////////////////////////////////////////////////////////
// Handler is called in main loop.
void AirQualityWingEvent()
{
  if (WiFi.ready())
  {
    if ( mqttConnect() ) 
    {
      SendMqttMessage (eeprom.node_name, "PM2.5", (int32_t)(AirQual.getData().hpma115.data.pm25));
      SendMqttMessage (eeprom.node_name, "PM10", (int32_t)(AirQual.getData().hpma115.data.pm10));
      SendMqttMessage (eeprom.node_name, "voc_index", AirQual.getData().sgp40.data.tvoc);
      SendMqttMessage (eeprom.node_name, "temperature", AirQual.getData().shtc3.data.temperature);
      SendMqttMessage (eeprom.node_name, "humidity", AirQual.getData().shtc3.data.humidity);
      SendMqttMessage (eeprom.node_name, "rssi", (int32_t)(WiFi.RSSI()));
      SendMqttMessage (eeprom.node_name, "signal", (int32_t)(rssiToPercent(WiFi.RSSI())));
      SendMqttMessage (eeprom.node_name, "ip", WiFi.localIP().toString().c_str());

      SendBatteryInfo();

      MqttClient.disconnect();
    }
    else
    {
      Log.error ("Failed to establish MQTT connection");
      ErrorStats.mqtt++;
    }
  }
  else
  {
    Log.error ("Attempted MQTT publish when wifi down");
    ErrorStats.wifi++;
  }

  Log.info ("Particle & VOC: PM 2.5: %d  PM 10: %d  VOC: %d", 
    (int)(AirQual.getData().hpma115.data.pm25), (int)(AirQual.getData().hpma115.data.pm10), (int)(AirQual.getData().sgp40.data.tvoc));
  Log.info ("Temperature: %.1fC  Humidity %.1f%%",
    AirQual.getData().shtc3.data.temperature, AirQual.getData().shtc3.data.humidity);
}

// Cloud function for setting interval
int set_interval (String period)
{
  Log.info ("Interval set to: %s", period.c_str());
  
  // Set the interval with the air quality code
  AirQual.setInterval((uint32_t)period.toInt());

  return -1;
}

////////////////////////////////////////////////////////
//
//  OLED support
//
////////////////////////////////////////////////////////
boolean OledPresent(short address)
{
  Wire.begin();
  Wire.beginTransmission (address);
  if (Wire.endTransmission() == 0)
    return true;
  else
    return false;
}

////////////////////////////////////////////////////////
//
//  Timer Queue support
//
////////////////////////////////////////////////////////
/*
 * When adding entries:
 * - If timeout != 0, text will be displayed for that long (CLEAR event automatically queued)
 * - If timeout == 0, text will be displayed after a CLEAR event
 */
void tqAdd(TQCMD cmd, unsigned long timeout, const char *text, unsigned short textSize, bool immediate)
{
  int nextPos;

  if (!AqwOled) return;

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
    Log.warn("Timer queue full!!");
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

void tqInit()
{
  tqTail = 0;
  tqHead = 0;
  memset (tq, 0, sizeof(TQENTRY) * TQSIZE);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.display();
}

////////////////////////////////////////////////////////
//
//  Serial console support
//
////////////////////////////////////////////////////////

void serialReadString(char *pch, int maxlen)
{
  const size_t READ_BUF_SIZE = 64;
  char readBuf[READ_BUF_SIZE];
  size_t readBufOffset = 0;

  // Read data from serial
  while (true)
  {
    if (Serial.available())
    {
      if (readBufOffset < READ_BUF_SIZE)
      {
        char c = Serial.read();
        if (c != '\r')
        {
          if (c != '\n')
          {
            // Add character to buffer
            readBuf[readBufOffset++] = c;
            Serial.printf("%c", c);
          }
        }
        else
        {
          // End of line character found, process line
          readBuf[readBufOffset] = 0;
          strncpy (pch, readBuf, maxlen);
          Serial.println("");
          return;
        }
      }
      else
      {
          Serial.println("readBuf overflow, emptying buffer");
          readBufOffset = 0;
          break;
      }
    }
    checkTimeQueue();
  }
}

void serialConsole()
{
  Serial.print ("Enter name of device (such as AQW): ");
  serialReadString (eeprom.node_name, 16);
  Serial.print ("Enter wifi SSID: ");
  serialReadString (eeprom.ssid, 16);
  Serial.print ("Enter wifi PSK: ");
  serialReadString (eeprom.psk, 16);
  Serial.print ("Enter MQTT Broker [<CR> to skip]: ");
  serialReadString (eeprom.mqtt_broker, 40);
  if (strlen(eeprom.mqtt_broker) > 0)
  {
    Serial.print ("Enter MQTT Broker username: ");
    serialReadString (eeprom.mqtt_user, 8);
    Serial.print ("Enter MQTT Broker password: ");
    serialReadString (eeprom.mqtt_pass, 8);
  }

  write_eeprom();   //@@@ Is this necessary? 
}


////////////////////////////////////////////////////////
//
//  Telnet console routines
//
////////////////////////////////////////////////////////

void regenerateLogHandler(LogLevel level)
{
    auto logManager = LogManager::instance();
    logManager->removeHandler(telnetLogger);
    telnetLogger = new TelnetLogger(telnetClient, TELNET_APP_NAME, level);
    logManager->addHandler(telnetLogger);
    telnetLogger->enable();
}

String GetCurrentTelnetLogLevel()
{
  if (telnetLogger->getLevel() == LOG_LEVEL_TRACE)
    return "Trace";
  else if (telnetLogger->getLevel() == LOG_LEVEL_INFO)
    return "Info";
  else if (telnetLogger->getLevel() == LOG_LEVEL_WARN)
    return "Warn";
  else if (telnetLogger->getLevel() == LOG_LEVEL_ERROR)
    return "Error";
  else
  {
    ErrorStats.internal++;
    Log.error ("Unknown log level set : %d", telnetLogger->getLevel());
  }
  return ("Unknown");
}

String setLogLevel(LogLevel level = (LogLevel)0)
{
  if (((LogLevel)level) == 0)
  {
    // Cycle to next level
    switch (telnetLogger->getLevel()) {
      case LOG_LEVEL_TRACE:
        regenerateLogHandler(LOG_LEVEL_INFO);
        break;
      case LOG_LEVEL_INFO:
        regenerateLogHandler(LOG_LEVEL_WARN);
        break;
      case LOG_LEVEL_WARN:
        regenerateLogHandler(LOG_LEVEL_ERROR);
        break;
      case LOG_LEVEL_ERROR:
        regenerateLogHandler(LOG_LEVEL_TRACE);
        break;
      default:
        regenerateLogHandler(LOG_LEVEL_INFO);
        break;
    }
  }
  else
  {
    regenerateLogHandler(level);
  }
  return GetCurrentTelnetLogLevel();
}

void AQWStatus()
{
  byte mac[6];
  telnetClient.println("Most recent status: ");
  telnetClient.println("==================================================");
  telnetClient.printlnf("AQW name:         %s", eeprom.node_name);
  telnetClient.printlnf("Telnet Log Level: %s", GetCurrentTelnetLogLevel().c_str());
  telnetClient.printlnf("IP Address:       %s", WiFi.localIP().toString().c_str());
  WiFi.macAddress(mac);
  telnetClient.printlnf("MAC Address:      %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  telnetClient.printlnf("Signal quality:   %d%%    RSSI: %d", rssiToPercent(WiFi.RSSI()), (uint)(WiFi.RSSI()));
  telnetClient.printlnf("MQTT Broker:      %s", eeprom.mqtt_broker);
  telnetClient.printlnf("FW Version:       %s", VERSION);
  telnetClient.printlnf("OLED Display:     %s", (AqwOled) ? "Detected" : "Not Detected");
  telnetClient.printlnf("Update Interval:  %ld ms", AirQual.getInterval());
  int sec = System.uptime(); int h = (sec/3600); byte m = (sec -(3600*h))/60; byte s = (sec -(3600*h)-(m*60));
  telnetClient.printlnf("Uptime:           %02d:%02d:%02d", h, m, s);
  telnetClient.println("Last readings:");
  telnetClient.printlnf("  PM 2.5:         %ld ug/m3", (int32_t)(AirQual.getData().hpma115.data.pm25));
  telnetClient.printlnf("  PM 10:          %ld ug/m3", (int32_t)(AirQual.getData().hpma115.data.pm10));
  telnetClient.printlnf("  Temperature:    %.2f\xC2\xB0 C", AirQual.getData().shtc3.data.temperature);
  telnetClient.printlnf("  Humidity:       %.2f%% rH", AirQual.getData().shtc3.data.humidity);
  telnetClient.printlnf("  VOC Index:      %ld", AirQual.getData().sgp40.data.tvoc);
  telnetClient.printlnf("  LiPo Battery:   %.2fV    %.2f%%", Battery_Voltage, Battery_Percent);
  telnetClient.printlnf("  Power Source:   %s", (onBatteryPower) ? "Battery" : "USB");
  telnetClient.println("Errors: ");
  telnetClient.printlnf("  Hardware:       %d", ErrorStats.hardware);
  telnetClient.printlnf("  MQTT:           %d", ErrorStats.mqtt);
  telnetClient.printlnf("  Wifi:           %d", ErrorStats.wifi);
  telnetClient.printlnf("  Internal:       %d", ErrorStats.internal);
  telnetClient.println("==================================================");
}

void telnetProcessInput(char *txt, TCPClient telnetClient)
{
//  Log.info ("Read from telnet; `%s`", txt);

  char ch = txt[0];
  switch (ch)
  {
    case 'R':
    case 'r':
      Log.warn("Restart requested from telnet client");
      telnetClient.write ("Restarting ESP32...");
      telnetLogger->disable();
      telnetClient.stop();
      delay(100);
      System.reset();
      break;
    case 'Q':
    case 'q':
      Log.info ("Closing telnet connection");
      telnetClient.write ("Goodbye...");
      telnetLogger->disable();
      telnetClient.stop();
      delay(100);
      break;
    case 'P':
    case 'p':
      telnetClient.println("EEPROM contents:");
      telnetClient.printlnf("Name:        %s", eeprom.node_name);
      telnetClient.printlnf("SSID:        %s", eeprom.ssid);
      telnetClient.printlnf("PSK:         %s", eeprom.psk);
      telnetClient.printlnf("mqtt Broker: %s", eeprom.mqtt_broker);
      telnetClient.printlnf("mqtt User:   %s", eeprom.mqtt_user);
      telnetClient.printlnf("mqtt Pass:   %s", eeprom.mqtt_pass);
      break;
    case 'L':
    case 'l':
      Log.warn("Changed telnet log level to: %s", setLogLevel().c_str());
      break;
    case 'T':
    case 't':
      Log.trace("Changed telnet log level to: %s", setLogLevel(LOG_LEVEL_TRACE).c_str());
      break;
    case 'I':
    case 'i':
      Log.info("Changed telnet log level to: %s", setLogLevel(LOG_LEVEL_INFO).c_str());
      break;
    case 'W':
    case 'w':
      Log.warn("Changed telnet log level to: %s", setLogLevel(LOG_LEVEL_WARN).c_str());
      break;
    case 'E':
    case 'e':
      if (tolower(txt[1]) == 'e')
      {
        /*char *p =*/ strtok (txt, " "); // "ee"
        char *f = strtok(NULL, " "); // EEPROM field
        char *v = strtok(NULL, " "); // data value
        //telnetClient.print("Modify EEPROM: "); telnetClient.print(f); telnetClient.print(" --> "); telnetClient.println(v);
        Log.info("Modify EEPROM: %s --> '%s'", f, v);
        if ( !eepromModifyValue(f, v) )
        {
          telnetClient.println ("Invalid field name specified!\n");
          telnetClient.println ("Valid field names:");
          telnetClient.println ("   node_name");
          telnetClient.println ("   ssid");
          telnetClient.println ("   psk");
          telnetClient.println ("   mqtt_broker");
          telnetClient.println ("   mqtt_user");
          telnetClient.println ("   mqtt_pass");
        }
        else
        {
          Log.warn("EEProm field '%s' changed to '%s'", f, v);
        }
      }
      else
      {
        Log.error("Changed telnet log level to: %s", setLogLevel(LOG_LEVEL_ERROR).c_str());
      }
      break;
    case 'D':
    case 'd':
      telnetClient.println("Dump log history: ");
      //
      // Everytime Log.Xxx() is called, entries have
      // been collected by the TelnetLogger object.
      // The RecallBufferPrint() member func will print
      // the last n entries to the telnet client.
      //
      telnetLogger->RecallBufferPrint();
      break;
    case 'U':
    case 'u':
      {
        strtok (txt, " "); // "u"
        char *f = strtok(NULL, " "); // update freq in ms
        if ((f == NULL) || (atoi(f) < 0))
          telnetClient.printlnf ("Invalid update freq: '%s'", (f == NULL) ? "<Empty>" : f);
        else
          set_interval(f);
      }
      break;
    case 'S':
    case 's':
      AQWStatus();
      break;
    case 'Z':
    case 'z':
      telnetClient.println("Zeroing stats");
      memset (&ErrorStats, 0, sizeof(ERRORS));
      break;
    case 'H':
    case 'h':
    case '?':
    default:
      telnetClient.println("Commands:");
      telnetClient.println("    Q - Quit Telnet session");
      telnetClient.println("    R - Reboot");
      telnetClient.println("    P - Dump EEPROM data");
      telnetClient.println("   EE - Modify EEPROM data");
      telnetClient.println("    L - Cycle log level");
      telnetClient.println("    T, I, W, E - Change to log level");
      telnetClient.println("      (Trace, Info, Warning, Error)");
      telnetClient.println("    D - Dump most recent log");
      telnetClient.println("    U <delay> - Change update interval (in milliseconds)");
      telnetClient.println("    S - Status");
      telnetClient.println("    Z - Clear statistics counts");
      break;
  }
}

void telnetLoop()
{
  static bool TelnetClientConnected = false;
  char ch;
  if (telnetClient.connected())
  {
    // echo all available bytes back to the client
    while (telnetClient.available())
    {
      ch = telnetClient.read();
      if (ch == '\r')
      {
        if (strlen(telnetCommand) > 0)
          telnetProcessInput(telnetCommand, telnetClient);
        memset (telnetCommand, 0, TELNET_BUFF_LEN);
      }
      else if ((strlen(telnetCommand) < TELNET_BUFF_LEN) && (isprint(ch)))
      {
        if ((strlen(telnetCommand) == 0) && (ch == ' ')) break;  // Can't start with space
        int p = strlen(telnetCommand);
        telnetClient.write(p);
        telnetCommand[p] = ch;
        telnetCommand[p+1] = '\0';
      }
    }
  }
  else
  {
    if (TelnetClientConnected == true)
    {
      Log.info ("Telnet connection was terminated");
      telnetLogger->disable();
      TelnetClientConnected = false;
    }
    // if no client is yet connected, check for a new connection
    telnetClient = telnetServer.available();
    if (telnetClient.connected())
    {
      telnetLogger->enable();
      TelnetClientConnected = true;
      Log.info ("Incoming telnet connection!");
      // Throw away any handshake bytes received during the first .25 secs
      unsigned long start = millis();
      while ((millis() - start) < 250)
        if (telnetClient.available())
          telnetClient.read();
      memset (telnetCommand, 0, TELNET_BUFF_LEN);
    }
  }
}


////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//
//  Setup
//
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

void setup()
{
  // Set up PC based UART (for debugging)
//  Serial.blockOnOverrun(false);
//  Serial.begin();
  Serial.begin(115200);
  waitFor(Serial.isConnected, 15000);

  delay(1000);

  // Set all error counts to 0
  memset (&ErrorStats, 0, sizeof(ERRORS));

  if ((AqwOled = OledPresent(SSD1306_ADDRESS)))
  {
    Log.info("OLED dislay found at address 0x%02x", SSD1306_ADDRESS);

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    display.begin(SSD1306_SWITCHCAPVCC, SSD1306_ADDRESS); // Address 0x3C for 128x32

    pinMode(BUTTON_A, INPUT_PULLUP);
    pinMode(BUTTON_B, INPUT_PULLUP);
    pinMode(BUTTON_C, INPUT_PULLUP);

    tqInit();
    delay(2000);    // To show Adafruit Logo
    tqAdd(TQ_SHOW, 2500, BANNER, 1, true);
    checkTimeQueue();

    tqAdd(TQ_SHOW, 5000, "Hello\n\rWorld!", 2, false);
  }

  // Read & validate EEPROM data
  if ( !read_eeprom() )
  {
    tqAdd(TQ_SHOW, 30000, "EEPROM Read\n\rFailed!", 2, true);
    serialConsole();
    tqAdd(TQ_SHOW, 30000, "Restarting", 2, true);
    delay(1000);
    System.reset();
  }

  setupWifi();
  if ( !(WiFi.ready()) )
  {
    tqAdd(TQ_SHOW, 30000, "WIFI Conn\n\rFailed!", 2, true);
    serialConsole();
  }

  telnetLogger = new TelnetLogger(telnetClient, TELNET_APP_NAME, LOG_LEVEL_INFO);
  memset (telnetCommand, 0, TELNET_BUFF_LEN);
  // Start the telnet server and connect
  telnetServer.begin();

  // Enable battery status pins
  pinMode(PWR, INPUT);
  pinMode(CHG, INPUT);

  onBatteryPower = !(digitalRead(PWR) == 1);
#ifdef PARTICLE_BENCH
  Particle.variable("onBatteryPower", onBatteryPower);
#endif

  // Set up I2C
  Wire.setSpeed(I2C_CLK_SPEED);
  Wire.begin();

  // Default settings
  AirQualityWingSettings_t defaultSettings =
  { MEASUREMENT_DELAY_MS, //Measurement Interval
    true,                 //Has HPMA115
    true,                 //Has SGP40
    true,                 //Has SHTC3
    HPMA1150_EN_PIN       //HPMA int pin
  };

  // Setup & Begin Air Quality
  AirQual.setup(AirQualityWingEvent, defaultSettings);
  AirQual.begin();

  // Set the default inerval (60 seconds)
  AirQual.setInterval(60000);

#ifdef PARTICLE_BENCH
  // Set up cloud variable
  Particle.function("set_interval", set_interval);
#endif

  if (WiFi.ready())
  {
    mqttConnect();
    mqttCheckConnection();
  }
}


////////////////////////////////////////////////////////
//
//  Periodic Routines
//
////////////////////////////////////////////////////////

/*
 * Look for any button presses on the OLED display
 */
void checkOledButtons()
{
  if (!AqwOled) return;

  if (!digitalRead(BUTTON_A))
  {
    snprintf(logMsg, LOGMSG_LEN, "PM2.5  %d\n\rPM10   %d", (int)(AirQual.getData().hpma115.data.pm25), (int)(AirQual.getData().hpma115.data.pm10));
    tqAdd(TQ_SHOW, 5000, logMsg, 2, true);
    // Debounce switches
    delay(200);
  }
  if (!digitalRead(BUTTON_B))
  {
    snprintf(logMsg, LOGMSG_LEN, "VOC  %d", (int)(AirQual.getData().sgp40.data.tvoc));
    tqAdd(TQ_SHOW, 5000, logMsg, 2, true);
    snprintf(logMsg, LOGMSG_LEN, "Temp %.1f%c\n\rHum  %.1f%%", AirQual.getData().shtc3.data.temperature, (char)247, AirQual.getData().shtc3.data.humidity);
    tqAdd(TQ_SHOW, 5000, logMsg, 2, false);
    // Debounce switches
    delay(200);
  }
  if (!digitalRead(BUTTON_C))
  {
    snprintf(logMsg, LOGMSG_LEN, "Battery: %.0f%%\n\rVersion: %s\n\rWifi:    %d%%\n\rIP:   %s",
      Battery_Percent, VERSION, rssiToPercent(WiFi.RSSI()), WiFi.localIP().toString().c_str());
    tqAdd(TQ_SHOW, 20000, logMsg, 1, true);
    // Debounce switches
    delay(200);
  }
}

/*
 * Check queue for any time-based updates to OLED display
 */
void checkTimeQueue()
{
  if (!AqwOled) return;

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

void serialLoop()
{
  if (Serial.available())
  {
    char c = Serial.read();
    if (c != 27) Serial.printlnf ("%c", c);
    switch (c) {
      case 'i':
      case 'I':
        Serial.printlnf ("IP Address: %s", WiFi.localIP().toString().c_str());
        break;
      case 'c':
      case 'C':
        Serial.printlnf ("Clearing EEPROM...");
        clear_eeprom();
        delay(500);
        Serial.printlnf ("Done. You should now restart the device");
        break;
      case 'r':
      case 'R':
        Serial.printlnf ("Rebooting controller");
        if (telnetClient.connected())
        {
          telnetClient.write ("Restarting ESP32...\n");
          telnetLogger->disable();
          telnetClient.stop();
        }
        delay(100);
        System.reset();
        break;
      case 27:
        // Assume it's the mouse scroll wheel
        Serial.read();
        Serial.read();
        break;
      default:
        Serial.printlnf ("Available Commands:");
        Serial.printlnf ("    I - Show IP address");
        Serial.printlnf ("    C - Clear EEPROM");
        Serial.printlnf ("    R - Reboot");
        break;
    }
  }
}

void aqwLoop()
{
  uint32_t err_code = AirQual.process();
  if( err_code != success )
  {
    switch(err_code)
    {
      case shtc3_error:
#ifdef PARTICLE_BENCH
        Particle.publish("err", "shtc3" , PRIVATE, NO_ACK);
#endif
        Log.error("Error shtc3");
        ErrorStats.hardware++;
        break;
      case sgp40_error:
#ifdef PARTICLE_BENCH
        Particle.publish("err", "sgp40" , PRIVATE, NO_ACK);
#endif
        Log.error("Error sgp40");
        ErrorStats.hardware++;
        break;
      case hpma115_error:
#ifdef PARTICLE_BENCH
        Particle.publish("err", "hpma115" , PRIVATE, NO_ACK);
#endif
        Log.error("Error hpma115");
        ErrorStats.hardware++;
        break;
      default:
        break;
    }
  }
}

void checkWifi()
{
  if ( !(WiFi.ready()) )
  {
    Log.warn("Reconnecting wifi...");
    setupWifi();
  }
}

// loop() runs over and over again, as quickly as it can execute.
void loop()
{
  checkWifi();
  if (AqwOled)
  {
    checkOledButtons();
    checkTimeQueue();
  }
  telnetLoop();
  serialLoop();
  aqwLoop();
}