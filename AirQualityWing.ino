/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Steve Meisner (steve@meisners.net)
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
 * The HUZZAH32 board is one of the few that has the Fetaher foot print and
 * supplies onboard wifi. The wifi allows for MQTT publiching to a broker.
 * 
 * Info on the HUZZAH32 can be found anywhere on the internet, but Adafruit has some good details.
 * https://learn.adafruit.com/adafruit-huzzah32-esp32-feather
 * 
 * The Air Quality Wing (aka, AQW) is a Feather board desgined and built by Jared Wolff. This makes
 * for a nice plug and play...especialy with the 3D printed case he made for it.
 * 
 * Feather spec:
 * https://learn.adafruit.com/adafruit-feather/feather-specification
 * 
 * Originally, Jared designed his software around the Partice line of boards, but Particle focuses
 * more on cellular connection instead of wifi, which is what I needed.
 * 
 * Details on the Particle Xenon board can be found here:
 * https://docs.particle.io/datasheets/discontinued/xenon-datasheet/
 * 
 * WARNING: Much of the online info I found related to the AQW specifies the CCS811 (which
 * was replaced with the SHTC3 on V6 AQW), the Si7021 (which was replaced with the AGP40 on V6  AQW). 
 * The HPMA115 is still used. As of Dec-2021, the current PCB version of the AQW is Version 6.
 * 
 * Details on the AQW can be found on Jared's web site:
 * [Specs] https://www.jaredwolff.com/documentation/air-quality-wing/
 * [Buy] https://www.jaredwolff.com/store/air-quality-wing/
 * [Old] https://www.jaredwolff.com/homemade-indoor-air-quality-sensor/
 * 
 * On Github at: [[ WARNING: This is for the Particle dev environment ]]
 * https://github.com/jaredwolff/air-quality-wing-code
 * https://github.com/jaredwolff/air-quality-wing-library
 * and
 * https://github.com/circuitdojo/air-quality-wing-hardware
 * https://github.com/circuitdojo/air-quality-wing-zephyr-demo
 * https://github.com/circuitdojo/air-quality-wing-zephyr-drivers
 * 
 * Code I used to process the HPM info came from:
 * https://medium.com/@boonsanti/esp32-air-quality-measurement-pm2-5-pm10-with-honeywell-hpma115s0-55f411d08fca
 * 
 * 
 * 
 * This code relies on the following libraries:
 * 
 * Adafruit SHTC3
 *    - Installed by name via IDE Library Manager
 *      https://github.com/adafruit/Adafruit_SHTC3
 *    - Dependencies
 *      - Adafruit Unified Sensor
 *        https://github.com/adafruit/Adafruit_Sensor
 * Adafruit SGP40
 *    - Installed by name via IDE Library Manager
 *      https://github.com/adafruit/Adafruit_SGP40
 *    - Dependencies
 *      - Adafruit SHT31
 *        https://github.com/adafruit/Adafruit_SHT31
 *      - Adafruit Unified Sensor
 *        https://github.com/adafruit/Adafruit_Sensor
 * Felix Galindo's HPMA Sensor
 *    - Manually download zip from repo and add to Arduino IDE
 *      https://github.com/felixgalindo/HPMA115S0.git
 *      [Possible alternative: https://github.com/jedp/PMSensor-HPMA115]
 *      
 * To support OTA updates;
 * ArduinoOTA
 *    - Installed by name via the IDE Library Manager
 *      https://github.com/jandrassy/ArduinoOTA
 * TelnetStream
 *    - Installed by name via IDE Library Manager
 *      https://github.com/jandrassy/TelnetStream
 * Logger
 *    - Installed by name via IDE Library Manager
 *      https://github.com/bakercp/Logger
 * 
 * 
 * I2C Addresses:
 *  - SHTC3 0x70
 *  - SGP40 0x59
 *  
 *  
 *  To do:
 *  - Look at power usage and minimize wherever possible
 *  - Use "EEPROM" for config storage
 *  - Create serial console for configuration
 *  - Enable OTA upgrades
 *  - Add support for SSD display
 *  - If operating on battery, turn on wifi only when needed
 *    ...and turn off fan in HPM until reading taken
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include "Adafruit_SGP40.h"
#include "Adafruit_SHTC3.h"
// OTA
#include "OTA.h"
#include <TelnetStream.h>
#include <Logger.h>
// Secret credentials
#include "arduino_secrets.h"

// Network credentials (set in arduino_secrets.h)
const char ssid[] = SECRET_SSID;
const char password[] = SECRET_PASS;
const char* mqtt_server = SECRET_MQTT_BROKER;

#define BANNER "HUZZAH32 Feather/Air Quality Wing -- V1.6"

// Global device structs
WiFiClient aqwClient;
PubSubClient mqttClient(aqwClient);
Adafruit_SGP40 sgp;
Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();
HardwareSerial HPMA115S0(1);  // Use the builtin UART

#define INTERVAL 60           // Interval to take readings (seconds)
#define LOGMSG_LEN 64         // Size of log message scratch buffer
char logMsg[LOGMSG_LEN];      // General use buffer for log messages
char mqtt_pub[50];            // Buffer for MQTT msg published

//
// Air Quality Wing hardware related details
//
// Set up sensor enable PIN
#define SENS_EN_PIN A8        // Enable pin for builtin AQW power supply (which provides 5V)
// HPM Sensor definitions
#define RXD2 16               // Pin assigments for onboard UART
#define TXD2 17
#define HPMA1150_EN_PIN A9    // Enable pin for HPM sensor

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

// Keeping the last 2 readigns to average
typedef struct {
  unsigned long pm25[2];
  unsigned long pm10[2];
  float temperature[2];
  float humidity[2];
  unsigned long voc_index[2];
} HISTORICAL_DATA;

HISTORICAL_DATA HistoricalData;
int ValidationFactor = 20;



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

  setupOTA("AirQualityWing", ssid, password);
  TelnetStream.begin();

  Logger::setLogLevel(Logger::NOTICE);
  Logger::setOutputFunction(localLogger);
  delay(5000);

  Logger::notice(BANNER);

  IPAddress ip = WiFi.localIP();
  Logger::notice("Connected to WiFi network. Connect with Telnet client to: ");
  snprintf (logMsg, LOGMSG_LEN, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  Logger::notice(logMsg);

  Logger::verbose("Connecting to MQTT broker");
  mqttClient.setServer(mqtt_server, 1883);

  Logger::verbose("Enabling HPM Sensor...");
  pinMode(HPMA1150_EN_PIN, OUTPUT);
  digitalWrite(HPMA1150_EN_PIN, HIGH);

  Logger::verbose("Enabling Sensor power...");
  pinMode(SENS_EN_PIN, OUTPUT);
  digitalWrite(SENS_EN_PIN, HIGH);

  Logger::verbose("Connecting to HPM sensor");
  hpma_Start();

  Logger::verbose("Connecting to i2C devices");
  sgp40_Start();
  shtc3_Start();

  memset (&HistoricalData, sizeof(HISTORICAL_DATA), 0);
  
  Logger::notice("Setup() done!");
  digitalWrite(BUILTIN_LED, LOW);
  Logger::notice("---------------------------------------------");
}

void loop()
{
  long now = millis();        // Grab timestamp on every entry
  static long lastMsg = 0;    // Timestamp of last loop() pass

  ArduinoOTA.handle();

  // Publish data every INTERVAL seconds
  if (now - lastMsg > INTERVAL*1000)
  {
    // Turn on LED to indicate we're taking a reading
    digitalWrite(BUILTIN_LED, HIGH);
    // Bring up MQTT broker connection
    connectMqtt();
    // Save the timestamp
    lastMsg = now;
    // Receive the particle data
    hpma_Read();
    // Read temp, humidity and TVOC/CO2
    sgp40_Read();
    // Drop MQTT connection
    disconnectMqtt();
    // Done...turn off the LED
    digitalWrite(BUILTIN_LED, LOW);
    Logger::notice("---------------------------------------------");
  }
}


//////////////////////////////////////////////////
///                                            ///
///               Logging                      ///
///                                            ///
//////////////////////////////////////////////////

void localLogger(Logger::Level level, const char* module, const char* message)
{
  Serial.print(F("AQW Logger: ["));
  Serial.print(Logger::asString(level));
  Serial.print(F("] "));

  if (strlen(module) > 0)
  {
      Serial.print(F(": "));
      Serial.print(module);
      Serial.print(" ");
  }

  Serial.println(message);

  if (TelnetStream.available())
  {
    TelnetStream.print(Logger::asString(level));
    TelnetStream.print(" : ");
    TelnetStream.println(message);
  }
}

//////////////////////////////////////////////////
///                                            ///
///               Wifi & MQTT                  ///
///                                            ///
//////////////////////////////////////////////////

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
      delay(5000);
      digitalWrite(BUILTIN_LED, LOW);
      Logger::error("MQTT cannot connect.");
    }
    ArduinoOTA.handle();
  }
}

void disconnectMqtt()
{
    mqttClient.disconnect();
}

void reconnect()
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
      delay(5000);
      digitalWrite(BUILTIN_LED, LOW);
      Logger::error("MQTT cannot connect.");
    }
    ArduinoOTA.handle();
  }
}


//////////////////////////////////////////////////
///                                            ///
///             Data Validation                ///
///                                            ///
//////////////////////////////////////////////////

bool validateReading(void *data, DataType type)
{
  switch (type)
  {
    case PM25:
      HistoricalData.pm25[0] = HistoricalData.pm25[1];
      HistoricalData.pm25[1] = *(unsigned long *)data;
      if (HistoricalData.pm25[0] == 0)
        return true;
      if (*(unsigned long *)data > (((HistoricalData.pm25[0] + HistoricalData.pm25[1]) / 2) * ValidationFactor))
      {
        Logger::error("PM2.5 value read out of bounds");
        return false;
      }
      else
        return true;
      break;
    case PM10:
      HistoricalData.pm10[0] = HistoricalData.pm10[1];
      HistoricalData.pm10[1] = *(unsigned long *)data;
      if (HistoricalData.pm10[0] == 0)
        return true;
      if (*(unsigned long *)data > (((HistoricalData.pm10[0] + HistoricalData.pm10[1]) / 2) * ValidationFactor))
      {
        Logger::error("PM10 value read out of bounds");
        return false;
      }
      else
        return true;
      break;
    case TEMPERATURE:
      HistoricalData.temperature[0] = HistoricalData.temperature[1];
      HistoricalData.temperature[1] = *(float *)data;
      if (HistoricalData.temperature[0] == 0)
        return true;
      if (*(float *)data > (((HistoricalData.temperature[0] + HistoricalData.temperature[1]) / 2.0) * (float)ValidationFactor))
      {
        Logger::error("Temperature value read out of bounds");
        return false;
      }
      else
        return true;
      break;
    case HUMIDITY:
      HistoricalData.humidity[0] = HistoricalData.humidity[1];
      HistoricalData.humidity[1] = *(float *)data;
      if (HistoricalData.humidity[0] == 0)
        return true;
      if (*(float *)data > (((HistoricalData.humidity[0] + HistoricalData.humidity[1]) / 2.0) * (float)ValidationFactor))
      {
        Logger::error("Humidity value read out of bounds");
        return false;
      }
      else
        return true;
      break;
    case VOC_INDEX:
      HistoricalData.voc_index[0] = HistoricalData.voc_index[1];
      HistoricalData.voc_index[1] = *(unsigned long *)data;
      if (HistoricalData.voc_index[0] == 0)
        return true;
      if (*(unsigned long *)data > (((HistoricalData.voc_index[0] + HistoricalData.voc_index[1]) / 2) * ValidationFactor))
      {
        Logger::error("VOC Index value read out of bounds");
        return false;
      }
      else
        return true;
      break;
    default:
      Logger::error("Data Valididater: Invalid type specified");
      return false;
  }
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
    int cnt=0;
    while (1) 
    {
      delay(10);
      cnt++; 
      if (cnt > 10000)
        digitalWrite(BUILTIN_LED, HIGH);
      else
        digitalWrite(BUILTIN_LED, LOW);
      ArduinoOTA.handle();
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
    while (1)
    {
      delay(10);
      cnt++; 
      if (cnt > 100000)
        digitalWrite(BUILTIN_LED, HIGH);
      else
        digitalWrite(BUILTIN_LED, LOW);
      ArduinoOTA.handle();
    }
  }

  snprintf(logMsg, LOGMSG_LEN, "Found SGP40 serial # %02x%02x%02x",
    sgp.serialnumber[0], sgp.serialnumber[1], sgp.serialnumber[2]);
  Logger::notice(logMsg);
}

void sgp40_Read(void)
{
  uint16_t sraw;
  int32_t voc_index;
  sensors_event_t humidity, temp;
  bool err;

  shtc3.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  
  snprintf(logMsg, LOGMSG_LEN, "Temperature:     %.2f degrees C", temp.temperature);
  Logger::notice(logMsg);
  snprintf(logMsg, LOGMSG_LEN, "Humidity:        %.2f%% rH", humidity.relative_humidity);
  Logger::notice(logMsg);

  sraw = sgp.measureRaw(temp.temperature, humidity.relative_humidity);
  snprintf(logMsg, LOGMSG_LEN, "Raw measurement: %D", sraw);
  Logger::notice(logMsg);

  voc_index = sgp.measureVocIndex(temp.temperature, humidity.relative_humidity);
  snprintf(logMsg, LOGMSG_LEN, "VOC Index:       %d", voc_index);
  Logger::notice(logMsg);

  if (validateReading(&temp.temperature, TEMPERATURE))
  {
    snprintf (mqtt_pub, 16, "%.2f", temp.temperature);
    err = mqttClient.publish("AQW/temperature", mqtt_pub);
  }
  if (validateReading(&humidity.relative_humidity, HUMIDITY))
  {
    snprintf (mqtt_pub, 16, "%.2f", humidity.relative_humidity);
    err &= mqttClient.publish("AQW/humidity", mqtt_pub);
  }
  if (validateReading(&voc_index, VOC_INDEX))
  {
    snprintf (mqtt_pub, 16, "%D", voc_index);
    err &= mqttClient.publish("AQW/voc_index", mqtt_pub);
  }

  if (err == false)
    Logger::error("Error while publiching MQTT data");
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
  HPMA115S0.begin(9600, SERIAL_8N1, RXD2, TXD2);
  while (!HPMA115S0) ArduinoOTA.handle();
  Logger::notice("Pausing 6s for particle sensor...");
  delay(6000);
  Logger::verbose("Starting HPM autosend");
  start_autosend();
}

void hpma_Read(void)
{
  int pm25;
  int pm10;
  bool err;

  if (!receive_measurement(&pm25, &pm10))
  {
    digitalWrite(BUILTIN_LED, 0);
    Logger::warning("Cannot receive data from HPMA115S0!");
    return;
  }

  snprintf(logMsg, LOGMSG_LEN, "PM 2.5:          %d ug/m3", pm25);
  Logger::notice(logMsg);
  snprintf(logMsg, LOGMSG_LEN, "PM 10:           %d ug/m3", pm10);
  Logger::notice(logMsg);

  if (validateReading(&pm25, PM25))
  {
    snprintf (mqtt_pub, 16, "%D", pm25);
    err = mqttClient.publish("AQW/PM2.5", mqtt_pub);
  }
  if (validateReading(&pm10, PM10))
  {
    snprintf (mqtt_pub, 16, "%D", pm10);
    err &= mqttClient.publish("AQW/PM10", mqtt_pub);
  }

  if (err == false)
    Logger::error("Error while publishing MQTT data");
  else
    Logger::verbose("Published HPMA115S0 data via MQTT");

}

bool receive_measurement (int *pm25, int *pm10)
{
  while(HPMA115S0.available() < 32) ArduinoOTA.handle();
  byte HEAD0 = HPMA115S0.read();
  byte HEAD1 = HPMA115S0.read();
  while (HEAD0 != 0x42)
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
    ArduinoOTA.handle();
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
      Logger::error("HPMA115S0 Checksum fail");
      return 0;
    }
    *pm25 = (Data1H * 256) + Data1L;
    *pm10 = (Data2H * 256) + Data2L;
    return 1;
  }
}
 
bool start_autosend(void)
{
  // Start auto send
  byte start_autosend[] = {0x68, 0x01, 0x40, 0x57 };
  HPMA115S0.write(start_autosend, sizeof(start_autosend));
  HPMA115S0.flush();
  delay(500);

  // Then we wait for the response
  while(HPMA115S0.available() < 2) ArduinoOTA.handle();
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
