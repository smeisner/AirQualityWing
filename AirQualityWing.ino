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

Adafruit_SGP40 sgp;
Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();

// Update these with values suitable for your network.
const char* mqtt_server = "mqtt.local";
const char* ssid = "wifi-ssid";
const char* password = "wifi-password";
WiFiClient espClient;
PubSubClient client(espClient);

#define BANNER "Feather/HUZZAH32/Air Quality Wing -- V1.4"

const int ledPin = BUILTIN_LED; // Onboard LED to indicate activity
#define INTERVAL 30           // Interval to take readings (seconds)
char mqtt_pub[50];            // Buffer for MQTT msg published
long lastMsg = 0;             // Timestamp of last loop() pass


//
// Air Quality Wing related details
//

// Set up sensor enable PIN
#define SENS_EN_PIN A8        // Enable pin for builtin AQW power supply (which provides 5V)
// HPM Sensor vriables
HardwareSerial HPMA115S0(1);  // Use the builtin UART
#define RXD2 16               // Pin assigments for onboard UART
#define TXD2 17
#define HPMA1150_EN_PIN A9    // Enable pin for HPM sensor


//////////////////////////////////////////////////
///                                            ///
///               Code Start                   ///
///                                            ///
//////////////////////////////////////////////////

void setup()
{
  Serial.begin(115200);
  while (!Serial) delay(100);

  delay(200);
  Serial.println(BANNER);
  Serial.println("Enabling HPM Sensor...");
  pinMode(HPMA1150_EN_PIN, OUTPUT);
  digitalWrite(HPMA1150_EN_PIN, HIGH);
  Serial.println("Enabling Sensor power...");
  pinMode(SENS_EN_PIN, OUTPUT);
  digitalWrite(SENS_EN_PIN, HIGH);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  setup_wifi();
  Serial.println("Connecting to MQTT broker");
  client.setServer(mqtt_server, 1883);

  Serial.println("Connecting to HPM sensor");
  hpma_Start();
  Serial.println("Connecting to i2C devices");
  sgp40_Start();
  shtc3_Start();
  
  Serial.println("Setup() done!");
  digitalWrite(ledPin, LOW);
}

void loop()
{
  long now = millis();

  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  // Publish data every INTERVAL seconds
  if (now - lastMsg > INTERVAL*1000)
  {
    // Turn on LED to indicate we're taking a reading
    digitalWrite(ledPin, HIGH);
    // Save the timestamp
    lastMsg = now;
    // Receive the particle data
    hpma_Read();
    // Read temp, humidity and TVOC/CO2
    sgp40_Read();
    // Done...turn off the LED
    digitalWrite(ledPin, LOW);
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
  Serial.write("Wifi connecting.\n");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.write(".\n");
  }
  Serial.write("Wifi connected.\n");
}

void reconnect()
{
  // Loop until we're reconnected
  Serial.write("MQTT connecting.\n");
  while (!client.connected())
  {
    // Attempt to connect
    if (client.connect("ESP32-AQW-Client"))
    {
      digitalWrite(ledPin, HIGH);
      Serial.write("MQTT connected.\n");
    }
    else
    {
      // Wait 5 seconds before retrying
      delay(5000);
      digitalWrite(ledPin, LOW);
      Serial.write("MQTT cannot connect.\n");
    }
  }
}


//////////////////////////////////////////////////
///                                            ///
///               SHTC3 Sensor                 ///
///                                            ///
//////////////////////////////////////////////////

void shtc3_Start(void)
{
  Serial.println("SHTC3 test");
  if (! shtc3.begin())
  {
    Serial.println("Couldn't find SHTC3");
    int cnt=0;
    while (1) 
    {
      delay(10);
      cnt++; 
      if (cnt > 1000)
        digitalWrite(ledPin, HIGH);
      else
        digitalWrite(ledPin, LOW);
    }
  }
  Serial.println("Found SHTC3 sensor");
}


//////////////////////////////////////////////////
///                                            ///
///               SGP40 Sensor                 ///
///                                            ///
//////////////////////////////////////////////////

void sgp40_Start(void)
{
  Serial.println("SGP40 test with SHT31 compensation");

  if (! sgp.begin())
  {
    int cnt=0;
    Serial.println("SGP40 sensor not found :(");
    while (1) 
    {
      delay(10);
      cnt++; 
      if (cnt > 10000)
        digitalWrite(ledPin, HIGH);
      else
        digitalWrite(ledPin, LOW);
    }
  }

  Serial.print("Found SGP40 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);
}

void sgp40_Read(void)
{
  uint16_t sraw;
  int32_t voc_index;
  sensors_event_t humidity, temp;
  
  shtc3.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  
  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");

  sraw = sgp.measureRaw(temp.temperature, humidity.relative_humidity);
  Serial.print("Raw measurement: ");
  Serial.println(sraw);

  voc_index = sgp.measureVocIndex(temp.temperature, humidity.relative_humidity);
  Serial.print("Voc Index: ");
  Serial.println(voc_index);

  snprintf (mqtt_pub, 16, "%f", temp.temperature);
  client.publish("AQW/temperature", mqtt_pub);
  snprintf (mqtt_pub, 16, "%f", humidity.relative_humidity);
  client.publish("AQW/humidity", mqtt_pub);
  snprintf (mqtt_pub, 16, "%D", voc_index);
  client.publish("AQW/voc_index", mqtt_pub);

}

//////////////////////////////////////////////////
///                                            ///
///               HPMA115S0 Sensor             ///
///                                            ///
//////////////////////////////////////////////////

void hpma_Start(void)
{
  HPMA115S0.begin(9600, SERIAL_8N1, RXD2, TXD2);
  while (!HPMA115S0);
  Serial.println("Pausing 6s for particle sensor...");
  delay(6000);
  Serial.println("Starting HPM autosend");
  start_autosend();
}

void hpma_Read(void)
{
  int pm25;
  int pm10;

  if (!receive_measurement(&pm25, &pm10))
  {
    digitalWrite(ledPin, 0);
    Serial.println("Cannot receive data from HPMA115S0!");
    return;
  }
  snprintf (mqtt_pub, 16, "%D", pm25);
  client.publish("AQW/PM2.5", mqtt_pub);
  snprintf (mqtt_pub, 16, "%D", pm10);
  client.publish("AQW/PM10", mqtt_pub);
  Serial.println("PM 2.5:\t" + String(pm25) + " ug/m3");
  Serial.println("PM 10:\t" + String(pm10) + " ug/m3");
  Serial.println("MQTT published HPMA115S0 data");
}

bool receive_measurement (int *pm25, int *pm10)
{
  while(HPMA115S0.available() < 32);
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
      Serial.println("Checksum fail");
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
  while(HPMA115S0.available() < 2);
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
