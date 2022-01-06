# AirQualityWing

Air Quality Monitor based on Wing board from Jared Wolff\
Air Quality Wing with HUZZAH32 uController

## Description

The HUZZAH32 board is one of the few that has the Fetaher foot print and\
supplies onboard wifi. The wifi allows for MQTT publiching to a broker.

Info on the HUZZAH32 can be found anywhere on the internet, but Adafruit has some good details.
- https://learn.adafruit.com/adafruit-huzzah32-esp32-feather
- https://makeabilitylab.github.io/physcomp/esp32/assets/images/AdafruitHuzzah32PinDiagram.png

The Air Quality Wing (aka, AQW) is a Feather board desgined and built by Jared Wolff. This makes\
for a nice plug and play...especialy with the 3D printed case he made for it.

Feather spec:
- https://learn.adafruit.com/adafruit-feather/feather-specification

Originally, Jared designed his software around the Partice line of boards, but Particle focuses\
more on cellular connection instead of wifi, which is what I needed.

Details on the Particle Xenon board can be found here:
- https://docs.particle.io/datasheets/discontinued/xenon-datasheet/

```WARNING:``` Much of the online info I found related to the AQW specifies the CCS811 (which\
was replaced with the SHTC3 on V6 AQW), the Si7021 (which was replaced with the AGP40 on V6  AQW).\
The HPMA115 is still used. As of Dec-2021, the current PCB version of the AQW is Version 6.

Details on the AQW can be found on Jared's web site:
- [Specs] https://www.jaredwolff.com/documentation/air-quality-wing/
- [Buy] https://www.jaredwolff.com/store/air-quality-wing/
- [Old] https://www.jaredwolff.com/homemade-indoor-air-quality-sensor/

On Github at: ```[[ WARNING: This is for the Particle dev environment ]]```
- https://github.com/jaredwolff/air-quality-wing-code
- https://github.com/jaredwolff/air-quality-wing-library \
...and
- https://github.com/circuitdojo/air-quality-wing-hardware
- https://github.com/circuitdojo/air-quality-wing-zephyr-demo
- https://github.com/circuitdojo/air-quality-wing-zephyr-drivers

Code I used to process the HPM info came from:
- https://medium.com/@boonsanti/esp32-air-quality-measurement-pm2-5-pm10-with-honeywell-hpma115s0-55f411d08fca


## Arduino Library Requirements

This code relies on the following libraries:

* Adafruit SHTC3
   - Installed by name via IDE Library Manager\
     https://github.com/adafruit/Adafruit_SHTC3
   - Dependencies
     - Adafruit Unified Sensor\
       https://github.com/adafruit/Adafruit_Sensor
* Adafruit SGP40
   - Installed by name via IDE Library Manager\
     https://github.com/adafruit/Adafruit_SGP40
   - Dependencies
     - Adafruit SHT31\
       https://github.com/adafruit/Adafruit_SHT31
     - Adafruit Unified Sensor\
       https://github.com/adafruit/Adafruit_Sensor
* Felix Galindo's HPMA Sensor
   - Manually download zip from repo and add to Arduino IDE\
     https://github.com/felixgalindo/HPMA115S0.git\ \
     [Possible alternative: https://github.com/jedp/PMSensor-HPMA115]
* To support OTA updates;
   * ArduinoOTA
     - Installed by name via the IDE Library Manager\
       https://github.com/jandrassy/ArduinoOTA
   * TelnetStream
     - Installed by name via IDE Library Manager\
       https://github.com/jandrassy/TelnetStream
   * Logger
     - Installed by name via IDE Library Manager\
       https://github.com/bakercp/Logger

## I2C Addresses:
| Device | I2C Address |
| ------ | ----------- |
| SHTC3 | 0x70 |
| SGP40 | 0x59 |

 
## To do:
 - Look at power usage and minimize wherever possible
 - Use "EEPROM" for config storage
 - Add support for SSD display
 - If operating on battery, turn on wifi only when needed\
   ...and turn off fan in HPM until reading taken\
   ...but this will effectively disable the telnet console & OTA updates
