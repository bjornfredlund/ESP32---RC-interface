# ESP32 - RC interface
---------

Project used to remotely interface with a rc car using esp32 microcontroller and esp-idf. Running a PI-controller for speed control using a hallsensor /w magnets. Transmitts telemetry and accepts speed & steering input through NRF24 module. First part of two-part project: [second part](https://github.com/bjornfredlund/ESP32---UART-NRF24-station)

Uses [esp-idf-mirf](https://github.com/nopnop2002/esp-idf-mirf) for NRF24 communication and [Minmea](https://github.com/kosma/minmea) for RMC parsing.

### Modules used:
- NRF24LO1
- Digital hallsensor
- GPS module like Neo-6m 

### Connections:

Module pin | |  ESP32 
--------- | ---------|-|
 Motor PWM |-- |GPIO32 
 Servo  | --|GPIO33 | 
  Hall sensor|-- | GPIO26 
 GPS Tx |-- |GPIO27
 GPS Rx | --|GPIO34
 
 |nRF24L01||ESP32|
|:-:|:-:|:-:|
|MISO|--|GPIO19|
|SCK|--|GPIO18|
|MOSI|--|GPIO23|
|CE|--|GPIO16|
|CSN|--|GPIO17|
|GND|--|GND|
|VCC|--|3.3V|

Power drawn from ESC

