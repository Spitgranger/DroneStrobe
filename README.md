# ESP32 remote light prototype

Fully remote controlled multifunction light with secured RF connection. Up to 7KM of range. Broadcasting on ISM 915MHz.

## Build

For building for esp32c6 targets, ensure that your ESP-IDF version is at least 5.3. 

```
$ idf.py set-target esp32c6/esp32s3
$ idf.py build
$ idf.py -p <port to serial device> flash
```
### Known Issues / Improvements
- Buttons are bouncy, need to be hardware debounced or software needs to be changed.
- Need to verify the implementation of the listener task, as interrupts are passing events to queue, may have potential implications when more than one button is pressed

### TODO

- Set up remote pairing system
- Add nonce based encryption to messages sent (used calculate HMAC?)
- Brightness button control is weird. (Maybe button needs to be hardware debounced)
- 

### Switch and LED Pin Configurations
|Device|ESP32 Transmitter|ESP32 Receiver|
|:-:|:-:|:-:|
| LED | N/A | GPIO10 |
| Switch 1 | GPIO15 | N/A|
| Switch 2 | GPIO23 | N/A|
| Toggle Switch | GPIO22 | N/A |


### Lora Pin Configurations
|SX127X||ESP32|ESP32-S2/S3|ESP32-C2/C3/C6|
|:-:|:-:|:-:|:-:|:-:|
|RST|--|GPIO16|GPIO38|GPIO3|
|MISO|--|GPIO19|GPIO37|GPIO4|
|SCK|--|GPIO18|GPIO36|GPIO5|
|MOSI|--|GPIO23|GPIO35|GPIO6|
|NSS|--|GPIO15|GPIO34|GPIO7|
|GND|--|GND|GND|GND|
|VCC|--|3.3V|3.3V|3.3V|
