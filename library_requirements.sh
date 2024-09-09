#! /bin/bash


arduino-cli config init
arduino-cli config add board_manager.additional_urls https://www.pjrc.com/teensy/package_teensy_index.json
arduino-cli update
arduino-cli core install teensy:avr
arduino-cli lib install SD
arduino-cli lib install "Adafruit Unified Sensor"
arduino-cli lib install "Adafruit GPS Library"
arduino-cli lib install "Adafruit ADXL375"
arduino-cli lib install "Adafruit BMP3XX Library"
arduino-cli lib install "Adafruit LSM6DS"
arduino-cli lib install "Adafruit BNO055"
