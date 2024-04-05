## AS7341-ESP32-I2C-MAIN ##
This repository contains drivers and example code for the AS7341 11-Channel Multi-Spectral Digital Sensor using the I2C interface. Built around an Adafruit breakout board for the ESP32-C3 devkit. The data sheet can be found here : https://look.ams-osram.com/m/24266a3e584de4db/original/AS7341-DS000504.pdf

### CHANGELOG ###
v1.0 - 4/5/2024 - MSD - Initial version. Utilizes the latest ESP IDF, version 5.2.1 due to updated I2C driver library. This code will initialize the I2C driver, attach the device and read the device ID from the associated register.

