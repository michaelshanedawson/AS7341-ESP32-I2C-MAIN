## AS7341-ESP32-I2C-MAIN ##
This repository contains drivers and example code for the AS7341 11-Channel Multi-Spectral Digital Sensor using the I2C interface. Built around an Adafruit breakout board for the ESP32-C3 devkit. The data sheet can be found here : https://look.ams-osram.com/m/24266a3e584de4db/original/AS7341-DS000504.pdf

### CHANGELOG ###
v1.0 - 4/5/2024 - MSD - Initial version. Utilizes the latest ESP IDF, version 5.2.1 due to updated I2C driver library. This code will initialize the I2C driver, attach the device and read the device ID from the associated register.
v2.0 - 4/5/2024 - MSD - The unit is reading spectral data and outputting the raw value to the serial console. Really sensitive and small changes are detected. There is more work to do like organize the code better and get better understanding of how to interpret the data.

