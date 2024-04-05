/*
*   main.c
*
*   Drivers and example code for the AS7341 11-Channel Multi-Spectral Digital Sensor using the I2C interface. Built around an Adafruit breakout board for the ESP32-C3 devkit.
*
*   Author: Michael Dawson
*   michaelshanedawson@gmail.com
*
*   Written: 4/5/2024
*   See README.md for changelog
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

/*Pin configurations for the I2C Bus*/
#define PIN_NUM_SCL 6 //I2C Clock pin - pull up to 3.3V with a 4.7KΩ resistor
#define PIN_NUM_SDA 5 //I2C Data pin - pull up to 3.3V with a 4.7KΩ resistor

/*I2C Address of the AS7341 device*/
#define DEV_ADDR 0x39 //This is the default I2C address for the Adafruit module

/*Register addresses for the AS7341*/
#define REG_ENABLE 0x80 //Register to enable the device - R/W - Needed to power the device up and set basic options
#define REG_AUXID 0x90 //Register for the auxillary ID - Read only
#define REG_REVID 0x91 //Register for the revision number of the device - Read only
#define REG_DEVID 0x92 //Register for the device ID - Read only

#define generic_gpio_pin 3

/*Declare prototypes*/
void device_address_scan(i2c_master_bus_handle_t bus);

void device_address_scan(i2c_master_bus_handle_t bus)
{
    uint8_t numDevices = 0;    
    printf("Scanning I2C Bus for devices ------------- \n");
    for(uint8_t address = 1; address < 127; address++)
    {
        uint8_t Status = i2c_master_probe(bus, address, -1);
        if(Status == 0)
        {
            printf("I2C device found at address: %#x \n",address);
            numDevices++;
        }                 
    }

    printf("Total number of devices on the I2C bus is: %u \n\n\r", numDevices);

    return;
}

void i2c_write(uint8_t devreg, uint8_t i2c_data)
{    
    uint8_t write_buf[2] = {devreg, i2c_data};
    //i2c_master_write_to_device(MASTER_PORT, DEV_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);   
}

uint8_t i2c_read(i2c_master_dev_handle_t i2c, uint8_t devreg)
{
    uint8_t write_buf[1] = {devreg};
    uint8_t read_buf[1] = {0x00};
    i2c_master_transmit_receive(i2c, write_buf, sizeof(write_buf), read_buf, sizeof(write_buf), -1);
    return(uint8_t)read_buf[0];
}

/*GPIO Pulse*/
void pulse(uint8_t pin)
{
    gpio_set_level(pin, 1);
    gpio_set_level(pin, 0);
}



void app_main(void)
{
    esp_err_t status;
    /*Configure GPIO pins*/
    gpio_reset_pin(generic_gpio_pin);
    gpio_set_direction(generic_gpio_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(generic_gpio_pin, 0);

    /*Configure the I2C driver*/
    i2c_master_bus_config_t i2cConf = {
        .i2c_port = -1,
        .sda_io_num = PIN_NUM_SDA,
        .scl_io_num = PIN_NUM_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };

    /*Initialize the I2C driver*/
    i2c_master_bus_handle_t i2cBus_handle;
    i2c_new_master_bus(&i2cConf, &i2cBus_handle);

    /*Configure the I2C device*/
    i2c_device_config_t i2cDevice = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DEV_ADDR,
        .scl_speed_hz = 100000,
    }; 

    /*Install the I2C device*/
    i2c_master_dev_handle_t i2c_handle;
    i2c_master_bus_add_device(i2cBus_handle, &i2cDevice, &i2c_handle);

    status = i2c_master_probe(i2cBus_handle, DEV_ADDR, -1);
    if(status != 0)
    {
        printf("Device not found, error code is: ");
        printf("%u \n\n", status);
    }

    /* Here we will start the code */
    device_address_scan(i2cBus_handle); //Just to check the bus for devices, basic I2C scanner
    uint8_t deviceID = i2c_read(i2c_handle, REG_DEVID); //Get and print out the device ID. As of this code version it is 0x24
    printf("Device ID is: %#x\n", (deviceID & 0xFC));    
    }

