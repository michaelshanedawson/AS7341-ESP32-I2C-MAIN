/*
*   main.c
*
*   Drivers and example code for the AS7341 11-Channel Multi-Spectral Digital Sensor using the I2C interface. Built around an Adafruit breakout board for the ESP32-C3 devkit.
*
*   Author: Michael Dawson
*   michaelshanedawson@gmail.com
*
*   Written: 4/5/2024
*   Version: 2.0
*   See README.md for changelog
*
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
#define REG_STATUS 0x71 //Status register, bit 0 = 0 if the system is busy taking a reading
#define CH0_LOW 0x95 //The lower 8 bits of the ADC data
#define CH0_HIGH 0x96 //The upper 8 bits of the ADC data

#define generic_gpio_pin 3

uint16_t spectralData = 0x0000; //Data store for the spectral data received

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

void i2c_write(i2c_master_dev_handle_t device, uint8_t devreg, uint8_t i2c_data)
{
    if(devreg >= 0x80)
    {
        uint8_t temp_buf[2] = {0xA9, 0x0};
        i2c_master_transmit(device, temp_buf, sizeof(temp_buf), -1);
    }

    else if(devreg < 0x80)
    {
        uint8_t temp_buf[2] = {0xA9, 0x10};
        i2c_master_transmit(device, temp_buf, sizeof(temp_buf), -1);
    }

    uint8_t write_buf[2] = {devreg, i2c_data};
    i2c_master_transmit(device, write_buf, sizeof(write_buf), -1);
}

uint8_t i2c_read(i2c_master_dev_handle_t device, uint8_t devreg)
{
     if(devreg >= 0x80)
    {
        uint8_t temp_buf[2] = {0xA9, 0x0};
        i2c_master_transmit(device, temp_buf, sizeof(temp_buf), -1);
    }

    else if(devreg < 0x80)
    {
        uint8_t temp_buf[2] = {0xA9, 0x10};
        i2c_master_transmit(device, temp_buf, sizeof(temp_buf), -1);
    }

    uint8_t write_buf[1] = {devreg};
    uint8_t read_buf[1] = {0x00};
    i2c_master_transmit_receive(device, write_buf, sizeof(write_buf), read_buf, sizeof(write_buf), -1);
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

    /* Some basic device testing to validate connection to the I2C bus */
    device_address_scan(i2cBus_handle); //Just to check the bus for devices, basic I2C scanner
    uint8_t deviceID = i2c_read(i2c_handle, REG_DEVID); //Get and print out the device ID. As of this code version it is 0x24
    printf("Device ID is: %#x\n\n", (deviceID & 0xFC));

    /* Here we will begin configuration of the device for us to use */
    uint8_t validateData = 0x00;
    uint8_t dataOut = 0x00;

    /* Enable and configuration register , 0x80 */
    dataOut = 0x00 | 1; //The first thing we do is power up the device itself, before doing anything else.
    i2c_write(i2c_handle, REG_ENABLE, dataOut); //Sends the configuration data

    validateData = i2c_read(i2c_handle,REG_ENABLE);
    printf("Register 0x80 is: %#x\n", validateData);

    /* CONFIG register, 0x70. */
    dataOut = 0x00 | 1 << 3; //Tie the LED control to the LDR pin
    i2c_write(i2c_handle, 0x70, dataOut);

    validateData = i2c_read(i2c_handle,0x70);
    printf("Register 0x70 is: %#x\n", validateData);

    /* LED register, 0x74. This controls the LED tied to the LDR pin. */
    dataOut = 0x00 | 0 << 7 | 0b0000100; //Turn the LED on with a drive strength of 12mA
    i2c_write(i2c_handle, 0x74, dataOut);

    validateData = i2c_read(i2c_handle,0x74);
    printf("Register 0x74 is: %#x\n", validateData);

    /* ATIME register, 0x81. This is part of the configuration of the ADC timing. */
    dataOut = 0x1D; // Sets ATIME to 29
    i2c_write(i2c_handle, 0x81, dataOut);

    validateData = i2c_read(i2c_handle,0x81);
    printf("Register 0x81 is: %#x\n", validateData);

    /* ASTEP, this is two 8bit registers equating to a 16bit value. 0xCA is bits 7:0 and 0xCB is bits 15:8 */
    uint16_t stepSize = 599; //This is the recommended step size
    dataOut = 0x00 | (stepSize & 0x00FF); //We will handle the lower 8 bits first
    i2c_write(i2c_handle, 0xCA, dataOut);
    validateData = i2c_read(i2c_handle,0xCA);
    printf("Register 0xCA is: %#x\n", validateData); 

    dataOut = 0x00 | (uint8_t)(stepSize >> 8); //Here we will handle the upper 8 bits
    i2c_write(i2c_handle, 0xCB, dataOut);
    validateData = i2c_read(i2c_handle,0xCB);
    printf("Register 0xCB is: %#x\n", validateData); 

    /* WTIME register, 0x83. Handles the wait time between measurements*/
    dataOut = 0x1; //Selecting 5.56mS
    i2c_write(i2c_handle, 0x83, dataOut);
    validateData = i2c_read(i2c_handle,0x83);
    printf("Register 0x83 is: %#x\n", validateData);

    /* ITIME registers, handles integration time. There are 3 registers, 0x62, 0x64 and 0x65 */

    /* Now we will enable measurements of the device */
    dataOut = 0x00 | 1 << 3 | 1 << 1 | 1; //Enables the device for measurements
    i2c_write(i2c_handle, REG_ENABLE, dataOut); //Sends the configuration data

    validateData = i2c_read(i2c_handle,REG_ENABLE);
    printf("Register 0x80 is: %#x\n", validateData);

    while(1)
    {
        uint8_t dataReady = i2c_read(i2c_handle,REG_STATUS);
        //printf("Status register is: %#x\n", REG_STATUS);

        dataReady = (dataReady & 0x1);
        //printf("Data ready bit is: %u \n", dataReady);

        if(dataReady == 1)
        {
            //printf("Data is ready! \n");
            spectralData = 0x00 | i2c_read(i2c_handle,CH0_HIGH) << 8 | i2c_read(i2c_handle,CH0_LOW);
            printf("Spectral Data is: %u\n", spectralData);       
        }
        //vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    }

