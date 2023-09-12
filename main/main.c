#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/i2c.h"

/*Pin configurations for the I2C Bus*/
#define PIN_NUM_SCL 6 //I2C Clock pin - pull up to 3.3V with a 4.7KΩ resistor
#define PIN_NUM_SDA 5 //I2C Data pin - pull up to 3.3V with a 4.7KΩ resistor

/*I2C Address of the AS7341 device*/
#define DEV_ADDR 0x39 //This is the default I2C address for the Adafruit module
#define MASTER_PORT 0
#define I2C_MASTER_TIMEOUT_MS 1000

/*Register addresses for the AS7341*/
#define REG_ENABLE 0x80 //Register to enable the device - R/W - Needed to power the device up and set basic options
#define REG_AUXID 0x90 //Register for the auxillary ID - Read only
#define REG_REVID 0x91 //Register for the revision number of the device - Read only
#define REG_ID 0x92 //Register for the device ID - Read only

#define generic_gpio_pin 3


void i2c_write(uint8_t devreg, uint8_t i2c_data)
{    
    uint8_t write_buf[2] = {devreg, i2c_data};
    i2c_master_write_to_device(MASTER_PORT, DEV_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);   
}

static esp_err_t i2c_read(uint8_t devreg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(MASTER_PORT, DEV_ADDR, &devreg, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/*GPIO Pulse*/
void pulse(uint8_t pin)
{
    gpio_set_level(pin, 1);
    gpio_set_level(pin, 0);
}



void app_main(void)
{
    /*Configure GPIO pins*/
    gpio_reset_pin(generic_gpio_pin);
    gpio_set_direction(generic_gpio_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(generic_gpio_pin, 0);

    /*Configure the I2C driver*/    
    i2c_config_t i2cConf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_NUM_SDA, //GPIO for the SDA line
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = PIN_NUM_SCL, //GPIO for the SCL line
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 2 * 1000 * 1000,
    };

    /*Initialize the I2C driver*/
    i2c_param_config(MASTER_PORT, &i2cConf);

    /*Install the I2C driver*/
    i2c_driver_install(MASTER_PORT,i2cConf.mode,0,0,0);


    /*Need to write a wake up command to the AS7341 prior to use*/
    uint8_t data[1];
    uint8_t dataByte = 0b01011001; 
    i2c_write(REG_ENABLE,dataByte);
    printf("Device configuration is: %u\n", dataByte);
    uint8_t deviceID = i2c_read(REG_ID, data, 1);
    printf("Device ID is: %u\n", deviceID);
    }

