#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

/*Pin Configurations for the SPI bus*/
#define SPI_HOST SPI2_HOST
#define PIN_NUM_MISO 19 //MISO Pin
#define PIN_NUM_MOSI 7 //MOSI Pin
#define PIN_NUM_CS 0 //CS (SS) Pin
#define PIN_NUM_CLK 1 //SPI CLK Pin

#define generic_gpio_pin 3

/*GPIO Pulse*/
void pulse(uint8_t pin)
{
    gpio_set_level(pin, 1);
    gpio_set_level(pin, 0);
}

void spi_transmit(uint8_t data1, uint8_t data2, spi_device_handle_t spi)
{    
    
    esp_err_t ret;
    /*When using SPI_TRANS_CS_KEEP_ACTIVE, bus must be locked/acquired*/
    spi_device_acquire_bus(spi, portMAX_DELAY);

    /*Perform the SPI transaction to send the data.*/
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Data Length in bits
    t.tx_buffer=&data1;               //The data is the calculated delta phase
    t.user=(void*)0;                //D/C needs to be set to 0
    t.flags = SPI_TRANS_CS_KEEP_ACTIVE  ;   //Keep CS active after data transfer
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
   
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&data2;               //The data is the phase data
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.

    /*Release SPI bus*/
    spi_device_release_bus(spi);
}

void app_main(void)
{
    /*Configure GPIO pins*/
    gpio_reset_pin(generic_gpio_pin);
    gpio_set_direction(generic_gpio_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(generic_gpio_pin, 0);

    /*Initializes the SPI driver*/
    esp_err_t ret;
    spi_device_handle_t spi;

    /*Configures the SPI driver*/
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=0,
    };

    /*Device specific SPI settings*/
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=20*1000*1000,           //Clock out at 20 MHz
        .mode=0,                                //SPI mode 0
        .flags=SPI_DEVICE_BIT_LSBFIRST,         //This tells the SPI driver to send the data LSB first, remove for the standard MSB format
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
    };

    /*Initialize the SPI bus*/
    ret=spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    /*Attach the AD9850 to the SPI bus*/
    ret=spi_bus_add_device(SPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);    

    
    spi_transmit(5, 0, spi);
    }

