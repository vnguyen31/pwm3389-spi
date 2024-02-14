//includes
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

//pin definitions
#define SCLK_PIN        13
#define MOSI_PIN        11
#define NCS_PIN         10
#define MISO_PIN        12

//sensor register definitions
#define Product_ID          0x00
#define Revision_ID         0x01
#define Delta_X_L           0x03
#define Delta_X_H           0x04
#define Delta_Y_L           0x05
#define Delta_Y_H           0x06

//handle for spi device (PMW3389 sensor)
spi_device_handle_t spi2;

//function to initialize Serial Peripherals Interface
static void spi_init(){
    //error return
    esp_err_t ret;

    //SPI bus configuration
    spi_bus_config_t bus_config = {
        .miso_io_num = MISO_PIN,
        .mosi_io_num = MOSI_PIN,
        .sclk_io_num = SCLK_PIN,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
        .max_transfer_sz = 16,    
    };
    //check for errors on initializing bus with above configs
    ret = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    //SPI device configuration
    spi_device_interface_config_t device_config = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .duty_cycle_pos = 128,
        .clock_speed_hz = 1000000,
        .mode = 3,
        .spics_io_num = -1,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
        .input_delay_ns = 0,
    };
    //check for errors initializing device 
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &device_config, &spi2));
}

//function to read from a register
static uint8_t spiRead(uint8_t reg){
    uint8_t txdata = reg;
    uint8_t rxdata;

    //create send address transaction
    spi_transaction_t t1 = {
        .length = 8,
        .tx_buffer = &txdata,
        .rx_buffer = NULL,
    };
    //send&receive
    gpio_set_level(NCS_PIN, 0);
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi2, &t1));

    //create send data transaction
    spi_transaction_t t2 = {
        .length = 8,
        .rx_buffer = &rxdata,
        .rxlength = 8,
        .tx_buffer = NULL,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi2, &t2));
    gpio_set_level(NCS_PIN, 1);

    return rxdata;
}

//function to write from a register
static void spiWrite(uint8_t reg, uint8_t value){
    uint8_t txdata[2] = {reg, value};

    //create a write to a register transaction
    spi_transaction_t t = {
        .tx_buffer = txdata,
        .length = 16,
    };
    //write
    gpio_set_level(NCS_PIN, 0);
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi2, &t));
    gpio_set_level(NCS_PIN, 1);
}

//Initialize the motion sensor
//static void PAW3389_init(const uint8_t DPI){

//}

void app_main() 
{   //gpio acting as NCS pin
    gpio_set_direction(NCS_PIN, GPIO_MODE_OUTPUT);
    gpio_pulldown_en(NCS_PIN);
    gpio_set_level(NCS_PIN, 1);

    //initialize SPI configurations
    spi_init();

    //print product ID and Revision ID
    uint8_t pid = spiRead(Product_ID);
    uint8_t rid = spiRead(Revision_ID);
    printf("productID: %d\nrevisionID: %d\n", pid, rid);

    //loop forever part
    while (1){
        spiWrite(0x3b, 0x01);
    }
}