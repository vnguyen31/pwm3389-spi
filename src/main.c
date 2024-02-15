//includes
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "srom.h"

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
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &device_config, &spi2));
}

//function to read from a register
static uint8_t spiRead(uint8_t reg){
    uint8_t txdata = reg;
    uint8_t rxdata;
    spi_transaction_t t1 = {
        .length = 8,
        .tx_buffer = &txdata,
        .rx_buffer = NULL,
    };
    gpio_set_level(NCS_PIN, 0);
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi2, &t1));
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
    spi_transaction_t t = {
        .tx_buffer = txdata,
        .rx_buffer = NULL,
        .length = 16,
    };
    gpio_set_level(NCS_PIN, 0);
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi2, &t));
    gpio_set_level(NCS_PIN, 1);
}

//Initialize the motion sensor
static void PAW3389_init(const uint8_t DPI){
    const uint8_t *psrom = srom;
    gpio_set_level(NCS_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(3));

    //shutdown first
    gpio_set_level(NCS_PIN, 0);
    spiWrite(0x3b, 0xb6);
    gpio_set_level(NCS_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(300));

    //reset SPI port
    gpio_set_level(NCS_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(NCS_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1));

    //power up reset
    gpio_set_level(NCS_PIN, 0);
    spiWrite(0x3a, 0x5a);
    gpio_set_level(NCS_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(30));

    //SROM download enable
    spiWrite(0x10, 0x20);
    spiWrite(0x13, 0x1d);
    vTaskDelay(pdMS_TO_TICKS(10));
    spiWrite(0x13, 0x18);
    //SROM data write
    spi_transaction_t srom_burst = {
        .addr = 0x62,
        .length = 4096,
        .rx_buffer = NULL,
        .tx_buffer = psrom++,
    };
    gpio_set_level(NCS_PIN, 0);
    for (uint16_t i = 0; i < SROM_LENGTH; i++){
        ESP_ERROR_CHECK(spi_device_polling_transmit(spi2, &srom_burst));
    }
    gpio_set_level(NCS_PIN, 1);

    //read from 0x02 to 0x06
    spiRead(0x02);
    spiRead(0x03);
    spiRead(0x04);
    spiRead(0x05);
    spiRead(0x06);
}

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
        
    }
}