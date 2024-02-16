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
#define Product_ID  0x00
#define Revision_ID 0x01
#define Motion      0x02
#define Delta_X_L   0x03
#define Delta_X_H   0x04
#define Delta_Y_L   0x05
#define Delta_Y_H   0x06
#define SQUAL 0x07
#define Raw_Data_Sum  0x08
#define Maximum_Raw_data  0x09
#define Minimum_Raw_data  0x0A
#define Shutter_Lower 0x0B
#define Shutter_Upper 0x0C
#define Control 0x0D
#define Config1 0x0F
#define Config2 0x10
#define Angle_Tune  0x11
#define Frame_Capture 0x12
#define SROM_Enable 0x13
#define Run_Downshift 0x14
#define Rest1_Rate_Lower  0x15
#define Rest1_Rate_Upper  0x16
#define Rest1_Downshift 0x17
#define Rest2_Rate_Lower  0x18
#define Rest2_Rate_Upper  0x19
#define Rest2_Downshift 0x1A
#define Rest3_Rate_Lower  0x1B
#define Rest3_Rate_Upper  0x1C
#define Observation 0x24
#define Data_Out_Lower  0x25
#define Data_Out_Upper  0x26
#define Raw_Data_Dump 0x29
#define SROM_ID 0x2A
#define Min_SQ_Run  0x2B
#define Raw_Data_Threshold  0x2C
#define Config5 0x2F
#define Power_Up_Reset  0x3A
#define Shutdown  0x3B
#define Inverse_Product_ID  0x3F
#define LiftCutoff_Tune3  0x41
#define Angle_Snap  0x42
#define LiftCutoff_Tune1  0x4A
#define Motion_Burst  0x50
#define LiftCutoff_Tune_Timeout 0x58
#define LiftCutoff_Tune_Min_Length  0x5A
#define SROM_Load_Burst 0x62
#define Lift_Config 0x63
#define Raw_Data_Burst  0x64
#define LiftCutoff_Tune2  0x65

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
        .max_transfer_sz = 4094,    
    };
    ret = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    //SPI device configuration
    spi_device_interface_config_t device_config = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .duty_cycle_pos = 128,
        .clock_speed_hz = 4000000,
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
    uint8_t txdata[2] = {reg | 0x80, value};
    spi_transaction_t t = {
        .tx_buffer = txdata,
        .rx_buffer = NULL,
        .length = 16,
    };
    gpio_set_level(NCS_PIN, 0);
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi2, &t));
    gpio_set_level(NCS_PIN, 1);
}

//Initialize the motion sensor 3 for 400dpi, 7 for 800dpi, 15 for 1600dpi
static void PMW3389_init(const uint8_t DPI){
    uint16_t i = 0;
    uint8_t sromburstaddress = SROM_Load_Burst;
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
    vTaskDelay(pdMS_TO_TICKS(50));

    //read from 0x02 to 0x06
    spiRead(0x02);
    spiRead(0x03);
    spiRead(0x04);
    spiRead(0x05);
    spiRead(0x06);

    //SROM download enable
    spiWrite(0x10, 0x20);
    spiWrite(0x13, 0x1d);
    vTaskDelay(pdMS_TO_TICKS(10));
    spiWrite(0x13, 0x18);
    //SROM address send
    spi_transaction_t sromaddress = {
        .length = 8,
        .rx_buffer = NULL,
        .tx_buffer = &sromburstaddress,
    };
    gpio_set_level(NCS_PIN, 0);
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi2, &sromaddress));
    //SROM data send
    spi_transaction_t sromdata = {
        .length = 4094,
        .rx_buffer = NULL,
        .tx_buffer = &srom[i],
    };
    for (i = 0; i < SROM_LENGTH; i++){
        ESP_ERROR_CHECK(spi_device_polling_transmit(spi2, &sromdata));
    }
    gpio_set_level(NCS_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(15));

    // configuration/settings
	spiWrite(0x10, 0x00); // Rest mode & independant X/Y DPI disabled
	spiWrite(0x0d, 0x00); // Camera angle
	spiWrite(0x11, 0x00); // Camera angle fine tuning
	spiWrite(0x0f, DPI); // DPI
	// LOD Stuff
	spiWrite(0x63, 0x02); // LOD: 0x00 disable lift detection, 0x02 = 2mm, 0x03 = 3mm
	spiWrite(0x2b, 0x10); // Minimum SQUAL for zero motion data (default: 0x10)
	spiWrite(0x2c, 0x0a); // Minimum Valid features (reduce SQUAL score) (default: 0x0a)
    vTaskDelay(pdMS_TO_TICKS(1));
}

void app_main() 
{   //gpio acting as NCS pin
    gpio_set_direction(NCS_PIN, GPIO_MODE_OUTPUT);
    gpio_pulldown_en(NCS_PIN);
    gpio_set_level(NCS_PIN, 1);

    //initialize SPI configurations
    spi_init();

    //initialize PMW3389
    PMW3389_init(15);

    while (1){
        uint8_t deltaxl = spiRead(Delta_X_L);
        uint8_t sromid = spiRead(SROM_ID);
        printf("sromid: %d \t deltax: %d\n", sromid, deltaxl);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}