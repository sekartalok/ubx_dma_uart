#pragma once
#ifndef UBX_DEVIDER
#define UBX_DEVIDER

#include <Arduino.h>
#include <driver/uart.h>
#include <esp_log.h>
#include "esp_intr_alloc.h"
#include "soc/uart_struct.h"
#include "soc/uart_reg.h"

#define UBX_MAX_PACKET_SIZE_GNSS 1024
#define NAV_PVT_SIZE 100

typedef struct {

    uint16_t queue_data_size;
    uint16_t queue_event_size;

    uint8_t data_delay;
    uint8_t event_delay;

    uint8_t update_delay;


} ubx_config;

typedef enum{
    UBX_BAUNDRATE_GNSS = 115200,
    UBX_HEADER_LEN_GNSS = 6,
    UBX_HEADER_SYNC_1_GNSS = 0xb5,
    UBX_HEADER_SYNC_2_GNSS = 0x62,
    UBX_CK_LEN_GNSS = 2
}UBX_GENERAL_GNSS;

class ubx_devider{
    private:

    // HALF PACKET DEVIDER
    uint8_t rx_half[UBX_MAX_PACKET_SIZE_GNSS];
    bool is_broken{false};

    uint32_t first_size;
    uint32_t full_size;

    //packet wraper
    QueueHandle_t packet_handler;
    QueueHandle_t event_handler;

    uint8_t packet_delay{1};
    uint8_t event_delay{1};
    uint8_t update_delay{1};
    
    uint16_t checksum_calculate(uint8_t *buffer,uint8_t *ck_a,uint8_t *ck_b);
    void packet_devider(uint8_t *buffer,uint32_t master_len,uint32_t start);
    uint32_t packet_assambler(uint8_t *buffer);

    void queue_manager(uint8_t *buffer);

    public:
        //ESP 32 UART CONFIG 8N1
    uart_config_t config = {
        .baud_rate = UBX_BAUNDRATE_GNSS,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };

    void begin(ubx_config *cfg);
    void addchecksum( uint8_t *buffer_rx);
    bool checksum( uint8_t *buffer_rx );
    void update_data(uint8_t *buffer_rx,uint32_t len);
    void header_converter( uint8_t *buffer_rx );
    esp_err_t receive(uint8_t *buffer);
    esp_err_t event_receive(uint8_t *buffer);
    uint32_t in_queue();
    uint32_t event_in_queue();

    //aux converter helper
    //16 
    uint16_t u2converter(uint8_t h, uint8_t l);
    int16_t i2converter(uint8_t h,uint8_t l);
    //32
    uint32_t u4converter(uint8_t h1, uint8_t l1, uint8_t h2, uint8_t l2);
    int32_t i4converter(uint8_t h1 ,uint8_t l1, uint8_t h2 ,uint8_t l2);




};


#endif