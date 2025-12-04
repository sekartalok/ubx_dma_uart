#pragma once

#ifndef GNSS_TRANSLATOR
#define GNSS_TRANSLATOR

#define MAX_EXTENSION_VER 10


#include <Arduino.h>
 
typedef enum{
    UBX_GPS,
    UBX_GPS_L1C_A,
    UBX_SBAS,
    UBX_SBAS_L1C_A,
    UBX_GALILEO,
    UBX_GALILEO_E1,
    UBX_BEIDOU,
    UBX_BEIDOU_B1I,
    UBX_BEIDOU_B1C,
    UBX_QZSS,
    UBX_QZSS_L1C_A,
    UBX_QZSS_L1S
    
}ubx_sattelite;

typedef enum{
    UBX_SETTING_DISABLE,
    UBX_SETTING_ENABLE
}ubx_enable_disable;

typedef enum{
    UBX_RAM_LAYER = 0x01,
    UBX_BBR_LAYER = 0x02,
    UBX_FLASH_LAYER = 0x04,
    UBX_ALL_LAYER = 0x07

}ubx_ram_layer;

typedef enum{
    UBX_NAV_PVT_S1 = 0x01,
    UBX_NAV_PVT_S2 = 0x07,
    UBX_MON_VER_S1 = 0x0A,
    UBX_MON_VER_S2 = 0x04,
    UBX_ACK_ACK_S1 = 0x05,
    UBX_ACK_ACK_S2 = 0x01,
    UBX_ACK_NAK_S1 = 0x05,
    UBX_ACK_NAK_S2 = 0x00,
    UBX_VAL_SET_S1 = 0x06,
    UBX_VAL_SET_S2 = 0x8A,
    UBX_VAL_GET_S1 = 0x06,
    UBX_VAL_GET_S2 = 0x8B
}ubx_msg_id;

typedef enum{
    UBX_HEADER_S1 = 0xB5,
    UBX_HEADER_S2 = 0x62

}ubx_header;

typedef struct __attribute__((packed)) {
    uint32_t itow;
    uint16_t year;
    uint8_t mouth;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t valid;
    uint32_t tacc;
    int32_t nano;
    uint8_t fix_type;
    uint8_t flag;
    uint8_t flag2;
    uint8_t num_sv;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t h_msl;
    uint32_t h_acc;
    uint32_t v_acc;
    int32_t vel_n;
    int32_t vel_e;
    int32_t vel_d;
    int32_t g_speed;
    int32_t head_mot;
    uint32_t s_acc;
    uint32_t head_acc;
    uint16_t p_dop;
    uint16_t flag3;
    uint8_t reserve[4];
    int32_t head_veh;
    int16_t mag_dec;
    uint16_t mag_acc;

    
}ubx_nav_pvt;

typedef struct __attribute__ ((packed)){
    uint8_t cls_id;
    uint8_t msg_id;

}ubx_ack_check;


typedef struct {
    char sofgver[31];
    char harwver[11];

    char extension[MAX_EXTENSION_VER][31];
    uint8_t extension_count;
}ubx_mon_ver;


class gnss_translator{
    private:

    const uint8_t header_valset[4] = {UBX_HEADER_S1,UBX_HEADER_S2,UBX_VAL_SET_S1,UBX_VAL_SET_S2};
    const uint8_t header_valget[6] = {UBX_HEADER_S1,UBX_HEADER_S2,UBX_VAL_GET_S1,UBX_VAL_GET_S2,0x08,0x00};

    const uint8_t sattelite_reg[12][4] =
    {{0x1F,0x00,0x31,0x10},
     {0x01,0x00,0x31,0x10},
     {0x20,0x00,0x31,0x10},
     {0x05,0x00,0x31,0x10},
     {0x21,0x00,0x31,0x10},
     {0x07,0x00,0x31,0x10},
     {0x22,0x00,0x31,0x10},
     {0x0D,0x00,0x31,0x10},
     {0x0F,0x00,0x31,0x10},
     {0x24,0x00,0x31,0x10},
     {0x12,0x00,0x31,0x10},
     {0x14,0x00,0x31,0x10}
    };

    typedef struct __attribute__((packed)) {
        char softver[30];
        char harwver[10];
    }ubx_mon_ver_fix;

    ubx_mon_ver_fix* ver_converter(uint8_t *buffer);

    public:

    void tx_sattelite_setting_check(uint8_t *buffer, uint8_t sattel, uint8_t save );
    void tx_sattelite_translate(uint8_t *buffer ,uint8_t *sattel,uint8_t sattek_size,uint8_t save, bool en);
    void rx_nav_translate(uint8_t *buffer ,ubx_nav_pvt *data);
    void rx_mon_ver_translate(uint8_t *buffer, ubx_mon_ver *ver,uint32_t size);
    int rx_ack_check(uint8_t*buffer , ubx_ack_check data);
    





};


#endif