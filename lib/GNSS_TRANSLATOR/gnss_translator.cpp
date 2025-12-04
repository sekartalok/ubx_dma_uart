#include"gnss_translator.h"

gnss_translator::ubx_mon_ver_fix* gnss_translator::ver_converter(uint8_t *buffer){
    return (ubx_mon_ver_fix *) buffer;
}
void gnss_translator::rx_mon_ver_translate(uint8_t *buffer, ubx_mon_ver *ver,uint32_t size){
    memset(ver,0,sizeof(ubx_mon_ver));
    if (size < 40) return;

    ubx_mon_ver_fix *ver_fix = ver_converter(buffer);
    memcpy(ver->sofgver,ver_fix->softver,30);
    memcpy(ver->harwver,ver_fix->harwver,10);

    uint16_t off_set = 40;

    while((off_set +30 <= size) && (ver->extension_count < MAX_EXTENSION_VER )){
        memcpy(ver->extension[ver->extension_count],buffer + off_set,30);
        ver->extension[ver->extension_count][30] = '\0';

        off_set +=30;
        ver->extension_count++;

    }
    
}


void gnss_translator::rx_nav_translate(uint8_t *buffer ,ubx_nav_pvt *data){
    
    memcpy(data,buffer,sizeof(ubx_nav_pvt));    
}

void gnss_translator::tx_sattelite_translate(uint8_t *buffer ,uint8_t *sattel,uint8_t sattel_size,uint8_t save, bool en){
 

    memcpy(buffer,header_valset,4);

    uint16_t size = sattel_size * 4;
    size += 4;

    uint8_t ADDH = size & 0xFF;
    uint8_t ADDL = (size >> 8) & 0xFF;

    buffer[4] = ADDH;
    buffer[5] = ADDL;

    buffer[6] = 0x00; //version
    buffer[7] = 0x07;
    buffer[8] = 0x00; //reserve
    buffer[9] = 0x00; //reserve

    uint8_t i = 4 ;
    uint8_t j = 0;

    while(j < sattel_size){
        memcpy(buffer + (6 + i), sattelite_reg[sattel[j]],4);
        buffer[14 + i] = en;
        buffer[15 + i] = 0x01;//size of en
        i+=6;
        j++;
        
    }


    

}
void gnss_translator::tx_sattelite_setting_check(uint8_t *buffer, uint8_t sattel, uint8_t save ){

    memcpy(buffer,header_valget,6);

    buffer[6] = 0x00; //version
    buffer[7] = 0x01; //ram
    buffer[8] = 0x00; //reserve
    buffer[9] = 0x00; //reserve
    
    memcpy(buffer + 10 , sattelite_reg[sattel],4);

}

int gnss_translator::rx_ack_check(uint8_t*buffer , ubx_ack_check *data){

    if(buffer[2] == UBX_ACK_ACK_S1 && buffer[3] == UBX_ACK_ACK_S2){
        memcpy(data,buffer,sizeof(ubx_ack_check));
        return 1;
    }else if(buffer[2] == UBX_ACK_NAK_S1 && buffer[3] == UBX_ACK_NAK_S2){
        memcpy(data,buffer,sizeof(ubx_ack_check));
        return 0;
    }
    return -1;
}