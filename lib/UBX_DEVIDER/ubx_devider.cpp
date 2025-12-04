#include "ubx_devider.h"


// helper
uint16_t ubx_devider::u2converter(uint8_t h, uint8_t l){
  return (uint16_t) h | (uint16_t) (l << 8);
}
int16_t ubx_devider:: i2converter(uint8_t h,uint8_t l){
  uint16_t val = (uint16_t) h | (uint16_t) (l << 8);
  return(int16_t) val;
}

uint32_t ubx_devider::u4converter(uint8_t h1, uint8_t l1, uint8_t h2, uint8_t l2){
  uint16_t temp_h;
  uint16_t temp_l;

  temp_h = u2converter(h1,l1);
  temp_l = u2converter(h2,l2);

  return (uint32_t) temp_h | (temp_l << 16);
}
int32_t ubx_devider::i4converter(uint8_t h1 ,uint8_t l1, uint8_t h2 ,uint8_t l2){
  uint16_t temp_h;
  uint16_t temp_l;

  temp_h = u2converter(h1,l1);
  temp_l = u2converter(h2,l2);

  uint32_t val = temp_h | (temp_l << 16);
  return (int32_t) val;
}


void ubx_devider::begin(uint8_t queue_size){
    packet_handler= xQueueCreate(queue_size,sizeof(uint8_t)*UBX_MAX_PACKET_SIZE_GNSS);
}
void ubx_devider::update_data(uint8_t *rx_buffer,uint32_t len,uint8_t delay){
    uint32_t start_read = 0;
    if(is_broken){
        is_broken = false;
        start_read = packet_assambler(rx_buffer,delay);
    }
    packet_devider(rx_buffer,len,start_read,delay);
    memset(rx_buffer,0x00,UBX_MAX_PACKET_SIZE_GNSS);
}
uint32_t ubx_devider::packet_assambler(uint8_t *rx_buffer,uint8_t delay){
    uint8_t buffer[UBX_MAX_PACKET_SIZE_GNSS];
    memset(buffer,0x00,UBX_MAX_PACKET_SIZE_GNSS);
    uint32_t last_size = full_size - first_size;

    memcpy(buffer,rx_half,first_size);
    memcpy(buffer + first_size ,rx_buffer,last_size);
    memset(rx_half,0x00,UBX_MAX_PACKET_SIZE_GNSS);

    if(!checksum(buffer)){  
        return 0;
    }
   

    xQueueSend(packet_handler,buffer,pdMS_TO_TICKS(delay));


    return last_size - 1;

}
void ubx_devider::packet_devider(uint8_t *rx_buffer,uint32_t master_len ,uint32_t start,uint8_t delay){
    uint32_t i = start;
    uint8_t buffer[UBX_MAX_PACKET_SIZE_GNSS];
    while(i < (master_len)){
        if(rx_buffer[i] == 0xb5 && rx_buffer[i + 1] == 0x62){
            uint16_t packet_len = u2converter(rx_buffer[4+i],rx_buffer[5+i]);
            packet_len += UBX_CK_LEN_GNSS + UBX_HEADER_LEN_GNSS;
            
            if(checksum(rx_buffer + i)){
           
                memset(buffer,0x00,UBX_MAX_PACKET_SIZE_GNSS);
                memcpy(buffer,rx_buffer + i,packet_len);

                xQueueSend(packet_handler,buffer,pdMS_TO_TICKS(delay));

                i += packet_len;

            }else{
                // to check if packet only half 
                
                if((i + packet_len) >= master_len){
                    is_broken = true;
                    first_size = master_len - i;
                    full_size = packet_len;

                    memcpy(rx_half,rx_buffer + i, master_len - i);
                    break;
                }
                i++;
            }
        }else{
            i++;
        }

        
    }
}

uint16_t ubx_devider::checksum_calculate(uint8_t *buffer,uint8_t *ck_a,uint8_t *ck_b){
    int16_t len = u2converter(buffer[4],buffer[5]);
    len += UBX_HEADER_LEN_GNSS;

    *ck_a =0;
    *ck_b =0;

    uint16_t i = 2;
    while(i < len){
        *ck_a += buffer[i];
        *ck_b += *ck_a;
        i++;
    }
    return len;

}
bool ubx_devider::checksum( uint8_t *buffer_rx ){
    uint8_t ck_a =0;
    uint8_t ck_b =0;

    uint16_t len = checksum_calculate(buffer_rx,&ck_a,&ck_b);

    return ((ck_a == buffer_rx[len]) && (ck_b == buffer_rx[len+1]));
}

void ubx_devider::addchecksum( uint8_t *buffer_tx){

    uint8_t ck_a =0;
    uint8_t ck_b =0;

    uint16_t len = checksum_calculate(buffer_tx,&ck_a,&ck_b);

    buffer_tx[len] = ck_a;
    buffer_tx[len + 1] = ck_b;




}

esp_err_t ubx_devider::receive(uint8_t *buffer_rx , uint8_t delay){
    esp_err_t err;

    memset(buffer_rx,0x00,UBX_MAX_PACKET_SIZE_GNSS);
    err = xQueueReceive(packet_handler,buffer_rx,pdMS_TO_TICKS(delay));


    return err;
}

uint32_t ubx_devider::in_queue(){
    return (uint32_t )uxQueueMessagesWaiting(packet_handler);
}





