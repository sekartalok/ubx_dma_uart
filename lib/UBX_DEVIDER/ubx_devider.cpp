#include "ubx_devider.h"


// helper
//16
uint16_t ubx_devider::u2converter(uint8_t h, uint8_t l){
  return (uint16_t) h | (uint16_t) (l << 8);
}
int16_t ubx_devider:: i2converter(uint8_t h,uint8_t l){
  uint16_t val = (uint16_t) h | (uint16_t) (l << 8);
  return(int16_t) val;
}

//32
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

//start the queue
void ubx_devider::begin(ubx_config *cfg){
    event_handler = xQueueCreate(cfg->queue_event_size,sizeof(uint8_t) * UBX_MAX_PACKET_SIZE_GNSS);
    packet_handler= xQueueCreate(cfg->queue_data_size,sizeof(uint8_t) * NAV_PVT_SIZE);

    packet_delay = cfg->data_delay;
    event_delay = cfg->event_delay;

    update_delay = cfg->update_delay;

}


//packet management 
void ubx_devider::update_data(uint8_t *rx_buffer,uint32_t len){
    uint32_t start_read = 0;
    if(is_broken_HL){
        is_broken_HL = false;
        start_read = packet_assembler_HL(rx_buffer);
    }
    else if(is_broken_HF){
        is_broken_HF = false;
        start_read = packet_assambler_HF(rx_buffer);
    }
    
    packet_devider(rx_buffer,len,start_read);
    
}

//queue manager 
void ubx_devider::queue_manager(uint8_t *buffer){
  
    if(buffer[2] == 0x01 && buffer[3] == 0x07 ){
        //gps data
        xQueueSend(packet_handler,buffer,pdMS_TO_TICKS(update_delay));
    }else{
        //gps event 
        xQueueSend(event_handler,buffer,pdMS_TO_TICKS(update_delay));
    }
}

//packet assembler 

uint32_t ubx_devider::packet_assembler_HL(uint8_t *rx_buffer){
    uint8_t buffer[UBX_MAX_PACKET_SIZE_GNSS];
    uint32_t sec_size = 6 - full_size_HL;

    //assemble the header
    memcpy(rx_half_HL + full_size_HL, rx_buffer , 6 - full_size_HL);
    //copy full header
    memcpy(buffer,rx_half_HL,6);

    // check the second header 
    if(buffer[1] !=UBX_HEADER_SYNC_2_GNSS){
        return 0;
    }

    uint32_t size = u2converter(buffer[4],buffer[5]);
    //copy full data
    memcpy(buffer + 6, rx_buffer + sec_size, size + 2);

    //check if data corrupted or not 
    if(!checksum(buffer)){
        return 0;
    }

    queue_manager(buffer);

    return sec_size + 2 + size;
}

uint32_t ubx_devider::packet_assambler_HF(uint8_t *rx_buffer){
    uint8_t buffer[UBX_MAX_PACKET_SIZE_GNSS];
    uint32_t last_size = full_size_HF - first_size_HF;

    memset(buffer,0x00,UBX_MAX_PACKET_SIZE_GNSS);
    //copy the save data
    memcpy(buffer,rx_half_HF,first_size_HF);
    //copy the current data 
    memcpy(buffer + first_size_HF ,rx_buffer,last_size);
    
    //check if data currupted or not
    if(!checksum(buffer)){  
        return 0;
    }
   
    queue_manager(buffer);

    return last_size;

}

//packet devider 
void ubx_devider::packet_devider(uint8_t *rx_buffer,uint32_t master_len ,uint32_t start){
    uint32_t i = start;
    uint8_t buffer[UBX_MAX_PACKET_SIZE_GNSS];


    //while for devide the packet to 1 chunck 
    while((master_len - i) > 0){
        if((master_len - i) < 6){
            //head is not full ( less than 6)
            if(rx_buffer[i] != UBX_HEADER_SYNC_1_GNSS)
            {
                //skip for corrupt data 
                i++;

            } else{

            full_size_HL = master_len - i;
            is_broken_HL = true;
            memset(rx_half_HL ,0x00 ,6);
            memcpy(rx_half_HL,rx_buffer + i,full_size_HL);
            break;

            }


        }
        else if(rx_buffer[i] == UBX_HEADER_SYNC_1_GNSS && rx_buffer[i + 1] == UBX_HEADER_SYNC_2_GNSS){
            uint16_t packet_len = u2converter(rx_buffer[4+i],rx_buffer[5+i]);
            packet_len += UBX_CK_LEN_GNSS + UBX_HEADER_LEN_GNSS;
            
            if(checksum(rx_buffer + i)){
                
                // full packet no lost 
                memset(buffer,0x00,UBX_MAX_PACKET_SIZE_GNSS);
                memcpy(buffer,rx_buffer + i,packet_len);

                queue_manager(buffer);

                i += packet_len;

            }else{
                // to check if packet only half but full head (6) 
                
              
                is_broken_HF = true;
                first_size_HF = master_len - i;
                full_size_HF = packet_len;
                memset(rx_half_HF,0x00,UBX_MAX_PACKET_SIZE_GNSS);
                memcpy(rx_half_HF,rx_buffer + i, master_len - i);
                break;
                
                
            }
        }else{
            // skip for corrupt data 
            i++;
        }

        
    }
}

//validate check and adder 
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

// data collector 
esp_err_t ubx_devider::event_receive(uint8_t *buffer_rx ){
    esp_err_t err;

    memset(buffer_rx,0x00,UBX_MAX_PACKET_SIZE_GNSS);
    err = xQueueReceive(event_handler,buffer_rx,pdMS_TO_TICKS(packet_delay));

    return err;
}

esp_err_t ubx_devider::receive(uint8_t *buffer_rx ){
    esp_err_t err;

    memset(buffer_rx,0x00,UBX_MAX_PACKET_SIZE_GNSS);
    err = xQueueReceive(packet_handler,buffer_rx,pdMS_TO_TICKS(packet_delay));


    return err;
}

uint32_t ubx_devider::in_queue(){
    return (uint32_t )uxQueueMessagesWaiting(packet_handler);
}

uint32_t ubx_devider::event_in_queue(){
    return (uint32_t )uxQueueMessagesWaiting(event_handler);

}



