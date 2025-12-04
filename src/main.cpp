#include <Arduino.h>
#include "ubx_devider.h"
#include "gnss_translator.h"


// --- HARDWARE CONFIG ---
#define UART_PORT       UART_NUM_2
#define UART_TX_PIN     GPIO_NUM_42
#define UART_RX_PIN     GPIO_NUM_46
#define RX_BUF_SIZE     2048
#define RX_QUEUE_SIZE   50
#define TX_BUF_SIZE     2048 
#define TX_QUEUE_SIZE   50
#define EVENT_QUEUE_SIZE 20
//rtos handler
TaskHandle_t rx_task{NULL};
TaskHandle_t tx_task{NULL};
//uart handler
static QueueHandle_t uart_event_queue;
DMA_ATTR uint8_t rx_buffer[RX_BUF_SIZE];
DMA_ATTR uint8_t tx_buffer[TX_BUF_SIZE];

//broken uart packet
uint8_t broken_buffer[RX_BUF_SIZE];
bool broken_pack{false};
uint32_t size_start;
uint32_t full_size;

//tx queue handler
QueueHandle_t tx_data_handler;

const char *RX_TAG = "GPS_RX";

uint32_t test{0};
bool s{false};
bool set_test{false};
ubx_devider my_gps;
gnss_translator my_ub;


/*
void disable_gps(){
  uint8_t buffer[TX_BUF_SIZE];

  memset(buffer,0x00,TX_BUF_SIZE);
  my_ub.tx_sattelite_translate(buffer,UBX_GALILEO,UBX_RAM_LAYER,1);
  my_gps.addchecksum(buffer);
  xQueueSend(tx_data_handler,buffer,portMAX_DELAY );

  

  memset(buffer,0x00,TX_BUF_SIZE);

  my_ub.tx_sattelite_translate(buffer,UBX_GPS,UBX_RAM_LAYER,0);
  my_gps.addchecksum(buffer);
  xQueueSend(tx_data_handler,buffer,portMAX_DELAY );

}
  */
// Print hex utility (your function)
void print_hex(const char *tag, const uint8_t* buffer, int len) {
    Serial.printf("[%s] Packet (%d bytes): ", tag, len);
    for (int i = 0; i < len; i++) {
        Serial.printf("%02X ", buffer[i]);
        if(buffer[i] == UBX_HEADER_SYNC_1_GNSS){
          Serial.println();
        }
    }
    Serial.println();
}

//test data sender
void send(){


  uint8_t buffer[TX_BUF_SIZE];

  memset(tx_buffer,0x00,sizeof(tx_buffer));
 /// print_hex("ARRAY M" ,ubx_cfg_valset_enable_gps,18);
 


  memset(buffer,0x00,sizeof(buffer));
  //
 

   my_ub.tx_sattelite_setting_check(buffer,UBX_BEIDOU,UBX_ALL_LAYER);
  
   my_gps.addchecksum(buffer);
   
   xQueueSend(tx_data_handler,buffer,portMAX_DELAY );

   /*
  memset(buffer,0x00,sizeof(buffer));

   memset(tx_buffer,0x00,sizeof(tx_buffer));
  my_ub.tx_sattelite_setting_check(buffer,UBX_GALILEO,UBX_ALL_LAYER);

     my_gps.addchecksum(buffer);
   
   xQueueSend(tx_data_handler,buffer,portMAX_DELAY );
*/


}

// data validation

/*
void print_packet (){

  if(!my_gps.checksum(rx_buffer)){
    Serial.println("not valid");
    return;
  }
  my_gps.header_converter(rx_buffer);
  if(!(my_gps.pck_header.m_class == 0x01 && my_gps.pck_header.m_id == 0x07) ){
    Serial.println("WRONG");
  }
  my_gps.packet_converter(0);
  /*
  Serial.println("LATIDUDE");
  Serial.println(my_gps.pck_nav_pvt.lat);
  Serial.println("longtidude");
  Serial.println(my_gps.pck_nav_pvt.lon);
  Serial.println("vertical_speed");
  Serial.println(my_gps.pck_nav_pvt.g_speed);
  Serial.println("HEADING");
  Serial.println(my_gps.pck_nav_pvt.head_veh);
  

}
*/

//tx sender
void uart_tx_task(void *arg){

  esp_err_t err;
  uint8_t buffer_temp[TX_BUF_SIZE];
  err = uart_wait_tx_done(UART_PORT, portMAX_DELAY);
  while(1){

    
    
    if(err == ESP_OK){
      if(xQueueReceive(tx_data_handler,buffer_temp,portMAX_DELAY)){
        memcpy(tx_buffer,buffer_temp,TX_BUF_SIZE);
        uart_write_bytes(UART_PORT,tx_buffer,TX_BUF_SIZE);
        err = uart_wait_tx_done(UART_PORT, portMAX_DELAY);
        print_hex("TX GNSS",buffer_temp,18);
        

      }
    }
    

  }

}

void datarechive(){
  uint8_t buffer[UBX_MAX_PACKET_SIZE_GNSS];
  //buffer[0] = 0x00;
  ubx_nav_pvt data;
  ubx_mon_ver ver;
  ubx_ack_check ack;
  //if(buffer[0] != 0x00){

  while(my_gps.in_queue() != 0){
    //Serial.println(my_gps.in_queue());
    my_gps.receive(buffer,10);


    
    if(buffer [2] == 0x0A && buffer[3] == 0x04){
      my_ub.rx_mon_ver_translate(buffer + 6,&ver,my_gps.u2converter(buffer[4],buffer[5]));
      Serial.printf("\n--- u-blox M10 Info ---\nSoftware: %s\nHardware: %s\n", ver.sofgver, ver.harwver); for(int i=0; i<ver.extension_count; i++) Serial.printf("[%d] %s\n", i+1, ver.extension[i]); Serial.println("-----------------------\n");


    }
    if(my_gps.checksum(buffer)){
      int a;
     // 
      a = my_ub.rx_ack_check(buffer ,&ack);
      if(a == 1){
        
        Serial.printf("%02X",ack.cls_id);
        Serial.println();
        Serial.printf("%02X",ack.msg_id);
        Serial.println();

        if(s){
          set_test = true;
        }
        s = true;

        
        
      }
    
    }
   


    print_hex("GNSS",buffer,my_gps.u2converter(buffer[4],buffer[5])+2+6);


  }
  
  //}
 // print_hex("GNSS",buffer,my_gps.u2converter(buffer[4],buffer[5])+2+6);
}

//rx rechiver 
void uart_rx_task(void *arg) {
    uart_event_t event;
 

    while (1) {
        if (xQueueReceive(uart_event_queue, &event, portMAX_DELAY)) {
            
            if (event.type == UART_DATA) {
          
                int len = uart_read_bytes(UART_PORT, rx_buffer, RX_BUF_SIZE, 0);
                


                if (len >= 1) { 
                  send();
                  Serial.println("vvvvvvvvvvv");
                  print_hex(RX_TAG,rx_buffer,len);
                  Serial.println("++++++");                  
            
                  my_gps.update_data(rx_buffer,len,20);
                  datarechive();


                  delay(500);
                 // print_packet();


                 
                 
                }
            }
            else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                uart_flush_input(UART_PORT);
                xQueueReset(uart_event_queue);
                Serial.println("Error: Buffer Overflow");
            }
        }
    }
}


void setup() {
  uint8_t buffer[RX_BUF_SIZE];
  //serial

  uint8_t st[] = {UBX_BEIDOU};
  my_ub.tx_sattelite_translate(buffer,st,sizeof(st),UBX_ALL_LAYER,false);
  
  Serial.begin(115200);
  my_gps.begin(20);

  //queuehandler 
  tx_data_handler = xQueueCreate(50,sizeof(uint8_t)*TX_BUF_SIZE);
  my_gps.addchecksum(buffer);
  xQueueSend(tx_data_handler,buffer,portMAX_DELAY );
  memset(buffer,0x00,TX_BUF_SIZE);



  //uart setup 
  uart_driver_install(UART_PORT, RX_BUF_SIZE , TX_BUF_SIZE  , EVENT_QUEUE_SIZE, &uart_event_queue, 0);
  uart_param_config(UART_PORT, &my_gps.config);
  uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    

  xTaskCreatePinnedToCore(uart_rx_task, "uart_rx_task", 4096, NULL, 10, &rx_task, 1);
  xTaskCreatePinnedToCore(uart_tx_task, "uart_tx_task", 4096, NULL, 9, &tx_task, 1);

  

}

void loop() {
  // put your main code here, to run repeatedly:
}

