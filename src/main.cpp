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
TaskHandle_t data_process{NULL};
//uart handler
static QueueHandle_t uart_event_queue;
DMA_ATTR uint8_t rx_buffer[RX_BUF_SIZE];
DMA_ATTR uint8_t tx_buffer[TX_BUF_SIZE];


//tx queue handler
QueueHandle_t tx_data_handler;

//data manager and translator 
ubx_devider my_gps;
gnss_translator my_ub;


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

//other task for simulate data proccess 
void data_process_f(void *arg){
  uint8_t buffer[UBX_MAX_PACKET_SIZE_GNSS];

  //struct declare 
  ubx_nav_pvt data;
  ubx_mon_ver ver;
  ubx_ack_check ack;

  while(1){
    //read the data 
    while(my_gps.in_queue()!=0){
      my_gps.receive(buffer );

      my_ub.rx_nav_translate(buffer + 6 ,&data);
      Serial.println("____________________");
      Serial.println(" DATA FROM GPS");
      Serial.println(data.year);
      Serial.printf("speed: %d\n", data.g_speed);
      Serial.printf("latitude: %d\n", data.lat);
      Serial.printf("longtidude: %d\n", data.lon);
      Serial.println("--------------------");

      print_hex("TELE",buffer,my_gps.u2converter(buffer[4],buffer[5])+2+6);
      
    }

    while(my_gps.event_in_queue() != 0){
      my_gps.event_receive(buffer);

      if(buffer [2] == 0x0A && buffer[3] == 0x04){
        my_ub.rx_mon_ver_translate(buffer + 6,&ver,my_gps.u2converter(buffer[4],buffer[5]));
        Serial.printf("\n--- u-blox M10 Info ---\nSoftware: %s\nHardware: %s\n", ver.sofgver, ver.harwver); for(int i=0; i<ver.extension_count; i++) Serial.printf("[%d] %s\n", i+1, ver.extension[i]); Serial.println("-----------------------\n");


      }
      if(my_gps.checksum(buffer)){
        int a;
      // 
        a = my_ub.rx_ack_check(buffer ,&ack);

        if(a == 1){
          Serial.println("ACK");

          Serial.printf("%02X",ack.cls_id);
          Serial.println();
          Serial.printf("%02X",ack.msg_id);
          Serial.println();

        }else if(a == 0){
          Serial.println(" NACK");

          Serial.printf("%02X",ack.cls_id);
          Serial.println();
          Serial.printf("%02X",ack.msg_id);
          Serial.println();
        }


      
      }
    
    }
    //core 0 avoid core pannic 
    vTaskDelay(1);
  }


  

}


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
        

      }
    }
    
    

  }

}



//rx rechiver 
void uart_rx_task(void *arg) {
    uart_event_t event;
 

    while (1) {
        if (xQueueReceive(uart_event_queue, &event, portMAX_DELAY)) {
            
            if (event.type == UART_DATA) {
          
                int len = uart_read_bytes(UART_PORT, rx_buffer, RX_BUF_SIZE, 0);
                


                if (len >= 1) { 
                                 
            
                  uint32_t startmillis = millis();
                  my_gps.update_data(rx_buffer,len);
                  /*
                  Serial.println("___________");
                  Serial.println("time to take");
                  uint32_t current = millis();
                  uint32_t timetake = current - startmillis;
                  Serial.println((uint32_t) timetake);
                  Serial.println("___________");
                  */

                 
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

void uart_setup(){
   //uart setup 
  uart_driver_install(UART_PORT, RX_BUF_SIZE , TX_BUF_SIZE  , EVENT_QUEUE_SIZE, &uart_event_queue, 0);
  uart_param_config(UART_PORT, &my_gps.config);
  uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    

}

void setup() {
  
  Serial.begin(115200);

  // config for ubx devider 
  ubx_config confi{
    .queue_data_size = 700,
    .queue_event_size = 700,
    .data_delay =  0,
    .event_delay = 0,
    .update_delay = 0
  };
  
  my_gps.begin(&confi);
  uart_setup();

  //queuehandler 
  tx_data_handler = xQueueCreate(50,sizeof(uint8_t)*TX_BUF_SIZE);

  xTaskCreatePinnedToCore(uart_rx_task, "uart_rx_task", 4096, NULL, 10, &rx_task, 1);
  xTaskCreatePinnedToCore(uart_tx_task, "uart_tx_task", 4096, NULL, 9, &tx_task, 1);
  xTaskCreatePinnedToCore(data_process_f,"proccess the data ", 50096,NULL,12 ,&data_process, 0);
  

}

void loop() {
  // put your main code here, to run repeatedly:
}

