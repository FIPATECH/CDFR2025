// ==================== uart_handler.cpp (Mega 2560) ====================
// Two‑port UART manager (Serial2 ⇄ STM32, Serial3 ⇄ Jetson Nano)
// Strategy is now split into 3 mini‑frames (color, own zone, enemy zone)
// so that every frame ≤10 bytes passes even on slow rise‑time lines.
// -----------------------------------------------------------------------
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include "task.h"

#include "uart_handler.h"
#include "uart_commands.h"
#include "circular_buffer.h"
#include <string.h>

// ---------- compile‑time parameters ----------
#define FIFO_SPACING_MS 1              // delay between mini‑frames
#define HEADER 0x4A

// ---------- buffers TX per port ----------
static CircularBuffer txCB2;  // Serial2 ↔ STM32
static CircularBuffer txCB3;  // Serial3 ↔ Jetson
static volatile bool  isTx2 = false, isTx3 = false;
static uint8_t tx_byte2 = 0, tx_byte3 = 0;

// ---------- RX decoders ----------
typedef struct {
  uint8_t  state;
  uint16_t fn;
  uint16_t len;
  uint8_t  payload[32];
  uint16_t idx;
} decoder_t;
static decoder_t dec2 = {0}, dec3 = {0};

// ---------- checksum ----------
static uint8_t calc_cs(uint16_t fn, uint16_t len, const uint8_t *pl) {
  uint8_t cs = HEADER;
  cs ^= fn >> 8;  cs ^= fn;
  cs ^= len>> 8;  cs ^= len;
  for (uint16_t i = 0; i < len; ++i) cs ^= pl[i];
  return cs;
}

// ---------- helpers ----------
static inline bool cb_is_full(CircularBuffer *cb) {
  return (((cb->head + 1) % CB_SIZE) == cb->tail);
}

static void UART_Send_Bytes_Port(uart_port_t port,const uint8_t *data,uint16_t len);
static void UART_Process_Decoded_Message_Port(uart_port_t src,uint16_t fn,uint16_t len,const uint8_t *pl);

// =========== API TX HIGH‑LEVEL ===========
void UART_Encode_And_Send_Message_Port(uart_port_t port,uint16_t fn,uint16_t len,const uint8_t *pl) {
  uint16_t tot = 1 + 2 + 2 + len + 1;
  uint8_t buf[32];               // len <= 25 in our use‑case
  uint16_t p = 0;
  buf[p++] = HEADER;
  buf[p++] = fn >> 8;   buf[p++] = fn & 0xFF;
  buf[p++] = len>> 8;   buf[p++] = len & 0xFF;
  if (len && pl) { memcpy(&buf[p], pl, len); p += len; }
  buf[p++] = calc_cs(fn, len, pl);
  UART_Send_Bytes_Port(port, buf, p);
}

void UART_Encode_And_Send_Message(uint16_t fn,uint16_t len,const uint8_t *pl) {
  UART_Encode_And_Send_Message_Port(UART_PORT_3, fn, len, pl); }

void UART_Send_Text(const char *txt) {
  UART_Encode_And_Send_Message_Port(UART_PORT_3, UART_CMD_TEXT, strlen(txt),(const uint8_t*)txt); }

// ---------- wrappers (legacy) ----------
void uart_send_start(){ UART_Encode_And_Send_Message_Port(UART_PORT_3, UART_CMD_START_MATCH, 0, nullptr);} 
void uart_send_stop (){ UART_Encode_And_Send_Message_Port(UART_PORT_3, UART_CMD_STOP_MATCH , 0, nullptr);} 
void uart_send_pong (){ UART_Encode_And_Send_Message_Port(UART_PORT_3, UART_CMD_PONG       , 0, nullptr);} 

// =========== TX low‑level ===========
static void UART_Send_Bytes_Port(uart_port_t port,const uint8_t *data,uint16_t len) {
  CircularBuffer *cb = (port==UART_PORT_2)?&txCB2:&txCB3;
  volatile bool  *flag = (port==UART_PORT_2)?&isTx2:&isTx3;
  uint8_t        *first = (port==UART_PORT_2)?&tx_byte2:&tx_byte3;
  for(uint16_t i=0;i<len;++i){ while(cb_is_full(cb)){ taskYIELD(); } circular_buffer_push(cb,data[i]); }
  if(!*flag && circular_buffer_pop(cb,first)){
     *flag=true;
     if(port==UART_PORT_2) Serial2.write(first,1); else Serial3.write(first,1);
  }
}

// =========== INIT ===========
void uart_init(){
  circular_buffer_init(&txCB2);
  circular_buffer_init(&txCB3);
  Serial2.begin(115200, SERIAL_8N1);
  Serial3.begin(115200, SERIAL_8N2);      // extra stop‑bit for stability
  Serial.println("uart_init: Serial2 & Serial3 ready");
}

// =========== STRATEGY mini‑frames ===========
static void uart_send_strategy(uint8_t color,uint8_t zone,uint8_t enemy){
  UART_Encode_And_Send_Message_Port(UART_PORT_3, 0x0200, 1, &color);
  vTaskDelay(pdMS_TO_TICKS(FIFO_SPACING_MS));
  UART_Encode_And_Send_Message_Port(UART_PORT_3, 0x0201, 1, &zone);
  vTaskDelay(pdMS_TO_TICKS(FIFO_SPACING_MS));
  UART_Encode_And_Send_Message_Port(UART_PORT_3, 0x0202, 1, &enemy);
}

// =========== DECODER helper ===========
static void decode_byte(decoder_t &d,uint8_t c,uart_port_t src){
  enum{W,FMSB,FLSB,LMSB,LLSB,PAY,CS};
  switch(d.state){
    case W:   if(c==HEADER){d.state=FMSB;d.idx=0;} break;
    case FMSB:d.fn=(uint16_t)c<<8; d.state=FLSB; break;
    case FLSB:d.fn|=c; d.state=LMSB; break;
    case LMSB:d.len=(uint16_t)c<<8; d.state=LLSB; break;
    case LLSB:d.len|=c; d.state=d.len?PAY:CS; break;
    case PAY: if(d.idx<sizeof d.payload) d.payload[d.idx++]=c; if(d.idx>=d.len) d.state=CS; break;
    case CS:  {
      uint8_t cs=calc_cs(d.fn,d.len,d.payload);
      if(cs==c) UART_Process_Decoded_Message_Port(src,d.fn,d.len,d.payload);
      d.state=W; } break;
    default: d.state=W; break;
  }
}

// =========== DISPATCH ===========
static void UART_Process_Decoded_Message_Port(uart_port_t src,uint16_t fn,uint16_t len,const uint8_t *pl){
  switch(fn){
    case UART_CMD_STRATEGY:
      Serial.println("strategy recu via STM32 — segmentation");
      if(src==UART_PORT_2 && len){
        /* extraire color, zone, enemy */
        uint8_t c=0,z=0,e=0;
        // format ascii "STRATEGY:c:z:e"
        char buf[32]={0}; memcpy(buf,pl,(len<31)?len:31);
        if(sscanf(buf+9,"%hhu:%hhu:%hhu",&c,&z,&e)==3){
          uart_send_strategy(c,z,e);
        }
      }
      break;

    case UART_CMD_PING:
      UART_Encode_And_Send_Message_Port(src, UART_CMD_PONG, 0, nullptr);
      break;

    default: break;
  }
}

// =========== TASK ===========
void uart_task(void *arg){
  for(;;){
    while(Serial2.available()) decode_byte(dec2,Serial2.read(),UART_PORT_2);
    while(Serial3.available()) decode_byte(dec3,Serial3.read(),UART_PORT_3);

    if(isTx2 && Serial2.availableForWrite()){
      if(circular_buffer_pop(&txCB2,&tx_byte2)) Serial2.write(&tx_byte2,1); else isTx2=false; }
    if(isTx3 && Serial3.availableForWrite()){
      if(circular_buffer_pop(&txCB3,&tx_byte3)) Serial3.write(&tx_byte3,1); else isTx3=false; }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
// =====================================================================
