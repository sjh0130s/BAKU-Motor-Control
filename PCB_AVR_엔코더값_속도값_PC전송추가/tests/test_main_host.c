#define UNIT_TEST
#include "../main.c"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t fake_time;
static CanFrame queued_can;
static bool can_pending;
static uint8_t queued_ip[4], queued_udp[9];
static uint16_t queued_len;
static bool udp_pending;
static uint8_t last_speed_packet[5];
static uint8_t last_speed_ip[4];
static unsigned speed_sends, speed_polls, motor_sends, stops;
static uint8_t last_control_byte;
void board_init(void) {}
uint32_t millis(void) { return fake_time; }
void delay_ms(uint16_t t) { (void)t; }
void log_text(const char *s) { (void)s; }
void log_auto(int32_t a, int32_t b, int16_t c) {(void)a;(void)b;(void)c;}
bool ethernet_init(void){return true;}
int8_t udp_receive(uint8_t ip[4], uint8_t *data, uint8_t capacity, uint16_t *len) {
  if (!udp_pending) return 0;
  memcpy(ip, queued_ip, 4);
  *len = queued_len;
  memcpy(data, queued_udp, queued_len < capacity ? queued_len : capacity);
  udp_pending = false;
  return 1;
}
void udp_speed_poll(void) { ++speed_polls; }
bool udp_speed_try_send(const uint8_t ip[4], const uint8_t *data, uint8_t len) {
  assert(len==5);
  memcpy(last_speed_ip, ip,4);
  memcpy(last_speed_packet,data,5);
  speed_sends++;
  return true;
}
bool can_init(void){return true;}
bool can_receive(CanFrame *f) {if(!can_pending)return false; *f=queued_can; can_pending=false;return true;}
bool can_send(uint32_t id,const uint8_t data[8]) {
  assert(id==VCU_ID);
  if (data[0] == 0xAA && data[1] == 0xAA) return true;
  ++motor_sends;
  last_control_byte=data[4];
  if (data[4] == 0) ++stops;
  return true;
}
static void enqueue_can(uint32_t id,uint16_t raw, bool hs){
  memset(&queued_can,0,sizeof(queued_can));
  queued_can.extended=true;queued_can.id=id;queued_can.len=8;
  if (hs) memset(queued_can.data,0x55,8);
  else {queued_can.data[6]=(uint8_t)raw;queued_can.data[7]=(uint8_t)(raw>>8);}
  can_pending=true;
}
static void enqueue_asms(uint8_t mode,int16_t x) {
  const uint8_t ip[4]={10,177,21,5};memcpy(queued_ip,ip,4);
  queued_len=5;queued_udp[0]=mode;
  queued_udp[1]=(uint8_t)x;queued_udp[2]=(uint8_t)((uint16_t)x>>8);
  queued_udp[3]=0;queued_udp[4]=0;udp_pending=true;
}
static int32_t last_rpm(void){return get_i32(last_speed_packet);}
int main(void){
  /* Before handshake, telemetry is never treated as valid. */
  fake_time=1;enqueue_can(MCU_ID,33000,false);control_step();
  assert(!handshake_ok && !hasSpeedFeedback);
  fake_time=2;enqueue_can(MCU_ID,0,true);control_step();assert(handshake_ok);
  fake_time=3;enqueue_can(0x1802D0EFUL,33000,false);control_step();assert(!hasSpeedFeedback);
  fake_time=4;enqueue_can(MCU_ID,33000,false);control_step();assert(actual_speed_rpm==1000 && hasSpeedFeedback);
  fake_time=100;control_step();assert(speed_sends==1 && last_rpm()==1000 && last_speed_packet[4]==1);
  const uint8_t pc[4]={10,177,21,1}; assert(memcmp(last_speed_ip,pc,4)==0);
  fake_time=101;enqueue_can(MCU_ID,30500,false);control_step();assert(actual_speed_rpm==-1500);
  fake_time=200;control_step();assert(speed_sends==2 && last_rpm()==-1500 && last_speed_packet[4]==1);
  fake_time=702;control_step();assert(speed_sends==3 && last_rpm()==0 && last_speed_packet[4]==0);
  /* Retain original manual speed path, 50ms CAN cadence and halt mode. */
  fake_time=703;enqueue_asms(2,-2048);control_step();assert(target_speed_rpm==-1500 && motor_enable_cmd);
  fake_time=753;control_step();assert(last_control_byte==0x03);
  fake_time=754;enqueue_asms(3,0);control_step();assert(!motor_enable_cmd && last_control_byte==0);
  assert(speed_polls>0 && motor_sends>0 && stops>0);
  puts("PASS: handshake, frame ID filter, +1000/-1500 RPM, stale flag, 5-byte UDP, manual/ESTOP, CAN cadence");
}
