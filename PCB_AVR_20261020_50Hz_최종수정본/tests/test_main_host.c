#define UNIT_TEST
#include "../main.c"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t fake_time;
static uint8_t fake_switch_mode = 1;  /* MANUAL */
uint8_t read_mode_pins(void) { return fake_switch_mode; }
static CanFrame queued_can;
static bool can_pending;
static uint8_t queued_ip[4], queued_udp[9];
static uint16_t queued_len;
static bool udp_pending;
static uint8_t last_speed_packet[8];
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
  assert(len==8);
  memcpy(last_speed_ip, ip,4);
  memcpy(last_speed_packet,data,8);
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
static uint32_t last_ts(void) {const uint8_t *p=last_speed_packet+4;return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
int main(void){
  fake_time=1;enqueue_can(MCU_ID,33000,false);control_step();
  assert(!handshake_ok && !hasSpeedFeedback && speed_sends==0);
  fake_time=2;enqueue_can(MCU_ID,0,true);control_step();assert(handshake_ok);
  fake_time=3;enqueue_can(0x1802D0EFUL,33000,false);control_step();assert(!hasSpeedFeedback);
  fake_time=4;enqueue_can(MCU_ID,33000,false);control_step();assert(actual_speed_rpm==1000 && hasSpeedFeedback);
  fake_time=19;control_step();assert(speed_sends==0);
  fake_time=20;control_step();assert(speed_sends==1 && last_rpm()==1000 && last_ts()==4);
  const uint8_t pc[4]={10,177,21,BAKU_PC_IP_LAST_OCTET};assert(memcmp(last_speed_ip,pc,4)==0);
  fake_time=39;control_step();assert(speed_sends==1);
  fake_time=40;control_step();assert(speed_sends==2 && last_rpm()==1000 && last_ts()==4); /* same sample retransmitted */
  fake_time=55;enqueue_can(MCU_ID,30500,false);control_step();assert(actual_speed_rpm==-1500 && lastSpeedSampleTimeMs==55);
  fake_time=60;control_step();assert(speed_sends==3 && last_rpm()==-1500 && last_ts()==55);
  fake_time=80;control_step();assert(speed_sends==4 && last_ts()==55);
  /* A genuine 50Hz CAN stream yields distinct 20ms timestamps. */
  fake_time=100;enqueue_can(MCU_ID,33000,false);control_step();assert(speed_sends==5 && last_ts()==100);
  fake_time=120;enqueue_can(MCU_ID,30500,false);control_step();assert(speed_sends==6 && last_rpm()==-1500 && last_ts()==120);
  /* Stale CAN: no 8-byte UDP packet is sent, even while send clock advances. */
  fake_time=621;control_step();assert(speed_sends==6 && !hasSpeedFeedback);
  fake_time=640;control_step();assert(speed_sends==6);
  fake_time=645;enqueue_can(MCU_ID,32000,false);control_step();assert(speed_sends==7 && last_rpm()==0 && last_ts()==645);
  /* Retain original manual command processing, CAN 50ms cadence and HALT. */
  fake_time=646;enqueue_asms(2,-2048);control_step();assert(target_speed_rpm==-1500 && motor_enable_cmd);
  fake_time=696;control_step();assert(last_control_byte==0x03);
  fake_time=697;enqueue_asms(3,0);control_step();
  assert(!motor_enable_cmd && last_control_byte==0 && estop_latched);
  fake_time=698;enqueue_asms(2,-2048);control_step();
  assert(!motor_enable_cmd);
  assert(speed_polls>0 && motor_sends>0 && stops>0);
  /* One simulated second with 50Hz fresh CAN feedback and 1ms scheduler ticks. */
  unsigned prior = speed_sends;
  for (uint32_t t=2000; t<3000; ++t) {
    fake_time=t;
    if ((t % 20)==0) enqueue_can(MCU_ID,33000,false);
    control_step();
  }
  assert(speed_sends-prior == 50);
  /* One simulated second with only 20Hz CAN update still transmits 50Hz UDP.
     Timestamp remains unchanged when repeating the last CAN sample. */
  prior=speed_sends;
  uint32_t last_observed_ts=0;
  unsigned fresh_count=0;
  for (uint32_t t=3000; t<4000; ++t) {
    fake_time=t;
    if ((t % 50)==0) enqueue_can(MCU_ID,32000,false);
    control_step();
    if ((t % 20)==0) {
      if (last_ts()!=last_observed_ts) {++fresh_count;last_observed_ts=last_ts();}
    }
  }
  assert(speed_sends-prior == 50);
  assert(fresh_count == 20);

  estop_latched = false; /* emulate MCU reset in the harness */
  fake_switch_mode = HW_THROTTLE;
  fake_time = 4000; control_step();
  assert(hw_mode == HW_INVALID && !motor_enable_cmd);
  fake_time = 4031; control_step();
  assert(hw_mode == HW_THROTTLE && !motor_enable_cmd);
  unsigned prior_motor = motor_sends;
  fake_time = 4100; control_step();
  assert(motor_sends == prior_motor);
  fake_switch_mode = HW_AUTO;
  fake_time = 4200; control_step();
  fake_time = 4231; control_step();
  assert(hw_mode == HW_AUTO);
  fake_time = 4232; enqueue_asms(2,-2048); control_step();
  assert(!motor_enable_cmd);
  fake_time = 4233; enqueue_asms(1,0); control_step();
  assert(!motor_enable_cmd);
  const uint8_t asms_ip[4] = {10,177,21,5};
  memcpy(queued_ip, asms_ip, 4);
  queued_len = 3; queued_udp[0] = 3; queued_udp[1] = 0; queued_udp[2] = 0;
  udp_pending = true; fake_time = 4240; control_step();
  assert(estop_latched && !motor_enable_cmd);
  fake_switch_mode = HW_INVALID;
  fake_time = 4300; control_step();
  fake_time = 4331; control_step();
  assert(hw_mode == HW_INVALID && !motor_enable_cmd); /* 20 distinct samples but 50 UDP messages */
  puts("PASS: 20ms 8-byte UDP, RPM+CAN-sample timestamp, repeated timestamp, stale silent, manual/ESTOP, CAN command unchanged");
}
