#define UNIT_TEST
#include "../main.c"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t fake_time;
static CanFrame incoming_can;
static bool can_pending;
static uint8_t queued_ip[4], queued_data[9];
static uint16_t queued_len;
static bool udp_pending;
static uint8_t last_speed_packet[8], last_speed_ip[4];
static unsigned rpm_sends, motor_sends, stops;
static uint8_t last_cmd;
void board_init(void){}
uint32_t millis(void){return fake_time;}
void delay_ms(uint16_t t){(void)t;}
void log_text(const char *s){(void)s;}
void log_auto(int32_t a,int32_t b,int16_t c){(void)a;(void)b;(void)c;}
bool ethernet_init(void){return true;}
int8_t udp_receive(uint8_t ip[4],uint8_t *p,uint8_t capacity,uint16_t *len){
  if(!udp_pending)return 0;
  memcpy(ip,queued_ip,4);
  *len=queued_len;
  memcpy(p,queued_data,queued_len<capacity?queued_len:capacity);
  udp_pending=false;return 1;
}
void udp_speed_poll(void){}
bool udp_speed_try_send(const uint8_t ip[4],const uint8_t *data,uint8_t len){
  assert(len==8);memcpy(last_speed_ip,ip,4);
  memcpy(last_speed_packet,data,8);++rpm_sends;return true;
}
bool can_init(void){return true;}
bool can_receive(CanFrame *f){
  if(!can_pending)return false;
  *f=incoming_can;can_pending=false;return true;
}
bool can_send(uint32_t id,const uint8_t p[8]){
  assert(id==VCU_ID);
  if(p[0]==0xAA && p[1]==0xAA)return true;
  ++motor_sends;last_cmd=p[4];if(p[4]==0)++stops;return true;
}
static void asms(uint8_t mode,int16_t joy){
  const uint8_t ip[4]={10,177,21,5};memcpy(queued_ip,ip,4);
  queued_len=5;queued_data[0]=mode;
  queued_data[1]=(uint8_t)joy;queued_data[2]=(uint8_t)((uint16_t)joy>>8);
  queued_data[3]=0;queued_data[4]=0;udp_pending=true;
}
static void pc_speed(int32_t speed){
  const uint8_t ip[4]={10,177,21,BAKU_PC_IP_LAST_OCTET};
  memcpy(queued_ip,ip,4);queued_len=9;
  memset(queued_data,0,9);
  uint32_t u=(uint32_t)speed;
  for(int i=0;i<4;++i)queued_data[4+i]=(uint8_t)(u>>(8*i));
  udp_pending=true;
}
static void hs(void){
  memset(&incoming_can,0,sizeof(incoming_can));incoming_can.id=MCU_ID;
  incoming_can.len=8;incoming_can.extended=true;
  memset(incoming_can.data,0x55,8);can_pending=true;
}
static void rpm(uint16_t raw){
  memset(&incoming_can,0,sizeof(incoming_can));incoming_can.id=MCU_ID;
  incoming_can.len=8;incoming_can.extended=true;
  incoming_can.data[6]=(uint8_t)raw;
  incoming_can.data[7]=(uint8_t)(raw>>8);can_pending=true;
}
static int32_t rpm_val(void){return get_i32(last_speed_packet);}
static uint32_t timestamp(void){return (uint32_t)last_speed_packet[4]|
  ((uint32_t)last_speed_packet[5]<<8)|((uint32_t)last_speed_packet[6]<<16)|
  ((uint32_t)last_speed_packet[7]<<24);}
static void at(uint32_t t){fake_time=t;control_step();}
int main(void){
  hs();at(1);assert(handshake_ok);
  at(51);assert(motor_sends>=1 && last_cmd==0); /* periodic HALT without ASMS */
  pc_speed(20);at(52);
#if DEBUG_ALLOW_PC_AUTO_WITHOUT_ASMS
  assert(motor_enable_cmd && target_speed_rpm==-300 && receivedMode==0);
  at(1053);assert(!motor_enable_cmd && receivedMode==0); /* PC watchdog */
#else
  assert(!motor_enable_cmd && target_speed_rpm==0 && receivedMode==0);
#endif
  asms(1,0);at(2000);assert(receivedMode==1 && !motor_enable_cmd);
  pc_speed(30);at(2001);assert(receivedMode==1 && motor_enable_cmd && target_speed_rpm==-450);
  // Repeated ASMS AUTO must not clear an active PC speed command.
  asms(1,0);at(2500);assert(receivedMode==1 && motor_enable_cmd);
  // Even after ASMS disappears for >1s, a fresh PC packet remains accepted.
  pc_speed(40);at(4500);assert(receivedMode==1 && motor_enable_cmd && target_speed_rpm==-600);
  at(5501);assert(receivedMode==1 && !motor_enable_cmd); /* speed goes stale */
  pc_speed(10);at(5502);assert(receivedMode==1 && motor_enable_cmd);
  // Manual selection overrides PC authority; joystick must remain fresh.
  asms(2,-2048);at(5600);assert(receivedMode==2 && motor_enable_cmd && target_speed_rpm==-1500);
  pc_speed(100);at(5601);assert(receivedMode==2 && target_speed_rpm==-1500);
  at(6601);assert(receivedMode==2 && !motor_enable_cmd);
  asms(2,-1000);at(6602);assert(receivedMode==2 && motor_enable_cmd);
  // ASMS AUTO changes mode; clears old manual target until new PC speed.
  asms(1,0);at(6700);assert(receivedMode==1 && !motor_enable_cmd);
  // RPM feedback behavior unchanged (8B signed RPM+timestamp).
  rpm(33000);at(6701);at(6720);
  assert(rpm_sends>0 && rpm_val()==1000 && timestamp()==6701);
  const uint8_t dest[4]={10,177,21,BAKU_PC_IP_LAST_OCTET};
  assert(memcmp(dest,last_speed_ip,4)==0);
  const uint8_t asms_ip[4]={10,177,21,5};
  memcpy(queued_ip,asms_ip,4);queued_len=3;
  queued_data[0]=3;queued_data[1]=0;queued_data[2]=0;
  udp_pending=true;at(6800);
  assert(estop_latched && !motor_enable_cmd && last_cmd==0);
  asms(1,0);at(6801);pc_speed(90);at(6802);
  assert(estop_latched && !motor_enable_cmd && motor_sends>0 && stops>0);
  puts("PASS: mode latch, fresh-PC watchdog, manual watchdog, 3B ESTOP, HALT heartbeat, RPM feedback");
  return 0;
}
