/* Mock W5500 SPI register/buffer simulator: tests socket separation and UDP TX. */
#include "../hal.h"
#include "../spi_bus.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
static uint8_t common[64],s0[64],s1[64],tx1[2048],rx0[16384],unused_sock[6][64];
static uint8_t cs_state,header_i,selected_block,write_mode;
static uint16_t address;
static uint32_t ticks;
static unsigned send_n, recv_n;
void board_init(void) {}
uint32_t millis(void){return ticks;}
void delay_ms(uint16_t ms){ticks+=ms;}
void log_text(const char *s){(void)s;}
void log_auto(int32_t x,int32_t y,int16_t z){(void)x;(void)y;(void)z;}
bool can_init(void){return true;}
bool can_receive(CanFrame *f){(void)f;return false;}
bool can_send(uint32_t id,const uint8_t data[8]){(void)id;(void)data;return true;}
void spi_none(void){cs_state=0;}
void spi_can(void){assert(false);}
void spi_eth(void){cs_state=1;header_i=0;address=0;selected_block=0;write_mode=0;}
static uint8_t* mem(uint8_t block,uint16_t address){
  switch(block){
    case 0:assert(address<64);return common+address;
    case 1:assert(address<64);return s0+address;
    case 5:assert(address<64);return s1+address;
    case 3: return rx0+(address&16383);
    case 6: return tx1+(address&2047);
  }
  if (block>=9 && block<=29 && ((block-1)%4)==0){assert(address<64);return unused_sock[(block-9)/4]+address;}
  fprintf(stderr,"bad W5500 block=%u addr=0x%X\n",block,address);
  assert(0);return common;
}
uint8_t spi_byte(uint8_t byte){
  assert(cs_state);
  if (header_i==0){address=(uint16_t)byte<<8;header_i++;return 0;}
  if (header_i==1){address|=byte;header_i++;return 0;}
  if (header_i==2){selected_block=byte>>3;write_mode=(byte&4)!=0;header_i++;return 0;}
  uint8_t *p=mem(selected_block,address);
  uint8_t out=*p;
  if(write_mode){
    *p=byte;
    if(selected_block==0 && address==0 && byte==0x80) *p=0;
    if((selected_block==1 || selected_block==5) && address==1) {
      if(byte==0x10) mem(selected_block,3)[0]=0;
      if(byte==0x01) mem(selected_block,3)[0]=0x22;
      if(byte==0x20){assert(selected_block==5);send_n++;}
      if(byte==0x40) recv_n++;
      *p=0; /* command executed */
    }
    if((selected_block==1 || selected_block==5) && address==2) {
      /* Sn_IR is write 1 to clear. */
      *p=(uint8_t)(out & ~byte);
    }
  }
  address++;
  return out;
}
int main(void){
  common[0x39]=4; /* W5500 version */
  s1[0x20]=0x08;s1[0x21]=0x00; /* 2048 TX bytes free */
  assert(ethernet_init());
  assert(s0[0x1e]==16 && s0[0x1f]==2 && s1[0x1e]==0 && s1[0x1f]==2);
  assert(s0[3]==0x22 && s1[3]==0x22);
  assert(s0[4]==0x13 && s0[5]==0x88); /* command RX port 5000 */
  const uint8_t pc[4]={10,177,21,1},payload[8]={0x18,0xFC,0xFF,0xFF,0x34,0x12,0x00,0x00};
  assert(udp_speed_try_send(pc,payload,8));
  assert(send_n==1 && memcmp(tx1,payload,8)==0);
  assert(memcmp(s1+0x0c,pc,4)==0 && s1[0x10]==0x13 && s1[0x11]==0x89);
  assert(s1[0x24]==0 && s1[0x25]==8);
  assert(!udp_speed_try_send(pc,payload,8)); /* busy drops sample */
  s1[2]|=0x10; udp_speed_poll(); assert(s1[2]==0);
  assert(udp_speed_try_send(pc,payload,8)); assert(send_n==2);
  /* Simulate UDP command reception through unchanged socket 0. */
  const uint8_t asms[4]={10,177,21,5};
  memcpy(rx0,asms,4);rx0[4]=0x13;rx0[5]=0x88;rx0[6]=0;rx0[7]=5;
  rx0[8]=2;rx0[9]=0;rx0[10]=0;rx0[11]=0;rx0[12]=0;
  s0[0x26]=0;s0[0x27]=13; /* UDP header + 5-byte payload */
  uint8_t rxip[4],data[9];uint16_t len;
  assert(udp_receive(rxip,data,9,&len)==1);
  assert(len==5 && data[0]==2 && memcmp(rxip,asms,4)==0 && recv_n==1);
  assert(s0[0x28]==0 && s0[0x29]==13);
  /* Send timeout: a subsequent transmission is permitted. */
  s1[2]|=0x08;udp_speed_poll();assert(udp_speed_try_send(pc,payload,8));
  /* Busy for too long: reset ONLY socket 1. */
  ticks+=501;udp_speed_poll();assert(s0[3]==0x22 && s1[3]==0x22);
  puts("PASS: W5500 TX separate socket, 8-byte UDP, busy/SEND_OK/TIMEOUT, RX5000 preserved, stale reset");
}
