#ifndef HAL_H
#define HAL_H
#include <stdint.h>
#include <stdbool.h>
/* Vehicle PC is .1. Define BAKU_PC_IP_LAST_OCTET=50 for a test laptop.
 * A different build definition requires rebuilding and flashing a new HEX. */
#ifndef BAKU_PC_IP_LAST_OCTET
#define BAKU_PC_IP_LAST_OCTET 1
#endif
#if BAKU_PC_IP_LAST_OCTET < 1 || BAKU_PC_IP_LAST_OCTET > 254 || BAKU_PC_IP_LAST_OCTET == 2 || BAKU_PC_IP_LAST_OCTET == 5
#error "Invalid PC IP last octet: avoid PCB .2 and ASMS .5"
#endif
/* SPI: PD7=ETH_CS, PB2=CAN_CS and hardware SS (output HIGH when idle).
 * PB3=MOSI, PB4=MISO, PB5=SCK. PD2=CAN_INT, PB1=ETH_INT,
 * PB0 is unconnected in the provided final schematic. Interrupt pins are inputs.
 * PD3/PD4 are unused mode switch inputs with external pull-ups. */
typedef struct {
    uint32_t id;
    uint8_t len, data[8];
    bool extended, remote;
} CanFrame;
void board_init(void);
uint8_t read_mode_pins(void);  /* 0 throttle, 1 manual, 2 auto, 3 invalid */
uint32_t millis(void);
void delay_ms(uint16_t ms);
void log_text(const char *text);
void log_auto(int32_t raw, int32_t limited, int16_t rpm);
bool ethernet_init(void);
/* 1 = one complete datagram consumed, 0 = none, -1 = hardware/protocol fault.
 * len is original payload length. At most capacity bytes are copied. */
int8_t udp_receive(uint8_t ip[4], uint8_t *data, uint8_t capacity, uint16_t *len);
/* Optional 8-byte RPM + timestamp telemetry, socket 1: destination PC 10.177.21.1:5001.
 * poll: check SEND_OK/TIMEOUT without waiting for network delivery.
 * try_send: returns false if busy/not ready. Does not block for ARP. */
void udp_speed_poll(void);
bool udp_speed_try_send(const uint8_t ip[4], const uint8_t *data, uint8_t len);
bool can_init(void);
bool can_receive(CanFrame *frame);
bool can_send(uint32_t id, const uint8_t data[8]);
#endif
