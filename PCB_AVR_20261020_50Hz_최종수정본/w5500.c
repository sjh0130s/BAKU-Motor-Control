/* W5500: socket 0 receives existing UDP commands (port 5000).
 * Socket 1 independently sends 8-byte signed RPM + uint32 timestamp telemetry to PC port 5001.
 * Never wait for Ethernet delivery or ARP resolution in the control loop. */
#include "hal.h"
#include "spi_bus.h"
#include <string.h>
#define COMMON 0
#define SOCK0 1
#define RX0 3
#define SOCK1 5
#define TX1 6
#define RX_BYTES 16384U
#define TX1_BYTES 2048U
#define SOCK_MR 0x0000
#define SOCK_CR 0x0001
#define SOCK_IR 0x0002
#define SOCK_SR 0x0003
#define SOCK_PORT 0x0004
#define SOCK_DIPR 0x000C
#define SOCK_DPORT 0x0010
#define SOCK_RXBUF_SIZE 0x001E
#define SOCK_TXBUF_SIZE 0x001F
#define SOCK_TX_FSR 0x0020
#define SOCK_TX_WR 0x0024
#define SOCK_RX_RSR 0x0026
#define SOCK_RX_RD 0x0028
#define IR_SEND_OK 0x10
#define IR_TIMEOUT 0x08
#define SOCK_UDP 0x22
#define TX_STALE_MS 500UL
static bool tx_busy;
static uint32_t tx_start;
static void read_bytes(uint8_t block, uint16_t addr, uint8_t *data, uint16_t n) {
    spi_eth();
    spi_byte((uint8_t)(addr >> 8)); spi_byte((uint8_t)addr);
    spi_byte((uint8_t)(block << 3));
    while (n--) *data++ = spi_byte(0);
    spi_none();
}
static void write_bytes(uint8_t block, uint16_t addr, const uint8_t *data, uint16_t n) {
    spi_eth();
    spi_byte((uint8_t)(addr >> 8)); spi_byte((uint8_t)addr);
    spi_byte((uint8_t)((block << 3) | 0x04));
    while (n--) spi_byte(*data++);
    spi_none();
}
static uint8_t read8(uint8_t block, uint16_t addr) {
    uint8_t v; read_bytes(block, addr, &v, 1); return v;
}
static void write8(uint8_t block, uint16_t addr, uint8_t v) {
    write_bytes(block, addr, &v, 1);
}
static uint16_t read16(uint8_t block, uint16_t addr) {
    uint8_t v[2]; read_bytes(block, addr, v, 2);
    return ((uint16_t)v[0] << 8) | v[1];
}
static void write16(uint8_t block, uint16_t addr, uint16_t v) {
    uint8_t b[2] = {(uint8_t)(v >> 8), (uint8_t)v};
    write_bytes(block, addr, b, 2);
}
static bool command(uint8_t sock, uint8_t cmd) {
    write8(sock, SOCK_CR, cmd);
    uint32_t start = millis();
    do {
        if (read8(sock, SOCK_CR) == 0) return true;
    } while ((uint32_t)(millis() - start) < 10);
    return false;
}
static bool stable_read16(uint8_t block, uint16_t addr, uint16_t *value) {
    uint16_t prev = read16(block, addr);
    for (uint8_t i = 0; i < 8; ++i) {
        uint16_t next = read16(block, addr);
        if (next == prev) { *value = next; return true; }
        prev = next;
    }
    return false;
}
static void ring_read(uint16_t ptr, uint8_t *data, uint8_t n) {
    while (n) {
        uint16_t offset = ptr & (RX_BYTES - 1U);
        uint16_t remaining = RX_BYTES - offset;
        uint8_t chunk = remaining < n ? (uint8_t)remaining : n;
        read_bytes(RX0, offset, data, chunk);
        ptr += chunk; data += chunk; n -= chunk;
    }
}
static void ring_write1(uint16_t ptr, const uint8_t *data, uint8_t n) {
    while (n) {
        uint16_t offset = ptr & (TX1_BYTES - 1U);
        uint16_t remaining = TX1_BYTES - offset;
        uint8_t chunk = remaining < n ? (uint8_t)remaining : n;
        write_bytes(TX1, offset, data, chunk);
        ptr += chunk; data += chunk; n -= chunk;
    }
}
static bool speed_socket_open(void) {
    tx_busy = false;
    /* Reinitialize only the telemetry socket; leave control RX socket alone. */
    if (!command(SOCK1, 0x10)) return false; /* CLOSE */
    write8(SOCK1, SOCK_IR, 0xFF);
    write8(SOCK1, SOCK_MR, 0x02);       /* UDP */
    write16(SOCK1, SOCK_PORT, 5001);   /* arbitrary local source port */
    if (!command(SOCK1, 0x01)) return false; /* OPEN */
    return read8(SOCK1, SOCK_SR) == SOCK_UDP;
}
bool ethernet_init(void) {
    const uint8_t mac[6] = {0xA8,0x61,0x0A,0xAE,0x21,0x68};
    const uint8_t ip[4] = {10,177,21,2};
    const uint8_t gateway[4] = {10,177,21,1};
    const uint8_t mask[4] = {255,255,255,0};
    delay_ms(100);
    write8(COMMON, 0x0000, 0x80);
    uint32_t start = millis();
    while (read8(COMMON, 0x0000) & 0x80) {
        if ((uint32_t)(millis() - start) >= 100) return false;
    }
    if (read8(COMMON, 0x0039) != 0x04) return false;
    write_bytes(COMMON, 0x0001, gateway, 4);
    write_bytes(COMMON, 0x0005, mask, 4);
    write_bytes(COMMON, 0x0009, mac, 6);
    write_bytes(COMMON, 0x000F, ip, 4);
    /* Keep original socket 0 memory allocation; add 2 KiB TX to socket 1. */
    for (uint8_t s = 0; s < 8; ++s) {
        write8((uint8_t)(1U + 4U*s), SOCK_RXBUF_SIZE, 0);
        write8((uint8_t)(1U + 4U*s), SOCK_TXBUF_SIZE, 0);
    }
    write8(SOCK0, SOCK_RXBUF_SIZE, 16);
    write8(SOCK0, SOCK_TXBUF_SIZE, 2);
    write8(SOCK1, SOCK_RXBUF_SIZE, 0);
    write8(SOCK1, SOCK_TXBUF_SIZE, 2);
    if (!command(SOCK0, 0x10)) return false;
    write8(SOCK0, SOCK_IR, 0xFF);
    write8(SOCK0, SOCK_MR, 0x02);
    write16(SOCK0, SOCK_PORT, 5000);
    if (!command(SOCK0, 0x01)) return false;
    start = millis();
    do {
        if (read8(SOCK0, SOCK_SR) == SOCK_UDP) break;
    } while ((uint32_t)(millis() - start) < 10);
    if (read8(SOCK0, SOCK_SR) != SOCK_UDP) return false;
    /* Telemetry is optional: a TX socket failure must not disable control. */
    (void)speed_socket_open();
    return true;
}
int8_t udp_receive(uint8_t ip[4], uint8_t *data, uint8_t capacity, uint16_t *len) {
    if (read8(SOCK0, SOCK_SR) != SOCK_UDP) return -1;
    uint16_t available;
    if (!stable_read16(SOCK0, SOCK_RX_RSR, &available) || available < 8) return 0;
    uint16_t ptr = read16(SOCK0, SOCK_RX_RD);
    uint8_t header[8];
    ring_read(ptr, header, 8);
    *len = ((uint16_t)header[6] << 8) | header[7];
    if (*len > RX_BYTES - 8U) return -1;
    if (available < (uint16_t)(8 + *len)) return 0;
    memcpy(ip, header, 4);
    uint8_t copied = *len < capacity ? (uint8_t)*len : capacity;
    ring_read((uint16_t)(ptr + 8), data, copied);
    write16(SOCK0, SOCK_RX_RD, (uint16_t)(ptr + 8 + *len));
    return command(SOCK0, 0x40) ? 1 : -1;
}
void udp_speed_poll(void) {
    uint8_t ir = read8(SOCK1, SOCK_IR);
    if (ir & (IR_SEND_OK | IR_TIMEOUT)) {
        write8(SOCK1, SOCK_IR, (uint8_t)(ir & (IR_SEND_OK | IR_TIMEOUT)));
        tx_busy = false;
    }
    if (tx_busy && (uint32_t)(millis() - tx_start) > TX_STALE_MS) {
        /* Abandon a stalled datagram without blocking the motor control loop. */
        (void)speed_socket_open();
    }
}
bool udp_speed_try_send(const uint8_t ip[4], const uint8_t *data, uint8_t len) {
    if (!len || len > 32 || tx_busy) return false;
    if (read8(SOCK1, SOCK_SR) != SOCK_UDP) {
        if (!speed_socket_open()) return false;
    }
    uint16_t free_space;
    if (!stable_read16(SOCK1, SOCK_TX_FSR, &free_space) || free_space < len) return false;
    write_bytes(SOCK1, SOCK_DIPR, ip, 4);
    write16(SOCK1, SOCK_DPORT, 5001);
    uint16_t ptr = read16(SOCK1, SOCK_TX_WR);
    ring_write1(ptr, data, len);
    write16(SOCK1, SOCK_TX_WR, (uint16_t)(ptr + len));
    if (!command(SOCK1, 0x20)) return false; /* SEND: do not wait for SEND_OK */
    tx_start = millis();
    tx_busy = true;
    return true;
}
