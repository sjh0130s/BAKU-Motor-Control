/* Minimal W5500 UDP receiver. SPI variable-length data mode, socket 0.
 * Original firmware does not transmit UDP; no unused TX stack is included.
 * Register layout: WIZnet W5500 datasheet, socket/UDP application notes. */
#include "hal.h"
#include "spi_bus.h"
#include <string.h>
#define COMMON 0
#define SOCK0 1
#define RX0 3
#define RX_BYTES 16384U
static void read_bytes(uint8_t block, uint16_t addr, uint8_t *data, uint16_t n) {
    spi_eth();
    spi_byte((uint8_t)(addr >> 8)); spi_byte((uint8_t)addr);
    spi_byte((uint8_t)(block << 3));
    while (n--) *data++ = spi_byte(0);
    spi_none();
}
static void write_bytes(uint8_t block, uint16_t addr, const uint8_t *data, uint8_t n) {
    spi_eth();
    spi_byte((uint8_t)(addr >> 8)); spi_byte((uint8_t)addr);
    spi_byte((uint8_t)((block << 3) | 0x04));
    while (n--) spi_byte(*data++);
    spi_none();
}
static uint8_t read8(uint8_t block, uint16_t addr) {
    uint8_t v; read_bytes(block, addr, &v, 1); return v;
}
static void write8(uint8_t block, uint16_t addr, uint8_t v) { write_bytes(block, addr, &v, 1); }
static uint16_t read16(uint16_t addr) {
    uint8_t v[2]; read_bytes(SOCK0, addr, v, 2);
    return ((uint16_t)v[0] << 8) | v[1];
}
static void write16(uint16_t addr, uint16_t v) {
    uint8_t b[2] = {(uint8_t)(v >> 8), (uint8_t)v};
    write_bytes(SOCK0, addr, b, 2);
}
static bool command(uint8_t cmd) {
    write8(SOCK0, 0x0001, cmd);
    uint32_t start = millis();
    do {
        if (read8(SOCK0, 0x0001) == 0) return true;
    } while ((uint32_t)(millis() - start) < 10);
    return false;
}
static void ring_read(uint16_t ptr, uint8_t *data, uint8_t n) {
    /* Explicit ring split also covers the 16-bit RX_RD rollover. */
    while (n) {
        uint16_t offset = ptr & (RX_BYTES - 1);
        uint16_t remaining = RX_BYTES - offset;
        uint8_t chunk = remaining < n ? (uint8_t)remaining : n;
        read_bytes(RX0, offset, data, chunk);
        ptr += chunk; data += chunk; n -= chunk;
    }
}
bool ethernet_init(void) {
    const uint8_t mac[6] = {0xA8,0x61,0x0A,0xAE,0x21,0x68};
    const uint8_t ip[4] = {10,177,21,2};
    const uint8_t gateway[4] = {10,177,21,1};
    const uint8_t mask[4] = {255,255,255,0};
    /* W5500 RST is on the board RESET net: use software reset. */
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
    /* Disable unused sockets before assigning all RX memory to socket 0. */
    for (uint8_t s = 0; s < 8; ++s) {
        write8((uint8_t)(1 + 4*s), 0x001E, 0);
        write8((uint8_t)(1 + 4*s), 0x001F, 0);
    }
    write8(SOCK0, 0x001E, 16);
    write8(SOCK0, 0x001F, 2);
    if (!command(0x10)) return false; /* CLOSE */
    write8(SOCK0, 0x0002, 0xFF);    /* clear socket interrupts */
    write8(SOCK0, 0x0000, 0x02);    /* UDP */
    write16(0x0004, 5000);
    if (!command(0x01)) return false; /* OPEN */
    start = millis();
    do {
        if (read8(SOCK0, 0x0003) == 0x22) return true;
    } while ((uint32_t)(millis() - start) < 10);
    return false;
}
int8_t udp_receive(uint8_t ip[4], uint8_t *data, uint8_t capacity, uint16_t *len) {
    if (read8(SOCK0, 0x0003) != 0x22) return -1;
    uint16_t available = read16(0x0026);
    bool stable = false;
    for (uint8_t i = 0; i < 8; ++i) {
        uint16_t next = read16(0x0026);
        if (available == next) { stable = true; break; }
        available = next;
    }
    if (!stable || available < 8) return 0;
    uint16_t ptr = read16(0x0028);
    uint8_t header[8];
    ring_read(ptr, header, 8);
    *len = ((uint16_t)header[6] << 8) | header[7];
    if (*len > RX_BYTES - 8) return -1;
    if (available < (uint16_t)(8 + *len)) return 0;
    memcpy(ip, header, 4);
    uint8_t copied = *len < capacity ? (uint8_t)*len : capacity;
    ring_read((uint16_t)(ptr + 8), data, copied);
    write16(0x0028, (uint16_t)(ptr + 8 + *len));
    return command(0x40) ? 1 : -1; /* RECV acknowledges the entire datagram */
}
