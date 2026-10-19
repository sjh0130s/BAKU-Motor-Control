/* MCP2515, 16 MHz oscillator, 250 kbit/s, extended CAN data frames.
 * CNF1/2/3 match Seeed CAN_BUS_Shield 2.3.4: 41/F1/85.
 * Receive all frames and apply the original handshake filter in main.c. */
#include "hal.h"
#include "spi_bus.h"
static void read_bytes(uint8_t addr, uint8_t *data, uint8_t n) {
    spi_can(); spi_byte(0x03); spi_byte(addr);
    while (n--) *data++ = spi_byte(0);
    spi_none();
}
static void write_bytes(uint8_t addr, const uint8_t *data, uint8_t n) {
    spi_can(); spi_byte(0x02); spi_byte(addr);
    while (n--) spi_byte(*data++);
    spi_none();
}
static uint8_t read8(uint8_t addr) { uint8_t v; read_bytes(addr, &v, 1); return v; }
static void write8(uint8_t addr, uint8_t v) { write_bytes(addr, &v, 1); }
static void modify(uint8_t addr, uint8_t mask, uint8_t value) {
    spi_can(); spi_byte(0x05); spi_byte(addr); spi_byte(mask); spi_byte(value); spi_none();
}
static bool mode(uint8_t value) {
    modify(0x0F, 0xE0, value);
    uint32_t start = millis();
    do { if ((read8(0x0E) & 0xE0) == value) return true; }
    while ((uint32_t)(millis() - start) < 10);
    return false;
}
bool can_init(void) {
    spi_can(); spi_byte(0xC0); spi_none(); delay_ms(10);
    if (!mode(0x80)) return false;
    write8(0x2A, 0x41); write8(0x29, 0xF1); write8(0x28, 0x85);
    write8(0x0C, 0); /* BFPCTRL: RX pins high impedance */
    write8(0x0D, 0); /* TXRTS pins digital inputs */
    write8(0x60, 0x64); /* RXB0: accept any, rollover to RXB1 */
    write8(0x70, 0x60); /* RXB1: accept any */
    write8(0x2B, 0x03); /* RX interrupts exposed, firmware polls registers */
    write8(0x2C, 0);
    write8(0x30, 0); write8(0x40, 0); write8(0x50, 0);
    /* Disable CLKOUT and use normal retransmission mode. */
    modify(0x0F, 0x1F, 0);
    if (read8(0x2A) != 0x41 || read8(0x29) != 0xF1 || read8(0x28) != 0x85) return false;
    return mode(0);
}
bool can_receive(CanFrame *frame) {
    uint8_t flags = read8(0x2C);
    uint8_t mask = (flags & 1) ? 1 : ((flags & 2) ? 2 : 0);
    if (!mask) return false;
    uint8_t raw[13];
    uint8_t base = mask == 1 ? 0x60 : 0x70;
    uint8_t ctrl = read8(base);
    read_bytes((uint8_t)(base + 1), raw, 13);
    modify(0x2C, mask, 0);
    frame->extended = (raw[1] & 0x08) != 0;
    frame->remote = (ctrl & 0x08) != 0;
    frame->id = ((uint32_t)raw[0] << 3) | (raw[1] >> 5);
    if (frame->extended) {
        frame->id = (frame->id << 18) | ((uint32_t)(raw[1] & 3) << 16)
                  | ((uint32_t)raw[2] << 8) | raw[3];
    }
    frame->len = raw[4] & 0x0F;
    if (frame->len > 8) return false;
    for (uint8_t i = 0; i < frame->len; ++i) frame->data[i] = raw[5+i];
    return true;
}
bool can_send(uint32_t id, const uint8_t data[8]) {
    if (id > 0x1FFFFFFFUL || (read8(0x2D) & 0x20)) return false; /* bus off */
    /* One synchronous TX buffer; bound a missing ACK to 10 ms. */
    if (read8(0x30) & 0x08) return false;
    modify(0x2C, 0x04, 0);
    write8(0x30, 0);
    uint8_t raw[13];
    raw[0] = (uint8_t)(id >> 21);
    raw[1] = (uint8_t)(((id >> 13) & 0xE0) | ((id >> 16) & 3) | 0x08);
    raw[2] = (uint8_t)(id >> 8); raw[3] = (uint8_t)id;
    raw[4] = 8;
    for (uint8_t i = 0; i < 8; ++i) raw[5+i] = data[i];
    write_bytes(0x31, raw, 13);
    spi_can(); spi_byte(0x81); spi_none();
    uint32_t start = millis();
    do {
        uint8_t ctrl = read8(0x30);
        if (!(ctrl & 0x08)) return (read8(0x2C) & 0x04) && !(ctrl & 0x70);
    } while ((uint32_t)(millis() - start) < 10);
    /* Abort this stale command so it cannot reappear after the link returns. */
    modify(0x30, 0x08, 0);
    return false;
}
