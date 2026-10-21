#include "hal.h"
#include "spi_bus.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdlib.h>
#if !defined(__AVR_ATmega328P__) || F_CPU != 16000000UL
#error "This board requires ATmega328P and a 16 MHz CPU clock."
#endif
static volatile uint32_t ticks;
ISR(TIMER0_COMPA_vect) { ++ticks; }
uint32_t millis(void) {
    uint32_t t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { t = ticks; }
    return t;
}
void delay_ms(uint16_t ms) {
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < ms) {}
}
void spi_none(void) { PORTD |= _BV(PD7); PORTB |= _BV(PB2); }
void spi_eth(void) { spi_none(); PORTD &= (uint8_t)~_BV(PD7); }
void spi_can(void) { spi_none(); PORTB &= (uint8_t)~_BV(PB2); }
uint8_t spi_byte(uint8_t value) {
    SPDR = value;
    while (!(SPSR & _BV(SPIF))) {}
    return SPDR;
}
void board_init(void) {
    cli();
    /* PB2 is CAN_CS and hardware SS: it must be an output for SPI master. */
    PORTD |= _BV(PD7);
    PORTB |= _BV(PB2);
    DDRD |= _BV(PD7);
    DDRB |= _BV(PB2) | _BV(PB3) | _BV(PB5);
    DDRB &= (uint8_t)~(_BV(PB0) | _BV(PB1) | _BV(PB4));
    /* Keep unused PD3/PD4 as inputs; no physical mode switch required. */
    DDRD &= (uint8_t)~(_BV(PD2) | _BV(PD3) | _BV(PD4));
    PORTD &= (uint8_t)~(_BV(PD2) | _BV(PD3) | _BV(PD4));
    PORTB &= (uint8_t)~(_BV(PB0) | _BV(PB1) | _BV(PB4));
    /* SPI mode 0, MSB first, 4 MHz, no SPI interrupt. */
    SPCR = _BV(SPE) | _BV(MSTR);
    SPSR = 0;
    /* Timer0 CTC: 16 MHz / 64 / 250 = 1000 Hz. */
    TCCR0A = _BV(WGM01);
    TCCR0B = _BV(CS01) | _BV(CS00);
    OCR0A = 249;
    TCNT0 = 0;
    TIFR0 = _BV(OCF0A);
    TIMSK0 = _BV(OCIE0A);
    /* UART 115200, 8N1, same U2X divisor as Arduino at 16 MHz. */
    UCSR0A = _BV(U2X0);
    UBRR0H = 0; UBRR0L = 16;
    UCSR0B = _BV(TXEN0) | _BV(RXEN0);
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
    sei();
}
void log_text(const char *text) {
    while (*text) {
        while (!(UCSR0A & _BV(UDRE0))) {}
        UDR0 = (uint8_t)*text++;
    }
}
void log_auto(int32_t raw, int32_t limited, int16_t rpm) {
    char number[12];
    log_text("[AUTO] speedRaw="); ltoa(raw, number, 10); log_text(number);
    log_text(" speedLimited="); ltoa(limited, number, 10); log_text(number);
    log_text(" targetRPM="); itoa(rpm, number, 10); log_text(number);
    log_text("\r\n");
}
