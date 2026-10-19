#ifndef SPI_BUS_H
#define SPI_BUS_H
#include <stdint.h>
void spi_none(void);
void spi_eth(void);
void spi_can(void);
uint8_t spi_byte(uint8_t value);
#endif
