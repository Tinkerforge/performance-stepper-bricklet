#ifndef TMC5160_H
#define TMC5160_H

#include "configs/config.h"
#include "bricklib2/hal/spi_fifo/spi_fifo.h"

typedef struct {
	SPIFifo spi_fifo;

    uint8_t last_status;
} TMC5160;

void tmc5160_init(void);
void tmc5160_tick(void);

extern TMC5160 tmc5160;

#endif