#include "tmc5160.h"

#include <stdbool.h>
#include <stdint.h>

#include "configs/config_tmc5160.h"

#include "bricklib2/hal/system_timer/system_timer.h"
#include "bricklib2/os/coop_task.h"
#include "bricklib2/logging/logging.h"

#define TMC5160_SPI_READ  (0 << 7)
#define TMC5160_SPI_WRITE (1 << 7)

TMC5160 tmc5160;
CoopTask tmc5160_task;

static uint32_t tmc5160_register_read(const uint8_t reg) {
	uint8_t tmp[5] = {
		TMC5160_SPI_READ | reg, 
		0, 0, 0, 0
	};

	if(!spi_fifo_coop_transceive(&tmc5160.spi_fifo, 5, tmp, tmp)) {
		// TODO: ERROR
	}

	tmc5160.last_status = tmp[0];
	return (tmp[1] << 24) | (tmp[2] << 16) | (tmp[3] << 8) | (tmp[4] << 0);
}

static void tmc5160_register_write(const uint8_t reg, uint32_t data) {
	uint8_t tmp[5] = {
		TMC5160_SPI_WRITE | reg,
		(data >> 24) & 0xFF,
		(data >> 16) & 0xFF,
		(data >>  8) & 0xFF,
		(data >>  0) & 0xFF,
	};

	if(!spi_fifo_coop_transceive(&tmc5160.spi_fifo, 5, tmp, tmp)) {
		// TODO: ERROR
	}

	tmc5160.last_status = tmp[0];
}

static void tmc5160_register_log(void) {
	for(uint8_t reg = 0; reg < 127; reg++) {
		uint32_t value = tmc5160_register_read(reg);
		logd("Register %d: %u (status: %b)\n\r", reg, value, tmc5160.last_status);
		coop_task_yield();
	}
	logd("\n\r");
}

static void tmc5160_init_spi(void) {
	tmc5160.spi_fifo.channel             = TMC5160_USIC_SPI;
	tmc5160.spi_fifo.baudrate            = TMC5160_SPI_BAUDRATE;

	tmc5160.spi_fifo.rx_fifo_size        = TMC5160_RX_FIFO_SIZE;
	tmc5160.spi_fifo.rx_fifo_pointer     = TMC5160_RX_FIFO_POINTER;
	tmc5160.spi_fifo.tx_fifo_size        = TMC5160_TX_FIFO_SIZE;
	tmc5160.spi_fifo.tx_fifo_pointer     = TMC5160_TX_FIFO_POINTER;

	tmc5160.spi_fifo.slave               = TMC5160_SLAVE;
	tmc5160.spi_fifo.clock_output        = TMC5160_CLOCK_OUTPUT;
	tmc5160.spi_fifo.clock_passive_level = TMC5160_CLOCK_PASSIVE_LEVEL;

	tmc5160.spi_fifo.sclk_pin            = TMC5160_SCLK_PIN;
	tmc5160.spi_fifo.sclk_port           = TMC5160_SCLK_PORT;
	tmc5160.spi_fifo.sclk_pin_mode       = TMC5160_SCLK_PIN_MODE;

	tmc5160.spi_fifo.select_pin          = TMC5160_SELECT_PIN;
	tmc5160.spi_fifo.select_port         = TMC5160_SELECT_PORT;
	tmc5160.spi_fifo.select_pin_mode     = TMC5160_SELECT_PIN_MODE;

	tmc5160.spi_fifo.mosi_pin            = TMC5160_MOSI_PIN;
	tmc5160.spi_fifo.mosi_port           = TMC5160_MOSI_PORT;
	tmc5160.spi_fifo.mosi_pin_mode       = TMC5160_MOSI_PIN_MODE;

	tmc5160.spi_fifo.miso_pin            = TMC5160_MISO_PIN;
	tmc5160.spi_fifo.miso_port           = TMC5160_MISO_PORT;
	tmc5160.spi_fifo.miso_input          = TMC5160_MISO_INPUT;
	tmc5160.spi_fifo.miso_source         = TMC5160_MISO_SOURCE;

	spi_fifo_init(&tmc5160.spi_fifo);
}

void tmc5160_tick_task(void) {
	const XMC_GPIO_CONFIG_t enable_pin_config = {
		.mode             = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
		.output_level     = XMC_GPIO_OUTPUT_LEVEL_LOW
	};
	XMC_GPIO_Init(TMC5160_ENABLE_PIN, &enable_pin_config);

	while(true) {
		coop_task_sleep_ms(1000);
		logd("tick\n\r");
		tmc5160_register_log();
	}
}

void tmc5160_init(void) {
	coop_task_init(&tmc5160_task, tmc5160_tick_task);

	tmc5160_init_spi();
}

void tmc5160_tick(void) {
	coop_task_tick(&tmc5160_task);
}