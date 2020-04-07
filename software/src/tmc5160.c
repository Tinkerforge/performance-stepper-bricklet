/* silent-stepper-v2-bricklet
 * Copyright (C) 2020 Olaf Lüke <olaf@tinkerforge.com>
 *
 * tmc5160.c: Driver for TMC5160
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "tmc5160.h"

#include <stdbool.h>
#include <stdint.h>

#include "configs/config_tmc5160.h"
#include "configs/config_gpio.h"

#include "bricklib2/hal/system_timer/system_timer.h"
#include "bricklib2/utility/util_definitions.h"
#include "bricklib2/os/coop_task.h"
#include "bricklib2/logging/logging.h"

#include "communication.h"
#include "gpio.h"
#include "voltage.h"

#define TMC5160_SPI_READ  (0 << 7)
#define TMC5160_SPI_WRITE (1 << 7)

TMC5160 tmc5160;
CoopTask tmc5160_task;

uint32_t tmc5160_task_register_read(const uint8_t reg) {
	uint8_t tmp[5] = {
		TMC5160_SPI_READ | reg, 
		0, 0, 0, 0
	};


	// First read sets pointer
	uint8_t tmp_read1[5] = {0};
	if(!spi_fifo_coop_transceive(&tmc5160.spi_fifo, 5, tmp, tmp_read1)) {
		// TODO: Reset SPI here in case of error?
		tmc5160.last_status = 0xFF;
		return 0xFFFFFFFF;
	}

	// Second read gets actual data
	uint8_t tmp_read2[5] = {0};
	if(!spi_fifo_coop_transceive(&tmc5160.spi_fifo, 5, tmp, tmp_read2)) {
		// TODO: Reset SPI here in case of error?
		tmc5160.last_status = 0xFF;
		return 0xFFFFFFFF;
	}

	tmc5160.last_status = tmp_read2[0];
	return (tmp_read2[1] << 24) | (tmp_read2[2] << 16) | (tmp_read2[3] << 8) | (tmp_read2[4] << 0);
}

void tmc5160_task_register_write(const uint8_t reg, const uint32_t data) {
	uint8_t tmp[5] = {
		TMC5160_SPI_WRITE | reg,
		(data >> 24) & 0xFF,
		(data >> 16) & 0xFF,
		(data >>  8) & 0xFF,
		(data >>  0) & 0xFF,
	};

	if(!spi_fifo_coop_transceive(&tmc5160.spi_fifo, 5, tmp, tmp)) {
		// TODO: Reset SPI here in case of error?
		tmc5160.last_status = 0xFF;
		return;
	}

	tmc5160.last_status = tmp[0];
}

static void tmc5160_task_register_log(void) {
	for(uint8_t reg = 0; reg < 127; reg++) {
		uint32_t value = tmc5160_task_register_read(reg);
		logd("Register %d: %u (status: %b)\n\r", reg, value, tmc5160.last_status);
		coop_task_yield();
	}
	logd("\n\r");
}

static void tmc5160_task_handle_gpio(void) {
	if(gpio.stop_emergency) {
		gpio.stop_emergency = false;
		gpio.stop_normal = false;

		// TODO: Handle emergency stop
	} else if(gpio.stop_normal) {
		gpio.stop_normal = false;
		tmc5160.registers.bits.rampmode.bit.rampmode = SILENT_STEPPER_V2_RAMPING_MODE_VELOCITY_POSITIVE;
		tmc5160.registers.bits.vmax.bit.vmax = 0;
		tmc5160.registers.bits.amax.bit.amax = gpio.stop_deceleration;

		tmc5160_task_register_write(TMC5160_REG_RAMPMODE, tmc5160.registers.regs[TMC5160_REG_RAMPMODE]);
		tmc5160_task_register_write(TMC5160_REG_VMAX, tmc5160.registers.regs[TMC5160_REG_VMAX]);
		tmc5160_task_register_write(TMC5160_REG_AMAX, tmc5160.registers.regs[TMC5160_REG_AMAX]);
	}
}

static void tmc5160_task_handle_error_led(const uint32_t t) {
	static uint32_t last_time = 0;

	uint8_t error = 0;
	if(voltage.value < 9000) {
		error = 500;
	}

	if(tmc5160.registers.bits.drv_status.bit.otpw) {
		error = 125;
	}

	if(tmc5160.registers.bits.drv_status.bit.s2gb || tmc5160.registers.bits.drv_status.bit.s2ga || tmc5160.registers.bits.drv_status.bit.ot) {
		error = 1;
	}

	if(error == 0) {
		XMC_GPIO_SetOutputHigh(TMC5160_ERROR_LED_PIN);
	} else if (error == 1) {
		XMC_GPIO_SetOutputHigh(TMC5160_ERROR_LED_PIN);
	} else {
		if(system_timer_is_time_elapsed_ms(last_time, error)) {
			XMC_GPIO_ToggleOutput(TMC5160_ERROR_LED_PIN);
			last_time = system_timer_get_ms();
		}
	}
}

static void tmc5160_task_handle_steps_led(const uint32_t t) {
	static uint32_t last_ontime = 0;
	static uint32_t last_time = 0;
	static int32_t last_xactual = 0;

	if(system_timer_is_time_elapsed_ms(last_ontime, 5)) {
		XMC_GPIO_SetOutputHigh(TMC5160_STEPS_LED_PIN);	
	}

	if(system_timer_is_time_elapsed_ms(last_time, 50)) {
		last_time = t;
		if(ABS(tmc5160.registers.bits.xactual.bit.xactual - last_xactual) > 256) {
			last_xactual = tmc5160.registers.bits.xactual.bit.xactual;
			XMC_GPIO_SetOutputLow(TMC5160_STEPS_LED_PIN);
			last_ontime = system_timer_get_ms();
		}
	}
}

static void tmc5160_task_handle_leds(void) {
	uint32_t t = system_timer_get_ms();
	if(tmc5160.error_led_flicker_state.config == LED_FLICKER_CONFIG_HEARTBEAT) {
		led_flicker_tick(&tmc5160.error_led_flicker_state, t, TMC5160_ERROR_LED_PIN);
	} else if(tmc5160.error_led_flicker_state.config == SILENT_STEPPER_V2_ERROR_LED_CONFIG_SHOW_ERROR) {
		tmc5160_task_handle_error_led(t);
	}

	if(tmc5160.enable_led_flicker_state.config == LED_FLICKER_CONFIG_HEARTBEAT) {
		led_flicker_tick(&tmc5160.enable_led_flicker_state, t, TMC5160_ENABLE_LED_PIN);
	}

	if(tmc5160.steps_led_flicker_state.config == LED_FLICKER_CONFIG_HEARTBEAT) {
		led_flicker_tick(&tmc5160.steps_led_flicker_state, t, TMC5160_STEPS_LED_PIN);
	} else if(tmc5160.steps_led_flicker_state.config == SILENT_STEPPER_V2_STEPS_LED_CONFIG_SHOW_STEPS) {
		tmc5160_task_handle_steps_led(t);
	}

	if(gpio.gpio_led_flicker_state[0].config == LED_FLICKER_CONFIG_HEARTBEAT) {
		led_flicker_tick(&gpio.gpio_led_flicker_state[0], t, GPIO_0_LED_PIN);
	}

	if(gpio.gpio_led_flicker_state[1].config == LED_FLICKER_CONFIG_HEARTBEAT) {
		led_flicker_tick(&gpio.gpio_led_flicker_state[1], t, GPIO_1_LED_PIN);
	}
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
		.output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH
	};
	XMC_GPIO_Init(TMC5160_ENABLE_PIN, &enable_pin_config);

	// We start off with reading all registers into memory
	for(uint8_t i = 0; i < TMC5160_NUM_REGISTERS; i++) {
		tmc5160.registers.regs[i] = tmc5160_task_register_read(i);
		logd("read %x -> %d\n\r", i, tmc5160.registers.regs[i]);
	}

	while(true) {
		tmc5160_task_handle_gpio();
		tmc5160_task_handle_leds();

		// Write necessary registers
		for(uint8_t i = 0; i < TMC5160_NUM_REGISTERS; i++) {
			if(tmc5160.registers_write[i]) {
				tmc5160_task_register_write(i, tmc5160.registers.regs[i]);
				logd("write %x -> %d\n\r", i, tmc5160.registers.regs[i]);
				tmc5160.registers_write[i] = false;
			}
		}

		// Read necessary registers
		for(uint8_t i = 0; i < TMC5160_NUM_REGISTERS; i++) {
			if(tmc5160.registers_read[i]) {
				tmc5160.registers.regs[i] = tmc5160_task_register_read(i);
//				logd("read %x -> %d\n\r", i, tmc5160.registers.regs[i]);
				tmc5160.registers_read[i] = false;
			}
		}

		// Read current position and velocity once per ms
		if(system_timer_is_time_elapsed_ms(tmc5160.last_read_time, 1)) {
			tmc5160.last_read_time = system_timer_get_ms();
			tmc5160.registers_read[TMC5160_REG_XACTUAL]    = true;
			tmc5160.registers_read[TMC5160_REG_VACTUAL]    = true;
			tmc5160.registers_read[TMC5160_REG_DRV_STATUS] = true;
		}

		coop_task_yield();
	}
}

void tmc5160_init(void) {
	memset(&tmc5160, 0, sizeof(TMC5160));

	const XMC_GPIO_CONFIG_t led_config = {
		.mode             = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
		.output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH
	};
	XMC_GPIO_Init(TMC5160_ENABLE_LED_PIN, &led_config);
	XMC_GPIO_Init(TMC5160_STEPS_LED_PIN,  &led_config);
	XMC_GPIO_Init(TMC5160_ERROR_LED_PIN,  &led_config);
	tmc5160.enable_led_flicker_state.config = SILENT_STEPPER_V2_ENABLE_LED_CONFIG_SHOW_ENABLE;
	tmc5160.steps_led_flicker_state.config  = SILENT_STEPPER_V2_STEPS_LED_CONFIG_SHOW_STEPS;
	tmc5160.error_led_flicker_state.config  = SILENT_STEPPER_V2_ERROR_LED_CONFIG_SHOW_ERROR;

	coop_task_init(&tmc5160_task, tmc5160_tick_task);

	tmc5160_init_spi();
}

void tmc5160_tick(void) {
	coop_task_tick(&tmc5160_task);
}