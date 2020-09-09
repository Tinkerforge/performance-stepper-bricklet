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
	while(tmc5160.spi_communication_in_progress) {
		coop_task_yield();
	}
	tmc5160.spi_communication_in_progress = true;

	uint8_t tmp[5] = {
		TMC5160_SPI_READ | reg, 
		0, 0, 0, 0
	};

	// First read sets pointer
	uint8_t tmp_read1[5] = {0};
	if(!spi_fifo_coop_transceive(&tmc5160.spi_fifo, 5, tmp, tmp_read1)) {
		// TODO: Reset SPI here in case of error?
		tmc5160.last_status = 0xFF;
		tmc5160.spi_communication_in_progress = false;
		return 0xFFFFFFFF;
	}

	// Second read gets actual data
	uint8_t tmp_read2[5] = {0};
	if(!spi_fifo_coop_transceive(&tmc5160.spi_fifo, 5, tmp, tmp_read2)) {
		// TODO: Reset SPI here in case of error?
		tmc5160.last_status = 0xFF;
		tmc5160.spi_communication_in_progress = false;
		return 0xFFFFFFFF;
	}

	tmc5160.last_status = tmp_read2[0];

	tmc5160.spi_communication_in_progress = false;
	return (tmp_read2[1] << 24) | (tmp_read2[2] << 16) | (tmp_read2[3] << 8) | (tmp_read2[4] << 0);
}

void tmc5160_task_register_write(const uint8_t reg, const uint32_t data) {
	while(tmc5160.spi_communication_in_progress) {
		coop_task_yield();
	}
	tmc5160.spi_communication_in_progress = true;

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
		tmc5160.spi_communication_in_progress = false;
		return;
	}

	tmc5160.last_status = tmp[0];
	tmc5160.spi_communication_in_progress = false;
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

	uint32_t error = 0;
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
	// Default enable pin high -> TMC5160 disabled
	const XMC_GPIO_CONFIG_t enable_pin_config = {
		.mode             = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
		.output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH
	};
	XMC_GPIO_Init(TMC5160_ENABLE_PIN, &enable_pin_config);

#if 0
	// We start off with reading all registers into memory
	for(uint8_t i = 0; i < TMC5160_NUM_REGISTERS; i++) {
		tmc5160.registers.regs[i] = tmc5160_task_register_read(i);
		logd("read %x -> %d\n\r", i, tmc5160.registers.regs[i]);
	}
#endif

	while(true) {
		tmc5160_task_handle_gpio();
		tmc5160_task_handle_leds();

		// Write necessary registers
		for(uint8_t i = 0; i < TMC5160_NUM_REGISTERS; i++) {
			if(tmc5160.registers_write[i]) {
				tmc5160_task_register_write(i, tmc5160.registers.regs[i]);
				logd("write %x -> %d (%d)\n\r", i, tmc5160.registers.regs[i], tmc5160.last_status);
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

		// Read current position, velocity and status once per ms
		if(system_timer_is_time_elapsed_ms(tmc5160.last_read_time, 1)) {
			tmc5160.last_read_time = system_timer_get_ms();
			tmc5160.registers_read[TMC5160_REG_XACTUAL]    = true;
			tmc5160.registers_read[TMC5160_REG_VACTUAL]    = true;
			tmc5160.registers_read[TMC5160_REG_DRV_STATUS] = true;
		}

		coop_task_yield();
	}
}

void tmc5160_init_registers(const bool read_from_tmc5160, const bool write_to_tmc5160, const bool set_default) {
	if(set_default) {
		tmc5160.registers.bits.gconf.bit.recalibrate            = 1;
		tmc5160.registers.bits.gconf.bit.faststandstill         = 0;
		tmc5160.registers.bits.gconf.bit.en_pwm_mode            = 1;
		tmc5160.registers.bits.gconf.bit.multistep_filt         = 1;
		tmc5160.registers.bits.gconf.bit.shaft                  = 0;
		tmc5160.registers.bits.gconf.bit.diag0_error            = 0;
		tmc5160.registers.bits.gconf.bit.diag0_otpw             = 0;
		tmc5160.registers.bits.gconf.bit.diag0_stall            = 0;
		tmc5160.registers.bits.gconf.bit.diag1_stall            = 0;
		tmc5160.registers.bits.gconf.bit.diag1_index            = 0;
		tmc5160.registers.bits.gconf.bit.diag1_index            = 0;
		tmc5160.registers.bits.gconf.bit.diag1_steps_skipped    = 0;
		tmc5160.registers.bits.gconf.bit.diag0_int_pushpull     = 0;
		tmc5160.registers.bits.gconf.bit.diag1_poscomp_pushpull = 0;
		tmc5160.registers.bits.gconf.bit.small_hysteresis       = 0;
		tmc5160.registers.bits.gconf.bit.stop_enable            = 0;
		tmc5160.registers.bits.gconf.bit.direct_mode            = 0;
		tmc5160.registers.bits.gconf.bit.test_mode              = 0;

		// tmc5160.registers.bits.gstat // R+C
		// tmc5160.registers.bits.ifcnt // R

		tmc5160.registers.bits.slaveconf.bit.saveaddr           = 0;
		tmc5160.registers.bits.slaveconf.bit.senddelay          = 0;

		// tmc5160.registers.bits.ioen // R
		tmc5160.registers.bits.x_compare.bit.compare            = 0;

		tmc5160.registers.bits.otp_prog.bit.otpbit              = 0;
		tmc5160.registers.bits.otp_prog.bit.otpbyte             = 0;
		tmc5160.registers.bits.otp_prog.bit.otpmagic            = 0;

		// tmc5160.registers.bits.otp_read // R

		tmc5160.registers.bits.factory_conf.bit.fclktrim        = 0;

		tmc5160.registers.bits.short_conf.bit.s2vs_level        = 6;
		tmc5160.registers.bits.short_conf.bit.s2g_level         = 6;
		tmc5160.registers.bits.short_conf.bit.shortfilter       = 1;
		tmc5160.registers.bits.short_conf.bit.shortdelay        = 0;

		tmc5160.registers.bits.drv_conf.bit.bbmtime             = 0;
		tmc5160.registers.bits.drv_conf.bit.bbmclks             = 4;
		tmc5160.registers.bits.drv_conf.bit.otselect            = 0;
		tmc5160.registers.bits.drv_conf.bit.drvstrength         = 2;
		tmc5160.registers.bits.drv_conf.bit.filt_isense         = 0;

		tmc5160.registers.bits.global_scaler.bit.global_scaler  = SCALE(tmc5160.high_level_current, 0, tmc5160.max_current, 0, 255);

		// tmc5160.registers.bits.offset_read // R

		tmc5160.registers.bits.ihold_irun.bit.ihold             = SCALE(tmc5160.high_level_standstill_current, 0, tmc5160.high_level_current, 0, 31);
		tmc5160.registers.bits.ihold_irun.bit.irun              = SCALE(tmc5160.high_level_motor_run_current, 0, tmc5160.high_level_current, 0, 31);
		tmc5160.registers.bits.ihold_irun.bit.ihold_delay       = SCALE(tmc5160.high_level_standstill_delay_time, 0, 327, 0, 15);
		tmc5160.registers.bits.tpowerdown.bit.tpowerdown        = SCALE(tmc5160.high_level_power_down_time, 0, 5570, 0, 255);
		// tmc5160.registers.bits.tstep // R
		tmc5160.registers.bits.tpwmthrs.bit.tpwmthrs            = MIN(TCP5160_CLOCK_FREQUENCY/(tmc5160.high_level_stealth_threshold*256), 0xFFFFF);
		tmc5160.registers.bits.tcoolthrs.bit.tcoolthrs          = MIN(TCP5160_CLOCK_FREQUENCY/(tmc5160.high_level_coolstep_threshold*256), 0xFFFFF);
		tmc5160.registers.bits.thigh.bit.thigh                  = MIN(TCP5160_CLOCK_FREQUENCY/(tmc5160.high_level_classic_threshold*256), 0xFFFFF);

		tmc5160.registers.bits.rampmode.bit.rampmode            = 0;
		tmc5160.registers.bits.xactual.bit.xactual              = 0;
		// tmc5160.registers.bits.vactual // R
		tmc5160.registers.bits.vstart.bit.vstart                = 10;
		tmc5160.registers.bits.a1.bit.a1                        = 2000;
		tmc5160.registers.bits.v1.bit.v1                        = 10000;
		tmc5160.registers.bits.amax.bit.amax                    = 1000;
		tmc5160.registers.bits.vmax.bit.vmax                    = 51200;
		tmc5160.registers.bits.dmax.bit.dmax                    = 1000;
		tmc5160.registers.bits.d1.bit.d1                        = 2000;
		tmc5160.registers.bits.vstop.bit.vstop                  = 20;
		tmc5160.registers.bits.tzerowait.bit.tzerowait          = SCALE(tmc5160.high_level_ramp_zero_wait, 0, 2796, 0, 0xFFFF);
		tmc5160.registers.bits.xtarget.bit.xtarget              = 0;

		tmc5160.registers.bits.vdcmin.bit.velocity              = 0;
		tmc5160.registers.bits.sw_mode.bit.stop_l_enable        = 0;
		tmc5160.registers.bits.sw_mode.bit.stop_r_enable        = 0;
		tmc5160.registers.bits.sw_mode.bit.pol_stop_l           = 0;
		tmc5160.registers.bits.sw_mode.bit.pol_stop_r           = 0;
		tmc5160.registers.bits.sw_mode.bit.swap_lr              = 0;
		tmc5160.registers.bits.sw_mode.bit.latch_l_active       = 0;
		tmc5160.registers.bits.sw_mode.bit.latch_l_inactive     = 0;
		tmc5160.registers.bits.sw_mode.bit.latch_r_active       = 0;
		tmc5160.registers.bits.sw_mode.bit.latch_r_inactive     = 0;
		tmc5160.registers.bits.sw_mode.bit.en_latch_encoder     = 0;
		tmc5160.registers.bits.sw_mode.bit.sg_stop              = 0;
		tmc5160.registers.bits.sw_mode.bit.en_softstop          = 0;

		tmc5160.registers.bits.ramp_stat.bit.stop_status_l      = 0;
		tmc5160.registers.bits.ramp_stat.bit.stop_status_r      = 0;
		tmc5160.registers.bits.ramp_stat.bit.status_latch_l     = 0;
		tmc5160.registers.bits.ramp_stat.bit.status_latch_r     = 0;
		tmc5160.registers.bits.ramp_stat.bit.event_stop_l       = 0;
		tmc5160.registers.bits.ramp_stat.bit.event_stop_r       = 0;
		tmc5160.registers.bits.ramp_stat.bit.event_stop_sg      = 0;
		tmc5160.registers.bits.ramp_stat.bit.event_pos_reached  = 0;
		tmc5160.registers.bits.ramp_stat.bit.velocity_reached   = 0;
		tmc5160.registers.bits.ramp_stat.bit.position_reached   = 0;
		tmc5160.registers.bits.ramp_stat.bit.vzero              = 0;
		tmc5160.registers.bits.ramp_stat.bit.t_zerowait_active  = 0;
		tmc5160.registers.bits.ramp_stat.bit.second_move        = 0;
		tmc5160.registers.bits.ramp_stat.bit.status_sg          = 0;

		// tmc5160.registers.bits.xlatch // R

		tmc5160.registers.bits.encmode.bit.pol_a                = 0;
		tmc5160.registers.bits.encmode.bit.pol_b                = 0;
		tmc5160.registers.bits.encmode.bit.pol_n                = 0;
		tmc5160.registers.bits.encmode.bit.ingore_ab            = 0;
		tmc5160.registers.bits.encmode.bit.clr_cont             = 0;
		tmc5160.registers.bits.encmode.bit.clr_once             = 0;
		tmc5160.registers.bits.encmode.bit.pos_edge             = 0;
		tmc5160.registers.bits.encmode.bit.neg_edge             = 0;
		tmc5160.registers.bits.encmode.bit.clr_enc_x            = 0;
		tmc5160.registers.bits.encmode.bit.latch_x_act          = 0;
		tmc5160.registers.bits.encmode.bit.enc_sel_decimal      = 0;

		tmc5160.registers.bits.x_enc.bit.x_enc                  = 0;
		tmc5160.registers.bits.enc_const.bit.enc_const          = 0;
		tmc5160.registers.bits.enc_status.bit.enc_status        = 0;
		// tmc5160.registers.bits.enc_latch // R
		tmc5160.registers.bits.enc_deviation.bit.enc_deviation  = 0;

		tmc5160.registers.bits.mslut[0].bit.table_entry         = 0;
		tmc5160.registers.bits.mslut[1].bit.table_entry         = 0;
		tmc5160.registers.bits.mslut[2].bit.table_entry         = 0;
		tmc5160.registers.bits.mslut[3].bit.table_entry         = 0;
		tmc5160.registers.bits.mslut[4].bit.table_entry         = 0;
		tmc5160.registers.bits.mslut[5].bit.table_entry         = 0;
		tmc5160.registers.bits.mslut[6].bit.table_entry         = 0;
		tmc5160.registers.bits.mslut[7].bit.table_entry         = 0;
		tmc5160.registers.bits.mslutsel.bit.w0                  = 0;
		tmc5160.registers.bits.mslutsel.bit.w1                  = 0;
		tmc5160.registers.bits.mslutsel.bit.w2                  = 0;
		tmc5160.registers.bits.mslutsel.bit.w3                  = 0;
		tmc5160.registers.bits.mslutsel.bit.x1                  = 0;
		tmc5160.registers.bits.mslutsel.bit.x2                  = 0;
		tmc5160.registers.bits.mslutsel.bit.x3                  = 0;
		tmc5160.registers.bits.mslutstart.bit.start_sin         = 0;
		tmc5160.registers.bits.mslutstart.bit.start_sin90       = 0;
		// tmc5160.registers.bits.mscnt // R
		// tmc5160.registers.bits.mscuract // R

		tmc5160.registers.bits.chopconf.bit.toff                = 5;
		tmc5160.registers.bits.chopconf.bit.hstrt               = 4;
		tmc5160.registers.bits.chopconf.bit.hend                = 0;
		tmc5160.registers.bits.chopconf.bit.fd3                 = 0;
		tmc5160.registers.bits.chopconf.bit.disfdcc             = 0;
		tmc5160.registers.bits.chopconf.bit.chm                 = 0;
		tmc5160.registers.bits.chopconf.bit.tbl                 = 2;
		tmc5160.registers.bits.chopconf.bit.vhighfs             = 0;
		tmc5160.registers.bits.chopconf.bit.vhighchm            = 0;
		tmc5160.registers.bits.chopconf.bit.tpfd                = 4;
		tmc5160.registers.bits.chopconf.bit.mres                = 0;
		tmc5160.registers.bits.chopconf.bit.intpol              = 1;
		tmc5160.registers.bits.chopconf.bit.dedge               = 0;
		tmc5160.registers.bits.chopconf.bit.diss2g              = 0;
		tmc5160.registers.bits.chopconf.bit.diss2vs             = 0;

		tmc5160.registers.bits.coolconf.bit.semin               = 2;
		tmc5160.registers.bits.coolconf.bit.seup                = 0;
		tmc5160.registers.bits.coolconf.bit.semax               = 10;
		tmc5160.registers.bits.coolconf.bit.sedn                = 0;
		tmc5160.registers.bits.coolconf.bit.seimin              = 0;
		tmc5160.registers.bits.coolconf.bit.sgt                 = 0;
		tmc5160.registers.bits.coolconf.bit.sfilt               = 0;

		tmc5160.registers.bits.dcctrl.bit.dc_time               = 0;
		tmc5160.registers.bits.dcctrl.bit.dc_sg                 = 0;

		// tmc5160.registers.bits.drv_status // R
		
		tmc5160.registers.bits.pwmconf.bit.pwm_ofs              = 30;
		tmc5160.registers.bits.pwmconf.bit.pwm_grad             = 1;
		tmc5160.registers.bits.pwmconf.bit.pwm_freq             = 1;
		tmc5160.registers.bits.pwmconf.bit.pwm_autoscale        = 1;
		tmc5160.registers.bits.pwmconf.bit.pwm_autograd         = 1;
		tmc5160.registers.bits.pwmconf.bit.freewheel            = 0;
		tmc5160.registers.bits.pwmconf.bit.pwm_reg              = 4;
		tmc5160.registers.bits.pwmconf.bit.pwm_lim              = 12;

		// tmc5160.registers.bits.pwm_scale // R
		// tmc5160.registers.bits.pwm_auto // R
		// tmc5160.registers.bits.lost_steps // R
	}

	if(read_from_tmc5160) {
		tmc5160.registers_read[TMC5160_REG_GSTAT]          = true;
		tmc5160.registers_read[TMC5160_REG_IFCNT]          = true;
		tmc5160.registers_read[TMC5160_REG_IOEN]           = true;
		tmc5160.registers_read[TMC5160_REG_OTP_READ]       = true;
		tmc5160.registers_read[TMC5160_REG_OFFSET_READ]    = true;
		tmc5160.registers_read[TMC5160_REG_TSTEP]          = true;
		tmc5160.registers_read[TMC5160_REG_VACTUAL]        = true;
		tmc5160.registers_read[TMC5160_REG_XLATCH]         = true;
		tmc5160.registers_read[TMC5160_REG_ENC_LATCH]      = true;
		tmc5160.registers_read[TMC5160_REG_MSCNT]          = true;
		tmc5160.registers_read[TMC5160_REG_MSCURACT]       = true;
		tmc5160.registers_read[TMC5160_REG_DRV_STATUS]     = true;
		tmc5160.registers_read[TMC5160_REG_PWM_SCALE]      = true;
		tmc5160.registers_read[TMC5160_REG_PWM_AUTO]       = true;
		tmc5160.registers_read[TMC5160_REG_LOST_STEPS]     = true;

		// We read the default value from the following
		// writable registers that we never want to write ourselves:
		tmc5160.registers_read[TMC5160_REG_SLAVECONF]      = true;
		tmc5160.registers_read[TMC5160_REG_OTP_PROG]       = true;
		tmc5160.registers_read[TMC5160_REG_FACTORY_CONF]   = true;
		tmc5160.registers_read[TMC5160_REG_MSLUT0]         = true;
		tmc5160.registers_read[TMC5160_REG_MSLUT1]         = true;
		tmc5160.registers_read[TMC5160_REG_MSLUT2]         = true;
		tmc5160.registers_read[TMC5160_REG_MSLUT3]         = true;
		tmc5160.registers_read[TMC5160_REG_MSLUT4]         = true;
		tmc5160.registers_read[TMC5160_REG_MSLUT5]         = true;
		tmc5160.registers_read[TMC5160_REG_MSLUT6]         = true;
		tmc5160.registers_read[TMC5160_REG_MSLUT7]         = true;
		tmc5160.registers_read[TMC5160_REG_MSLUTSEL]       = true;
		tmc5160.registers_read[TMC5160_REG_MSLUTSTART]     = true;
	}

	if(write_to_tmc5160) {
		tmc5160.registers_write[TMC5160_REG_GCONF]         = true;
		tmc5160.registers_write[TMC5160_REG_SLAVECONF]     = false;
		tmc5160.registers_write[TMC5160_REG_X_COMPARE]     = true;
		tmc5160.registers_write[TMC5160_REG_OTP_PROG]      = false;
		tmc5160.registers_write[TMC5160_REG_FACTORY_CONF]  = false;
		tmc5160.registers_write[TMC5160_REG_SHORT_CONF]    = true;
		tmc5160.registers_write[TMC5160_REG_DRV_CONF]      = true;
		tmc5160.registers_write[TMC5160_REG_GLOBAL_SCALER] = true;
		tmc5160.registers_write[TMC5160_REG_IHOLD_IRUN]    = true;
		tmc5160.registers_write[TMC5160_REG_TPOWERDOWN]    = true;
		tmc5160.registers_write[TMC5160_REG_TPWMTHRS]      = true;
		tmc5160.registers_write[TMC5160_REG_TCOOLTHRS]     = true;
		tmc5160.registers_write[TMC5160_REG_THIGH]         = true;
		tmc5160.registers_write[TMC5160_REG_RAMPMODE]      = true;
		tmc5160.registers_write[TMC5160_REG_XACTUAL]       = true;
		tmc5160.registers_write[TMC5160_REG_VSTART]        = true;
		tmc5160.registers_write[TMC5160_REG_A1]            = true;
		tmc5160.registers_write[TMC5160_REG_V1]            = true;
		tmc5160.registers_write[TMC5160_REG_AMAX]          = true;
		tmc5160.registers_write[TMC5160_REG_VMAX]          = true;
		tmc5160.registers_write[TMC5160_REG_D1]            = true;
		tmc5160.registers_write[TMC5160_REG_VSTOP]         = true;
		tmc5160.registers_write[TMC5160_REG_TZEROWAIT]     = true;
		tmc5160.registers_write[TMC5160_REG_XTARGET]       = true;
		tmc5160.registers_write[TMC5160_REG_VDCMIN]        = true;
		tmc5160.registers_write[TMC5160_REG_SW_MODE]       = true;
		tmc5160.registers_write[TMC5160_REG_RAMP_STAT]     = true;
		tmc5160.registers_write[TMC5160_REG_ENCMODE]       = true;
		tmc5160.registers_write[TMC5160_REG_X_ENC]         = true;
		tmc5160.registers_write[TMC5160_REG_ENC_CONST]     = true;
		tmc5160.registers_write[TMC5160_REG_ENC_STATUS]    = true;
		tmc5160.registers_write[TMC5160_REG_ENC_DEVIATION] = true;
		tmc5160.registers_write[TMC5160_REG_MSLUT0]        = false;
		tmc5160.registers_write[TMC5160_REG_MSLUT1]        = false;
		tmc5160.registers_write[TMC5160_REG_MSLUT2]        = false;
		tmc5160.registers_write[TMC5160_REG_MSLUT3]        = false;
		tmc5160.registers_write[TMC5160_REG_MSLUT4]        = false;
		tmc5160.registers_write[TMC5160_REG_MSLUT5]        = false;
		tmc5160.registers_write[TMC5160_REG_MSLUT6]        = false;
		tmc5160.registers_write[TMC5160_REG_MSLUT7]        = false;
		tmc5160.registers_write[TMC5160_REG_MSLUTSEL]      = false;
		tmc5160.registers_write[TMC5160_REG_MSLUTSTART]    = false;
		tmc5160.registers_write[TMC5160_REG_CHOPCONF]      = true;
		tmc5160.registers_write[TMC5160_REG_COOLCONF]      = true;
		tmc5160.registers_write[TMC5160_REG_DCCTRL]        = true;
		tmc5160.registers_write[TMC5160_REG_PWMCONF]       = true;
	}
}

void tmc5160_init(void) {
	memset(&tmc5160, 0, sizeof(TMC5160));

	// TODO: Get max current from eeprom or solder bridge across pin or similar?
#if 0
	tmc5160.max_current                      = 1000; // 2300

	tmc5160.high_level_current               = 800;
	tmc5160.high_level_motor_run_current     = 800;
	tmc5160.high_level_standstill_current    = 600;
#else
	tmc5160.max_current                      = 4700; // 2300

	tmc5160.high_level_current               = 4700;
	tmc5160.high_level_motor_run_current     = 4700;
	tmc5160.high_level_standstill_current    = 4700;
#endif

	tmc5160.high_level_stealth_threshold     = 500;
	tmc5160.high_level_coolstep_threshold    = 500;
	tmc5160.high_level_classic_threshold     = 1000;
	tmc5160.high_level_last_steps            = 0;
	tmc5160.high_level_ramp_zero_wait        = 100;
	tmc5160.high_level_standstill_delay_time = 50;
	tmc5160.high_level_power_down_time       = 250;

	tmc5160_init_registers(true, true, true);

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