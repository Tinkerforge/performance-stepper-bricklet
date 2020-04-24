/* silent-stepper-v2-bricklet
 * Copyright (C) 2020 Olaf Lüke <olaf@tinkerforge.com>
 *
 * communication.c: TFP protocol message handling
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

#include "communication.h"

#include <stdint.h>
#include <stdbool.h>

#include "bricklib2/utility/communication_callback.h"
#include "bricklib2/utility/util_definitions.h"
#include "bricklib2/protocols/tfp/tfp.h"
#include "bricklib2/logging/logging.h"

#include "configs/config_tmc5160.h"
#include "configs/config_gpio.h"
#include "tmc5160.h"
#include "gpio.h"
#include "voltage.h"

BootloaderHandleMessageResponse handle_message(const void *message, void *response) {
	switch(tfp_get_fid_from_message(message)) {
		case FID_SET_MOTION_CONFIGURATION: return set_motion_configuration(message);
		case FID_GET_MOTION_CONFIGURATION: return get_motion_configuration(message, response);
		case FID_SET_CURRENT_POSITION: return set_current_position(message);
		case FID_GET_CURRENT_POSITION: return get_current_position(message, response);
		case FID_GET_CURRENT_VELOCITY: return get_current_velocity(message, response);
		case FID_SET_TARGET_POSITION: return set_target_position(message);
		case FID_GET_TARGET_POSITION: return get_target_position(message, response);
		case FID_SET_STEPS: return set_steps(message);
		case FID_GET_STEPS: return get_steps(message, response);
		case FID_GET_REMAINING_STEPS: return get_remaining_steps(message, response);
		case FID_SET_STEP_CONFIGURATION: return set_step_configuration(message);
		case FID_GET_STEP_CONFIGURATION: return get_step_configuration(message, response);
		case FID_SET_MOTOR_CURRENT: return set_motor_current(message);
		case FID_GET_MOTOR_CURRENT: return get_motor_current(message, response);
		case FID_SET_ENABLED: return set_enabled(message);
		case FID_GET_ENABLED: return get_enabled(message, response);
		case FID_SET_BASIC_CONFIGURATION: return set_basic_configuration(message);
		case FID_GET_BASIC_CONFIGURATION: return get_basic_configuration(message, response);
		case FID_SET_SPREADCYCLE_CONFIGURATION: return set_spreadcycle_configuration(message);
		case FID_GET_SPREADCYCLE_CONFIGURATION: return get_spreadcycle_configuration(message, response);
		case FID_SET_STEALTH_CONFIGURATION: return set_stealth_configuration(message);
		case FID_GET_STEALTH_CONFIGURATION: return get_stealth_configuration(message, response);
		case FID_SET_COOLSTEP_CONFIGURATION: return set_coolstep_configuration(message);
		case FID_GET_COOLSTEP_CONFIGURATION: return get_coolstep_configuration(message, response);
		case FID_SET_MISC_CONFIGURATION: return set_misc_configuration(message);
		case FID_GET_MISC_CONFIGURATION: return get_misc_configuration(message, response);
		case FID_GET_DRIVER_STATUS: return get_driver_status(message, response);
		case FID_GET_INPUT_VOLTAGE: return get_input_voltage(message, response);
		case FID_SET_GPIO_CONFIGURATION: return set_gpio_configuration(message);
		case FID_GET_GPIO_CONFIGURATION: return get_gpio_configuration(message, response);
		case FID_SET_GPIO_ACTION: return set_gpio_action(message);
		case FID_GET_GPIO_ACTION: return get_gpio_action(message, response);
		case FID_GET_GPIO_STATE: return get_gpio_state(message, response);
		case FID_SET_ERROR_LED_CONFIG: return set_error_led_config(message);
		case FID_GET_ERROR_LED_CONFIG: return get_error_led_config(message, response);
		case FID_SET_ENABLE_LED_CONFIG: return set_enable_led_config(message);
		case FID_GET_ENABLE_LED_CONFIG: return get_enable_led_config(message, response);
		case FID_SET_STEPS_LED_CONFIG: return set_steps_led_config(message);
		case FID_GET_STEPS_LED_CONFIG: return get_steps_led_config(message, response);
		case FID_SET_GPIO_LED_CONFIG: return set_gpio_led_config(message);
		case FID_GET_GPIO_LED_CONFIG: return get_gpio_led_config(message, response);
		case FID_WRITE_REGISTER: return write_register(message, response);
		case FID_READ_REGISTER: return read_register(message, response);
		default: return HANDLE_MESSAGE_RESPONSE_NOT_SUPPORTED;
	}
}


BootloaderHandleMessageResponse set_motion_configuration(const SetMotionConfiguration *data) {
	if(data->ramping_mode > SILENT_STEPPER_V2_RAMPING_MODE_HOLD) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}
	if((data->velocity_start < 0)   || (data->velocity_start > 0x1FFFF)) { // 18 bit
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}
	if((data->acceleration_1 < 0)   || (data->acceleration_1 > 0xFFFF)) { // 16 bit
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}
	if((data->velocity_1 < 0)       || (data->velocity_1 > 0xFFFFF)) { //20 bit
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}
	if((data->acceleration_max < 0) || (data->acceleration_max > 0xFFFF)) { // 16 bit
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}
	if((data->velocity_max < 0)     || (data->velocity_max > (0x7fffff - 511))) { // 23 bit - 512
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}
	if((data->deceleration_max < 0) || (data->deceleration_max > 0xFFFF)) { // 16 bit
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}
	if((data->deceleration_1 < 1)   || (data->deceleration_1 > 0xFFFF)) { // 16 bit, 0 not allowed
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}
	if((data->velocity_stop < 1)    || (data->velocity_stop > 0x1FFFF)) { // 18 bit, 0 not allowed
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}
	if((data->ramp_zero_wait < 0)   || (data->ramp_zero_wait > 2796)) { // 0-2796ms -> 16 bit
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	tmc5160.registers.bits.rampmode.bit.rampmode   = data->ramping_mode;
	tmc5160.registers_write[TMC5160_REG_RAMPMODE]  = true;

	tmc5160.registers.bits.vstart.bit.vstart       = data->velocity_start;
	tmc5160.registers_write[TMC5160_REG_VSTART]    = true;

	tmc5160.registers.bits.a1.bit.a1               = data->acceleration_1;
	tmc5160.registers_write[TMC5160_REG_A1]        = true;

	tmc5160.registers.bits.v1.bit.v1               = data->velocity_1;
	tmc5160.registers_write[TMC5160_REG_V1]        = true;

	tmc5160.registers.bits.amax.bit.amax           = data->acceleration_max;
	tmc5160.registers_write[TMC5160_REG_AMAX]      = true;

	tmc5160.registers.bits.vmax.bit.vmax           = data->velocity_max;
	tmc5160.registers_write[TMC5160_REG_VMAX]      = true;

	tmc5160.registers.bits.dmax.bit.dmax           = data->deceleration_max;
	tmc5160.registers_write[TMC5160_REG_DMAX]      = true;

	tmc5160.registers.bits.d1.bit.d1               = data->deceleration_1;
	tmc5160.registers_write[TMC5160_REG_D1]        = true;

	tmc5160.registers.bits.vstop.bit.vstop         = data->velocity_stop;
	tmc5160.registers_write[TMC5160_REG_VSTOP]     = true;

	tmc5160.high_level_ramp_zero_wait              = data->ramp_zero_wait;
	tmc5160.registers.bits.tzerowait.bit.tzerowait = SCALE(data->ramp_zero_wait, 0, 2796, 0, 0xFFFF);
	tmc5160.registers_write[TMC5160_REG_TZEROWAIT] = true;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_motion_configuration(const GetMotionConfiguration *data, GetMotionConfiguration_Response *response) {
	response->header.length    = sizeof(GetMotionConfiguration_Response);
	response->ramping_mode     = tmc5160.registers.bits.rampmode.bit.rampmode;
	response->velocity_start   = tmc5160.registers.bits.vstart.bit.vstart;
	response->acceleration_1   = tmc5160.registers.bits.a1.bit.a1;
	response->velocity_1       = tmc5160.registers.bits.v1.bit.v1;
	response->acceleration_max = tmc5160.registers.bits.amax.bit.amax;
	response->velocity_max     = tmc5160.registers.bits.vmax.bit.vmax;
	response->deceleration_max = tmc5160.registers.bits.dmax.bit.dmax;
	response->deceleration_1   = tmc5160.registers.bits.d1.bit.d1;
	response->velocity_stop    = tmc5160.registers.bits.vstop.bit.vstop;
	response->ramp_zero_wait   = tmc5160.high_level_ramp_zero_wait;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_current_position(const SetCurrentPosition *data) {
	tmc5160.registers.bits.xactual.bit.xactual   = (uint32_t)data->position;
	tmc5160.registers_write[TMC5160_REG_XACTUAL] = true;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_current_position(const GetCurrentPosition *data, GetCurrentPosition_Response *response) {
	response->header.length = sizeof(GetCurrentPosition_Response);
	response->position      = (int32_t)tmc5160.registers.bits.xactual.bit.xactual;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_current_velocity(const GetCurrentVelocity *data, GetCurrentVelocity_Response *response) {
	response->header.length = sizeof(GetCurrentVelocity_Response);
	response->velocity      = INTN_TO_INT32((int32_t)tmc5160.registers.bits.vactual.bit.vactual, 24);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_target_position(const SetTargetPosition *data) {
	tmc5160.registers.bits.xtarget.bit.xtarget   = (uint32_t)data->position;
	tmc5160.registers_write[TMC5160_REG_XTARGET] = true;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_target_position(const GetTargetPosition *data, GetTargetPosition_Response *response) {
	response->header.length = sizeof(GetTargetPosition_Response);
	response->position      = tmc5160.registers.bits.xtarget.bit.xtarget;
	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_steps(const SetSteps *data) {
	tmc5160.registers.bits.xtarget.bit.xtarget   = tmc5160.registers.bits.xactual.bit.xactual + data->steps;
	tmc5160.high_level_last_steps                = data->steps;
	tmc5160.registers_write[TMC5160_REG_XTARGET] = true;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_steps(const GetSteps *data, GetSteps_Response *response) {
	response->header.length = sizeof(GetSteps_Response);
	response->steps         = tmc5160.high_level_last_steps;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_remaining_steps(const GetRemainingSteps *data, GetRemainingSteps_Response *response) {
	response->header.length = sizeof(GetRemainingSteps_Response);
	response->steps         = tmc5160.registers.bits.xtarget.bit.xtarget - tmc5160.registers.bits.xactual.bit.xactual;
	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_step_configuration(const SetStepConfiguration *data) {
	if(data->step_resolution > SILENT_STEPPER_V2_STEP_RESOLUTION_1) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	tmc5160.registers.bits.chopconf.bit.mres      = data->step_resolution;
	tmc5160.registers.bits.chopconf.bit.intpol    = data->interpolation;
	tmc5160.registers_write[TMC5160_REG_CHOPCONF] = true;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_step_configuration(const GetStepConfiguration *data, GetStepConfiguration_Response *response) {
	response->header.length   = sizeof(GetStepConfiguration_Response);
	response->step_resolution = tmc5160.registers.bits.chopconf.bit.mres;
	response->interpolation   = tmc5160.registers.bits.chopconf.bit.intpol;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_motor_current(const SetMotorCurrent *data) {
	// 2300mA max (TODO: Depends on resistor)

	tmc5160.high_level_current = data->current;

	tmc5160.registers.bits.global_scaler.bit.global_scaler = SCALE(data->current, 0, 2300, 0, 255);
	tmc5160.registers_write[TMC5160_REG_GLOBAL_SCALER]     = true;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_motor_current(const GetMotorCurrent *data, GetMotorCurrent_Response *response) {
	response->header.length = sizeof(GetMotorCurrent_Response);
	response->current       = tmc5160.high_level_current;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_enabled(const SetEnabled *data) {
	if(data->enabled) {
		XMC_GPIO_SetOutputLow(TMC5160_ENABLE_PIN);
	} else {
		XMC_GPIO_SetOutputHigh(TMC5160_ENABLE_PIN);
	}

	if(tmc5160.enable_led_flicker_state.config == SILENT_STEPPER_V2_ENABLE_LED_CONFIG_SHOW_ENABLE) {
		if(data->enabled) {
			XMC_GPIO_SetOutputLow(TMC5160_ENABLE_LED_PIN);
		} else {
			XMC_GPIO_SetOutputHigh(TMC5160_ENABLE_LED_PIN);
		}
	}

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_enabled(const GetEnabled *data, GetEnabled_Response *response) {
	response->header.length = sizeof(GetEnabled_Response);
	response->enabled       = !XMC_GPIO_GetInput(TMC5160_ENABLE_PIN);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_basic_configuration(const SetBasicConfiguration *data) {
	if(data->standstill_delay_time > 327) { // 0-327ms -> 0-15
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	if(data->power_down_time > 5570) { // 0-5570ms -> 0-255
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	tmc5160.high_level_standstill_current             = MIN(data->standstill_current, tmc5160.high_level_current);
	tmc5160.high_level_motor_run_current              = MIN(data->motor_run_current,  tmc5160.high_level_current);
	tmc5160.high_level_standstill_delay_time          = data->standstill_delay_time;
	tmc5160.high_level_power_down_time                = data->power_down_time;

	tmc5160.registers.bits.ihold_irun.bit.ihold       = BETWEEN(0, SCALE(data->standstill_current, 0, tmc5160.high_level_current, 0, 31), 31);
	tmc5160.registers.bits.ihold_irun.bit.irun        = BETWEEN(0, SCALE(data->motor_run_current,  0, tmc5160.high_level_current, 0, 31), 31);
	tmc5160.registers.bits.ihold_irun.bit.ihold_delay = SCALE(data->standstill_delay_time, 0, 327, 0, 15);
	tmc5160.registers_write[TMC5160_REG_IHOLD_IRUN]   = true;

	tmc5160.registers.bits.tpowerdown.bit.tpowerdown  = SCALE(data->power_down_time, 0, 5570, 0, 255);
	tmc5160.registers_write[TMC5160_REG_TPOWERDOWN]   = true;

	tmc5160.registers.bits.tpwmthrs.bit.tpwmthrs      = data->stealth_threshold;
	tmc5160.registers_write[TMC5160_REG_TPWMTHRS]     = true;

	tmc5160.registers.bits.tcoolthrs.bit.tcoolthrs    = data->coolstep_threshold;
	tmc5160.registers_write[TMC5160_REG_TCOOLTHRS]    = true;

	tmc5160.registers.bits.thigh.bit.thigh            = data->classic_threshold;
	tmc5160.registers_write[TMC5160_REG_THIGH]        = true;

	tmc5160.registers.bits.chopconf.bit.vhighchm      = data->high_velocity_chopper_mode;
	tmc5160.registers_write[TMC5160_REG_CHOPCONF]     = true;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_basic_configuration(const GetBasicConfiguration *data, GetBasicConfiguration_Response *response) {
	response->header.length              = sizeof(GetBasicConfiguration_Response);
	response->standstill_current         = tmc5160.high_level_standstill_current;
	response->motor_run_current          = tmc5160.high_level_motor_run_current;
	response->standstill_delay_time      = tmc5160.high_level_standstill_delay_time;
	response->power_down_time            = tmc5160.high_level_power_down_time;
	response->stealth_threshold          = tmc5160.registers.bits.tpwmthrs.bit.tpwmthrs;
	response->coolstep_threshold         = tmc5160.registers.bits.tcoolthrs.bit.tcoolthrs;
	response->classic_threshold          = tmc5160.registers.bits.thigh.bit.thigh;
	response->high_velocity_chopper_mode = tmc5160.registers.bits.chopconf.bit.vhighchm;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_spreadcycle_configuration(const SetSpreadcycleConfiguration *data) {
	if((data->slow_decay_duration > 15) ||
	   (data->fast_decay_duration > 15) ||
	   (data->hysteresis_start_value > 7) ||
	   (data->hysteresis_end_value < -3 || data->hysteresis_end_value > 12) ||
	   (data->sine_wave_offset < -3 || data->sine_wave_offset > 12) ||
	   (data->chopper_mode > 1) ||
	   (data->comparator_blank_time > 3)) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}


	tmc5160.registers.bits.chopconf.bit.toff      = data->slow_decay_duration;
//	tmc5160.registers.bits.chopconf.bit.rndtf     = data->enable_random_slow_decay;
	if(data->chopper_mode) {
		tmc5160.registers.bits.chopconf.bit.hstrt = data->fast_decay_duration & 0b111;
		tmc5160.registers.bits.chopconf.bit.fd3   = (data->fast_decay_duration >> 3) & 0b1;
		tmc5160.registers.bits.chopconf.bit.hend  = data->sine_wave_offset; // TODO: handle signednes correctly!
	} else {
		tmc5160.registers.bits.chopconf.bit.hstrt = data->hysteresis_start_value & 0b111;
		tmc5160.registers.bits.chopconf.bit.fd3   = 0;
		tmc5160.registers.bits.chopconf.bit.hend  = data->hysteresis_end_value; // TODO: handle signednes correctly!
	}
	tmc5160.registers.bits.chopconf.bit.chm       = data->chopper_mode;
	tmc5160.registers.bits.chopconf.bit.tbl       = data->comparator_blank_time;
	tmc5160.registers.bits.chopconf.bit.disfdcc   = data->fast_decay_without_comparator;

	tmc5160.registers_write[TMC5160_REG_CHOPCONF] = true;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_spreadcycle_configuration(const GetSpreadcycleConfiguration *data, GetSpreadcycleConfiguration_Response *response) {
	response->header.length = sizeof(GetSpreadcycleConfiguration_Response);
	response->slow_decay_duration           = tmc5160.registers.bits.chopconf.bit.toff;
//	response->enable_random_slow_decay      = tmc5160.registers.bits.chopconf.bit.rndtf;
	if(tmc5160.registers.bits.chopconf.bit.chm) {
		response->fast_decay_duration       = tmc5160.registers.bits.chopconf.bit.hstrt | (tmc5160.registers.bits.chopconf.bit.fd3 << 3);
		response->sine_wave_offset          = tmc5160.registers.bits.chopconf.bit.hend;
		response->hysteresis_start_value    = 0;
		response->hysteresis_end_value      = 0;
	} else {
		response->fast_decay_duration       = 0;
		response->sine_wave_offset          = 0;
		response->hysteresis_start_value    = tmc5160.registers.bits.chopconf.bit.hstrt;
		response->hysteresis_end_value      = tmc5160.registers.bits.chopconf.bit.hend;
	}
	response->chopper_mode                  = tmc5160.registers.bits.chopconf.bit.chm;
	response->comparator_blank_time         = tmc5160.registers.bits.chopconf.bit.tbl;
	response->fast_decay_without_comparator = tmc5160.registers.bits.chopconf.bit.disfdcc;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_stealth_configuration(const SetStealthConfiguration *data) {
	if(data->freewheel_mode > 3) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	tmc5160.registers.bits.gconf.bit.en_pwm_mode     = data->enable_stealth;
	tmc5160.registers.bits.pwmconf.bit.freewheel     = data->freewheel_mode;
	tmc5160.registers_write[TMC5160_REG_GCONF]       = true;

	tmc5160.registers.bits.pwmconf.bit.pwm_ofs       = data->amplitude; // TODO: amplitude -> offfset?
	tmc5160.registers.bits.pwmconf.bit.pwm_grad      = data->gradient;
	tmc5160.registers.bits.pwmconf.bit.pwm_autoscale = data->enable_autoscale;
	tmc5160.registers.bits.pwmconf.bit.pwm_autograd  = data->force_symmetric; // TODO: autograd
	tmc5160.registers.bits.pwmconf.bit.freewheel     = data->freewheel_mode;
	tmc5160.registers_write[TMC5160_REG_PWMCONF]     = true;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_stealth_configuration(const GetStealthConfiguration *data, GetStealthConfiguration_Response *response) {
	response->header.length = sizeof(GetStealthConfiguration_Response);
	response->enable_stealth   = tmc5160.registers.bits.gconf.bit.en_pwm_mode;
	response->amplitude        = tmc5160.registers.bits.pwmconf.bit.pwm_ofs;
	response->gradient         = tmc5160.registers.bits.pwmconf.bit.pwm_grad;
	response->enable_autoscale = tmc5160.registers.bits.pwmconf.bit.pwm_autoscale;
	response->force_symmetric  = tmc5160.registers.bits.pwmconf.bit.pwm_autograd;
	response->freewheel_mode   = tmc5160.registers.bits.pwmconf.bit.freewheel;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_coolstep_configuration(const SetCoolstepConfiguration *data) {
	if((data->minimum_stallguard_value > 15) ||
	   (data->maximum_stallguard_value > 15) ||
	   (data->current_up_step_width > 3) ||
	   (data->current_down_step_width > 3) ||
	   (data->minimum_current > 1) ||
	   (data->stallguard_threshold_value < -64 || data->stallguard_threshold_value > 63) ||
	   (data->stallguard_mode > 1)) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	tmc5160.registers.bits.coolconf.bit.semin     = data->minimum_stallguard_value;
	tmc5160.registers.bits.coolconf.bit.semax     = data->maximum_stallguard_value;
	tmc5160.registers.bits.coolconf.bit.seup      = data->current_up_step_width;
	tmc5160.registers.bits.coolconf.bit.sedn      = data->current_down_step_width;
	tmc5160.registers.bits.coolconf.bit.seimin    = data->minimum_current;
	tmc5160.registers.bits.coolconf.bit.sgt       = data->stallguard_threshold_value;
	tmc5160.registers.bits.coolconf.bit.sfilt     = data->stallguard_mode;
	tmc5160.registers_write[TMC5160_REG_COOLCONF] = true;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_coolstep_configuration(const GetCoolstepConfiguration *data, GetCoolstepConfiguration_Response *response) {
	response->header.length = sizeof(GetCoolstepConfiguration_Response);
	response->minimum_stallguard_value   = tmc5160.registers.bits.coolconf.bit.semin;
	response->maximum_stallguard_value   = tmc5160.registers.bits.coolconf.bit.semax;
	response->current_up_step_width      = tmc5160.registers.bits.coolconf.bit.seup;
	response->current_down_step_width    = tmc5160.registers.bits.coolconf.bit.sedn;
	response->minimum_current            = tmc5160.registers.bits.coolconf.bit.seimin;
	response->stallguard_threshold_value = tmc5160.registers.bits.coolconf.bit.sgt;
	response->stallguard_mode            = tmc5160.registers.bits.coolconf.bit.sfilt;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_misc_configuration(const SetMiscConfiguration *data) {
	if(data->synchronize_phase_frequency > 15) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	tmc5160.registers.bits.chopconf.bit.diss2g    = data->disable_short_to_ground_protection;
	tmc5160.registers.bits.chopconf.bit.sync      = data->synchronize_phase_frequency;
	tmc5160.registers_write[TMC5160_REG_CHOPCONF] = true;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_misc_configuration(const GetMiscConfiguration *data, GetMiscConfiguration_Response *response) {
	response->header.length = sizeof(GetMiscConfiguration_Response);
	response->disable_short_to_ground_protection = tmc5160.registers.bits.chopconf.bit.diss2g;
	response->synchronize_phase_frequency        = tmc5160.registers.bits.chopconf.bit.sync;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_driver_status(const GetDriverStatus *data, GetDriverStatus_Response *response) {
	response->header.length = sizeof(GetDriverStatus_Response);

	response->open_load                 = (tmc5160.registers.bits.drv_status.bit.ola << 0) | (tmc5160.registers.bits.drv_status.bit.olb << 1);
	response->short_to_ground           = (tmc5160.registers.bits.drv_status.bit.s2ga << 0) | (tmc5160.registers.bits.drv_status.bit.s2gb << 1);
	response->over_temperature          = 0;
	if(tmc5160.registers.bits.drv_status.bit.otpw) {
		response->over_temperature      = 1;
	}
	if(tmc5160.registers.bits.drv_status.bit.ot) {
		response->over_temperature      = 2;
	}
	response->motor_stalled             = tmc5160.registers.bits.drv_status.bit.stall_guard;
	response->actual_motor_current      = tmc5160.registers.bits.drv_status.bit.cs_actual;
	response->full_step_active          = tmc5160.registers.bits.drv_status.bit.fsactive;
	response->stallguard_result         = tmc5160.registers.bits.drv_status.bit.sg_result;
	response->stealth_voltage_amplitude = tmc5160.registers.bits.pwm_scale.bit.amplitude_scalar;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_input_voltage(const GetInputVoltage *data, GetInputVoltage_Response *response) {
	response->header.length = sizeof(GetInputVoltage_Response);
	response->voltage       = voltage.value;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_gpio_configuration(const SetGPIOConfiguration *data) {
	if(data->stop_deceleration > 0xFFFF) { // 16 bit
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	gpio.debounce          = data->debounce;
	gpio.stop_deceleration = data->stop_deceleration;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_gpio_configuration(const GetGPIOConfiguration *data, GetGPIOConfiguration_Response *response) {
	response->header.length     = sizeof(GetGPIOConfiguration_Response);
	response->debounce          = gpio.debounce;
	response->stop_deceleration = gpio.stop_deceleration;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_gpio_action(const SetGPIOAction *data) {
	if(data->channel >= GPIO_CHANNEL_NUM) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	gpio.action[data->channel] = data->action;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_gpio_action(const GetGPIOAction *data, GetGPIOAction_Response *response) {
	if(data->channel >= GPIO_CHANNEL_NUM) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	response->header.length = sizeof(GetGPIOAction_Response);
	response->action        = gpio.action[data->channel];

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_gpio_state(const GetGPIOState *data, GetGPIOState_Response *response) {
	response->header.length = sizeof(GetGPIOState_Response);
	response->gpio_state[0] = (gpio.last_interrupt_value[0] << 0) | (gpio.last_interrupt_value[1] << 1);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_error_led_config(const SetErrorLEDConfig *data) {
	if(data->config > SILENT_STEPPER_V2_ERROR_LED_CONFIG_SHOW_ERROR) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	tmc5160.error_led_flicker_state.config = data->config;
	switch(data->config) {
		case SILENT_STEPPER_V2_ERROR_LED_CONFIG_OFF:
			XMC_GPIO_SetOutputHigh(TMC5160_ERROR_LED_PIN);
			break;

		case SILENT_STEPPER_V2_ERROR_LED_CONFIG_ON:
			XMC_GPIO_SetOutputLow(TMC5160_ERROR_LED_PIN);
			break;

		default: break;
	}

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_error_led_config(const GetErrorLEDConfig *data, GetErrorLEDConfig_Response *response) {
	response->header.length = sizeof(GetErrorLEDConfig_Response);
	response->config        = tmc5160.error_led_flicker_state.config;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_enable_led_config(const SetEnableLEDConfig *data) {
	if(data->config > SILENT_STEPPER_V2_ENABLE_LED_CONFIG_SHOW_ENABLE) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	tmc5160.enable_led_flicker_state.config = data->config;
	switch(data->config) {
		case SILENT_STEPPER_V2_ENABLE_LED_CONFIG_OFF:
			XMC_GPIO_SetOutputHigh(TMC5160_ENABLE_LED_PIN);
			break;

		case SILENT_STEPPER_V2_ENABLE_LED_CONFIG_ON:
			XMC_GPIO_SetOutputLow(TMC5160_ENABLE_LED_PIN);
			break;

		case SILENT_STEPPER_V2_ENABLE_LED_CONFIG_SHOW_ENABLE:
			if(XMC_GPIO_GetInput(TMC5160_ENABLE_PIN)) {
				XMC_GPIO_SetOutputHigh(TMC5160_ENABLE_LED_PIN);
			} else {
				XMC_GPIO_SetOutputLow(TMC5160_ENABLE_LED_PIN);
			}
			break;

		default: break;
	}

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_enable_led_config(const GetEnableLEDConfig *data, GetEnableLEDConfig_Response *response) {
	response->header.length = sizeof(GetEnableLEDConfig_Response);
	response->config        = tmc5160.enable_led_flicker_state.config;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_steps_led_config(const SetStepsLEDConfig *data) {
	if(data->config > SILENT_STEPPER_V2_STEPS_LED_CONFIG_SHOW_STEPS) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	tmc5160.steps_led_flicker_state.config = data->config;
	switch(data->config) {
		case SILENT_STEPPER_V2_STEPS_LED_CONFIG_OFF:
			XMC_GPIO_SetOutputHigh(TMC5160_STEPS_LED_PIN);
			break;

		case SILENT_STEPPER_V2_STEPS_LED_CONFIG_ON:
			XMC_GPIO_SetOutputLow(TMC5160_STEPS_LED_PIN);
			break;

		case SILENT_STEPPER_V2_STEPS_LED_CONFIG_SHOW_STEPS:
			XMC_GPIO_SetOutputHigh(TMC5160_STEPS_LED_PIN);
			break;

		default: break;
	}

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_steps_led_config(const GetStepsLEDConfig *data, GetStepsLEDConfig_Response *response) {
	response->header.length = sizeof(GetStepsLEDConfig_Response);
	response->config        = tmc5160.steps_led_flicker_state.config;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_gpio_led_config(const SetGPIOLEDConfig *data) {
	if(data->config > SILENT_STEPPER_V2_GPIO_LED_CONFIG_SHOW_GPIO_ACTIVE_LOW) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	if(data->channel > GPIO_CHANNEL_NUM) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	gpio.gpio_led_flicker_state[data->channel].config = data->config;
	switch(data->config) {
		case SILENT_STEPPER_V2_GPIO_LED_CONFIG_OFF:
			if(data->channel == 0) {
				XMC_GPIO_SetOutputHigh(GPIO_0_LED_PIN);
			} else {
				XMC_GPIO_SetOutputHigh(GPIO_1_LED_PIN);
			}
			break;

		case SILENT_STEPPER_V2_GPIO_LED_CONFIG_ON:
			if(data->channel == 0) {
				XMC_GPIO_SetOutputLow(GPIO_0_LED_PIN);
			} else {
				XMC_GPIO_SetOutputLow(GPIO_1_LED_PIN);
			}
			break;

		case SILENT_STEPPER_V2_GPIO_LED_CONFIG_SHOW_GPIO_ACTIVE_HIGH:
			if(data->channel == 0) {
				if(gpio.last_interrupt_value[0])	{
					XMC_GPIO_SetOutputLow(GPIO_0_LED_PIN);
				} else {
					XMC_GPIO_SetOutputHigh(GPIO_0_LED_PIN);
				}
			} else {
				if(gpio.last_interrupt_value[1])	{
					XMC_GPIO_SetOutputLow(GPIO_1_LED_PIN);
				} else {
					XMC_GPIO_SetOutputHigh(GPIO_1_LED_PIN);
				}
			}
			break;

		case SILENT_STEPPER_V2_GPIO_LED_CONFIG_SHOW_GPIO_ACTIVE_LOW:
			if(data->channel == 0) {
				if(gpio.last_interrupt_value[0])	{
					XMC_GPIO_SetOutputHigh(GPIO_0_LED_PIN);
				} else {
					XMC_GPIO_SetOutputLow(GPIO_0_LED_PIN);
				}
			} else {
				if(gpio.last_interrupt_value[1])	{
					XMC_GPIO_SetOutputHigh(GPIO_1_LED_PIN);
				} else {
					XMC_GPIO_SetOutputLow(GPIO_1_LED_PIN);
				}
			}
			break;
		default: break;
	}

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_gpio_led_config(const GetGPIOLEDConfig *data, GetGPIOLEDConfig_Response *response) {
	if(data->channel > GPIO_CHANNEL_NUM) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	response->header.length = sizeof(GetGPIOLEDConfig_Response);
	response->config        = gpio.gpio_led_flicker_state[data->channel].config;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse write_register(const WriteRegister *data, WriteRegister_Response *response) {
	tmc5160_task_register_write(data->reg, data->value);

	response->header.length = sizeof(WriteRegister_Response);
	response->status        = tmc5160.last_status;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse read_register(const ReadRegister *data, ReadRegister_Response *response) {
	if(data->reg > 127) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	logd("read_register: %d\n\r", data->reg);

	response->header.length = sizeof(ReadRegister_Response);
	response->value         = tmc5160_task_register_read(data->reg);
	response->status        = tmc5160.last_status;

	logd(" -> %b %x\n\r", response->status, response->value);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}


void communication_tick(void) {
	communication_callback_tick();
}

void communication_init(void) {
	communication_callback_init();
}
