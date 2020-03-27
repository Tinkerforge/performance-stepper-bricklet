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

#include "bricklib2/utility/communication_callback.h"
#include "bricklib2/protocols/tfp/tfp.h"
#include "bricklib2/logging/logging.h"

#include "tmc5160.h"

BootloaderHandleMessageResponse handle_message(const void *message, void *response) {
	switch(tfp_get_fid_from_message(message)) {
		case FID_SET_MOTION_CONFIGURATION: return set_motion_configuration(message);
		case FID_GET_MOTION_CONFIGURATION: return get_motion_configuration(message, response);
		case FID_SET_CURRENT_POSITION: return set_current_position(message);
		case FID_GET_CURRENT_POSITION: return get_current_position(message, response);
		case FID_SET_TARGET_POSITION: return set_target_position(message);
		case FID_GET_TARGET_POSITION: return get_target_position(message, response);
		case FID_SET_STEPS: return set_steps(message);
		case FID_GET_STEPS: return get_steps(message, response);
		case FID_GET_REMAINING_STEPS: return get_remaining_steps(message, response);
		case FID_SET_STEP_CONFIGURATION: return set_step_configuration(message);
		case FID_SET_ENABLED: return set_enabled(message);
		case FID_GET_ENABLED: return get_enabled(message, response);
		case FID_WRITE_REGISTER: return write_register(message, response);
		case FID_READ_REGISTER: return read_register(message, response);
		default: return HANDLE_MESSAGE_RESPONSE_NOT_SUPPORTED;
	}
}


BootloaderHandleMessageResponse set_motion_configuration(const SetMotionConfiguration *data) {

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_motion_configuration(const GetMotionConfiguration *data, GetMotionConfiguration_Response *response) {
	response->header.length = sizeof(GetMotionConfiguration_Response);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_current_position(const SetCurrentPosition *data) {

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_current_position(const GetCurrentPosition *data, GetCurrentPosition_Response *response) {
	response->header.length = sizeof(GetCurrentPosition_Response);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_target_position(const SetTargetPosition *data) {

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_target_position(const GetTargetPosition *data, GetTargetPosition_Response *response) {
	response->header.length = sizeof(GetTargetPosition_Response);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_steps(const SetSteps *data) {

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_steps(const GetSteps *data, GetSteps_Response *response) {
	response->header.length = sizeof(GetSteps_Response);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_remaining_steps(const GetRemainingSteps *data, GetRemainingSteps_Response *response) {
	response->header.length = sizeof(GetRemainingSteps_Response);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_step_configuration(const SetStepConfiguration *data) {

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse set_enabled(const SetEnabled *data) {

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_enabled(const GetEnabled *data, GetEnabled_Response *response) {
	response->header.length = sizeof(GetEnabled_Response);

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
