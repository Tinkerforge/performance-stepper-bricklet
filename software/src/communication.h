/* silent-stepper-v2-bricklet
 * Copyright (C) 2020 Olaf Lüke <olaf@tinkerforge.com>
 *
 * communication.h: TFP protocol message handling
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

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdint.h>
#include <stdbool.h>

#include "bricklib2/protocols/tfp/tfp.h"
#include "bricklib2/bootloader/bootloader.h"

// Default functions
BootloaderHandleMessageResponse handle_message(const void *data, void *response);
void communication_tick(void);
void communication_init(void);

// Constants

#define SILENT_STEPPER_V2_STEP_RESOLUTION_1 8
#define SILENT_STEPPER_V2_STEP_RESOLUTION_2 7
#define SILENT_STEPPER_V2_STEP_RESOLUTION_4 6
#define SILENT_STEPPER_V2_STEP_RESOLUTION_8 5
#define SILENT_STEPPER_V2_STEP_RESOLUTION_16 4
#define SILENT_STEPPER_V2_STEP_RESOLUTION_32 3
#define SILENT_STEPPER_V2_STEP_RESOLUTION_64 2
#define SILENT_STEPPER_V2_STEP_RESOLUTION_128 1
#define SILENT_STEPPER_V2_STEP_RESOLUTION_256 0

#define SILENT_STEPPER_V2_RAMPING_MODE_POSITIONING 0
#define SILENT_STEPPER_V2_RAMPING_MODE_VELOCITY_NEGATIVE 1
#define SILENT_STEPPER_V2_RAMPING_MODE_VELOCITY_POSITIVE 2
#define SILENT_STEPPER_V2_RAMPING_MODE_HOLD 3

#define SILENT_STEPPER_V2_BOOTLOADER_MODE_BOOTLOADER 0
#define SILENT_STEPPER_V2_BOOTLOADER_MODE_FIRMWARE 1
#define SILENT_STEPPER_V2_BOOTLOADER_MODE_BOOTLOADER_WAIT_FOR_REBOOT 2
#define SILENT_STEPPER_V2_BOOTLOADER_MODE_FIRMWARE_WAIT_FOR_REBOOT 3
#define SILENT_STEPPER_V2_BOOTLOADER_MODE_FIRMWARE_WAIT_FOR_ERASE_AND_REBOOT 4

#define SILENT_STEPPER_V2_BOOTLOADER_STATUS_OK 0
#define SILENT_STEPPER_V2_BOOTLOADER_STATUS_INVALID_MODE 1
#define SILENT_STEPPER_V2_BOOTLOADER_STATUS_NO_CHANGE 2
#define SILENT_STEPPER_V2_BOOTLOADER_STATUS_ENTRY_FUNCTION_NOT_PRESENT 3
#define SILENT_STEPPER_V2_BOOTLOADER_STATUS_DEVICE_IDENTIFIER_INCORRECT 4
#define SILENT_STEPPER_V2_BOOTLOADER_STATUS_CRC_MISMATCH 5

#define SILENT_STEPPER_V2_STATUS_LED_CONFIG_OFF 0
#define SILENT_STEPPER_V2_STATUS_LED_CONFIG_ON 1
#define SILENT_STEPPER_V2_STATUS_LED_CONFIG_SHOW_HEARTBEAT 2
#define SILENT_STEPPER_V2_STATUS_LED_CONFIG_SHOW_STATUS 3

// Function and callback IDs and structs
#define FID_SET_MOTION_CONFIGURATION 1
#define FID_GET_MOTION_CONFIGURATION 2
#define FID_SET_CURRENT_POSITION 3
#define FID_GET_CURRENT_POSITION 4
#define FID_SET_TARGET_POSITION 5
#define FID_GET_TARGET_POSITION 6
#define FID_SET_STEPS 7
#define FID_GET_STEPS 8
#define FID_GET_REMAINING_STEPS 9
#define FID_SET_STEP_CONFIGURATION 10
#define FID_SET_ENABLED 11
#define FID_GET_ENABLED 12
#define FID_WRITE_REGISTER 13
#define FID_READ_REGISTER 14


typedef struct {
	TFPMessageHeader header;
	uint8_t ramping_mode;
	uint16_t velocity_start;
	uint16_t acceleration_1;
	uint16_t velocity_1;
	uint16_t acceleration_max;
	uint16_t velocity_max;
	uint16_t deceleration_max;
	uint16_t deceleration_1;
	uint16_t velocity_stop;
	uint16_t ramp_zero_wait;
} __attribute__((__packed__)) SetMotionConfiguration;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetMotionConfiguration;

typedef struct {
	TFPMessageHeader header;
	uint8_t ramping_mode;
	uint16_t velocity_start;
	uint16_t acceleration_1;
	uint16_t velocity_1;
	uint16_t acceleration_max;
	uint16_t velocity_max;
	uint16_t deceleration_max;
	uint16_t deceleration_1;
	uint16_t velocity_stop;
	uint16_t ramp_zero_wait;
} __attribute__((__packed__)) GetMotionConfiguration_Response;

typedef struct {
	TFPMessageHeader header;
	int32_t position;
} __attribute__((__packed__)) SetCurrentPosition;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetCurrentPosition;

typedef struct {
	TFPMessageHeader header;
	int32_t position;
} __attribute__((__packed__)) GetCurrentPosition_Response;

typedef struct {
	TFPMessageHeader header;
	int32_t position;
} __attribute__((__packed__)) SetTargetPosition;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetTargetPosition;

typedef struct {
	TFPMessageHeader header;
	int32_t position;
} __attribute__((__packed__)) GetTargetPosition_Response;

typedef struct {
	TFPMessageHeader header;
	int32_t steps;
} __attribute__((__packed__)) SetSteps;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetSteps;

typedef struct {
	TFPMessageHeader header;
	int32_t steps;
} __attribute__((__packed__)) GetSteps_Response;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetRemainingSteps;

typedef struct {
	TFPMessageHeader header;
	int32_t steps;
} __attribute__((__packed__)) GetRemainingSteps_Response;

typedef struct {
	TFPMessageHeader header;
	uint8_t step_resolution;
	bool interpolation;
} __attribute__((__packed__)) SetStepConfiguration;

typedef struct {
	TFPMessageHeader header;
	bool enabled;
} __attribute__((__packed__)) SetEnabled;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetEnabled;

typedef struct {
	TFPMessageHeader header;
	bool enabled;
} __attribute__((__packed__)) GetEnabled_Response;

typedef struct {
	TFPMessageHeader header;
	uint8_t reg;
	uint32_t value;
} __attribute__((__packed__)) WriteRegister;

typedef struct {
	TFPMessageHeader header;
	uint8_t status;
} __attribute__((__packed__)) WriteRegister_Response;

typedef struct {
	TFPMessageHeader header;
	uint8_t reg;
} __attribute__((__packed__)) ReadRegister;

typedef struct {
	TFPMessageHeader header;
	uint8_t status;
	uint32_t value;
} __attribute__((__packed__)) ReadRegister_Response;


// Function prototypes
BootloaderHandleMessageResponse set_motion_configuration(const SetMotionConfiguration *data);
BootloaderHandleMessageResponse get_motion_configuration(const GetMotionConfiguration *data, GetMotionConfiguration_Response *response);
BootloaderHandleMessageResponse set_current_position(const SetCurrentPosition *data);
BootloaderHandleMessageResponse get_current_position(const GetCurrentPosition *data, GetCurrentPosition_Response *response);
BootloaderHandleMessageResponse set_target_position(const SetTargetPosition *data);
BootloaderHandleMessageResponse get_target_position(const GetTargetPosition *data, GetTargetPosition_Response *response);
BootloaderHandleMessageResponse set_steps(const SetSteps *data);
BootloaderHandleMessageResponse get_steps(const GetSteps *data, GetSteps_Response *response);
BootloaderHandleMessageResponse get_remaining_steps(const GetRemainingSteps *data, GetRemainingSteps_Response *response);
BootloaderHandleMessageResponse set_step_configuration(const SetStepConfiguration *data);
BootloaderHandleMessageResponse set_enabled(const SetEnabled *data);
BootloaderHandleMessageResponse get_enabled(const GetEnabled *data, GetEnabled_Response *response);
BootloaderHandleMessageResponse write_register(const WriteRegister *data, WriteRegister_Response *response);
BootloaderHandleMessageResponse read_register(const ReadRegister *data, ReadRegister_Response *response);

// Callbacks


#define COMMUNICATION_CALLBACK_TICK_WAIT_MS 1
#define COMMUNICATION_CALLBACK_HANDLER_NUM 0
#define COMMUNICATION_CALLBACK_LIST_INIT \


#endif
