/* silent-stepper-v2-bricklet
 * Copyright (C) 2020 Olaf Lüke <olaf@tinkerforge.com>
 *
 * gpio.c: Driver for GPIO inputs/interrupts
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

#include "gpio.h"
#include "configs/config_gpio.h"

#include "communication.h"

#include "xmc_gpio.h"
#include "xmc_eru.h"
#include "xmc_scu.h"

#define gpio_0_handler IRQ_Hdlr_5
#define gpio_1_handler IRQ_Hdlr_6

GPIO gpio;

volatile bool gpio_0_interrupt = false;
volatile bool gpio_1_interrupt = false;

void __attribute__((optimize("-O3"))) __attribute__ ((section (".ram_code"))) gpio_0_handler(void) {
    gpio_0_interrupt = XMC_GPIO_GetInput(GPIO_0_PIN);
	if(gpio_0_interrupt) {
		XMC_GPIO_SetOutputHigh(P1_5);
	} else {
		XMC_GPIO_SetOutputLow(P1_5);
	}
}

void __attribute__((optimize("-O3"))) __attribute__ ((section (".ram_code"))) gpio_1_handler(void) {
    gpio_1_interrupt = XMC_GPIO_GetInput(GPIO_1_PIN);
	if(gpio_1_interrupt) {
		XMC_GPIO_SetOutputHigh(P1_6);
	} else {
		XMC_GPIO_SetOutputLow(P1_6);
	}
}

void gpio_init(void) {
    memset(&gpio, 0, sizeof(GPIO));

	gpio.action[0]         = SILENT_STEPPER_V2_GPIO_ACTION_NONE;
	gpio.action[1]         = SILENT_STEPPER_V2_GPIO_ACTION_NONE;
	gpio.debounce          = 200; // 200ms default
	gpio.stop_deceleration = 0xFFFF;


	const XMC_GPIO_CONFIG_t gpio_pin_config = {
		.mode             = XMC_GPIO_MODE_INPUT_TRISTATE,
		.input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_LARGE
	};

	const XMC_ERU_ETL_CONFIG_t sync_etl_0_config = {
		.input_a                = GPIO_0_INPUT_A,
		.input_b                = GPIO_0_INPUT_B,
		.enable_output_trigger  = 1,
		.edge_detection         = XMC_ERU_ETL_EDGE_DETECTION_BOTH,
		.output_trigger_channel = GPIO_0_TRG_CHANNEL,
		.source                 = GPIO_0_SOURCE
	};

	XMC_GPIO_Init(GPIO_0_PIN, &gpio_pin_config);
	XMC_ERU_ETL_Init(GPIO_0_ERU, GPIO_0_ETL_CHANNEL, &sync_etl_0_config);
	XMC_ERU_OGU_SetServiceRequestMode(GPIO_0_ERU, GPIO_0_OGU_CHANNEL, XMC_ERU_OGU_SERVICE_REQUEST_ON_TRIGGER);
	NVIC_SetPriority(GPIO_0_IRQ_N, GPIO_0_IRQ_PRIO);
    XMC_SCU_SetInterruptControl(GPIO_0_IRQ_N, GPIO_0_IRQ_CTRL);
	NVIC_ClearPendingIRQ(GPIO_0_IRQ_N);
	NVIC_EnableIRQ(GPIO_0_IRQ_N);


    const XMC_ERU_ETL_CONFIG_t sync_etl_1_config = {
		.input_a                = GPIO_1_INPUT_A,
		.input_b                = GPIO_1_INPUT_B,
		.enable_output_trigger  = 1,
		.edge_detection         = XMC_ERU_ETL_EDGE_DETECTION_BOTH,
		.output_trigger_channel = GPIO_1_TRG_CHANNEL,
		.source                 = GPIO_1_SOURCE
	};

	XMC_GPIO_Init(GPIO_1_PIN, &gpio_pin_config);
	XMC_ERU_ETL_Init(GPIO_1_ERU, GPIO_1_ETL_CHANNEL, &sync_etl_1_config);
	XMC_ERU_OGU_SetServiceRequestMode(GPIO_1_ERU, GPIO_1_OGU_CHANNEL, XMC_ERU_OGU_SERVICE_REQUEST_ON_TRIGGER);
	NVIC_SetPriority(GPIO_1_IRQ_N, GPIO_1_IRQ_PRIO);
    XMC_SCU_SetInterruptControl(GPIO_1_IRQ_N, GPIO_1_IRQ_CTRL);
	NVIC_ClearPendingIRQ(GPIO_1_IRQ_N);
	NVIC_EnableIRQ(GPIO_1_IRQ_N);


	const XMC_GPIO_CONFIG_t led_pin_config = {
		.mode         = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
		.output_level = XMC_GPIO_OUTPUT_LEVEL_LOW
	};

	XMC_GPIO_Init(P1_5, &led_pin_config);
	XMC_GPIO_Init(P1_6, &led_pin_config);
}

void gpio_tick(void) {

}