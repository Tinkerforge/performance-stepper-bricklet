/* performance-stepper-bricklet
 * Copyright (C) 2020 Olaf Lüke <olaf@tinkerforge.com>
 *
 * config_tmc5160.h: TMC5160 driver configurations
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


#ifndef CONFIG_TMC5160_H
#define CONFIG_TMC5160_H

#define TMC5160_ENABLE_PIN             P0_0
#define TMC5160_DIAGNOSIS0_PIN         P0_1
#define TMC5160_DIAGNOSIS1_PIN         P0_2

#define TMC5160_ENABLE_LED_PIN         P1_3
#define TMC5160_STEPS_LED_PIN          P1_4
#define TMC5160_ERROR_LED_PIN          P1_2

#define TMC5160_SPI_BAUDRATE           2000000
#define TMC5160_USIC_CHANNEL           USIC0_CH1
#define TMC5160_USIC_SPI               XMC_SPI0_CH1

#define TMC5160_RX_FIFO_SIZE           XMC_USIC_CH_FIFO_SIZE_16WORDS
#define TMC5160_RX_FIFO_POINTER        32
#define TMC5160_TX_FIFO_SIZE           XMC_USIC_CH_FIFO_SIZE_16WORDS
#define TMC5160_TX_FIFO_POINTER        48

#define TMC5160_CLOCK_PASSIVE_LEVEL    XMC_SPI_CH_BRG_SHIFT_CLOCK_PASSIVE_LEVEL_1_DELAY_DISABLED
#define TMC5160_CLOCK_OUTPUT           XMC_SPI_CH_BRG_SHIFT_CLOCK_OUTPUT_SCLK
#define TMC5160_SLAVE                  XMC_SPI_CH_SLAVE_SELECT_0

#define TMC5160_SCLK_PORT              XMC_GPIO_PORT0
#define TMC5160_SCLK_PIN               8
#define TMC5160_SCLK_PIN_MODE          (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7 | P0_8_AF_U0C1_SCLKOUT)

#define TMC5160_SELECT_PORT            XMC_GPIO_PORT0
#define TMC5160_SELECT_PIN             9
#define TMC5160_SELECT_PIN_MODE        (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7 | P0_9_AF_U0C1_SELO0)

#define TMC5160_MOSI_PORT              XMC_GPIO_PORT0
#define TMC5160_MOSI_PIN               7
#define TMC5160_MOSI_PIN_MODE          (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7 | P0_7_AF_U0C1_DOUT0)

#define TMC5160_MISO_PORT              XMC_GPIO_PORT0
#define TMC5160_MISO_PIN               6
#define TMC5160_MISO_INPUT             XMC_USIC_CH_INPUT_DX0
#define TMC5160_MISO_SOURCE            0b010 // DX0C

#endif