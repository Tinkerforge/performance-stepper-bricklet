#ifndef CONFIG_TMC5160_H
#define CONFIG_TMC5160_H

#define TMC5160_SPI_BAUDRATE         400000
#define TMC5160_USIC_CHANNEL         USIC0_CH1
#define TMC5160_USIC                 XMC_SPI0_CH1

#define TMC5160_SCLK_PIN             P0_8
#define TMC5160_SCLK_PIN_AF          (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7 | P0_8_AF_U0C1_SCLKOUT)

#define TMC5160_SELECT_PIN           P0_9
#define TMC5160_SELECT_PIN_AF        (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7 | P0_9_AF_U0C1_SELO0)

#define TMC5160_MOSI_PIN             P0_7
#define TMC5160_MOSI_PIN_AF          (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7 | P0_7_AF_U0C1_DOUT0)

#define TMC5160_MISO_PIN             P0_6
#define TMC5160_MISO_INPUT           XMC_USIC_CH_INPUT_DX0
#define TMC5160_MISO_SOURCE          0b010 // DX0C

#define TMC5160_ENABLE_PIN           P0_5
#define TMC5160_DIAGNOSIS_PIN        P0_4

#define TMC5160_SERVICE_REQUEST_RX   2
#define TMC5160_SERVICE_REQUEST_TX   3

#define TMC5160_IRQ_RX               11
#define TMC5160_IRQ_RX_PRIORITY      2
#define TMC5160_IRQ_TX               12
#define TMC5160_IRQ_TX_PRIORITY      3

#endif