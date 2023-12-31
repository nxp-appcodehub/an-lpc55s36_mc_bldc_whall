/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v10.0
processor: LPC55S36
package_id: LPC55S36JBD100
mcu_data: ksdk2_0
processor_version: 0.11.4
board: LPCXpresso55S36
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_iocon.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: cm33_core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '93', peripheral: FLEXCOMM0, signal: RXD_SDA_MOSI_DATA, pin_signal: PIO0_29/FC0_RXD_SDA_MOSI_DATA/CTIMER2_MAT3/SCT0_OUT8/TRACEDATA2/FC6_RXD_SDA_MOSI_DATA/CMP0_OUT/SECURE_GPIO0_29/PWM0_A1/SPI_DIN/EXTTRIG_IN3,
    mode: inactive, slew_rate: standard, invert: disabled, open_drain: disabled}
  - {pin_num: '95', peripheral: FLEXCOMM0, signal: TXD_SCL_MISO_WS, pin_signal: PIO0_30/FC0_TXD_SCL_MISO_WS/CTIMER0_MAT0/SCT0_OUT9/TRACEDATA1/CAN0_TD/FC6_TXD_SCL_MISO_WS/SECURE_GPIO0_30/PWM1_A1/AOI1_OUT0,
    mode: inactive, slew_rate: standard, invert: disabled, open_drain: disabled}
  - {pin_num: '11', peripheral: PWM0, signal: 'A, 0', pin_signal: PIO1_20/FC7_RTS_SCL_SSEL1/CT_INP14/FC4_TXD_SCL_MISO_WS/PWM0_A0/AOI0_OUT1/ADC1_8A}
  - {pin_num: '91', peripheral: PWM0, signal: 'B, 0', pin_signal: PIO1_17/FC6_RTS_SCL_SSEL1/SCT0_OUT4/PWM0_B0/AOI1_OUT3}
  - {pin_num: '50', peripheral: PWM0, signal: 'A, 1', pin_signal: PIO1_6/FC0_TXD_SCL_MISO_WS/CTIMER2_MAT1/SCT_GPI3/PWM0_A1/TRIGOUT_5/HSCMP0_OUT}
  - {pin_num: '40', peripheral: PWM0, signal: 'B, 1', pin_signal: PIO1_22/CTIMER2_MAT3/SCT_GPI5/FC4_SSEL3/CAN0_RD/QSPI_DIN3/PWM0_B1/TRIGOUT_2/HSCMP1_IN1/DAC0_OUT}
  - {pin_num: '36', peripheral: PWM0, signal: 'A, 2', pin_signal: PIO1_8/FC0_CTS_SDA_SSEL0/SCT0_OUT1/FC4_SSEL2/FC1_SCK/PWM0_A2/AOI1_OUT2/TRIGOUT_6}
  - {pin_num: '75', peripheral: PWM0, signal: 'B, 2', pin_signal: PIO1_4/FC0_SCK/CTIMER2_MAT1/SCT0_OUT0/FREQME_GPIO_CLK_A/FC4_TXD_SCL_MISO_WS/SPI_DIN/PWM0_B2/TRIGOUT_7/EXTTRIG_IN8}
  - {pin_num: '70', peripheral: GPIO, signal: 'PIO0, 13', pin_signal: PIO0_13/FC1_CTS_SDA_SSEL0/UTICK_CAP0/CT_INP0/SCT_GPI0/FC1_RXD_SDA_MOSI_DATA/SECURE_GPIO0_13/EXTTRIG_IN3,
    direction: INPUT}
  - {pin_num: '71', peripheral: GPIO, signal: 'PIO0, 14', pin_signal: PIO0_14/FC1_RTS_SCL_SSEL1/UTICK_CAP1/CT_INP1/SCT_GPI1/FC1_TXD_SCL_MISO_WS/SECURE_GPIO0_14/EXTTRIG_IN2,
    direction: INPUT}
  - {pin_num: '78', peripheral: GPIO, signal: 'PIO0, 22', pin_signal: PIO0_22/FC6_TXD_SCL_MISO_WS/UTICK_CAP1/CT_INP15/SCT0_OUT3/FLEXSPI0_SCLK_N/FLEXSPI0_SS1_N/USB0_VBUS/FC7_RTS_SCL_SSEL1/SECURE_GPIO0_22/PWM1_X0/EXTTRIG_IN5,
    direction: OUTPUT}
  - {pin_num: '72', peripheral: GPIO, signal: 'PIO1, 28', pin_signal: PIO1_28/FC7_SCK/CT_INP2/TRIGOUT_4/PWM1_X3/SPI_CS1_DIS/HSCMP1_OUT, direction: OUTPUT}
  - {pin_num: '41', peripheral: GPIO, signal: 'PIO0, 17', pin_signal: PIO0_17/FC4_SSEL2/SCT_GPI7/SCT0_OUT0/FC5_RXD_SDA_MOSI_DATA/QSPI_SCLK/SECURE_GPIO0_17/TRIGOUT_7/HSCMP1_OUT/HSCMP2_IN0,
    identifier: SW3, direction: INPUT}
  - {pin_num: '94', peripheral: GPIO, signal: 'PIO1, 11', pin_signal: PIO1_11/FC1_TXD_SCL_MISO_WS/CT_INP5/USB0_VBUS/HS_SPI_SSEL0/FC6_SCK/PWM0_A0/SPI_SCLK/EXTTRIG_IN8,
    direction: INPUT}
  - {pin_num: '37', peripheral: GPIO, signal: 'PIO1, 21', pin_signal: PIO1_21/FC7_CTS_SDA_SSEL0/CTIMER3_MAT2/FC4_RXD_SDA_MOSI_DATA/PWM1_A0/TRIGOUT_1, direction: OUTPUT}
  - {pin_num: '35', peripheral: HSCMP0, signal: 'IN, 3', pin_signal: PIO1_5/FC0_RXD_SDA_MOSI_DATA/CTIMER2_MAT0/SCT_GPI0/PWM1_A3/TRIGOUT_0/HSCMP0_IN3}
  - {pin_num: '30', peripheral: ADC0, signal: 'CH, 4B', pin_signal: PIO1_19/SCT0_OUT7/CTIMER3_MAT1/SCT_GPI7/FC4_SCK/QSPI_DIN0/AOI1_OUT2/DAC1_OUT/ADC0_4B/HSCMP1_IN5}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M33 */
void BOARD_InitPins(void)
{
    /* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);

    /* Enables the clock for the GPIO0 module */
    CLOCK_EnableClock(kCLOCK_Gpio0);

    /* Enables the clock for the GPIO1 module */
    CLOCK_EnableClock(kCLOCK_Gpio1);

    gpio_pin_config_t gpio0_pin70_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO0_13 (pin 70)  */
    GPIO_PinInit(GPIO, 0U, 13U, &gpio0_pin70_config);

    gpio_pin_config_t gpio0_pin71_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO0_14 (pin 71)  */
    GPIO_PinInit(GPIO, 0U, 14U, &gpio0_pin71_config);

    gpio_pin_config_t SW3_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO0_17 (pin 41)  */
    GPIO_PinInit(BOARD_INITPINS_SW3_GPIO, BOARD_INITPINS_SW3_PORT, BOARD_INITPINS_SW3_PIN, &SW3_config);

    gpio_pin_config_t LED_GREEN_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO0_22 (pin 78)  */
    GPIO_PinInit(BOARD_INITPINS_LED_GREEN_GPIO, BOARD_INITPINS_LED_GREEN_PORT, BOARD_INITPINS_LED_GREEN_PIN, &LED_GREEN_config);

    gpio_pin_config_t LED_BLUE_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO1_11 (pin 94)  */
    GPIO_PinInit(BOARD_INITPINS_LED_BLUE_GPIO, BOARD_INITPINS_LED_BLUE_PORT, BOARD_INITPINS_LED_BLUE_PIN, &LED_BLUE_config);

    gpio_pin_config_t gpio1_pin30_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO1_19 (pin 30)  */
    GPIO_PinInit(GPIO, 1U, 19U, &gpio1_pin30_config);

    gpio_pin_config_t gpio1_pin37_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO1_21 (pin 37)  */
    GPIO_PinInit(GPIO, 1U, 21U, &gpio1_pin37_config);

    gpio_pin_config_t LED_RED_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO1_28 (pin 72)  */
    GPIO_PinInit(BOARD_INITPINS_LED_RED_GPIO, BOARD_INITPINS_LED_RED_PORT, BOARD_INITPINS_LED_RED_PIN, &LED_RED_config);

    IOCON->PIO[0][13] = ((IOCON->PIO[0][13] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Signal(function) select: PORT013 (pin 70) is configured as PIO0_13. */
                         | IOCON_PIO_FUNC(0x00u)

                         /* Select Digital mode: Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_13_DIGIMODE_DIGITAL));

    IOCON->PIO[0][14] = ((IOCON->PIO[0][14] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Signal(function) select: PORT014 (pin 71) is configured as PIO0_14. */
                         | IOCON_PIO_FUNC(0x00u)

                         /* Select Digital mode: Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_14_DIGIMODE_DIGITAL));

    IOCON->PIO[0][17] = ((IOCON->PIO[0][17] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Signal(function) select: PORT017 (pin 41) is configured as PIO0_17. */
                         | IOCON_PIO_FUNC(0x00u)

                         /* Select Digital mode: Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_17_DIGIMODE_DIGITAL));

    IOCON->PIO[0][22] = ((IOCON->PIO[0][22] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Signal(function) select: PORT022 (pin 78) is configured as PIO0_22. */
                         | IOCON_PIO_FUNC(0x00u)

                         /* Select Digital mode: Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_22_DIGIMODE_DIGITAL));

    IOCON->PIO[0][29] = ((IOCON->PIO[0][29] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Signal(function) select: PORT029 (pin 93) is configured as FC0_RXD_SDA_MOSI_DATA. */
                         | IOCON_PIO_FUNC(0x01u)

                         /* Select Digital mode: Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_29_DIGIMODE_DIGITAL));

    IOCON->PIO[0][30] = ((IOCON->PIO[0][30] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Signal(function) select: PORT030 (pin 95) is configured as FC0_TXD_SCL_MISO_WS. */
                         | IOCON_PIO_FUNC(0x01u)

                         /* Select Digital mode: Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_30_DIGIMODE_DIGITAL));

    IOCON->PIO[1][11] = ((IOCON->PIO[1][11] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Signal(function) select: PORT111 (pin 94) is configured as PIO1_11. */
                         | IOCON_PIO_FUNC(0x00u)

                         /* Select Digital mode: Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO1_11_DIGIMODE_DIGITAL));

    IOCON->PIO[1][17] = ((IOCON->PIO[1][17] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Signal(function) select: PORT117 (pin 91) is configured as PWM0_B0. */
                         | IOCON_PIO_FUNC(0x0Bu)

                         /* Select Digital mode: Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO1_17_DIGIMODE_DIGITAL));

    IOCON->PIO[1][19] = ((IOCON->PIO[1][19] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Signal(function) select: PORT119 (pin 30) is configured as ADC0_4B. */
                         | IOCON_PIO_FUNC(0x00u)

                         /* Select Digital mode: Disable digital mode.
                          * Digital input set to 0. */
                         | IOCON_PIO_DIGIMODE(PIO1_19_DIGIMODE_ANALOG));

    IOCON->PIO[1][20] = ((IOCON->PIO[1][20] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Signal(function) select: PORT120 (pin 11) is configured as PWM0_A0. */
                         | IOCON_PIO_FUNC(0x0Bu)

                         /* Select Digital mode: Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO1_20_DIGIMODE_DIGITAL));

    IOCON->PIO[1][21] = ((IOCON->PIO[1][21] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Signal(function) select: PORT121 (pin 37) is configured as PIO1_21. */
                         | IOCON_PIO_FUNC(0x00u)

                         /* Select Digital mode: Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO1_21_DIGIMODE_DIGITAL));

    IOCON->PIO[1][22] = ((IOCON->PIO[1][22] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Signal(function) select: PORT122 (pin 40) is configured as PWM0_B1. */
                         | IOCON_PIO_FUNC(0x0Bu)

                         /* Select Digital mode: Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO1_22_DIGIMODE_DIGITAL));

    IOCON->PIO[1][28] = ((IOCON->PIO[1][28] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Signal(function) select: PORT128 (pin 72) is configured as PIO1_28. */
                         | IOCON_PIO_FUNC(0x00u)

                         /* Select Digital mode: Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO1_28_DIGIMODE_DIGITAL));

    IOCON->PIO[1][4] = ((IOCON->PIO[1][4] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Signal(function) select: PORT14 (pin 75) is configured as PWM0_B2. */
                        | IOCON_PIO_FUNC(0x0Bu)

                        /* Select Digital mode: Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO1_4_DIGIMODE_DIGITAL));

    IOCON->PIO[1][5] = ((IOCON->PIO[1][5] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_MODE_MASK | IOCON_PIO_DIGIMODE_MASK | IOCON_PIO_ASW0_MASK)))

                        /* Signal(function) select: PORT15 (pin 35) is configured as HSCMP0_IN3. */
                        | IOCON_PIO_FUNC(0x00u)

                        /* Mode select (on-chip pull-up/pull-down resistor control): Inactive.
                         * Inactive (no pull-down/pull-up resistor enabled). */
                        | IOCON_PIO_MODE(PIO1_5_MODE_INACTIVE)

                        /* Select Digital mode: Disable digital mode.
                         * Digital input set to 0. */
                        | IOCON_PIO_DIGIMODE(PIO1_5_DIGIMODE_ANALOG)

                        /* Analog switch input control: Analog switch is closed.
                         * (enable). */
                        | IOCON_PIO_ASW0(PIO1_5_ASW0_ENABLE));

    IOCON->PIO[1][6] = ((IOCON->PIO[1][6] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Signal(function) select: PORT16 (pin 50) is configured as PWM0_A1. */
                        | IOCON_PIO_FUNC(0x0Bu)

                        /* Select Digital mode: Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO1_6_DIGIMODE_DIGITAL));

    IOCON->PIO[1][8] = ((IOCON->PIO[1][8] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Signal(function) select: PORT18 (pin 36) is configured as PWM0_A2. */
                        | IOCON_PIO_FUNC(0x0Bu)

                        /* Select Digital mode: Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO1_8_DIGIMODE_DIGITAL));
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
