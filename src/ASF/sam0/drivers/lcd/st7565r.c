/**
 * \file
 *
 * \brief ST7565R display controller driver.
 *
 * Copyright (c) 2011-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include "st7565r.h"

struct spi_module spi_master_instance;
//uint8_t ucLCDRAM[128*9];

/**
 * \internal
 * \brief Initialize the hardware interface
 *
 * Depending on what interface used for interfacing the LCD controller this
 * function will initialize the necessary hardware.
 */
static void st7565r_interface_init(void)
{
	struct spi_config config_spi_master;
	
	//struct spi_slave_inst_config slave_dev_config;
	/* Configure and initialize software device instance of peripheral slave */
	//spi_slave_inst_get_config_defaults(&slave_dev_config);
	//slave_dev_config.ss_pin = SLAVE_SELECT_PIN;
	//spi_attach_slave(&slave, &slave_dev_config);
	/* Configure, initialize and enable SERCOM SPI module */
	
	spi_get_config_defaults(&config_spi_master);
	config_spi_master.mux_setting = SPI_SIGNAL_MUX_SETTING_A;

	/* Configure pad 0 for data in */
	config_spi_master.pinmux_pad0 = PINMUX_PA08C_SERCOM0_PAD0;
	/* Configure pad 1 as unused */
	config_spi_master.pinmux_pad1 = PINMUX_PA09C_SERCOM0_PAD1;
	/* Configure pad 2 for data out */
	config_spi_master.pinmux_pad2 = PINMUX_UNUSED;
	/* Configure pad 3 for SCK */
	config_spi_master.pinmux_pad3 = PINMUX_UNUSED;
	config_spi_master.transfer_mode = SPI_TRANSFER_MODE_3;
	config_spi_master.generator_source = GCLK_GENERATOR_0;
	spi_init(&spi_master_instance, ST7565R_USART_SPI, &config_spi_master);
	spi_set_baudrate(&spi_master_instance,ST7565R_CLOCK_SPEED);
	//spi_enable(&spi_master_instance);
}

/**
 * \brief Initialize the LCD controller
 *
 * Call this function to initialize the hardware interface and the LCD
 * controller. When initialization is done the display is turned on and ready
 * to receive data.
 */
void st7565r_init(void)
{
	// Do a hard reset of the LCD display controller
	st7565r_hard_reset();

	// Initialize the interface
	st7565r_interface_init();

	// Set the A0 pin to the default state (command)
	//ioport_set_pin_low(ST7565R_A0_PIN);
	port_pin_set_output_level(ST7565R_CS_PIN,true);
    port_pin_set_output_level(ST7565R_A0_PIN, false);

	// The column address is set to increasing
	st7565r_write_command(ST7565R_CMD_ADC_NORMAL);

	// Non-inverted display
	st7565r_display_invert_enable();

	// The common mode scan direction is reversed COM31->COM0
	st7565r_write_command(ST7565R_CMD_REVERSE_SCAN_DIRECTION);

	// Set the voltage bias ratio to 1/6
	st7565r_write_command(ST7565R_CMD_LCD_BIAS_1_DIV_6_DUTY33);

	// Set booster circuit, voltage regulator and voltage follower all to on
	st7565r_write_command(ST7565R_CMD_POWER_CTRL_ALL_ON);

	// Set the booster ratio to 2X,3X,4X
	st7565r_write_command(ST7565R_CMD_BOOSTER_RATIO_SET);
	st7565r_write_command(ST7565R_CMD_BOOSTER_RATIO_2X_3X_4X);

	// Set voltage resistor ratio to 1
	st7565r_write_command(ST7565R_CMD_VOLTAGE_RESISTOR_RATIO_7);

	/* Set contrast to min value, no need to check return value as the contrast
	is set to the defined min*/
	st7565r_set_contrast(50);

	// Turn on the display
	st7565r_display_on();
}
