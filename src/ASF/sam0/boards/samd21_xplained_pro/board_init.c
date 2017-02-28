/**
 * \file
 *
 * \brief SAM D21 Xplained Pro board initialization
 *
 * Copyright (c) 2013-2015 Atmel Corporation. All rights reserved.
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

#include <compiler.h>
#include <board.h>
#include <conf_board.h>
#include <port.h>

#include "conf_st7565r.h"

#if defined(__GNUC__)
void board_init(void) WEAK __attribute__((alias("system_board_init")));
#elif defined(__ICCARM__)
void board_init(void);
#  pragma weak board_init=system_board_init
#endif

void system_board_init(void)
{
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);

	///* Configure LEDs as outputs, turn them off */
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
    port_pin_set_config(LED_0_PIN, &pin_conf);
    port_pin_set_output_level(LED_0_PIN,true);
	
    port_pin_set_config(RELAY_1, &pin_conf);
    port_pin_set_output_level(RELAY_1,false);
	
    port_pin_set_config(RELAY_2, &pin_conf);
    port_pin_set_output_level(RELAY_2,false);

	//
	//port_get_config_defaults(&pin_conf);
//
	///* Configure LEDs as outputs, turn them off */
	//pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	//port_pin_set_config(LED_1_PIN, &pin_conf);
	//port_pin_set_output_level(LED_1_PIN,false);

	//pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	//port_pin_set_config(ST7565R_A0_PIN, &pin_conf);
	//port_pin_set_output_level(ST7565R_A0_PIN,true);
	//pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	//port_pin_set_config(ST7565R_CS_PIN, &pin_conf);
	//port_pin_set_output_level(ST7565R_CS_PIN,true);
	//pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	//port_pin_set_config(ST7565R_RESET_PIN, &pin_conf);
	//port_pin_set_output_level(ST7565R_RESET_PIN,true);
	//
	//pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	//port_pin_set_config(WIFI_RESET_PIN, &pin_conf);
	//port_pin_set_output_level(WIFI_RESET_PIN,false);	
	//
    //pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	//port_pin_set_config(DEBUG_PIN, &pin_conf);
	//port_pin_set_output_level(DEBUG_PIN,false);	
	
	//pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	//port_pin_set_config(PIN_PA00,&pin_conf);
	//port_pin_set_output_level(PIN_PA00,false);
		//
	//pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	//port_pin_set_config(PIN_PA01,&pin_conf);
	//port_pin_set_output_level(PIN_PA01,false);
	
	/* Set buttons as inputs */
	//pin_conf.direction  = PORT_PIN_DIR_INPUT;
	//pin_conf.input_pull = PORT_PIN_PULL_UP;
	//port_pin_set_config(BUTTON_0_PIN, &pin_conf);
	
#ifdef CONF_BOARD_AT86RFX
	port_get_config_defaults(&pin_conf);
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(AT86RFX_SPI_SCK, &pin_conf);
	port_pin_set_config(AT86RFX_SPI_MOSI, &pin_conf);
	port_pin_set_config(AT86RFX_SPI_CS, &pin_conf);
	port_pin_set_config(AT86RFX_RST_PIN, &pin_conf);
	port_pin_set_config(AT86RFX_SLP_PIN, &pin_conf);
	port_pin_set_output_level(AT86RFX_SPI_SCK, true);
	port_pin_set_output_level(AT86RFX_SPI_MOSI, true);
	port_pin_set_output_level(AT86RFX_SPI_CS, true);
	port_pin_set_output_level(AT86RFX_RST_PIN, true);
	port_pin_set_output_level(AT86RFX_SLP_PIN, true);
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	port_pin_set_config(AT86RFX_SPI_MISO, &pin_conf);
#endif	
}
