/**
 * \file
 *
 * \brief SD/MMC card example with FatFs
 *
 * Copyright (c) 2014-2015 Atmel Corporation. All rights reserved.
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

/**
 * \mainpage SD/MMC Card with FatFs Example
 *
 * \section Purpose
 *
 * This example shows how to implement the SD/MMC stack with the FatFS.
 * It will mount the file system and write a file in the card.
 *
 * The example outputs the information through the standard output (stdio).
 * To know the output used on the board, look in the conf_example.h file
 * and connect a terminal to the correct port.
 *
 * While using Xplained Pro evaluation kits, please attach I/O1 Xplained Pro
 * extension board to EXT1.
 *
 * \section Usage
 *
 * -# Build the program and download it into the board.
 * -# On the computer, open and configure a terminal application.
 * Refert to conf_example.h file.
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 *    \code
 *     -- SD/MMC Card Example on FatFs --
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     Please plug an SD, MMC card in slot.
 *    \endcode
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include "conf_example.h"
#include "taskslist.h"
#include "st7565r.h"
#include "ili9341.h"
#include "ili9341_regs.h"
#include "IdleHook.h"
#include "eepromemul.h"
#include "time.h"
#include "RC522.h"
#include "RC522Pic.h"
#include "dma.h"
#include "winc.h"
#include "DHT22.h"
#include "LCDFunctions/DriverLCD.h"
//#include <string.h>
		 
//static void cdc_uart_init(void)
//{
	//struct usart_config usart_conf;
//
	///* Configure USART for unit test output */
	//usart_get_config_defaults(&usart_conf);
	//usart_conf.mux_setting = CONF_STDIO_MUX_SETTING;
	//usart_conf.pinmux_pad0 = CONF_STDIO_PINMUX_PAD0;
	//usart_conf.pinmux_pad1 = CONF_STDIO_PINMUX_PAD1;
	//usart_conf.pinmux_pad2 = CONF_STDIO_PINMUX_PAD2;
	//usart_conf.pinmux_pad3 = CONF_STDIO_PINMUX_PAD3;
	//usart_conf.baudrate    = CONF_STDIO_BAUDRATE;
//
	//stdio_serial_init(&cdc_uart_module, CONF_STDIO_USART, &usart_conf);
	//usart_enable(&cdc_uart_module);
//}


//#define DATA_LENGTH (512)
//static uint8_t source_memory[DATA_LENGTH];
//static uint8_t destination_memory[DATA_LENGTH];
//static volatile bool transfer_is_done = false;
//COMPILER_ALIGNED(16)
//DmacDescriptor example_descriptor;


//static void transfer_done(struct dma_resource* const resource )
//{
	//transfer_is_done = true;
//}
//static void configure_dma_resource(struct dma_resource *resource)
//{
	//struct dma_resource_config config;
	//dma_get_config_defaults(&config);
	//dma_allocate(resource, &config);
//}
//static void setup_transfer_descriptor(DmacDescriptor *descriptor )
//{
	//struct dma_descriptor_config descriptor_config;
	//dma_descriptor_get_config_defaults(&descriptor_config);
	//descriptor_config.block_transfer_count = sizeof(source_memory);
	//descriptor_config.source_address = (uint32_t)source_memory +
	//sizeof(source_memory);
	//descriptor_config.destination_address = (uint32_t)destination_memory +
	//sizeof(source_memory);
	//dma_descriptor_create(descriptor, &descriptor_config);
//}


////////////////////////////////////////////////////////////////////////////////////////

static void configure_wdt(void)
{
    /* Create a new configuration structure for the Watchdog settings and fill
     * with the default module settings. */
    struct wdt_conf config_wdt;
    wdt_get_config_defaults(&config_wdt);
    /* Set the Watchdog configuration settings */
    config_wdt.always_on      = false;
    config_wdt.clock_source   = GCLK_GENERATOR_1;
    config_wdt.timeout_period = WDT_PERIOD_16384CLK;
    /* Initialize and enable the Watchdog with the user settings */
    wdt_set_config(&config_wdt);
}


///////////////////////////////////////////////////////////////////////////////////////

//struct rtc_module rtc_instance;

//static void configure_rtc_calendar(void)
//{   
	///* Initialize RTC in calendar mode. */
	//struct rtc_calendar_config config_rtc_calendar;
	//rtc_calendar_get_config_defaults(&config_rtc_calendar);
	//config_rtc_calendar.clock_24h     = true;
	//rtc_calendar_init(&rtc_instance, RTC, &config_rtc_calendar);
	//rtc_calendar_enable(&rtc_instance);
//}

///////////////////////////////////////////////////////////////////////////////////////

//static void extint_detection_callback(void)
//{ 
  //if (port_pin_get_input_level(PIN_PA11)) system_reset();
  //else 
  //{
   //EIC->CONFIG[1].bit.SENSE3=EIC_CONFIG_SENSE0_NONE_Val;
   //xPowerDown=true;
  //}
//}

//static void configure_extint_channel(void)
//{
	//struct extint_chan_conf config_extint_chan;
	//extint_chan_get_config_defaults(&config_extint_chan);
	//config_extint_chan.gpio_pin           = PIN_PA11A_EIC_EXTINT11;
	//config_extint_chan.gpio_pin_mux       = MUX_PA11A_EIC_EXTINT11;
	//config_extint_chan.gpio_pin_pull      = EXTINT_PULL_DOWN;
	//config_extint_chan.detection_criteria = EXTINT_DETECT_LOW;
	//extint_chan_set_config(11, &config_extint_chan);
   //
//}

//static void configure_extint_callbacks(void)
//{
	//extint_register_callback(extint_detection_callback,11, EXTINT_CALLBACK_TYPE_DETECT);
	//extint_chan_enable_callback(11,EXTINT_CALLBACK_TYPE_DETECT);
//}

///////////////////////////////////////////////////////////////////////////////////////

static void set_fuses(void)
{struct nvm_fusebits fuses;
	
 nvm_get_fuses(&fuses);
 
#ifdef DEBUG
 
  if ((fuses.eeprom_size!=4) || (fuses.bootloader_size!=7) || (fuses.lockbits!=0xFFFF))
 {
  fuses.eeprom_size=4;
  fuses.bootloader_size=7;
  fuses.lockbits=0xFFFF;
  nvm_set_fuses(&fuses);
 }
 
#else

 
 if ((fuses.eeprom_size!=4) || (fuses.bootloader_size!=0) || (fuses.lockbits!=0xFFFC))
 {
  fuses.eeprom_size=4;
  fuses.bootloader_size=0;
  fuses.lockbits=0xFFFC;
  nvm_set_fuses(&fuses);
  nvm_execute_command(NVM_COMMAND_SET_SECURITY_BIT,0,0);
  system_reset();
 }
 
 #endif
	
}

///////////////////////////////////////////////////////////////////////////////////////

//static void wifi_cb(uint8_t u8MsgType, void *pvMsg) {};

////////////////////////////////

/**
 * \brief Application entry point.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{   
	//uint8_t color;
    //uint8_t counter=0;
	//struct dma_resource example_resource;

    //system_board_init();
    //port_pin_set_output_level(DEBUG_PIN,true);

	system_init();
	xPowerDown=false;
	
	lcd_init();
	nm_bsp_init();
	
	/* Initialize SD MMC stack */
    nm_bsp_init();	
	
    sd_mmc_init();
	DHT_Init();

	
    struct nvm_config nvm_cfg;
    nvm_get_config_defaults(&nvm_cfg);
	nvm_cfg.manual_page_write=false;
    nvm_set_config(&nvm_cfg);
	

    set_fuses();	

    configure_wdt();
	//configure_eeprom();
	
	tasks_init();
	vCreateRFIDTask();
	vCreateWincTask();
	
	// ..and let FreeRTOS run tasks!
	vTaskStartScheduler();
	
}
