/**
 * \file
 *
 * \brief ILI9341 display controller component driver
 *
 * Copyright (c) 2012-2015 Atmel Corporation. All rights reserved.
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
#include "conf_ili9341.h"
#include "ili9341.h"
#include "ili9341_regs.h"
//#include <sysclk.h>
//#include <ioport.h>
//#include <delay.h>

#if (SAM3S || SAM3N || SAM4S)
#  include <pdc.h>
#elif UC3
#  include <pdca.h>
#endif

#if (SAM3S || SAM3N || SAM4S) && defined(CONF_ILI9341_SPI)
#  define ILI9341_DMA_ENABLED
#  define ILI9341_DMA_CHUNK_SIZE   16
#elif UC3 && defined(CONF_ILI9341_PDCA_CHANNEL)
#  define ILI9341_DMA_ENABLED
#  define ILI9341_DMA_CHUNK_SIZE   16
#endif

#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif

struct spi_module spi_master_instance;

COMPILER_ALIGNED(16)
DmacDescriptor example_descriptor_tx;

static volatile bool transfer_tx_is_done = false;
TaskHandle_t xcbTaskH;

#define BUF_LENGTH 240
static ili9341_color_t buffer_tx[BUF_LENGTH]; // = {
//        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
//        0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14,
//};

struct dma_resource example_resource_tx;

const int16_t WIDTH=320, HEIGHT=240;  
int16_t _width=320;
int16_t _height=240;
 
int16_t cursor_x=0, cursor_y=0;
uint16_t textcolor=ILI9340_GREEN, textbgcolor=ILI9340_BLACK;
uint8_t textsize=2,rotation=0;
bool wrap=true;   // If set, 'wrap' text at right edge of display
bool _cp437=false; // If set, use correct CP437 charset (default is off)
GFXfont *gfxFont=NULL;

static void transfer_tx_done(struct dma_resource* const resource )
{
transfer_tx_is_done=true;
 portENTER_CRITICAL();
 xTaskResumeFromISR(xcbTaskH);
 portEXIT_CRITICAL();
}


static void configure_dma_resource_tx(struct dma_resource *tx_resource)
{
    struct dma_resource_config tx_config;
    dma_get_config_defaults(&tx_config);
    tx_config.peripheral_trigger = CONF_ILI9341_TRIGGER_TX;
    tx_config.trigger_action = DMA_TRIGGER_ACTON_BEAT;
    dma_allocate(tx_resource, &tx_config);
}

static void setup_transfer_descriptor_tx(DmacDescriptor *tx_descriptor)
{
    struct dma_descriptor_config tx_descriptor_config;
    dma_descriptor_get_config_defaults(&tx_descriptor_config);
    tx_descriptor_config.beat_size = DMA_BEAT_SIZE_BYTE;
    tx_descriptor_config.dst_increment_enable = false;
    tx_descriptor_config.block_transfer_count =  BUF_LENGTH<<1; //sizeof(buffer_tx)/sizeof(uint8_t);
    tx_descriptor_config.source_address = (uint32_t)buffer_tx + sizeof(buffer_tx);
    tx_descriptor_config.destination_address =
        (uint32_t)(&spi_master_instance.hw->SPI.DATA.reg);
    dma_descriptor_create(tx_descriptor, &tx_descriptor_config);
}


/**
 * \internal
 * \brief Helper function to select the CS of the controller on the bus
 */
static inline void ili9341_select_chip(void)
{
	//port_pin_set_output_level(CONF_ILI9341_CS_PIN, false);
	PORT->Group[0].OUTCLR.reg=(1<<CONF_ILI9341_CS_PIN);
}

/**
 * \internal
 * \brief Helper function to de-select the CS of the controller on the bus
 */
static inline void ili9341_deselect_chip(void)
{
	//port_pin_set_output_level(CONF_ILI9341_CS_PIN, true);
	PORT->Group[0].OUTSET.reg=(1<<CONF_ILI9341_CS_PIN);
}

/**
 * \internal
 * \brief Helper function to select command byte transmission mode
 */
static inline void ili9341_select_command_mode(void)
{
	//port_pin_set_output_level(CONF_ILI9341_DC_PIN, false);
	PORT->Group[0].OUTCLR.reg=(1<<CONF_ILI9341_DC_PIN);
}

/**
 * \internal
 * \brief Helper function to select data byte transmission mode
 */
static inline void ili9341_select_data_mode(void)
{
	//port_pin_set_output_level(CONF_ILI9341_DC_PIN, true);
	PORT->Group[0].OUTSET.reg=(1<<CONF_ILI9341_DC_PIN);
}

/**
 * \internal
 * \brief Helper function to wait for the last send operation to complete
 */
static inline void ili9341_wait_for_send_done(void)
{
//#if defined(CONF_ILI9341_USART_SPI)
//#  if XMEGA
	//while (!usart_tx_is_complete(CONF_ILI9341_USART_SPI)) {
		///* Do nothing */
	//}
	//usart_clear_tx_complete(CONF_ILI9341_USART_SPI);
//#  else
	///* Wait for TX to complete */
	//while (!usart_spi_is_tx_empty(CONF_ILI9341_USART_SPI)) {
		///* Do nothing */
	//}
//#  endif
//#elif defined(CONF_ILI9341_SPI)
	///* Wait for TX to complete */
	//while (!spi_is_tx_empty(CONF_ILI9341_SPI)) {
		///* Do nothing */
	//}
//#endif

while (!spi_master_instance.hw->SPI.INTFLAG.bit.TXC);




//while (!spi_is_write_complete(&spi_master_instance));

}

/**
 * \internal
 * \brief Helper function to send a byte over an arbitrary interface
 *
 * This function is used to hide what interface is used by the component
 * driver, e.g.  the component driver does not need to know if USART in SPI
 * mode is used or the native SPI module.
 *
 * \param data The byte to be transfered
 */
static void ili9341_send_byte(uint8_t data)
{
//#if defined(CONF_ILI9341_USART_SPI)
//#  if XMEGA
	//while (!usart_data_register_is_empty(CONF_ILI9341_USART_SPI)) {
		///* Do nothing */
	//}
//
	//irqflags_t flags = cpu_irq_save();
	//usart_clear_tx_complete(CONF_ILI9341_USART_SPI);
	//usart_put(CONF_ILI9341_USART_SPI, data);
	//cpu_irq_restore(flags);
//#  else
	//usart_spi_write_single(CONF_ILI9341_USART_SPI, data);
//#  endif
//#elif defined(CONF_ILI9341_SPI)
	///* Wait for any previously running send data */
	//ili9341_wait_for_send_done();
//
	//spi_write_single(CONF_ILI9341_SPI, data);
//#endif

//spi_write_buffer_wait(&spi_master_instance,&data,1);
 
//spi_write(&spi_master_instance,data);

while (!spi_master_instance.hw->SPI.INTFLAG.bit.DRE);
spi_master_instance.hw->SPI.DATA.bit.DATA = data; // & SERCOM_SPI_DATA_MASK;

}

/**
 * \internal
 * \brief Helper function to read a byte from an arbitrary interface
 *
 * This function is used to hide what interface is used by the component
 * driver, e.g.  the component driver does not need to know if USART in SPI
 * mode is used or the native SPI module.
 *
 * \retval uint8_t Byte of data read from the display controller
 */
static uint8_t ili9341_read_byte(void)
{
	//uint8_t data;
//
//#if defined(CONF_ILI9341_USART_SPI)
//#  if XMEGA
	///* Workaround for clearing the RXCIF for XMEGA */
	//usart_rx_enable(CONF_ILI9341_USART_SPI);
//
	//usart_put(CONF_ILI9341_USART_SPI, 0xFF);
	//while (!usart_rx_is_complete(CONF_ILI9341_USART_SPI)) {
		///* Do nothing */
	//}
	//data = usart_get(CONF_ILI9341_USART_SPI);
//
	///* Workaround for clearing the RXCIF for XMEGA */
	//usart_rx_disable(CONF_ILI9341_USART_SPI);
//#  else
	//usart_spi_read_single(CONF_ILI9341_USART_SPI, &data);
//#  endif
//#elif defined(CONF_ILI9341_SPI)
	//spi_write_single(CONF_ILI9341_SPI, 0xFF);
//
	//ili9341_wait_for_send_done();
//
	///* Wait for RX to complete */
	//while (!spi_is_rx_full(CONF_ILI9341_SPI)) {
		///* Do nothing */
	//}
//
	//spi_read_single(CONF_ILI9341_SPI, &data);
//#endif
//
	return 0;
}

/**
 * \internal
 * \brief Sends a command to the controller, and prepares for parameter transfer
 *
 * Helper function to use for sending a command to the controller.
 *
 * \note After the command is sent, the display remains selected.
 *
 * \param command The command to send
 */
static void ili9341_send_command(uint8_t command)
{
	ili9341_select_command_mode();
	ili9341_select_chip();
	ili9341_send_byte(command);
	ili9341_wait_for_send_done();
	ili9341_select_data_mode();
}

static ili9341_coord_t limit_start_x, limit_start_y;
static ili9341_coord_t limit_end_x, limit_end_y;

/**
 * \internal
 * \brief Helper function to send the drawing limits (boundaries) to the display
 *
 * This function is used to send the currently set upper-left and lower-right
 * drawing limits to the display, as set through the various limit functions.
 *
 * \param send_end_limits  True to also send the lower-right drawing limits
 */
static void ili9341_send_draw_limits(const bool send_end_limits)
{
	ili9341_send_command(ILI9341_CMD_COLUMN_ADDRESS_SET);
	ili9341_send_byte(limit_start_x >> 8);
	ili9341_send_byte(limit_start_x & 0xFF);
	if (send_end_limits) {
		ili9341_send_byte(limit_end_x >> 8);
		ili9341_send_byte(limit_end_x & 0xFF);
	}
	ili9341_wait_for_send_done();
	ili9341_deselect_chip();

	ili9341_send_command(ILI9341_CMD_PAGE_ADDRESS_SET);
	ili9341_send_byte(limit_start_y >> 8);
	ili9341_send_byte(limit_start_y & 0xFF);
	if (send_end_limits) {
		ili9341_send_byte(limit_end_y >> 8);
		ili9341_send_byte(limit_end_y & 0xFF);
	}
	ili9341_wait_for_send_done();
	ili9341_deselect_chip();
}

/**
 * \brief Set the display top left drawing limit
 *
 * Use this function to set the top left limit of the drawing limit box.
 *
 * \param x The x coordinate of the top left corner
 * \param y The y coordinate of the top left corner
 */
void ili9341_set_top_left_limit(ili9341_coord_t x, ili9341_coord_t y)
{
	limit_start_x = x;
	limit_start_y = y;

	ili9341_send_draw_limits(false);
}

/**
 * \brief Set the display bottom right drawing limit
 *
 * Use this function to set the bottom right corner of the drawing limit box.
 *
 * \param x The x coordinate of the bottom right corner
 * \param y The y coordinate of the bottom right corner
 */
void ili9341_set_bottom_right_limit(ili9341_coord_t x, ili9341_coord_t y)
{
	limit_end_x = x;
	limit_end_y = y;

	ili9341_send_draw_limits(true);
}

/**
 * \brief Set the full display drawing limits
 *
 * Use this function to set the full drawing limit box.
 *
 * \param start_x The x coordinate of the top left corner
 * \param start_y The y coordinate of the top left corner
 * \param end_x The x coordinate of the bottom right corner
 * \param end_y The y coordinate of the bottom right corner
 */
void ili9341_set_limits(ili9341_coord_t start_x, ili9341_coord_t start_y,
		ili9341_coord_t end_x, ili9341_coord_t end_y)
{
	limit_start_x = start_x;
	limit_start_y = start_y;
	limit_end_x = end_x;
	limit_end_y = end_y;

	ili9341_send_draw_limits(true);
}

/**
 * \brief Read a single color from the graphical memory
 *
 * Use this function to read a color from the graphical memory of the
 * controller.
 *
 * Limits have to be set prior to calling this function, e.g.:
 * \code
	ili9341_set_top_left_limit(0, 0);
	ili9341_set_bottom_right_limit(320, 240);
	...
\endcode
 *
 * \retval ili9341_color_t The read color pixel
 */
ili9341_color_t ili9341_read_gram(void)
{
	uint8_t red, green, blue;

	ili9341_send_command(ILI9341_CMD_MEMORY_READ);

	/* No interesting data in the first byte, hence read and discard */
	red = ili9341_read_byte();

	red = ili9341_read_byte();
	green = ili9341_read_byte();
	blue = ili9341_read_byte();

	ili9341_deselect_chip();

	return ILI9341_COLOR(red, green, blue);
}

/**
 * \brief Write the graphical memory with a single color pixel
 *
 * Use this function to write a single color pixel to the controller memory.
 *
 * Limits have to be set prior to calling this function, e.g.:
 * \code
	ili9341_set_top_left_limit(0, 0);
	ili9341_set_bottom_right_limit(320, 240);
	...
\endcode
 *
 * \param color The color pixel to write to the screen
 */
void ili9341_write_gram(ili9341_color_t color)
{
	/* Only 16-bit color supported */
	Assert(sizeof(color) == 2);

	ili9341_send_command(ILI9341_CMD_MEMORY_WRITE);
	ili9341_send_byte(color);
	ili9341_send_byte(color >> 8);
	ili9341_wait_for_send_done();
	ili9341_deselect_chip();
}

/**
 * \brief Copy pixels from SRAM to the screen
 *
 * Used to copy a large quantitative of data to the screen in one go.
 *
 * Limits have to be set prior to calling this function, e.g.:
 * \code
	ili9341_set_top_left_limit(0, 0);
	ili9341_set_bottom_right_limit(320, 240);
	...
\endcode
 *
 * \param pixels Pointer to the pixel data
 * \param count Number of pixels to copy to the screen
 */
void ili9341_copy_pixels_to_screen(const ili9341_color_t *pixels, uint32_t count)
{
	const ili9341_color_t *pixel = pixels;
	
	uint8_t red=0;
	uint32_t counter=0;

	/* Sanity check to make sure that the pixel count is not zero */
	Assert(count > 0);

	ili9341_send_command(ILI9341_CMD_MEMORY_WRITE);

#if defined(ILI9341_DMA_ENABLED)
	ili9341_color_t chunk_buf[ILI9341_DMA_CHUNK_SIZE];
	uint32_t chunk_len;

#  if SAM
	Pdc *SPI_DMA = spi_get_pdc_base(CONF_ILI9341_SPI);
	pdc_packet_t spi_pdc_data;

	pdc_enable_transfer(SPI_DMA, PERIPH_PTCR_TXTEN);
	spi_pdc_data.ul_addr = (uint32_t)chunk_buf;
#  elif UC3
	pdca_set_transfer_size(CONF_ILI9341_PDCA_CHANNEL,
			PDCA_TRANSFER_SIZE_BYTE);
	pdca_set_peripheral_select(CONF_ILI9341_PDCA_CHANNEL,
			CONF_ILI9341_PDCA_PID);
#  endif

	while (count)
	{
		/* We need to copy out the data to send in chunks into RAM, as the PDC
		 * does not allow FLASH->Peripheral transfers */
		chunk_len = min(ILI9341_DMA_CHUNK_SIZE, count);

		/* Wait for pending transfer to complete */
		ili9341_wait_for_send_done();

		for (uint32_t i = 0; i < chunk_len; i++) {
			chunk_buf[i] = le16_to_cpu(pixel[i]);
		}

#  if SAM
		spi_pdc_data.ul_size = (uint32_t)sizeof(ili9341_color_t) * chunk_len;
		pdc_tx_init(SPI_DMA, NULL, &spi_pdc_data);
#  elif UC3
		pdca_reload_channel(CONF_ILI9341_PDCA_CHANNEL, chunk_buf,
				(uint32_t)sizeof(ili9341_color_t) * chunk_len);
		pdca_enable(CONF_ILI9341_PDCA_CHANNEL);
#  endif

		pixel += chunk_len;
		count -= chunk_len;
	}

	ili9341_wait_for_send_done();
	ili9341_deselect_chip();

#  if SAM
	pdc_disable_transfer(SPI_DMA, PERIPH_PTCR_TXTEN);
#  elif UC3
	pdca_disable(CONF_ILI9341_PDCA_CHANNEL);
#  endif
#else
 
	while (count--) {
		//ili9341_send_byte(*pixel);
		//ili9341_send_byte(*pixel >> 8);
		
		//ILI9341_GREEN
		
		if (counter<106)
		{
		ili9341_send_byte(ILI9341_COLOR(0xff,0,0));
        ili9341_send_byte(ILI9341_COLOR(0xff,0,0) >> 8);
		}
		else if (counter<212)
		{
		ili9341_send_byte(ILI9341_COLOR(0xff,0xff,0xff));
        ili9341_send_byte(ILI9341_COLOR(0xff,0xff,0xff) >> 8);
		}
		else
		{
		ili9341_send_byte(ILI9341_COLOR(0,0xff,0));
		ili9341_send_byte(ILI9341_COLOR(0,0xff,0) >> 8);
		}			
		
		red=(red+0x01) % 240;
		
		if (!red) counter++;
		
		
		
		pixel++;
	}

	ili9341_wait_for_send_done();
	ili9341_deselect_chip();
#endif
}

#if XMEGA
/**
 * \brief Copy pixels from progmem to the screen
 *
 * This function can be used to copy data from program memory (flash) to the
 * display.
 *
 * Limits have to be set prior to calling this function, e.g.:
 * \code
	ili9341_set_top_left_limit(0, 0);
	ili9341_set_bottom_right_limit(320, 240);
	...
\endcode
 *
 * \param pixels Pointer to the progmem data
 * \param count Number of pixels to write
 */
void ili9341_copy_progmem_pixels_to_screen(
		ili9341_color_t PROGMEM_PTR_T pixels, uint32_t count)
{
	ili9341_color_t color;

	/* Sanity check to make sure that the pixel count is not zero */
	Assert(count > 0);

	ili9341_send_command(ILI9341_CMD_MEMORY_WRITE);

	while (count--) {
		color = PROGMEM_READ_WORD(pixels);

		ili9341_send_byte(color);
		ili9341_send_byte(color >> 8);

		pixels++;
	}

	ili9341_wait_for_send_done();
	ili9341_deselect_chip();
}
#endif

/**
 * \brief Set a given number of pixels to the same color
 *
 * Use this function to write a certain number of pixels to the same color
 * within a set limit.
 *
 * Limits have to be set prior to calling this function, e.g.:
 * \code
	ili9341_set_top_left_limit(0, 0);
	ili9341_set_bottom_right_limit(320, 240);
	...
\endcode
 *
 * \param color The color to write to the display
 * \param count The number of pixels to write with this color
 */
void ili9341_duplicate_pixel(const ili9341_color_t color, uint32_t count)
{
	/* Sanity check to make sure that the pixel count is not zero */
	Assert(count > 0);

	ili9341_send_command(ILI9341_CMD_MEMORY_WRITE);

#if defined(ILI9341_DMA_ENABLED)
	ili9341_color_t chunk_buf[ILI9341_DMA_CHUNK_SIZE];
	uint32_t chunk_len;

#  if SAM
	Pdc *SPI_DMA = spi_get_pdc_base(CONF_ILI9341_SPI);
	pdc_packet_t spi_pdc_data;

	pdc_enable_transfer(SPI_DMA, PERIPH_PTCR_TXTEN);
	spi_pdc_data.ul_addr = (uint32_t)chunk_buf;
#  elif UC3
	pdca_set_transfer_size(CONF_ILI9341_PDCA_CHANNEL,
			PDCA_TRANSFER_SIZE_BYTE);
	pdca_set_peripheral_select(CONF_ILI9341_PDCA_CHANNEL,
			CONF_ILI9341_PDCA_PID);
#  endif

	for (uint32_t i = 0; i < ILI9341_DMA_CHUNK_SIZE; i++) {
		chunk_buf[i] = le16_to_cpu(color);
	}

	while (count)
	{
		chunk_len = min(ILI9341_DMA_CHUNK_SIZE, count);

		ili9341_wait_for_send_done();

#  if SAM
		spi_pdc_data.ul_size = (uint32_t)sizeof(ili9341_color_t) * chunk_len;
		pdc_tx_init(SPI_DMA, NULL, &spi_pdc_data);
#  elif UC3
		pdca_reload_channel(CONF_ILI9341_PDCA_CHANNEL, chunk_buf,
				(uint32_t)sizeof(ili9341_color_t) * chunk_len);
		pdca_enable(CONF_ILI9341_PDCA_CHANNEL);
#  endif

		count -= chunk_len;
	}

	ili9341_wait_for_send_done();
	//ili9341_deselect_chip();

#  if SAM
	pdc_disable_transfer(SPI_DMA, PERIPH_PTCR_TXTEN);
#  elif UC3
	pdca_disable(CONF_ILI9341_PDCA_CHANNEL);
#  endif
#else
    uint16_t ucBCount=0;
	while (count--) {
		//ili9341_send_byte(color);
		//ili9341_send_byte(color >> 8);
		//buffer_tx[ucBCount++]=color;
		//buffer_tx[ucBCount++]=color>>8;
		buffer_tx[ucBCount++]=color;
	}
	
	transfer_tx_is_done=false;
	portDISABLE_INTERRUPTS();
    dma_start_transfer_job(&example_resource_tx);
	vTaskSuspend(NULL);
    //while (!transfer_tx_is_done) {
	//    /* Wait for transfer done */
    //}		

	//ili9341_wait_for_send_done();
	ili9341_deselect_chip();
#endif
}

/**
 * \brief Copy pixels from the screen to a pixel buffer
 *
 * Use this function to copy pixels from the display to an internal SRAM buffer.
 *
 * Limits have to be set prior to calling this function, e.g.:
 * \code
	ili9341_set_top_left_limit(0, 0);
	ili9341_set_bottom_right_limit(320, 240);
	...
\endcode
 *
 * \param pixels Pointer to the pixel buffer to read to
 * \param count Number of pixels to read
 */
void ili9341_copy_pixels_from_screen(ili9341_color_t *pixels, uint32_t count)
{
	uint8_t red, green, blue;

	/* Sanity check to make sure that the pixel count is not zero */
	Assert(count > 0);

	ili9341_send_command(ILI9341_CMD_MEMORY_READ);

	/* No interesting data in the first byte, hence read and discard */
	red = ili9341_read_byte();

	while (count--) {
		red = ili9341_read_byte();
		green = ili9341_read_byte();
		blue = ili9341_read_byte();

		*pixels = ILI9341_COLOR(red, green, blue);
		pixels++;
	}

	ili9341_deselect_chip();
}

/**
 * \internal
 * \brief Initialize the hardware interface to the controller
 *
 * This will initialize the module used for communication with the controller.
 * Currently supported interfaces by this component driver are the SPI
 * interface through either the SPI module in master mode or the USART in
 * Master SPI mode.  Configuration must be done in the associated
 * conf_ili9341.h file.
 */
static void ili9341_interface_init(void)
{
//#if defined(CONF_ILI9341_USART_SPI) || defined(CONF_ILI9341_SPI)
	//spi_flags_t spi_flags = SPI_MODE_0;
	//board_spi_select_id_t spi_select_id = 0;
//#else
	//#error Interface for ILI9341 has not been selected or interface not
	//supported, please configure component driver using the conf_ili9341.h
	//file!
//#endif
//
//#if defined(CONF_ILI9341_USART_SPI)
	//struct usart_spi_device device = {
		//.id = 0,
	//};
//
	//usart_spi_init(CONF_ILI9341_USART_SPI);
	//usart_spi_setup_device(CONF_ILI9341_USART_SPI, &device, spi_flags,
			//CONF_ILI9341_CLOCK_SPEED, spi_select_id);
//
//#elif defined(CONF_ILI9341_SPI)
	//struct spi_device device = {
		//.id = 0,
	//};
//
	//spi_master_init(CONF_ILI9341_SPI);
	//spi_master_setup_device(CONF_ILI9341_SPI, &device, spi_flags,
			//CONF_ILI9341_CLOCK_SPEED, spi_select_id);
	//spi_enable(CONF_ILI9341_SPI);
//
//#  if UC3
	//spi_set_chipselect(CONF_ILI9341_SPI, ~(1 << 0));
//
//#    if defined(ILI9341_DMA_ENABLED)
	//sysclk_enable_peripheral_clock(&AVR32_PDCA);
//#    endif
//#  endif
//
	///* Send one dummy byte for the spi_is_tx_ok() to work as expected */
	//spi_write_single(CONF_ILI9341_SPI, 0);
//#endif

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
	config_spi_master.transfer_mode = SPI_TRANSFER_MODE_0;
	config_spi_master.generator_source = GCLK_GENERATOR_0;
	spi_init(&spi_master_instance, CONF_ILI9341_SPI, &config_spi_master);
	spi_set_baudrate(&spi_master_instance,CONF_ILI9341_CLOCK_SPEED);
	//spi_enable(&spi_master_instance);
	
	
    configure_dma_resource_tx(&example_resource_tx);
    setup_transfer_descriptor_tx(&example_descriptor_tx);
    dma_add_descriptor(&example_resource_tx, &example_descriptor_tx);
    dma_register_callback(&example_resource_tx, transfer_tx_done,DMA_CALLBACK_TRANSFER_DONE);

    dma_enable_callback(&example_resource_tx, DMA_CALLBACK_TRANSFER_DONE);
	

    //dma_start_transfer_job(&example_resource_tx);
    //while (!transfer_tx_is_done) {
	    ///* Wait for transfer done */
    //}	
	
}

/**
 * \internal
 * \brief Initialize all the display registers
 *
 * This function will set up all the internal registers according the the
 * manufacturer's description.
 */
static void ili9341_controller_init_registers(void)
{
	//ili9341_send_command(ILI9341_CMD_POWER_CONTROL_A);
	//ili9341_send_byte(0x39);
	//ili9341_send_byte(0x2C);
	//ili9341_send_byte(0x00);
	//ili9341_send_byte(0x34);
	//ili9341_send_byte(0x02);
	//ili9341_wait_for_send_done();
	//ili9341_deselect_chip();
//
	//ili9341_send_command(ILI9341_CMD_POWER_CONTROL_B);
	//ili9341_send_byte(0x00);
	//ili9341_send_byte(0xAA);
	//ili9341_send_byte(0XB0);
	//ili9341_wait_for_send_done();
	//ili9341_deselect_chip();
//
	//ili9341_send_command(ILI9341_CMD_PUMP_RATIO_CONTROL);
	//ili9341_send_byte(0x30);
	//ili9341_wait_for_send_done();
	//ili9341_deselect_chip();
//
	//ili9341_send_command(ILI9341_CMD_POWER_CONTROL_1);
	//ili9341_send_byte(0x25);
	//ili9341_wait_for_send_done();
	//ili9341_deselect_chip();
//
	//ili9341_send_command(ILI9341_CMD_POWER_CONTROL_2);
	//ili9341_send_byte(0x11);
	//ili9341_wait_for_send_done();
	//ili9341_deselect_chip();
//
	//ili9341_send_command(ILI9341_CMD_VCOM_CONTROL_1);
	//ili9341_send_byte(0x5C);
	//ili9341_send_byte(0x4C);
	//ili9341_wait_for_send_done();
	//ili9341_deselect_chip();
//
	//ili9341_send_command(ILI9341_CMD_VCOM_CONTROL_2);
	//ili9341_send_byte(0x94);
	//ili9341_wait_for_send_done();
	//ili9341_deselect_chip();
//
	//ili9341_send_command(ILI9341_CMD_DRIVER_TIMING_CONTROL_A);
	//ili9341_send_byte(0x85);
	//ili9341_send_byte(0x01);
	//ili9341_send_byte(0x78);
	//ili9341_wait_for_send_done();
	//ili9341_deselect_chip();
//
	//ili9341_send_command(ILI9341_CMD_DRIVER_TIMING_CONTROL_B);
	//ili9341_send_byte(0x00);
	//ili9341_send_byte(0x00);
	//ili9341_wait_for_send_done();
	//ili9341_deselect_chip();
//
	//ili9341_send_command(ILI9341_CMD_COLMOD_PIXEL_FORMAT_SET);
	//ili9341_send_byte(0x05);
	//ili9341_wait_for_send_done();
	//ili9341_deselect_chip();
//
	//ili9341_set_orientation(0);
	//ili9341_set_limits(0, 0, ILI9341_DEFAULT_WIDTH,
			//ILI9341_DEFAULT_HEIGHT);
			

//if (hwSPI) spi_begin();
ili9341_send_command(0xEF);
ili9341_send_byte(0x03);
ili9341_send_byte(0x80);
ili9341_send_byte(0x02);

ili9341_send_command(0xCF);
ili9341_send_byte(0x00);
ili9341_send_byte(0XC1);
ili9341_send_byte(0X30);

ili9341_send_command(0xED);
ili9341_send_byte(0x64);
ili9341_send_byte(0x03);
ili9341_send_byte(0X12);
ili9341_send_byte(0X81);

ili9341_send_command(0xE8);
ili9341_send_byte(0x85);
ili9341_send_byte(0x00);
ili9341_send_byte(0x78);

ili9341_send_command(0xCB);
ili9341_send_byte(0x39);
ili9341_send_byte(0x2C);
ili9341_send_byte(0x00);
ili9341_send_byte(0x34);
ili9341_send_byte(0x02);

ili9341_send_command(0xF7);
ili9341_send_byte(0x20);

ili9341_send_command(0xEA);
ili9341_send_byte(0x00);
ili9341_send_byte(0x00);

ili9341_send_command(ILI9341_PWCTR1);    //Power control
ili9341_send_byte(0x23);   //VRH[5:0]

ili9341_send_command(ILI9341_PWCTR2);    //Power control
ili9341_send_byte(0x10);   //SAP[2:0];BT[3:0]

ili9341_send_command(ILI9341_VMCTR1);    //VCM control
ili9341_send_byte(0x3e); //对比度调节
ili9341_send_byte(0x28);

ili9341_send_command(ILI9341_VMCTR2);    //VCM control2
ili9341_send_byte(0x86);  //--

ili9341_send_command(ILI9341_MADCTL);    // Memory Access Control
ili9341_send_byte(0x48);

ili9341_send_command(ILI9341_PIXFMT);
ili9341_send_byte(0x55);

ili9341_send_command(ILI9341_FRMCTR1);
ili9341_send_byte(0x00);
ili9341_send_byte(0x18);

ili9341_send_command(ILI9341_DFUNCTR);    // Display Function Control
ili9341_send_byte(0x08);
ili9341_send_byte(0x82);
ili9341_send_byte(0x27);

ili9341_send_command(0xF2);    // 3Gamma Function Disable
ili9341_send_byte(0x00);

ili9341_send_command(ILI9341_GAMMASET);    //Gamma curve selected
ili9341_send_byte(0x01);

ili9341_send_command(ILI9341_GMCTRP1);    //Set Gamma
ili9341_send_byte(0x0F);
ili9341_send_byte(0x31);
ili9341_send_byte(0x2B);
ili9341_send_byte(0x0C);
ili9341_send_byte(0x0E);
ili9341_send_byte(0x08);
ili9341_send_byte(0x4E);
ili9341_send_byte(0xF1);
ili9341_send_byte(0x37);
ili9341_send_byte(0x07);
ili9341_send_byte(0x10);
ili9341_send_byte(0x03);
ili9341_send_byte(0x0E);
ili9341_send_byte(0x09);
ili9341_send_byte(0x00);

ili9341_send_command(ILI9341_GMCTRN1);    //Set Gamma
ili9341_send_byte(0x00);
ili9341_send_byte(0x0E);
ili9341_send_byte(0x14);
ili9341_send_byte(0x03);
ili9341_send_byte(0x11);
ili9341_send_byte(0x07);
ili9341_send_byte(0x31);
ili9341_send_byte(0xC1);
ili9341_send_byte(0x48);
ili9341_send_byte(0x08);
ili9341_send_byte(0x0F);
ili9341_send_byte(0x0C);
ili9341_send_byte(0x31);
ili9341_send_byte(0x36);
ili9341_send_byte(0x0F);

ili9341_send_command(ILI9341_SLPOUT);    //Exit Sleep
//if (hwSPI) spi_end();
//delay(120);
//if (hwSPI) spi_begin();
ili9341_send_command(ILI9341_DISPON);    //Display on
//if (hwSPI) spi_end();			

			
}

/**
 * \internal
 * \brief Send display commands to exit standby mode
 *
 * This function is used to exit the display standby mode, which is the default
 * mode after a reset signal to the display.
 */
static void ili9341_exit_standby(void)
{
	ili9341_send_command(ILI9341_CMD_SLEEP_OUT);
	ili9341_deselect_chip();
	delay_ms(150);
	ili9341_send_command(ILI9341_CMD_DISPLAY_ON);
	//ili9341_send_command(ILI9341_CMD_DISPLAY_OFF);
    //ili9341_send_command(ILI9341_CMD_DISPLAY_ON);
	//ili9341_send_command(ILI9341_CMD_DISPLAY_OFF);
	ili9341_deselect_chip();
}

/**
 * \internal
 * \brief Reset the display using the digital control interface
 *
 * Controls the reset pin of the display controller to reset the display.
 */
static void ili9341_reset_display(void)
{
	port_pin_set_output_level(CONF_ILI9341_RESET_PIN, true);
	delay_ms(10);
	port_pin_set_output_level(CONF_ILI9341_RESET_PIN, false);
	delay_ms(10);
	port_pin_set_output_level(CONF_ILI9341_RESET_PIN, true);
	delay_ms(150);
}

/**
 * \brief Initialize the controller
 *
 * Used to initialize the ILI9341 display controller by setting up the hardware
 * interface, and setting up the controller according to the manufacturer's
 * description. It also set up the screen orientation to the default state
 * (portrait).
 */
void ili9341_init(TaskHandle_t xTaskH)
{
	/* Initialize the communication interface */
	
	xcbTaskH=xTaskH;
	
	ili9341_interface_init();

	/* Reset the display */
	ili9341_reset_display();

	/* Send commands to exit standby mode */
	ili9341_exit_standby();

	/* Write all the controller registers with correct values */
	ili9341_controller_init_registers();
}

/**
 * \brief Sets the orientation of the display data
 *
 * Configures the display for a given orientation, including mirroring and/or
 * screen rotation.
 *
 * \param flags Orientation flags to use, see \ref ILI9341_FLIP_X, \ref ILI9341_FLIP_Y
 *        and \ref ILI9341_SWITCH_XY.
 */
void ili9341_set_orientation(uint8_t flags)
{
	uint8_t madctl = 0x48;

	/* Pretend the display is in landscape mode by default to match other display drivers */
	flags ^= ILI9341_SWITCH_XY | ILI9341_FLIP_X;

	if (flags & ILI9341_FLIP_X) {
		madctl &= ~(1 << 6);
	}

	if (flags & ILI9341_FLIP_Y) {
		madctl |= 1 << 7;
	}

	if (flags & ILI9341_SWITCH_XY) {
		madctl |= 1 << 5;
	}

	ili9341_send_command(ILI9341_CMD_MEMORY_ACCESS_CONTROL);
	ili9341_send_byte(madctl);
	ili9341_wait_for_send_done();
	ili9341_deselect_chip();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void writecommand(uint8_t c) {
	
	PORT->Group[0].OUTCLR.reg=(1<<CONF_ILI9341_DC_PIN);
	//CLEAR_BIT(dcport, dcpinmask);
	//digitalWrite(_dc, LOW);
	
	//CLEAR_BIT(clkport, clkpinmask);
	//digitalWrite(_sclk, LOW);
	
	PORT->Group[0].OUTCLR.reg=(1<<CONF_ILI9341_CS_PIN);
	//CLEAR_BIT(csport, cspinmask);
	//digitalWrite(_cs, LOW);

    spi_master_instance.hw->SPI.DATA.bit.DATA = c; // & SERCOM_SPI_DATA_MASK;
	while (!spi_master_instance.hw->SPI.INTFLAG.bit.DRE);
	//spiwrite(c);

	PORT->Group[0].OUTSET.reg=(1<<CONF_ILI9341_CS_PIN);
	//SET_BIT(csport, cspinmask);
	//digitalWrite(_cs, HIGH);
}


void writedata(uint8_t c) {
	
	PORT->Group[0].OUTSET.reg=(1<<CONF_ILI9341_DC_PIN);
	//SET_BIT(dcport,  dcpinmask);
	//digitalWrite(_dc, HIGH);
	
	//CLEAR_BIT(clkport, clkpinmask);
	//digitalWrite(_sclk, LOW);
	
	PORT->Group[0].OUTCLR.reg=(1<<CONF_ILI9341_CS_PIN);
	//CLEAR_BIT(csport, cspinmask);
	//digitalWrite(_cs, LOW);

    spi_master_instance.hw->SPI.DATA.bit.DATA = c; 
	while (!spi_master_instance.hw->SPI.INTFLAG.bit.DRE);	
	//spiwrite(c);

	PORT->Group[0].OUTSET.reg=(1<<CONF_ILI9341_CS_PIN);
	//digitalWrite(_cs, HIGH);
	//SET_BIT(csport, cspinmask);
}


void Adafruit_ILI9340_setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
uint16_t y1) {

	writecommand(ILI9340_CASET); // Column addr set
	writedata(x0 >> 8);
	writedata(x0 & 0xFF);     // XSTART
	writedata(x1 >> 8);
	writedata(x1 & 0xFF);     // XEND

	writecommand(ILI9340_PASET); // Row addr set
	writedata(y0>>8);
	writedata(y0);     // YSTART
	writedata(y1>>8);
	writedata(y1);     // YEND

	writecommand(ILI9340_RAMWR); // write to RAM
}


void Adafruit_ILI9340_pushColor(uint16_t color) {
	
	PORT->Group[0].OUTSET.reg=(1<<CONF_ILI9341_DC_PIN);
	//digitalWrite(_dc, HIGH);
	//SET_BIT(dcport, dcpinmask);
	
	PORT->Group[0].OUTCLR.reg=(1<<CONF_ILI9341_CS_PIN);
	//digitalWrite(_cs, LOW);
	//CLEAR_BIT(csport, cspinmask);

    spi_master_instance.hw->SPI.DATA.bit.DATA = color >> 8;
	while (!spi_master_instance.hw->SPI.INTFLAG.bit.DRE);
	spi_master_instance.hw->SPI.DATA.bit.DATA = color;
    while (!spi_master_instance.hw->SPI.INTFLAG.bit.DRE);
	//spiwrite(color >> 8);
	//spiwrite(color);

	PORT->Group[0].OUTSET.reg=(1<<CONF_ILI9341_CS_PIN);
	//SET_BIT(csport, cspinmask);
	//digitalWrite(_cs, HIGH);
}

void Adafruit_ILI9340_drawPixel(int16_t x, int16_t y, uint16_t color) {

	if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

	Adafruit_ILI9340_setAddrWindow(x,y,x+1,y+1);

    PORT->Group[0].OUTSET.reg=(1<<CONF_ILI9341_DC_PIN);
	//digitalWrite(_dc, HIGH);
	//SET_BIT(dcport, dcpinmask);
	
	PORT->Group[0].OUTCLR.reg=(1<<CONF_ILI9341_CS_PIN);
	//digitalWrite(_cs, LOW);
	//CLEAR_BIT(csport, cspinmask);

    spi_master_instance.hw->SPI.DATA.bit.DATA = color;
    while (!spi_master_instance.hw->SPI.INTFLAG.bit.DRE);
    spi_master_instance.hw->SPI.DATA.bit.DATA = color >> 8;
	while (!spi_master_instance.hw->SPI.INTFLAG.bit.DRE);
	//spiwrite(color >> 8);
	//spiwrite(color);

    PORT->Group[0].OUTSET.reg=(1<<CONF_ILI9341_CS_PIN);
	//SET_BIT(csport, cspinmask);
	//digitalWrite(_cs, HIGH);
}


void Adafruit_ILI9340_drawFastVLine(int16_t x, int16_t y, int16_t h,
uint16_t color) {

	// Rudimentary clipping
	if((x >= _width) || (y >= _height)) return;

	if((y+h-1) >= _height)
	h = _height-y;

	Adafruit_ILI9340_setAddrWindow(x, y, x, y+h-1);

	uint8_t hi = color >> 8, lo = color;

    PORT->Group[0].OUTSET.reg=(1<<CONF_ILI9341_DC_PIN);
	//SET_BIT(dcport, dcpinmask);
	//digitalWrite(_dc, HIGH);
	PORT->Group[0].OUTCLR.reg=(1<<CONF_ILI9341_CS_PIN);
	//CLEAR_BIT(csport, cspinmask);
	//digitalWrite(_cs, LOW);

	while (h--) {
          spi_master_instance.hw->SPI.DATA.bit.DATA = hi;
          while (!spi_master_instance.hw->SPI.INTFLAG.bit.DRE);
          spi_master_instance.hw->SPI.DATA.bit.DATA = lo;
		  while (!spi_master_instance.hw->SPI.INTFLAG.bit.DRE);		
		//spiwrite(hi);
		//spiwrite(lo);
	}
	
	PORT->Group[0].OUTSET.reg=(1<<CONF_ILI9341_CS_PIN);
	//SET_BIT(csport, cspinmask);
	//digitalWrite(_cs, HIGH);
}


void Adafruit_ILI9340_drawFastHLine(int16_t x, int16_t y, int16_t w,
uint16_t color) {

	// Rudimentary clipping
	if((x >= _width) || (y >= _height)) return;
	if((x+w-1) >= _width)  w = _width-x;
	Adafruit_ILI9340_setAddrWindow(x, y, x+w-1, y);

	uint8_t hi = color >> 8, lo = color;
	PORT->Group[0].OUTSET.reg=(1<<CONF_ILI9341_DC_PIN);
	//SET_BIT(dcport, dcpinmask);
	PORT->Group[0].OUTCLR.reg=(1<<CONF_ILI9341_CS_PIN);
	//CLEAR_BIT(csport, cspinmask);
	//digitalWrite(_dc, HIGH);
	//digitalWrite(_cs, LOW);
	while (w--) {
          spi_master_instance.hw->SPI.DATA.bit.DATA = hi;
          while (!spi_master_instance.hw->SPI.INTFLAG.bit.DRE);
          spi_master_instance.hw->SPI.DATA.bit.DATA = lo;	
		  while (!spi_master_instance.hw->SPI.INTFLAG.bit.DRE);	
		//spiwrite(hi);
		//spiwrite(lo);
	}
	PORT->Group[0].OUTSET.reg=(1<<CONF_ILI9341_CS_PIN);	
	//SET_BIT(csport, cspinmask);
	//digitalWrite(_cs, HIGH);
}

void Adafruit_ILI9340_fillScreen(uint16_t color) {
	Adafruit_ILI9340_fillRect(0, 0,  _width, _height, color);
}

// fill a rectangle
void Adafruit_ILI9340_fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
uint16_t color) {

	// rudimentary clipping (drawChar w/big text requires this)
	if((x >= _width) || (y >= _height)) return;
	if((x + w - 1) >= _width)  w = _width  - x;
	if((y + h - 1) >= _height) h = _height - y;

	Adafruit_ILI9340_setAddrWindow(x, y, x+w-1, y+h-1);
	
	uint8_t hi = color >> 8, lo = color;

	PORT->Group[0].OUTSET.reg=(1<<CONF_ILI9341_DC_PIN);
	//SET_BIT(dcport, dcpinmask);
	//digitalWrite(_dc, HIGH);
    PORT->Group[0].OUTCLR.reg=(1<<CONF_ILI9341_CS_PIN);
	//CLEAR_BIT(csport, cspinmask);
	//digitalWrite(_cs, LOW);

	for(y=h; y>0; y--) {
		for(x=w; x>0; x--) {
          spi_master_instance.hw->SPI.DATA.bit.DATA = hi;
          while (!spi_master_instance.hw->SPI.INTFLAG.bit.DRE);
          spi_master_instance.hw->SPI.DATA.bit.DATA = lo;
		  while (!spi_master_instance.hw->SPI.INTFLAG.bit.DRE);			
	      //spiwrite(hi);
		  //spiwrite(lo);
		}
	}
	//digitalWrite(_cs, HIGH);
	PORT->Group[0].OUTSET.reg=(1<<CONF_ILI9341_CS_PIN);	
	//SET_BIT(csport, cspinmask);
}

//Circle
void Adafruit_GFX_drawCircle(int16_t x0, int16_t y0, int16_t r,
 uint16_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  Adafruit_ILI9340_drawPixel(x0  , y0+r, color);
  Adafruit_ILI9340_drawPixel(x0  , y0-r, color);
  Adafruit_ILI9340_drawPixel(x0+r, y0  , color);
  Adafruit_ILI9340_drawPixel(x0-r, y0  , color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    Adafruit_ILI9340_drawPixel(x0 + x, y0 + y, color);
    Adafruit_ILI9340_drawPixel(x0 - x, y0 + y, color);
    Adafruit_ILI9340_drawPixel(x0 + x, y0 - y, color);
    Adafruit_ILI9340_drawPixel(x0 - x, y0 - y, color);
    Adafruit_ILI9340_drawPixel(x0 + y, y0 + x, color);
    Adafruit_ILI9340_drawPixel(x0 - y, y0 + x, color);
    Adafruit_ILI9340_drawPixel(x0 + y, y0 - x, color);
    Adafruit_ILI9340_drawPixel(x0 - y, y0 - x, color);
  }
}

void Adafruit_GFX_drawCircleHelper( int16_t x0, int16_t y0,
 int16_t r, uint8_t cornername, uint16_t color) {
  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (cornername & 0x4) {
      Adafruit_ILI9340_drawPixel(x0 + x, y0 + y, color);
      Adafruit_ILI9340_drawPixel(x0 + y, y0 + x, color);
    }
    if (cornername & 0x2) {
      Adafruit_ILI9340_drawPixel(x0 + x, y0 - y, color);
      Adafruit_ILI9340_drawPixel(x0 + y, y0 - x, color);
    }
    if (cornername & 0x8) {
      Adafruit_ILI9340_drawPixel(x0 - y, y0 + x, color);
      Adafruit_ILI9340_drawPixel(x0 - x, y0 + y, color);
    }
    if (cornername & 0x1) {
      Adafruit_ILI9340_drawPixel(x0 - y, y0 - x, color);
      Adafruit_ILI9340_drawPixel(x0 - x, y0 - y, color);
    }
  }
}

void Adafruit_GFX_fillCircle(int16_t x0, int16_t y0, int16_t r,
 uint16_t color) {
  Adafruit_GFX_drawFastVLine(x0, y0-r, 2*r+1, color);
  Adafruit_GFX_fillCircleHelper(x0, y0, r, 3, 0, color);
}

// Used to do circles and roundrects
void Adafruit_GFX_fillCircleHelper(int16_t x0, int16_t y0, int16_t r,
 uint8_t cornername, int16_t delta, uint16_t color) {

  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;

    if (cornername & 0x1) {
      Adafruit_GFX_drawFastVLine(x0+x, y0-y, 2*y+1+delta, color);
      Adafruit_GFX_drawFastVLine(x0+y, y0-x, 2*x+1+delta, color);
    }
    if (cornername & 0x2) {
      Adafruit_GFX_drawFastVLine(x0-x, y0-y, 2*y+1+delta, color);
      Adafruit_GFX_drawFastVLine(x0-y, y0-x, 2*x+1+delta, color);
    }
  }
}

// Bresenham's algorithm - thx wikpedia
void Adafruit_GFX_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
 uint16_t color) {
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    _swap_int16_t(x0, y0);
    _swap_int16_t(x1, y1);
  }

  if (x0 > x1) {
    _swap_int16_t(x0, x1);
    _swap_int16_t(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++) {
    if (steep) {
      Adafruit_ILI9340_drawPixel(y0, x0, color);
    } else {
      Adafruit_ILI9340_drawPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

// Draw a rectangle
void Adafruit_GFX_drawRect(int16_t x, int16_t y, int16_t w, int16_t h,
 uint16_t color) {
  Adafruit_GFX_drawFastHLine(x, y, w, color);
  Adafruit_GFX_drawFastHLine(x, y+h-1, w, color);
  Adafruit_GFX_drawFastVLine(x, y, h, color);
  Adafruit_GFX_drawFastVLine(x+w-1, y, h, color);
}

void Adafruit_GFX_drawFastVLine(int16_t x, int16_t y,
 int16_t h, uint16_t color) {
  // Update in subclasses if desired!
  Adafruit_GFX_drawLine(x, y, x, y+h-1, color);
}

void Adafruit_GFX_drawFastHLine(int16_t x, int16_t y,
 int16_t w, uint16_t color) {
  // Update in subclasses if desired!
  Adafruit_GFX_drawLine(x, y, x+w-1, y, color);
}

void Adafruit_GFX_fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
 uint16_t color) {
  // Update in subclasses if desired!
  for (int16_t i=x; i<x+w; i++) {
    Adafruit_GFX_drawFastVLine(i, y, h, color);
  }
}

void Adafruit_GFX_fillScreen(uint16_t color) {
  Adafruit_GFX_fillRect(0, 0, _width, _height, color);
}

size_t Adafruit_GFX_write(uint8_t c) {

		if(!gfxFont) { // 'Classic' built-in font

			if(c == '\n') {
				cursor_y += textsize*8;
				cursor_x  = 0;
				} else if(c == '\r') {
				// skip em
				} else {
				if(wrap && ((cursor_x + textsize * 6) >= _width)) { // Heading off edge?
					cursor_x  = 0;            // Reset x to zero
					cursor_y += textsize * 8; // Advance y one line
				}
				Adafruit_GFX_drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
				cursor_x += textsize * 6;
			}

			} else { // Custom font

			//if(c == '\n') {
				//cursor_x  = 0;
				//cursor_y += (int16_t)textsize *
				//(uint8_t)pgm_read_byte(&gfxFont->yAdvance);
				//} else if(c != '\r') {
				//uint8_t first = pgm_read_byte(&gfxFont->first);
				//if((c >= first) && (c <= (uint8_t)pgm_read_byte(&gfxFont->last))) {
					//uint8_t   c2    = c - pgm_read_byte(&gfxFont->first);
					//GFXglyph *glyph = &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c2]);
					//uint8_t   w     = pgm_read_byte(&glyph->width),
					//h     = pgm_read_byte(&glyph->height);
					//if((w > 0) && (h > 0)) { // Is there an associated bitmap?
						//int16_t xo = (int8_t)pgm_read_byte(&glyph->xOffset); // sic
						//if(wrap && ((cursor_x + textsize * (xo + w)) >= _width)) {
							//// Drawing character would go off right edge; wrap to new line
							//cursor_x  = 0;
							//cursor_y += (int16_t)textsize *
							//(uint8_t)pgm_read_byte(&gfxFont->yAdvance);
						//}
						//drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
					//}
					//cursor_x += pgm_read_byte(&glyph->xAdvance) * (int16_t)textsize;
				//}
			//}

		}
		return 1;
	}
	
// Draw a character
void Adafruit_GFX_drawChar(int16_t x, int16_t y, unsigned char c,
uint16_t color, uint16_t bg, uint8_t size) {

	if(!gfxFont) { // 'Classic' built-in font

		if((x >= _width)            || // Clip right
		(y >= _height)           || // Clip bottom
		((x + 6 * size - 1) < 0) || // Clip left
		((y + 8 * size - 1) < 0))   // Clip top
		return;

		if(!_cp437 && (c >= 176)) c++; // Handle 'classic' charset behavior

		for(int8_t i=0; i<6; i++ ) {
			uint8_t line;
			if(i < 5) line = font[(c*5)+i];
			else      line = 0x0;
			for(int8_t j=0; j<8; j++, line >>= 1) {
				if(line & 0x1) {
					if(size == 1) Adafruit_ILI9340_drawPixel(x+i, y+j, color);
					else          Adafruit_GFX_fillRect(x+(i*size), y+(j*size), size, size, color);
					} else if(bg != color) {
					if(size == 1) Adafruit_ILI9340_drawPixel(x+i, y+j, bg);
					else          Adafruit_GFX_fillRect(x+i*size, y+j*size, size, size, bg);
				}
			}
		}

		} else { // Custom font

		//// Character is assumed previously filtered by write() to eliminate
		//// newlines, returns, non-printable characters, etc.  Calling drawChar()
		//// directly with 'bad' characters of font may cause mayhem!
//
		//c -= pgm_read_byte(&gfxFont->first);
		//GFXglyph *glyph  = &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c]);
		//uint8_t  *bitmap = (uint8_t *)pgm_read_pointer(&gfxFont->bitmap);
//
		//uint16_t bo = pgm_read_word(&glyph->bitmapOffset);
		//uint8_t  w  = pgm_read_byte(&glyph->width),
		//h  = pgm_read_byte(&glyph->height),
		//xa = pgm_read_byte(&glyph->xAdvance);
		//int8_t   xo = pgm_read_byte(&glyph->xOffset),
		//yo = pgm_read_byte(&glyph->yOffset);
		//uint8_t  xx, yy, bits, bit = 0;
		//int16_t  xo16, yo16;
//
		//if(size > 1) {
			//xo16 = xo;
			//yo16 = yo;
		//}
//
		//// Todo: Add character clipping here
//
		//// NOTE: THERE IS NO 'BACKGROUND' COLOR OPTION ON CUSTOM FONTS.
		//// THIS IS ON PURPOSE AND BY DESIGN.  The background color feature
		//// has typically been used with the 'classic' font to overwrite old
		//// screen contents with new data.  This ONLY works because the
		//// characters are a uniform size; it's not a sensible thing to do with
		//// proportionally-spaced fonts with glyphs of varying sizes (and that
		//// may overlap).  To replace previously-drawn text when using a custom
		//// font, use the getTextBounds() function to determine the smallest
		//// rectangle encompassing a string, erase the area with fillRect(),
		//// then draw new text.  This WILL infortunately 'blink' the text, but
		//// is unavoidable.  Drawing 'background' pixels will NOT fix this,
		//// only creates a new set of problems.  Have an idea to work around
		//// this (a canvas object type for MCUs that can afford the RAM and
		//// displays supporting setAddrWindow() and pushColors()), but haven't
		//// implemented this yet.
//
		//for(yy=0; yy<h; yy++) {
			//for(xx=0; xx<w; xx++) {
				//if(!(bit++ & 7)) {
					//bits = pgm_read_byte(&bitmap[bo++]);
				//}
				//if(bits & 0x80) {
					//if(size == 1) {
						//drawPixel(x+xo+xx, y+yo+yy, color);
						//} else {
						//fillRect(x+(xo16+xx)*size, y+(yo16+yy)*size, size, size, color);
					//}
				//}
				//bits <<= 1;
			//}
		//}

	} // End classic vs custom font
}

void Adafruit_GFX_setCursor(int16_t x, int16_t y) {
	cursor_x = x;
	cursor_y = y;
}

int16_t Adafruit_GFX_getCursorX(void) {
	return cursor_x;
}

int16_t Adafruit_GFX_getCursorY(void) {
	return cursor_y;
}

void Adafruit_GFX_setTextSize(uint8_t s) {
	textsize = (s > 0) ? s : 1;
}

//void Adafruit_GFX_setTextColor(uint16_t c) {
///	// For 'transparent' background, we'll set the bg
//	// to the same as fg instead of using a flag
//	textcolor = textbgcolor = c;
//}

void Adafruit_GFX_setTextColor(uint16_t c, uint16_t b) {
	textcolor   = c;
	textbgcolor = b;
}

void Adafruit_GFX_setTextWrap(bool w) {
	wrap = w;
}

uint8_t Adafruit_GFX_getRotation(void) {
	return rotation;
}

void Adafruit_GFX_setRotation(uint8_t x) {
	rotation = (x & 3);
	switch(rotation) {
		case 0:
		case 2:
		_width  = WIDTH;
		_height = HEIGHT;
		break;
		case 1:
		case 3:
		_width  = HEIGHT;
		_height = WIDTH;
		break;
	}
}

// Enable (or disable) Code Page 437-compatible charset.
// There was an error in glcdfont.c for the longest time -- one character
// (#176, the 'light shade' block) was missing -- this threw off the index
// of every character that followed it.  But a TON of code has been written
// with the erroneous character indices.  By default, the library uses the
// original 'wrong' behavior and old sketches will still work.  Pass 'true'
// to this function to use correct CP437 character values in your code.
void Adafruit_GFX_cp437(bool x) {
	_cp437 = x;
}

void Adafruit_GFX_setFont(const GFXfont *f) {
	if(f) {          // Font struct pointer passed in?
		if(!gfxFont) { // And no current font struct?
			// Switching from classic to new font behavior.
			// Move cursor pos down 6 pixels so it's on baseline.
			cursor_y += 6;
		}
		} else if(gfxFont) { // NULL passed.  Current font struct defined?
		// Switching from new to classic font behavior.
		// Move cursor pos up 6 pixels so it's at top-left of char.
		cursor_y -= 6;
	}
	gfxFont = (GFXfont *)f;
}



