/*
 * RC522.c
 *
 * Created: 9/20/2016 13:24:02
 *  Author: ROBERTOC
 */ 

#include "asf.h"
#include "RC522.h"
#include "RTC.h"
#include "taskslist.h"

/////////////////////////////////////////////////////////////////////////

#define RC522_SCK                  PIN_PB23
#define RC522_MISO                 PIN_PB16
#define RC522_MOSI                 PIN_PB22
#define RC522_SS                   PIN_PB17
//#define RC522_IRQ                  PIN_PA28
#define RC522_RST				   PIN_PA08

#define RDM6300_RX				   PIN_PB11
#define RDM6300_INT				   PIN_PB11

//void PCD_ClearRegisterBitMask( uint8_t , uint8_t );	
//StatusCode PCD_TransceiveData( uint8_t * ,	uint8_t , uint8_t * , uint8_t * , uint8_t * , uint8_t ,	bool );
//StatusCode PICC_Select(	Uid *,uint8_t );
//StatusCode PICC_RequestA( uint8_t *,	uint8_t * ); 
//void PCD_WriteRegisterN( uint8_t , uint8_t , uint8_t * );

#define RFID_TASK_PRIORITY     (tskIDLE_PRIORITY + 1)

static void RFID_task(void *);

/////////////////////////////////////////////////////////////////////////

uint8_t ucBufferPointer;
uint16_t usRxBuffer;

void usart_received_callback(struct usart_module *const );

///////////////////////////////////////////////////////////////////////////////////////////////////////

static uint8_t ucGetBytes(uint8_t ucStr[])
{
 uint8_t ucTemp=0x00;
 
 if (ucStr[0]<=0x39) ucTemp=ucStr[0]-0x30;
 else ucTemp=ucStr[0]-0x37;
 
 if (ucStr[1]<=0x39) ucTemp|=(ucStr[1]-0x30)<<4;
 else ucTemp|=(ucStr[1]-0x37)<<4;
 
 return ucTemp;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////// 

void usart_received_callback(struct usart_module *const usart_module)
{
	
	if (!xStart)
	{
	 if (usRxBuffer==0x02) {xStart=true;ucBufferPointer=0x00;}
	}
	else
	{
	 if (usRxBuffer==0x03)
	 {
	  xStart=false;
	  uint8_t ucChecksum=0x00;
	  for (uint8_t ucIndex=0x00;ucIndex<0x05;ucIndex++)
	  {
	   uint8_t ucTemp=ucGetBytes(&rdm_buffer[ucIndex<<1]);
	   ucChecksum ^= ucTemp;
	   rdm_buffer[ucIndex]=ucTemp;
	  }
	  if (ucChecksum==ucGetBytes(&rdm_buffer[0x0A])) 
	  {
	   xTagRECV=true;
	   usart_disable_callback(&usart_instance,USART_CALLBACK_BUFFER_RECEIVED); 
	   xTaskAbortDelay(xRFIDTask);
	  }
	 }
	 else if (ucBufferPointer<RDM_BUFFER) rdm_buffer[ucBufferPointer++]=usRxBuffer;
	}
	
	usart_read_job(usart_module,&usRxBuffer);
		
}

///////////////////////////////////////////////////////////////////////

void vInitRDM6300(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate = 9600;
	config_usart.data_order  = USART_DATAORDER_LSB;
	config_usart.parity = USART_PARITY_NONE;
	config_usart.stopbits = USART_STOPBITS_1;
	config_usart.mux_setting = USART_RX_3_TX_0_XCK_1;
	config_usart.pinmux_pad0 = PINMUX_UNUSED;
	config_usart.pinmux_pad1 = PINMUX_UNUSED;
	config_usart.pinmux_pad2 = PINMUX_UNUSED;
	config_usart.pinmux_pad3 = PINMUX_PB11D_SERCOM4_PAD3;
	while (usart_init(&usart_instance,
	SERCOM4, &config_usart) != STATUS_OK) {};
    usart_enable(&usart_instance);
	
	usart_register_callback(&usart_instance,usart_received_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_disable_callback(&usart_instance,USART_CALLBACK_BUFFER_RECEIVED);
	
	//ucBufferPointer=0x00;
	//xStart=false;xTagRECV=false;
    //usart_read_job(&usart_instance,&usRxBuffer);
		
}

///////////////////////////////////////////////////////////////////////

void vCreateRFIDTask(void)
{
 xTaskCreate(RFID_task,(const char *)"RFID",configMINIMAL_STACK_SIZE,NULL, RFID_TASK_PRIORITY,&xRFIDTask);
}

///////////////////////////////////////////////////////////////////////

static void RFID_task(void *params)
{
 
  vTaskSuspend(NULL);
  
  for (;;)	
  {
   xStart=false;ucBufferPointer=0x00;xTagRECV=false;
   usart_enable_callback(&usart_instance,USART_CALLBACK_BUFFER_RECEIVED);  
   usart_read_job(&usart_instance,&usRxBuffer);  
   vTaskDelay(TASK_DELAY_MS(1000));
   if (xTagRECV) 
   {
	xValidTAG=false;
	for (uint8_t ucIndex=0x00;ucIndex<MAX_TAGS;ucIndex++)
	{
	 if (!memcmp(&xTags[ucIndex].ucTag,rdm_buffer,5))
	 {
	  xValidTAG=true;
	  break;	 
	 }
	}
	vTaskDelay(TASK_DELAY_MS(1000));
   }
   else xValidTAG=false;
  }
		
}

///////////////////////////////////////////////////////////////////////

void vInitRC522SPI(void)
{
	struct spi_config config_spi_master;
	struct port_config pin_conf;
	
	port_get_config_defaults(&pin_conf);
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(RC522_SS, &pin_conf);
	port_pin_set_output_level(RC522_SS,true);
	port_pin_set_config(RC522_RST, &pin_conf);
	port_pin_set_output_level(RC522_RST,true);
	port_pin_set_config(RTC_CS, &pin_conf);
	port_pin_set_output_level(RTC_CS,true);
		
	spi_get_config_defaults(&config_spi_master);
	config_spi_master.mux_setting = SPI_SIGNAL_MUX_SETTING_E;

	/* Configure pad 0 for MOSI */
	config_spi_master.pinmux_pad0 = PINMUX_PB16C_SERCOM5_PAD0;
	/* Configure pad 1 as unused */
	config_spi_master.pinmux_pad1 = PINMUX_UNUSED;
	/* Configure pad 2 for MISO*/
	config_spi_master.pinmux_pad2 = PINMUX_PB22D_SERCOM5_PAD2;
	/* Configure pad 3 for SCK */
	config_spi_master.pinmux_pad3 = PINMUX_PB23D_SERCOM5_PAD3;
	config_spi_master.transfer_mode = SPI_TRANSFER_MODE_0;
	config_spi_master.generator_source = GCLK_GENERATOR_0;
	//config_spi_master.data_order = SPI_DATA_ORDER_MSB;
	spi_init(&service_spi_master_instance,SERCOM5, &config_spi_master);
	spi_set_baudrate(&service_spi_master_instance,1000000);
	spi_enable(&service_spi_master_instance);
	
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
static void PCD_WriteRegister(	uint8_t reg,		///< The register to write to. One of the PCD_Register enums.
						uint8_t value		///< The value to write.
					   ) {
	uint16_t LocalValue;				   
	//SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0));	// Set the settings to work with SPI bus
	//digitalWrite(_chipSelectPin, LOW);		// Select slave
	port_pin_set_output_level(RC522_SS,false);
	//SPI.transfer(reg & 0x7E);				// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	spi_transceive_wait(&service_spi_master_instance,reg & 0x7E,&LocalValue);
    //SPI.transfer(value);
    spi_transceive_wait(&service_spi_master_instance,value,&LocalValue);
	//digitalWrite(_chipSelectPin, HIGH);		// Release slave again
	port_pin_set_output_level(RC522_SS,true);
	//SPI.endTransaction(); // Stop using the SPI bus
} // End PCD_WriteRegister()

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Writes a number of bytes to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
static void PCD_WriteRegisterN(	uint8_t reg,		///< The register to write to. One of the PCD_Register enums.
									uint8_t count,		///< The number of bytes to write to the register
									uint8_t *values	///< The values to write. Byte array.
								) {
	uint16_t LocalValue;							
	//SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0));	// Set the settings to work with SPI bus
	//digitalWrite(_chipSelectPin, LOW);		// Select slave
	port_pin_set_output_level(RC522_SS,false);
	//SPI.transfer(reg & 0x7E);	
	spi_transceive_wait(&service_spi_master_instance,reg & 0x7E,&LocalValue);
	// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	for (uint8_t index = 0; index < count; index++) {
		//SPI.transfer(values[index]);
		spi_transceive_wait(&service_spi_master_instance,values[index],&LocalValue);
	}
	//digitalWrite(_chipSelectPin, HIGH);		// Release slave again
	port_pin_set_output_level(RC522_SS,true);
	//SPI.endTransaction(); // Stop using the SPI bus
} // End PCD_WriteRegister()

/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Reads a byte from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
static uint8_t PCD_ReadRegister(	uint8_t reg	///< The register to read from. One of the PCD_Register enums.
								) {
	uint16_t value=0;
	//SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0));	// Set the settings to work with SPI bus
	//digitalWrite(_chipSelectPin, LOW);			// Select slave
	port_pin_set_output_level(RC522_SS,false);
	//SPI.transfer(0x80 | (reg & 0x7E));			// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	spi_transceive_wait(&service_spi_master_instance,0x80 | (reg & 0x7E),&value);
	//value = SPI.transfer(0);					    // Read the value back. Send 0 to stop reading.
	spi_transceive_wait(&service_spi_master_instance,0x00,&value);
	//digitalWrite(_chipSelectPin, HIGH);			// Release slave again
	port_pin_set_output_level(RC522_SS,true);
	//SPI.endTransaction(); // Stop using the SPI bus
	return (uint8_t)value;
} // End PCD_ReadRegister()

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
static void PCD_ReadRegisterN(	uint8_t reg,		///< The register to read from. One of the PCD_Register enums.
								uint8_t count,		///< The number of bytes to read
								uint8_t *values,	///< Byte array to store the values in.
								uint8_t rxAlign	///< Only bit positions rxAlign..7 in values[0] are updated.
								) {
	if (count == 0) {
		return;
	}
	uint16_t LocalValue=0;
	//Serial.print(F("Reading ")); 	Serial.print(count); Serial.println(F(" bytes from register."));
	uint8_t address = 0x80 | (reg & 0x7E);		// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	uint8_t index = 0;							// Index in values array.
	//SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0));	// Set the settings to work with SPI bus
	//digitalWrite(_chipSelectPin, LOW);		// Select slave
	port_pin_set_output_level(RC522_SS,false);
	count--;								// One read is performed outside of the loop
	//SPI.transfer(address);					// Tell MFRC522 which address we want to read
	spi_transceive_wait(&service_spi_master_instance,address,&LocalValue);
	while (index < count) {
		if (index == 0 && rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
			// Create bit mask for bit positions rxAlign..7
			uint8_t mask = 0;
			for (uint8_t i = rxAlign; i <= 7; i++) {
				mask |= (1 << i);
			}
			// Read value and tell that we want to read the same address again.
			//uint8_t value= SPI.transfer(address);
			spi_transceive_wait(&service_spi_master_instance,address,&LocalValue);
			// Apply mask to both current value of values[0] and the new data in value.
			values[0] = (values[index] & ~mask) | ((uint8_t)LocalValue & mask);
		}
		else { // Normal case
			//values[index] = SPI.transfer(address);	// Read value and tell that we want to read the same address again.
			spi_transceive_wait(&service_spi_master_instance,address,&LocalValue);
			values[index] = (uint8_t)LocalValue;
		}
		index++;
	}
	//values[index] = SPI.transfer(0);			// Read the final byte. Send 0 to stop reading.
	spi_transceive_wait(&service_spi_master_instance,0,&LocalValue);
	values[index] = (uint8_t)LocalValue;	
	//digitalWrite(_chipSelectPin, HIGH);			// Release slave again
	port_pin_set_output_level(RC522_SS,true);
	//SPI.endTransaction(); // Stop using the SPI bus
} // End PCD_ReadRegister()

/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Sets the bits given in mask in register reg.
 */
static void PCD_SetRegisterBitMask(	uint8_t reg,	///< The register to update. One of the PCD_Register enums.
										uint8_t mask	///< The bits to set.
									) { 
	uint8_t tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp | mask);			// set bit mask
} // End PCD_SetRegisterBitMask()

//////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Clears the bits given in mask from register reg.
 */
static void PCD_ClearRegisterBitMask(	uint8_t reg,	///< The register to update. One of the PCD_Register enums.
uint8_t mask	///< The bits to clear.
) {
	uint8_t tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp & (~mask));		// clear bit mask
} // End PCD_ClearRegisterBitMask()

/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
static StatusCode PCD_CalculateCRC(	uint8_t *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
												uint8_t length,	///< In: The number of bytes to transfer.
												uint8_t *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
					 ) {
	PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop any active command.
	PCD_WriteRegister(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);		// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegisterN(FIFODataReg, length, data);	// Write data to the FIFO
	PCD_WriteRegister(CommandReg, PCD_CalcCRC);		// Start the calculation
	
	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73?s.
	int16_t i = 5000;
	int8_t n;
	while (1) {
		n = PCD_ReadRegister(DivIrqReg);	// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
		if (n & 0x04) {						// CRCIRq bit set - calculation done
			break;
		}
		if (--i == 0) {						// The emergency break. We will eventually terminate on this one after 89ms. Communication with the MFRC522 might be down.
			return STATUS_TIMEOUT;
		}
	}
	PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop calculating CRC for new content in the FIFO.
	
	// Transfer the result from the registers to the result buffer
	result[0] = PCD_ReadRegister(CRCResultRegL);
	result[1] = PCD_ReadRegister(CRCResultRegH);
	return STATUS_OK;
} // End PCD_CalculateCRC()

/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
static StatusCode PCD_CommunicateWithPICC(	uint8_t command,		///< The command to execute. One of the PCD_Command enums.
														uint8_t waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
														uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
														uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
														uint8_t *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
														uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
														uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
														uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
														bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
									 ) {
	uint8_t n, _validBits;
	unsigned int i;
	
	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	
	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
	PCD_WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegisterN(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	PCD_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
	PCD_WriteRegister(CommandReg, command);				// Execute the command
	if (command == PCD_Transceive) {
		PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}
	
	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86?s.
	i = 2000;
	while (1) {
		n = PCD_ReadRegister(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
		if (--i == 0) {						// The emergency break. If all other conditions fail we will eventually terminate on this one after 35.7ms. Communication with the MFRC522 might be down.
			return STATUS_TIMEOUT;
		}
	}
	
	// Stop now if any errors except collisions were detected.
	uint8_t errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}	

	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		n = PCD_ReadRegister(FIFOLevelReg);			// Number of bytes in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;											// Number of bytes returned
		PCD_ReadRegisterN(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = PCD_ReadRegister(ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}
	
	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		return STATUS_COLLISION;
	}
	
	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		uint8_t controlBuffer[2];
		StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != RC522_STATUS_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}
	
	return RC522_STATUS_OK;
} // End PCD_CommunicateWithPICC()

/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
static StatusCode PCD_TransceiveData(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
													uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
													uint8_t *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
													uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
													uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
													uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
													bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
								 ) {
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
static StatusCode PICC_REQA_or_WUPA(	uint8_t command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
										uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
										uint8_t *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
											) {
	uint8_t validBits;
	StatusCode status;
	
	if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;
	}
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits,0,false);
	if (status != RC522_STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return RC522_STATUS_OK;
} // End PICC_REQA_or_WUPA()

/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
static StatusCode PICC_RequestA(	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
											uint8_t *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
										) {
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()

/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
static void PCD_AntennaOn(void) {
	uint8_t value = PCD_ReadRegister(TxControlReg);
	if ((value & 0x03) != 0x03) {
		PCD_WriteRegister(TxControlReg, value | 0x03);
	}
} // End PCD_AntennaOn()

//////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 * 
 * @return bool
 */
static bool PICC_IsNewCardPresent(void) {
	uint8_t bufferATQA[2];
	uint8_t bufferSize = sizeof(bufferATQA);
	StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);
	return (result == RC522_STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 * 
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
static StatusCode PICC_Select(	Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
											uint8_t validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
										 ) {
	bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	uint8_t cascadeLevel = 1;
	StatusCode result;
	uint8_t count;
	uint8_t index;
	uint8_t uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	uint8_t bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	uint8_t rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
	uint8_t *responseBuffer;
	uint8_t responseLength;
	
	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9
	
	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}
	
	// Prepare MFRC522
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	
	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;
			
			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;
			
			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;
			
			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}
		
		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}
		
		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while (!selectDone) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != RC522_STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}
			
			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			
			// Transmit the buffer and receive the response.
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign,NULL);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				uint8_t valueOfCollReg = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits = collisionPos;
				count			= (currentLevelKnownBits - 1) % 8; // The bit to modify
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << count);
			}
			else if (result != RC522_STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision 
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)
		
		// We do not check the CBB - it was constructed by us above.
		
		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}
		
		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != RC522_STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)
	
	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;
	
	return RC522_STATUS_OK;
} // End PICC_Select()

/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Dumps card info (UID,SAK,Type) about the selected PICC to Serial.
 */
static void PICC_DumpDetailsToSerial(Uid *uid	///< Pointer to Uid struct returned from a successful PICC_Select().
									) {
	//// UID
	//Serial.print(F("Card UID:"));
	//for (byte i = 0; i < uid->size; i++) {
		//if(uid->uidByte[i] < 0x10)
			//Serial.print(F(" 0"));
		//else
			//Serial.print(F(" "));
		//Serial.print(uid->uidByte[i], HEX);
	//} 
	//Serial.println();
	//
	//// SAK
	//Serial.print(F("Card SAK: "));
	//if(uid->sak < 0x10)
		//Serial.print(F("0"));
	//Serial.println(uid->sak, HEX);
	//
	//// (suggested) PICC type
	//PICC_Type piccType = PICC_GetType(uid->sak);
	//Serial.print(F("PICC type: "));
	//Serial.println(PICC_GetTypeName(piccType));
} // End PICC_DumpDetailsToSerial()

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void PICC_DumpMifareUltralightToSerial(void) {
	//StatusCode status;
	//uint8_t byteCount;
	//uint8_t buffer[18];
	//uint8_t i;
	//
	//Serial.println(F("Page  0  1  2  3"));
	//// Try the mpages of the original Ultralight. Ultralight C has more pages.
	//for (byte page = 0; page < 16; page +=4) { // Read returns data for 4 pages at a time.
		//// Read pages
		//byteCount = sizeof(buffer);
		//status = MIFARE_Read(page, buffer, &byteCount);
		//if (status != STATUS_OK) {
			//Serial.print(F("MIFARE_Read() failed: "));
			//Serial.println(GetStatusCodeName(status));
			//break;
		//}
		//// Dump data
		//for (byte offset = 0; offset < 4; offset++) {
			//i = page + offset;
			//if(i < 10)
			//Serial.print(F("  ")); // Pad with spaces
			//else
			//Serial.print(F(" ")); // Pad with spaces
			//Serial.print(i);
			//Serial.print(F("  "));
			//for (byte index = 0; index < 4; index++) {
				//i = 4 * offset + index;
				//if(buffer[i] < 0x10)
				//Serial.print(F(" 0"));
				//else
				//Serial.print(F(" "));
				//Serial.print(buffer[i], HEX);
			//}
			//Serial.println();
		//}
	//}
} // End PICC_DumpMifareUltralightToSerial()

/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Dumps memory contents of a MIFARE Classic PICC.
 * On success the PICC is halted after dumping the data.
 */
static void PICC_DumpMifareClassicToSerial(	Uid *uid,			///< Pointer to Uid struct returned from a successful PICC_Select().
												PICC_Type piccType,	///< One of the PICC_Type enums.
												MIFARE_Key *key		///< Key A used for all sectors.
											) {
	//byte no_of_sectors = 0;
	//switch (piccType) {
		//case PICC_TYPE_MIFARE_MINI:
			//// Has 5 sectors * 4 blocks/sector * 16 bytes/block = 320 bytes.
			//no_of_sectors = 5;
			//break;
			//
		//case PICC_TYPE_MIFARE_1K:
			//// Has 16 sectors * 4 blocks/sector * 16 bytes/block = 1024 bytes.
			//no_of_sectors = 16;
			//break;
			//
		//case PICC_TYPE_MIFARE_4K:
			//// Has (32 sectors * 4 blocks/sector + 8 sectors * 16 blocks/sector) * 16 bytes/block = 4096 bytes.
			//no_of_sectors = 40;
			//break;
			//
		//default: // Should not happen. Ignore.
			//break;
	//}
	//
	//// Dump sectors, highest address first.
	//if (no_of_sectors) {
		//Serial.println(F("Sector Block   0  1  2  3   4  5  6  7   8  9 10 11  12 13 14 15  AccessBits"));
		//for (int8_t i = no_of_sectors - 1; i >= 0; i--) {
			//PICC_DumpMifareClassicSectorToSerial(uid, key, i);
		//}
	//}
	//PICC_HaltA(); // Halt the PICC before stopping the encrypted session.
	//PCD_StopCrypto1();
} // End PICC_DumpMifareClassicToSerial()

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///**
 //* Dumps memory contents of a sector of a MIFARE Classic PICC.
 //* Uses PCD_Authenticate(), MIFARE_Read() and PCD_StopCrypto1.
 //* Always uses PICC_CMD_MF_AUTH_KEY_A because only Key A can always read the sector trailer access bits.
 //*/
//static void PICC_DumpMifareClassicSectorToSerial(Uid *uid,			///< Pointer to Uid struct returned from a successful PICC_Select().
													//MIFARE_Key *key,	///< Key A for the sector.
													//uint8_t sector			///< The sector to dump, 0..39.
													//) {
	////MFRC522::StatusCode status;
	////byte firstBlock;		// Address of lowest address to dump actually last block dumped)
	////byte no_of_blocks;		// Number of blocks in sector
	////bool isSectorTrailer;	// Set to true while handling the "last" (ie highest address) in the sector.
	////
	////// The access bits are stored in a peculiar fashion.
	////// There are four groups:
	//////		g[3]	Access bits for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
	//////		g[2]	Access bits for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
	//////		g[1]	Access bits for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
	//////		g[0]	Access bits for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
	////// Each group has access bits [C1 C2 C3]. In this code C1 is MSB and C3 is LSB.
	////// The four CX bits are stored together in a nible cx and an inverted nible cx_.
	////byte c1, c2, c3;		// Nibbles
	////byte c1_, c2_, c3_;		// Inverted nibbles
	////bool invertedError;		// True if one of the inverted nibbles did not match
	////byte g[4];				// Access bits for each of the four groups.
	////byte group;				// 0-3 - active group for access bits
	////bool firstInGroup;		// True for the first block dumped in the group
	////
	////// Determine position and size of sector.
	////if (sector < 32) { // Sectors 0..31 has 4 blocks each
		////no_of_blocks = 4;
		////firstBlock = sector * no_of_blocks;
	////}
	////else if (sector < 40) { // Sectors 32-39 has 16 blocks each
		////no_of_blocks = 16;
		////firstBlock = 128 + (sector - 32) * no_of_blocks;
	////}
	////else { // Illegal input, no MIFARE Classic PICC has more than 40 sectors.
		////return;
	////}
		////
	////// Dump blocks, highest address first.
	////byte byteCount;
	////byte buffer[18];
	////byte blockAddr;
	////isSectorTrailer = true;
	////for (int8_t blockOffset = no_of_blocks - 1; blockOffset >= 0; blockOffset--) {
		////blockAddr = firstBlock + blockOffset;
		////// Sector number - only on first line
		////if (isSectorTrailer) {
			////if(sector < 10)
				////Serial.print(F("   ")); // Pad with spaces
			////else
				////Serial.print(F("  ")); // Pad with spaces
			////Serial.print(sector);
			////Serial.print(F("   "));
		////}
		////else {
			////Serial.print(F("       "));
		////}
		////// Block number
		////if(blockAddr < 10)
			////Serial.print(F("   ")); // Pad with spaces
		////else {
			////if(blockAddr < 100)
				////Serial.print(F("  ")); // Pad with spaces
			////else
				////Serial.print(F(" ")); // Pad with spaces
		////}
		////Serial.print(blockAddr);
		////Serial.print(F("  "));
		////// Establish encrypted communications before reading the first block
		////if (isSectorTrailer) {
			////status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, firstBlock, key, uid);
			////if (status != STATUS_OK) {
				////Serial.print(F("PCD_Authenticate() failed: "));
				////Serial.println(GetStatusCodeName(status));
				////return;
			////}
		////}
		////// Read block
		////byteCount = sizeof(buffer);
		////status = MIFARE_Read(blockAddr, buffer, &byteCount);
		////if (status != STATUS_OK) {
			////Serial.print(F("MIFARE_Read() failed: "));
			////Serial.println(GetStatusCodeName(status));
			////continue;
		////}
		////// Dump data
		////for (byte index = 0; index < 16; index++) {
			////if(buffer[index] < 0x10)
				////Serial.print(F(" 0"));
			////else
				////Serial.print(F(" "));
			////Serial.print(buffer[index], HEX);
			////if ((index % 4) == 3) {
				////Serial.print(F(" "));
			////}
		////}
		////// Parse sector trailer data
		////if (isSectorTrailer) {
			////c1  = buffer[7] >> 4;
			////c2  = buffer[8] & 0xF;
			////c3  = buffer[8] >> 4;
			////c1_ = buffer[6] & 0xF;
			////c2_ = buffer[6] >> 4;
			////c3_ = buffer[7] & 0xF;
			////invertedError = (c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF)) || (c3 != (~c3_ & 0xF));
			////g[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0);
			////g[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1);
			////g[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
			////g[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);
			////isSectorTrailer = false;
		////}
		////
		////// Which access group is this block in?
		////if (no_of_blocks == 4) {
			////group = blockOffset;
			////firstInGroup = true;
		////}
		////else {
			////group = blockOffset / 5;
			////firstInGroup = (group == 3) || (group != (blockOffset + 1) / 5);
		////}
		////
		////if (firstInGroup) {
			////// Print access bits
			////Serial.print(F(" [ "));
			////Serial.print((g[group] >> 2) & 1, DEC); Serial.print(F(" "));
			////Serial.print((g[group] >> 1) & 1, DEC); Serial.print(F(" "));
			////Serial.print((g[group] >> 0) & 1, DEC);
			////Serial.print(F(" ] "));
			////if (invertedError) {
				////Serial.print(F(" Inverted access bits did not match! "));
			////}
		////}
		////
		////if (group != 3 && (g[group] == 1 || g[group] == 6)) { // Not a sector trailer, a value block
			////long value = (long(buffer[3])<<24) | (long(buffer[2])<<16) | (long(buffer[1])<<8) | long(buffer[0]);
			////Serial.print(F(" Value=0x")); Serial.print(value, HEX);
			////Serial.print(F(" Adr=0x")); Serial.print(buffer[12], HEX);
		////}
		////Serial.println();
	////}
	////
	////return;
//} // End PICC_DumpMifareClassicSectorToSerial()

/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Translates the SAK (Select Acknowledge) to a PICC type.
 * 
 * @return PICC_Type
 */
static PICC_Type PICC_GetType(uint8_t sak		///< The SAK byte returned from PICC_Select().
										) {
	// http://www.nxp.com/documents/application_note/AN10833.pdf 
	// 3.2 Coding of Select Acknowledge (SAK)
	// ignore 8-bit (iso14443 starts with LSBit = bit 1)
	// fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
	sak &= 0x7F;
	switch (sak) {
		case 0x04:	return PICC_TYPE_NOT_COMPLETE;	// UID not complete
		case 0x09:	return PICC_TYPE_MIFARE_MINI;
		case 0x08:	return PICC_TYPE_MIFARE_1K;
		case 0x18:	return PICC_TYPE_MIFARE_4K;
		case 0x00:	return PICC_TYPE_MIFARE_UL;
		case 0x10:
		case 0x11:	return PICC_TYPE_MIFARE_PLUS;
		case 0x01:	return PICC_TYPE_TNP3XXX;
		case 0x20:	return PICC_TYPE_ISO_14443_4;
		case 0x40:	return PICC_TYPE_ISO_18092;
		default:	return PICC_TYPE_UNKNOWN;
	}
} // End PICC_GetType()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
static StatusCode PICC_HaltA(void) {
	StatusCode result;
	uint8_t buffer[4];
	
	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != RC522_STATUS_OK) {
		return result;
	}
	
	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	result = PCD_TransceiveData(buffer, sizeof(buffer), NULL, 0,NULL,0,false);
	if (result == STATUS_TIMEOUT) {
		return RC522_STATUS_OK;
	}
	if (result == RC522_STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return result;
} // End PICC_HaltA()


/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Dumps debug info about the selected PICC to Serial.
 * On success the PICC is halted after dumping the data.
 * For MIFARE Classic the factory default key of 0xFFFFFFFFFFFF is tried. 
 */
static void PICC_DumpToSerial(Uid *uid	///< Pointer to Uid struct returned from a successful PICC_Select().
								) {
	MIFARE_Key key;
	
	// Dump UID, SAK and Type
	PICC_DumpDetailsToSerial(uid);
	
	// Dump contents
	PICC_Type piccType = PICC_GetType(uid->sak);
	switch (piccType) {
		case PICC_TYPE_MIFARE_MINI:
		case PICC_TYPE_MIFARE_1K:
		case PICC_TYPE_MIFARE_4K:
			// All keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
			for (uint8_t i = 0; i < 6; i++) {
				key.keyByte[i] = 0xFF;
			}
			PICC_DumpMifareClassicToSerial(uid, piccType, &key);
			break;
			
		case PICC_TYPE_MIFARE_UL:
			PICC_DumpMifareUltralightToSerial();
			break;
			
		case PICC_TYPE_ISO_14443_4:
		case PICC_TYPE_ISO_18092:
		case PICC_TYPE_MIFARE_PLUS:
		case PICC_TYPE_TNP3XXX:
			//Serial.println(F("Dumping memory contents not implemented for that PICC type."));
			break;
			
		case PICC_TYPE_UNKNOWN:
		case PICC_TYPE_NOT_COMPLETE:
		default:
			break; // No memory dump here
	}
	
	//Serial.println();
	PICC_HaltA(); // Already done if it was a MIFARE Classic PICC.
} // End PICC_DumpToSerial()

/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 * 
 * @return bool
 */
static bool PICC_ReadCardSerial(void) {
	StatusCode result = PICC_Select(&g_uid,0);
	return (result == RC522_STATUS_OK);
} // End 

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void MFRC522_PCD_Init(void) {
	//// Set the chipSelectPin as digital output, do not select the slave yet
	//pinMode(_chipSelectPin, OUTPUT);
	//digitalWrite(_chipSelectPin, HIGH);
	//
	//// Set the resetPowerDownPin as digital output, do not reset or power down.
	//pinMode(_resetPowerDownPin, OUTPUT);
	//
	//if (digitalRead(_resetPowerDownPin) == LOW) {	//The MFRC522 chip is in power down mode.
		//digitalWrite(_resetPowerDownPin, HIGH);		// Exit power down mode. This triggers a hard reset.
		//// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74?s. Let us be generous: 50ms.
		//delay(50);
	//}
	//else { // Perform a soft reset
		//PCD_Reset();
	//}
	
	port_pin_set_output_level(RC522_RST,false);
	delay_ms(50);
	port_pin_set_output_level(RC522_RST,true);	
	
	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	PCD_WriteRegister(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	PCD_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25?s.
	PCD_WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	PCD_WriteRegister(TReloadRegL, 0xE8);
	
	PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	PCD_WriteRegister(RFCfgReg, 0x70);      //RxGain = 48dB
	PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
} // End PCD_Init()

/////////////////////////////////////////////////////////////////////////////////////////////////////////

//void setup(void) {
	////Serial.begin(9600);	// Initialize serial communications with the PC
	////SPI.begin();			// Init SPI bus
	//MFRC522_PCD_Init();	// Init MFRC522 card
	////Serial.println("Scan PICC to see UID and type...");
//}

bool RFIDCheck(void) {
	// Look for new cards
	if ( ! PICC_IsNewCardPresent()) {
		return false;
	}

	// Select one of the cards
	if ( ! PICC_ReadCardSerial()) {
		return false;
	}

	// Dump debug info about the card. PICC_HaltA() is automatically called.
	PICC_DumpToSerial(&(g_uid));
	return true;
}

