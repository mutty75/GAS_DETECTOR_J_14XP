/*
 * eeprom.c
 *
 * Created: 19/05/2016 11.43.59
 *  Author: ROBERTOC
 */ 

#include <asf.h>
#include "eepromemul.h"
#include "taskslist.h"

//! [setup]
//void configure_eeprom(void)
//{
	///* Setup EEPROM emulator service */
////! [init_eeprom_service]
	//enum status_code error_code = eeprom_emulator_init();
////! [init_eeprom_service]
//
////! [check_init_ok]
	//if (error_code == STATUS_ERR_NO_MEMORY) {
		//while (true) {
			///* No EEPROM section has been set in the device's fuses */
		//}
	//}
////! [check_init_ok]
////! [check_re-init]
	//else if (error_code != STATUS_OK) {
		///* Erase the emulated EEPROM memory (assume it is unformatted or
		 //* irrecoverably corrupt) */
		//eeprom_emulator_erase_memory();
		//eeprom_emulator_init();
	//}
////! [check_re-init]
//}
//
//#if (SAMD || SAMR21)
//void SYSCTRL_Handler(void)
//{
	//if (SYSCTRL->INTFLAG.reg & SYSCTRL_INTFLAG_BOD33DET) {
		//SYSCTRL->INTFLAG.reg |= SYSCTRL_INTFLAG_BOD33DET;
		//eeprom_emulator_commit_page_buffer();
	//}
//}
//#endif
//
//void configure_bod(void)
//{
//#if (SAMD || SAMR21)
	//struct bod_config config_bod33;
	//bod_get_config_defaults(&config_bod33);
	//config_bod33.action = BOD_ACTION_INTERRUPT;
	///* BOD33 threshold level is about 3.2V */
	//config_bod33.level = 48;
	//bod_set_config(BOD_BOD33, &config_bod33);
	//bod_enable(BOD_BOD33);
//
	//SYSCTRL->INTENSET.reg |= SYSCTRL_INTENCLR_BOD33DET;
	//system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_SYSCTRL);
//#endif
//
//}

/////////////////////////////////////////////////////////////////

//void vLoadSetup(void)
//{
 //eeprom_emulator_read_buffer(EE_OFFSET,(uint8_t *)&xMachineSetup,sizeof(xMachineSetup));
//}

/////////////////////////////////////////////////////////////////

//bool xLoadPattern(void)
//{
 //uint8_t ucPattern;
 //eeprom_emulator_read_buffer(0,&ucPattern,sizeof(ucPattern));
	//
 //return (ucPattern==PATTERN_EE) ? true : false;
//}

/////////////////////////////////////////////////////////////////

//void vSaveSetup(void)
//{
 //eeprom_emulator_write_buffer(EE_OFFSET,(uint8_t *)&xMachineSetup,sizeof(xMachineSetup));
 //eeprom_emulator_commit_page_buffer();
//}

/////////////////////////////////////////////////////////////////

//void vSavePattern(void)
//{
 //uint8_t ucPattern=PATTERN_EE;
 //
 //eeprom_emulator_write_buffer(0,&ucPattern,sizeof(ucPattern));
 //eeprom_emulator_commit_page_buffer();
//}

/////////////////////////////////////////////////////////////////