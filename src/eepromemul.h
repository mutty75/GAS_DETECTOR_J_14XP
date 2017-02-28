/*
 * eeprom.h
 *
 * Created: 19/05/2016 11.42.05
 *  Author: ROBERTOC
 */ 


#ifndef EEPROM_H_
#define EEPROM_H_

#define PATTERN_EE 0x55
#define EE_OFFSET 0x0A

#define EEPROM_SIZE 4*NVMCTRL_ROW_SIZE

void configure_eeprom(void);
void configure_bod(void);

void vLoadSetup(void);
void vSaveSetup(void);

bool xLoadPattern(void);
void vSavePattern(void);

#endif /* EEPROM_H_ */