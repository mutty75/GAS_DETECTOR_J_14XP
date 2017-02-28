/*
 * IdleHook.h
 *
 * Created: 13/05/2016 07.59.21
 *  Author: ROBERTOC
 */ 

#include "asf.h"

#ifndef IDLEHOOK_H_
#define IDLEHOOK_H_

volatile bool xPowerDown;

void vApplicationIdleHook( void );
void vApplicationTickHook( void );

#endif /* IDLEHOOK_H_ */



