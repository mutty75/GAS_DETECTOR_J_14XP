/*
 * IdleHook.c
 *
 * Created: 13/05/2016 07.59.42
 *  Author: ROBERTOC
 */ 

#include "IdleHook.h"
#include "conf_st7565r.h"

void vApplicationIdleHook( void )
{
	
  //if (xPowerDown) 
  //{
   //struct port_config  config;
   ////port_pin_set_output_level(DEBUG_PIN,false);
  ///// SysTick->CTRL=0;
  //// delay_ms(500);
   //if (!port_pin_get_input_level(PIN_PA11))
   //{
	//port_get_config_defaults(&config);
    //config.direction=PORT_PIN_DIR_INPUT;
    //config.input_pull=PORT_PIN_PULL_DOWN;
    //port_group_set_config(&PORTA,~(PORT_PA11) ,&config);  
    //while ((WDT->STATUS.reg & WDT_STATUS_SYNCBUSY));
    //WDT->CTRL.reg &= ~WDT_CTRL_ENABLE;
    //bod_disable(BOD_BOD33);
    //SysTick->CTRL=0; //Ferma il SysTick
    //EIC->CONFIG[1].bit.SENSE3=EIC_CONFIG_SENSE0_RISE_Val;
    //system_sleep();
   //}
   //else xPowerDown=false;
  //}  	
	
}

//////////////////////////////////////////////////////////////////////////////////////////

void vApplicationTickHook( void )
{
	wdt_reset_count();
}