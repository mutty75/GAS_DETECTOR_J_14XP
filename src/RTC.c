/*
 * RTC.c
 *
 * Created: 10/2/2016 13:13:31
 *  Author: ROBERTOC
 */ 

#include "asf.h"
#include "RTC.h"
#include "RC522.h"
#include "taskslist.h"


#define REG_READ 0x13
#define REG_WRITE 0x12

//////////////////////////////////////////////////////////////////////////////

//static uint8_t RTC_ReadRegister(uint8_t reg) 
//{
	//uint16_t LocalValue;
//
	//port_pin_set_output_level(RTC_CS,false);
	//spi_transceive_wait(&service_spi_master_instance,REG_READ,&LocalValue);
	//spi_transceive_wait(&service_spi_master_instance,reg,&LocalValue);
	//spi_transceive_wait(&service_spi_master_instance,0,&LocalValue);
	//port_pin_set_output_level(RTC_CS,true);
	//
	//return (uint8_t)LocalValue;
//} 

//////////////////////////////////////////////////////////////////////////////

void RTC_WriteRegister(uint8_t reg, uint8_t value) 
{
	uint16_t LocalValue;

	port_pin_set_output_level(RTC_CS,false);
	spi_transceive_wait(&service_spi_master_instance,REG_WRITE,&LocalValue);
	spi_transceive_wait(&service_spi_master_instance,reg,&LocalValue);
	spi_transceive_wait(&service_spi_master_instance,value,&LocalValue);
	port_pin_set_output_level(RTC_CS,true);

} 

//////////////////////////////////////////////////////////////////////////////

void RTC_WriteRegisters(xRTCStruct *xRTCValue)
{
	uint16_t LocalValue;

	port_pin_set_output_level(RTC_CS,false);
	spi_transceive_wait(&service_spi_master_instance,REG_WRITE,&LocalValue);
	spi_transceive_wait(&service_spi_master_instance,0x01,&LocalValue);
	spi_write_buffer_wait(&service_spi_master_instance,(uint8_t *)xRTCValue,0x07);
	port_pin_set_output_level(RTC_CS,true);
}

//////////////////////////////////////////////////////////////////////////////

static void RTC_ReadRegisters(xRTCStruct *xRTCValue) 
{
	uint16_t LocalValue;
	
	port_pin_set_output_level(RTC_CS,false);
	spi_transceive_wait(&service_spi_master_instance,REG_READ,&LocalValue);
	spi_transceive_wait(&service_spi_master_instance,0x01,&LocalValue);
	spi_read_buffer_wait(&service_spi_master_instance,(uint8_t *)xRTCValue,0x07,0x00);
	port_pin_set_output_level(RTC_CS,true);
	
    if (!xRTCValue->xStart)
	{
	 xRTCValue->xSec=0;
	 xRTCValue->xSec10=0;
	 xRTCValue->xStart=0;
	 xRTCValue->xMin=0;
	 xRTCValue->xMin10=0;
	 xRTCValue->xHour=0;
	 xRTCValue->xHour10=0;
	 xRTCValue->x1224=0;
	 xRTCValue->xCalSign=0;
     xRTCValue->xDay=1;
	 xRTCValue->xBatEN=1;
	 xRTCValue->xVBat=0;
	 xRTCValue->xOSCon=0;			
	 xRTCValue->xDate=1;
	 xRTCValue->xDate10=0;	
	 xRTCValue->xMonth=1;
	 xRTCValue->xMonth10=0;
     xRTCValue->xLeap=0;
	 xRTCValue->xYear=0;
	 xRTCValue->xYear10=0;
	 RTC_WriteRegisters(xRTCValue);
	 RTC_WriteRegister(0x00,0x00);
	 RTC_WriteRegister(0x01,(1<<7)+(xRTCValue->xSec10<<4)+xRTCValue->xSec);
	}
	
	RTC_WriteRegister(0x08,(0<<6) | (0<<1) | (0<<0));
	RTC_WriteRegister(0x09,00);
} 

///////////////////////////////////////////////////////////////////////////////

void RTCRead(void)
{
  xRTCStruct xRTCValue;
  
  RTC_ReadRegisters(&xRTCValue);
  rtc_calendar_get_time_defaults(&Machinetime);
  Machinetime.year=xRTCValue.xYear+xRTCValue.xYear10*10+2000;
  Machinetime.hour=xRTCValue.xHour+xRTCValue.xHour10*10;
  Machinetime.minute=xRTCValue.xMin+xRTCValue.xMin10*10;
  Machinetime.second=xRTCValue.xSec+xRTCValue.xSec10*10;
  Machinetime.month=xRTCValue.xMonth+xRTCValue.xMonth10*10;
  Machinetime.day=xRTCValue.xDate+xRTCValue.xDate10*10;
  
  rtc_calendar_set_time(&rtc_instance, &Machinetime);
  timeinfo.tm_year = Machinetime.year - 1900;
  timeinfo.tm_mon  = Machinetime.month -1;
  timeinfo.tm_mday = Machinetime.day;
  timeinfo.tm_hour = Machinetime.hour;
  timeinfo.tm_min  = Machinetime.minute;
  timeinfo.tm_sec = Machinetime.second; 
  xMasterTime = mktime(&timeinfo);
	
}

///////////////////////////////////////////////////////////////////////////////