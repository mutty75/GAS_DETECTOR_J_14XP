/**
 * \file
 *
 * \brief FreeRTOS demo task implementations.
 *
 * Copyright (C) 2013-2015 Atmel Corporation. All rights reserved.
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

#include <asf.h>
#include <ctype.h>
#include "FreeRTOS.h"
#include "DHT22.h"
#include "crc.h"
#include "portmacro.h"
#include "task.h"
#include "st7565r.h"
#include "RC522.h"
#include "RTC.h"
#include "ili9341.h"
#include "limits.h"
#include "ili9341_regs.h"
#include "taskslist.h"
#include "IdleHook.h"
#include "eepromemul.h"
#include "LCDFunctions\loadbitmap.h"
#include "LCDFunctions\lcdleddriver.h"
#include "LCDFunctions\DriverLCD.h"

#define GRAPH_TASK_PRIORITY     (tskIDLE_PRIORITY + 1)

#define APP_START_ADDRESS (0x8000)

uint8_t ucActiveIndex;


static void main_task(void *params);
//static void graph_task(void *params);
//static void SD_task(void *params);
bool scan_files (const char*);
void vCheckFirmware(void);

TaskHandle_t xLCDTask;
TaskHandle_t xSDTask;

uint8_t rx_buffer[512];
struct tm *timeinfTmp;
float T,H;

char LCDlabels[30][MAXLSIZE]={"     INSERT",
							  "  MEMORY CARD",
							  "%02d%c%02d %02d/%02d/%02d %c",
							  "H:%2d.%d%%  T:%2d.%dC",
							  "     REFILL"};
							  
char WDay[7]={'S','M','T','W','T','F','S'};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void tasks_init(void)
{
	xTaskCreate(main_task,(const char *)"Main",configMINIMAL_STACK_SIZE*3,NULL,GRAPH_TASK_PRIORITY,&xLCDTask);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void vCheckFirmware()
{
 UINT BytesRead;
 DRESULT result;
 enum status_code error_code;
 struct dma_crc_config crc_config;
 
 result=f_open(&file_object,"0:/output.bin",FA_READ);
 if (result!=RES_OK) return;
 
  /* Start the DMA CRC with I/O mode */
 DMAC->CRCCHKSUM.bit.CRCCHKSUM=0;
 dma_crc_get_config_defaults(&crc_config);
 crc_config.type = CRC_TYPE_16;
 dma_crc_io_enable(&crc_config);
 
 uint16_t usCRC=0;
 f_lseek(&file_object,APP_START_ADDRESS);
 for (uint32_t ulPageCounter=APP_START_ADDRESS;ulPageCounter<(FLASH_SIZE-EEPROM_SIZE);ulPageCounter+=NVMCTRL_ROW_SIZE)
 {
  f_read(&file_object,rx_buffer,NVMCTRL_ROW_SIZE,&BytesRead);	
  if (BytesRead!=NVMCTRL_ROW_SIZE) return;
  dma_crc_io_calculation(rx_buffer,NVMCTRL_ROW_SIZE);
  usCRC=usCrc16(rx_buffer,NVMCTRL_ROW_SIZE,usCRC);
 }
 
 
 usCRC = dma_crc_get_checksum();
 dma_crc_disable();
 
 if (usCRC) return;
 
 f_close(&file_object);
 
 cpu_irq_disable();
 
do 
{
  error_code=nvm_read_buffer(FLASH_SIZE-EEPROM_SIZE-NVMCTRL_PAGE_SIZE,rx_buffer,NVMCTRL_PAGE_SIZE);
} while (error_code == STATUS_BUSY);

rx_buffer[NVMCTRL_PAGE_SIZE-2]^=0xFF;
rx_buffer[NVMCTRL_PAGE_SIZE-1]^=0xFF;

do
{
  error_code = nvm_erase_row(FLASH_SIZE-EEPROM_SIZE-NVMCTRL_ROW_SIZE);
} while (error_code == STATUS_BUSY);



do
{
	error_code=nvm_write_buffer(FLASH_SIZE-EEPROM_SIZE-NVMCTRL_PAGE_SIZE,rx_buffer,NVMCTRL_PAGE_SIZE);
} while (error_code == STATUS_BUSY);	

 system_reset();
 //start_application_with_wdt();
 
 
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void configure_rtc_calendar(void)
{
	/* Initialize RTC in calendar mode. */
	struct rtc_calendar_config config_rtc_calendar;
	rtc_calendar_get_config_defaults(&config_rtc_calendar);
	//struct rtc_calendar_time alarm;
	//rtc_calendar_get_time_defaults(&alarm);
	//alarm.year   = 2013;
	//alarm.month  = 1;
	//alarm.day    = 1;
	//alarm.hour   = 0;
	//alarm.minute = 0;
	//alarm.second = 4;
	config_rtc_calendar.clock_24h     = true;
	//config_rtc_calendar.alarm[0].time = alarm;
	//config_rtc_calendar.alarm[0].mask = RTC_CALENDAR_ALARM_MASK_YEAR;
	rtc_calendar_init(&rtc_instance, RTC, &config_rtc_calendar);
	rtc_calendar_enable(&rtc_instance);
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void vOpenFiles(void)
{   UINT uSize;
	DRESULT result;
	
	result=f_open(&file_object,"0:/PrgList.bin",FA_READ);
	if (result!=RES_OK)
	{
	 f_open(&file_object,"0:/PrgList.bin",FA_CREATE_ALWAYS | FA_WRITE);
	 memset(xProgramList,0x00,sizeof(xProgramList));
	 for (uint8_t ucIndex=0x00;ucIndex<MAX_PRG;ucIndex++)	
	  xProgramList[ucIndex].xProgramType=P_OFF;
	 result=f_write(&file_object,xProgramList,sizeof(xProgramList),&uSize);
	 f_close(&file_object);
	}
	else
	{
	 f_read(&file_object,xProgramList,sizeof(xProgramList),&uSize);	
	 f_close(&file_object);
	}
	
	result=f_open(&file_object,"0:/Setup.bin",FA_READ);
	if (result!=RES_OK)
	{
		f_open(&file_object,"0:/Setup.bin",FA_CREATE_ALWAYS | FA_WRITE);
	    xMachineSetup.ucIPAddr[0]=0x01;
	    xMachineSetup.ucIPAddr[1]=0x01;
	    xMachineSetup.ucIPAddr[2]=0xA8;
	    xMachineSetup.ucIPAddr[3]=0xC0;
	    xMachineSetup.ucLanguage=0;
	    xMachineSetup.ucWEPKey[0]=0x11;
	    xMachineSetup.ucWEPKey[1]=0x22;
	    xMachineSetup.ucWEPKey[2]=0x33;
	    xMachineSetup.ucWEPKey[3]=0x44;
	    xMachineSetup.ucWEPKey[4]=0x55;
	    xMachineSetup.ulPump1=0x00;
	    xMachineSetup.ulPump2=0x00;
	    xMachineSetup.usRevTime=300;
	    xMachineSetup.xDateTime=0x00;		
		result=f_write(&file_object,&xMachineSetup,sizeof(xMachineSetup),&uSize);
		f_close(&file_object);
	}
	else
	{
		f_read(&file_object,&xMachineSetup,sizeof(xMachineSetup),&uSize);
		f_close(&file_object);
	}
	
	result=f_open(&file_object,"0:/LogTable.bin",FA_READ);
	if (result!=RES_OK)
	{
		f_open(&file_object,"0:/LogTable.bin",FA_CREATE_ALWAYS | FA_WRITE);
		memset(&xLogTable,0x00,sizeof(xLogTable));
		for (uint8_t ucIndex=0x00;ucIndex<MAX_LOGS;ucIndex++)
		 xLogTable[ucIndex].ucPrgIndex=0xFF;
		result=f_write(&file_object,&xLogTable,sizeof(xLogTable),&uSize);
		f_close(&file_object);
	}
	else
	{
		f_read(&file_object,&xLogTable,sizeof(xLogTable),&uSize);
		f_close(&file_object);
	}
	
	//if (xLoadPattern()) vLoadSetup();
	//else
	//{
	 //vSavePattern();
	 //
	 //xMachineSetup.ucIPAddr[0]=0x01;
	 //xMachineSetup.ucIPAddr[1]=0x01;
	 //xMachineSetup.ucIPAddr[2]=0xA8;
	 //xMachineSetup.ucIPAddr[3]=0xC0;
	 //xMachineSetup.ucLanguage=0;
	 //xMachineSetup.ucWEPKey[0]=0x11;
	 //xMachineSetup.ucWEPKey[1]=0x22;
	 //xMachineSetup.ucWEPKey[2]=0x33;
	 //xMachineSetup.ucWEPKey[3]=0x44;
	 //xMachineSetup.ucWEPKey[4]=0x55;
	 //xMachineSetup.ulPump1=0x00;
	 //xMachineSetup.ulPump2=0x00;
	 //xMachineSetup.usRevTime=300;
	 //xMachineSetup.xDateTime=0x00;
	 //memset(&xMachineSetup.ucSpare,0x00,sizeof(xMachineSetup.ucSpare));
	 //
	 //vSaveSetup();	
	//}		
	
	result=f_open(&file_object,"0:/Labels.bin",FA_READ);
	if (result!=RES_OK)
	{
		f_open(&file_object,"0:/Labels.bin",FA_CREATE_ALWAYS | FA_WRITE);
		for (uint8_t ucIndex=0;ucIndex<MAX_LANGUAGE;ucIndex++)
		 result=f_write(&file_object,&LCDlabels,sizeof(LCDlabels),&uSize); 
		f_close(&file_object);
	}
	else
	{
		//Aggiungere Offset Lingua
		f_lseek(&file_object,(uint32_t)xMachineSetup.ucLanguage*sizeof(LCDlabels));
		f_read(&file_object,&LCDlabels,sizeof(LCDlabels),&uSize);
		f_close(&file_object);
	}	
	
	result=f_open(&file_object,"0:/RunTime.bin",FA_READ);
	if (result!=RES_OK)
	{
		f_open(&file_object,"0:/RunTime.bin",FA_CREATE_ALWAYS | FA_WRITE);
		result=f_write(&file_object,&ulMachineWorkTime,sizeof(ulMachineWorkTime),&uSize);
		f_close(&file_object);
	}
	else
	{
		f_read(&file_object,&ulMachineWorkTime,sizeof(ulMachineWorkTime),&uSize);
		f_close(&file_object);
	}
	
	result=f_open(&file_object,"0:/TagList.bin",FA_READ);
	if (result!=RES_OK)
	{
		f_open(&file_object,"0:/TagList.bin",FA_CREATE_ALWAYS | FA_WRITE);
		memset(&xTags,0x00,sizeof(xTags));
		result=f_write(&file_object,&xTags,sizeof(xTags),&uSize);
		f_close(&file_object);
	}
	else
	{
		f_read(&file_object,&xTags,sizeof(xTags),&uSize);
		f_close(&file_object);
	}
	
	f_mkdir("Logs");
	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void vMainScreen(uint16_t counter,struct rtc_calendar_time *ptime)
{   //unsigned char ucPixels=((unsigned short)40*perc)/100;
	unsigned char ucPixels=counter % (16*5+1);
	unsigned char ucBOX=ucPixels/5;
	unsigned char ucFract=ucPixels%5;
	
    memset(LCDData,0x00,sizeof(LCDData));
	
	//sprintf(LCDData[0],LCDlabels[2],ptime->hour,(ptime->second & 0x01) ? ' ' : ':',
	//ptime->minute,ptime->day,ptime->month,ptime->year-2000,WDay[ptime->weekday]);
	
	//sprintf(LCDData[0],LCDlabels[2],ptime->hour,(ptime->second & 0x01) ? ' ' : ':',
	//ptime->minute,ptime->second,ptime->month,ptime->year-2000,WDay[ptime->weekday]);

	sprintf(LCDData[0],LCDlabels[2],ptime->hour,(ptime->second & 0x01) ? ' ' : ':',
	ptime->minute,ptime->second,ptime->month,ptime->year-2000,(xValidTAG) ? 'T' : ' ');
	

	//if ((!((uint16_t)H)) && (!((uint16_t)(H*10)%10)))
	//{
	 //return;
	//}
	//
	//if ((!((uint16_t)T)) && (!((uint16_t)(T*10)%10)))
	//{
	 //return;	
	//}
	
	sprintf(LCDData[1],LCDlabels[3],(uint16_t)H,((uint16_t)(H*10))%10,(uint16_t)T,((uint16_t)(T*10))%10);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void vPrgScreen(uint8_t perc)
{   unsigned char ucPixels=((unsigned short)35*perc)/100+1;
	//unsigned char ucPixels=counter % (16*5+1);
	unsigned char ucBOX=ucPixels/5;
	unsigned char ucFract=ucPixels%5;
	
    memset(LCDData,0x00,sizeof(LCDData));
	sprintf(LCDData[0],"%s",xProgramList[ucActiveIndex].cPrgName);
	
	switch (xProgramSequence)
	{	
	 case TS:
	 {
      time_t xResTime=xProgramList[ucActiveIndex].usTs-xElapTime;
      sprintf(&LCDData[1][0],"%02d:%02d %s",(uint16_t)xResTime/60,(uint16_t)xResTime%60,"Ts"); 
	 }
	 break; 
	 case TI:
	 {
      time_t xResTime=xProgramList[ucActiveIndex].usTi-xElapTime;
      sprintf(&LCDData[1][0],"%02d:%02d %s",(uint16_t)xResTime/60,(uint16_t)xResTime%60,"Ti"); 
	 }
	 break; 
	 case TFP:
	 {
      time_t xResTime=xProgramList[ucActiveIndex].usTfp-xElapTime;
	  memset (LCDData,0x00,sizeof(LCDData));
	  sprintf(LCDData[0],"%s",LCDlabels[4]);
      sprintf(&LCDData[1][0],"%02d:%02d %s",(uint16_t)xResTime/60,(uint16_t)xResTime%60,"Tf"); 
	 }
	 break; 
	 case TP:
	 {
      time_t xResTime=xProgramList[ucActiveIndex].usTp-xElapTime;
      sprintf(&LCDData[1][0],"%02d:%02d %s",(uint16_t)xResTime/60,(uint16_t)xResTime%60,"Tp"); 
	 }
	 break; 
	 case TA:
	 {
		 time_t xResTime=xProgramList[ucActiveIndex].usTa-xElapTime;
		 sprintf(&LCDData[1][0],"%02d:%02d %s",(uint16_t)xResTime/60,(uint16_t)xResTime%60,"Ta");
	 }
	 break;
	 case TW1:
	 {
		 time_t xResTime=14*60-xElapTime;
		 sprintf(&LCDData[1][0],"%02d:%02d %s",(uint16_t)xResTime/60,(uint16_t)xResTime%60,"W1");
	 }
	 break;
	 case TW2:
	 {
		 time_t xResTime=7*60-xElapTime;
		 sprintf(&LCDData[1][0],"%02d:%02d %s",(uint16_t)xResTime/60,(uint16_t)xResTime%60,"W2");
	 }
	 break;
	 case TIW:
	 {
		 time_t xResTime=60-xElapTime;
		 sprintf(&LCDData[1][0],"%02d:%02d %s",(uint16_t)xResTime/60,(uint16_t)xResTime%60,"Wi");
	 }
	 break;
	 default: break;
	}
	
	if (ucPixels>35) ucPixels=35;
	for (uint8_t ucIndex=0x00;ucIndex<ucBOX;ucIndex++)
	LCDData[1][9+ucIndex]=5;
	if (ucFract) LCDData[1][9+ucBOX]=ucFract;
	

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint8_t ucSeekLogPos(void)
{   time_t OlderLog=LONG_MAX;
	uint8_t ucOlderIndex=0;
	
	for (uint8_t ucIndex=0x00;ucIndex<MAX_LOGS;ucIndex++)
	{
	 if (xLogTable[ucIndex].ucPrgIndex==0xFF) return ucIndex;
	 if (xLogTable[ucIndex].xDateTime<OlderLog) 
	 {
	   OlderLog=xLogTable[ucIndex].xDateTime;
	   ucOlderIndex=ucIndex;
	 }
	}
	
	return ucOlderIndex;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static bool xCheckPrograms(uint8_t *ucPrgIndex)
{
 UINT uSize;
 DRESULT result;	

 for (uint8_t ucIndex=0;ucIndex<MAX_PRG;ucIndex++)
 {
  if ((xProgramList[ucIndex].xProgramType==P_OFF) || (!xProgramList[ucIndex].xStartTime)) continue;
  if ((xProgramList[ucIndex].xStartTime<xMasterTime) && ((xMasterTime-xProgramList[ucIndex].xStartTime)<PRGDELTATIME))
  {
	*ucPrgIndex=ucIndex;
	
	switch (xProgramList[ucIndex].xProgramType)
	{
	 case P_STD:
	 case P_WASH:
	 
	 xProgramStartTime=xProgramList[ucIndex].xStartTime;
	 xProgramList[ucIndex].xStartTime=0;
	 xProgramSequence=TS;
	 ucLogPos=ucSeekLogPos();
	 
	 break;
	 
	 case P_WEEK:
	 
	 xProgramStartTime=xProgramList[ucIndex].xStartTime;
	 xProgramList[ucIndex].xStartTime+=24*3600*7;
	 xProgramSequence=TS;
	 ucLogPos=ucSeekLogPos();
	 	 
	 break;	
	 
	 case P_DAY:
	 
	 xProgramStartTime=xProgramList[ucIndex].xStartTime;
	 xProgramList[ucIndex].xStartTime+=24*3600;
	 xProgramSequence=TS;
	 ucLogPos=ucSeekLogPos();
	
	 break;
	}
	
	xLogTable[ucLogPos].ucPrgIndex=ucIndex;
	xLogTable[ucLogPos].xDateTime=xProgramStartTime;
	xLogTable[ucLogPos].ucLogPages=((xProgramList[ucIndex].usTp/xProgramList[ucIndex].usTr)/MAX_REC_PAGE)+1;
	
	f_open(&file_object,"0:/PrgList.bin",FA_WRITE);
	f_lseek(&file_object,sizeof(xProgram)*ucIndex);
	result=f_write(&file_object,&xProgramList[ucIndex],sizeof(xProgram),&uSize);
	f_close(&file_object);
	
	f_open(&file_object,"0:/LogTable.bin",FA_WRITE);
	f_lseek(&file_object,sizeof(xLogTb)*ucLogPos);
	result=f_write(&file_object,&xLogTable[ucLogPos],sizeof(xLogTb),&uSize);
	f_close(&file_object);	
	
	sprintf(TempString,"0:/Logs/Log%03d.bin",ucLogPos);
	f_unlink(TempString);
	f_open(&file_object,TempString,FA_CREATE_ALWAYS | FA_WRITE);
	f_close(&file_object);	
	
	return true;  
  }
  
 }
 
 //controlla se seve aggiornare i programmi week o day nel caso la macchina fosse spenta
 for (uint8_t ucIndex=0;ucIndex<MAX_PRG;ucIndex++)
 if ((xProgramList[ucIndex].xProgramType==P_WEEK) || (xProgramList[ucIndex].xProgramType==P_DAY))
 {
   if ((xProgramList[ucIndex].xProgramType==P_OFF) || (!xProgramList[ucIndex].xStartTime)) continue;	
   if ((xMasterTime>xProgramList[ucIndex].xStartTime) && ((xMasterTime-xProgramList[ucIndex].xStartTime)>=PRGDELTATIME))
   {
    switch (xProgramList[ucIndex].xProgramType)
	{
	 case P_WEEK: xProgramList[ucIndex].xStartTime+=24*3600*7;break;
	 case P_DAY:  xProgramList[ucIndex].xStartTime+=24*3600;break;
	 case P_STD:
	 case P_WASH: xProgramList[ucIndex].xStartTime=0;
	}
		 
	f_open(&file_object,"0:/PrgList.bin",FA_WRITE);
	f_lseek(&file_object,sizeof(xProgram)*ucIndex);
	result=f_write(&file_object,&xProgramList[ucIndex],sizeof(xProgram),&uSize);
	f_close(&file_object);	 
   }
 } 
	
 return false;
 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void vWriteLog(time_t xETime)
{
 UINT uSize;
 DRESULT result;
 xLogData xTempLogData;
 static time_t LastRecTime=0;
 
 if (!(xETime % xProgramList[ucActiveIndex].usTr))	
 {
  if (LastRecTime==xETime) return;
  LastRecTime=xETime;
  xTempLogData.xDateTime=xMasterTime;
  xTempLogData.xMachineState=xMachineState;
  xTempLogData.usTemp=(uint16_t)(T*10);
  xTempLogData.usH=(uint16_t)(H*10);
  xTempLogData.usWRKTime=0x00;
  memset(&xTempLogData.ucSpare,0x00,sizeof(xTempLogData.ucSpare));

  sprintf(TempString,"0:/Logs/Log%03d.bin",ucLogPos);
  f_open(&file_object,TempString,FA_WRITE);
  f_lseek(&file_object,f_size(&file_object));
  result=f_write(&file_object,&xTempLogData,sizeof(xLogData),&uSize);
  f_close(&file_object);	  
 }
	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


static void vDecreaseTime(time_t xETime)
{
 UINT uSize;
 DRESULT result;	
	
 if (!(xETime % 60))	
 {
  if (ulMachineWorkTime) 
  {
   ulMachineWorkTime--;  
   f_open(&file_object,"0:/RunTime.bin",FA_CREATE_ALWAYS | FA_WRITE);
   result=f_write(&file_object,&ulMachineWorkTime,sizeof(ulMachineWorkTime),&uSize);
   f_close(&file_object);
  }
  else xMachineState=REVISION;
 }
	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void vCardError(void)
{
  memset(LCDData,0x00,sizeof(LCDData));
  sprintf(&LCDData[0][0],LCDlabels[0]);
  sprintf(&LCDData[1][0],LCDlabels[1]);
  vLCDUpdate();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void vSTDPRogram(void)
{
	  switch (xProgramSequence)
	  {
	   case TS:
	   
	   if (xElapTime & 0x01) port_pin_set_output_level(RELAY_2,true);
	    else port_pin_set_output_level(RELAY_2,false);
	   vPrgScreen(100*xElapTime/xProgramList[ucActiveIndex].usTs);
	   
	   if (xElapTime>=xProgramList[ucActiveIndex].usTs) 
	   {
		xProgramSequence=TI; 
		xProgramStartTime=xMasterTime;
	   }
	   
	   break;
	   
	   case TI:
	   
	   port_pin_set_output_level(RELAY_1,true);
	   vPrgScreen(100*xElapTime/xProgramList[ucActiveIndex].usTi);
	   
	   if (xElapTime>=xProgramList[ucActiveIndex].usTi) 
	   {
		xProgramSequence=TFP; 
		xProgramStartTime=xMasterTime;
	   }
	   
	   break;
	   
	   case TFP:
	   
	   port_pin_set_output_level(RELAY_1,false);
	   vPrgScreen(100*xElapTime/xProgramList[ucActiveIndex].usTfp);
	   
	   if (xElapTime>=xProgramList[ucActiveIndex].usTfp) 
	   {
		xProgramSequence=TP; 
		xProgramStartTime=xMasterTime;
	   }
	   
	   break;
		  
	   case TP:
		  
	   port_pin_set_output_level(RELAY_1,true);
	   vWriteLog(xElapTime);
	   vDecreaseTime(xElapTime);
	   vPrgScreen(100*xElapTime/xProgramList[ucActiveIndex].usTp);
	  
	   if (xElapTime>=xProgramList[ucActiveIndex].usTp) 
	   {
		xProgramSequence=TA;
		xProgramStartTime=xMasterTime;
	   }
	  
	   break;
	   
	   case TA:
	   
	   port_pin_set_output_level(RELAY_1,true);
	   vWriteLog(xElapTime);
	   vPrgScreen(100*xElapTime/xProgramList[ucActiveIndex].usTa);
	   
	   if (xElapTime>=xProgramList[ucActiveIndex].usTa) xMachineState=STOP;
	   
	   break;
	   
	   default: break;
	  
	  }
	 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void vWASHPRG(void)
{
	  switch (xProgramSequence)
	  {
	   case TS:
	   
	   if (xElapTime & 0x01) port_pin_set_output_level(RELAY_2,true);
	    else port_pin_set_output_level(RELAY_2,false);
	   vPrgScreen(100*xElapTime/xProgramList[ucActiveIndex].usTs);
	   
	   if (xElapTime>=xProgramList[ucActiveIndex].usTs) 
	   {
		xProgramSequence=TW1; 
		xProgramStartTime=xMasterTime;
	   }
	   
	   break;
	   
	   case TW1:
	   
	   port_pin_set_output_level(RELAY_1,true);
	   vPrgScreen(100*xElapTime/(14*60));
	   
	   if (xElapTime>=(14*60)) 
	   {
		xProgramSequence=TIW; 
		xProgramStartTime=xMasterTime;
	   }
	   
	   break;
	   
	   case TIW:
	   
	   port_pin_set_output_level(RELAY_1,false);
	   vPrgScreen(100*xElapTime/60);
	   
	   if (xElapTime>=60) 
	   {
		xProgramSequence=TW2; 
		xProgramStartTime=xMasterTime;
	   }
	   
	   break;
		  
	   case TW2:
		  
	   port_pin_set_output_level(RELAY_1,true);
	   vWriteLog(xElapTime);
	   vPrgScreen(100*xElapTime/(7*60));
	  
	   if (xElapTime>=(7*60)) 
	   {
		xProgramSequence=TA;
		xProgramStartTime=xMasterTime;
	   }
	  
	   break;
	   
	   case TA:
	   
	   port_pin_set_output_level(RELAY_1,true);
	   vWriteLog(xElapTime);
	   vPrgScreen(100*xElapTime/xProgramList[ucActiveIndex].usTa);
	   
	   if (xElapTime>=xProgramList[ucActiveIndex].usTa) xMachineState=STOP;
	   
	   break;
	   
	   default: break;
	  
	  }
	 	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void main_task(void *params)
{   //uint8_t page_address=0;
	uint16_t counter=0;
	uint8_t tempPrescaler=0;

	memset(LCDData,0x00,sizeof(LCDData));
	
    sprintf(&LCDData[0][0],"   NOCOSYSTEM");
    sprintf(&LCDData[1][0],"    MEDIBIOS");
    vLCDUpdate();

    for (uint8_t ucIndex=0x00;ucIndex<0x14;ucIndex++)
    {
	 if (sd_mmc_test_unit_ready(0)==CTRL_GOOD) break;
	 vTaskDelay(TASK_DELAY_MS(100));
    }
	vTaskDelay(TASK_DELAY_MS(1500));
			
	Retry:
    while (sd_mmc_test_unit_ready(0)!=CTRL_GOOD) 
	{
	  vCardError();
	  vTaskDelay(TASK_DELAY_MS(100));	
	}
	memset(&fs, 0, sizeof(FATFS));
	FRESULT r = f_mount(&fs, "0:", 1);
	if (r==FR_INVALID_DRIVE)
	{
	 while (sd_mmc_check(0)!=CTRL_NO_PRESENT) 
	 {
	  vCardError();
	  
	  vTaskDelay(TASK_DELAY_MS(100));
	 }
	 goto Retry;
	}
		

	vCheckFirmware();
	vOpenFiles();
	
	configure_rtc_calendar();
	vInitRC522SPI();
    vInitRDM6300();
	MFRC522_PCD_Init();

    
    RTCRead();
	
	if (!ulMachineWorkTime) xMachineState=REVISION;
	
	vTaskResume(xRFIDTask);

	while (1)
	{
	 vTaskDelay(TASK_DELAY_MS(50));
	 
	 switch (xMachineState)
	 {
	  case STOP:
	  
	  port_pin_set_output_level(RELAY_1,false);
	  vMainScreen(counter,&Machinetime);
	  if (xCheckPrograms(&ucActiveIndex)) xMachineState=RUN;
	
	  break;
	  
	  case RUN:
	  
	  xElapTime=xMasterTime-xProgramStartTime;
	  
      if (xProgramList[ucActiveIndex].xProgramType==P_WASH) vWASHPRG();
	  else vSTDPRogram();

	  break;
	  
	  case ALARM:
	  
	  break;
	  
	  default: break;	  
     }
	 
	 //
	 tempPrescaler=(tempPrescaler+0x01) % 60;
	 if (!tempPrescaler)
	 {
	  //if (RFIDCheck()) 
	  //{
		//sprintf(&LCDData[0][5],"   %02X%02X%02X%02X",g_uid.uidByte[0],g_uid.uidByte[1],g_uid.uidByte[2],g_uid.uidByte[3]);
		//vLCDUpdate();
		//vTaskDelay(TASK_DELAY_MS(3000));
	  //}
	  //if (xTagRECV)
	  //{
	   //xTagRECV=false;  
	   //sprintf(&LCDData[0][3],"   %02X%02X%02X%02X%02X",rdm_buffer[0],rdm_buffer[1],rdm_buffer[2],rdm_buffer[3],rdm_buffer[4]);
	   //vLCDUpdate();
	    //vTaskDelay(TASK_DELAY_MS(3000));
	  //}
	  vReadTHAll (&T,&H);
	 }
	 
	 vLCDUpdate();
	 rtc_calendar_get_time(&rtc_instance, &Machinetime);
	 timeinfo.tm_year = Machinetime.year - 1900;
	 timeinfo.tm_mon  = Machinetime.month -1;
	 timeinfo.tm_mday = Machinetime.day;
	 timeinfo.tm_hour = Machinetime.hour;
     timeinfo.tm_min  = Machinetime.minute;
	 timeinfo.tm_sec = Machinetime.second; 
	 xMasterTime = mktime(&timeinfo);
	 timeinfTmp = localtime ( &xMasterTime );
	 Machinetime.weekday = timeinfTmp->tm_wday;
	 
	 
	}   

}

///////////////////////////////////////////////////////////////////////////////////////////////////////

