/**
 * \file
 *
 * \brief FreeRTOS demo tasks header.
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

#ifndef DEMOTASKS_H
#define DEMOTASKS_H

#include "time.h"

#define MAX_TAGS 20
#define MAX_PRG 10
#define MAX_LANGUAGE 10
#define MAX_LOGS 100
#define MAX_REC_PAGE 50
#define MAXLSIZE 32
#define PRGDELTATIME 10

/**
 * \defgroup freertos_sam0_demo_tasks_group FreeRTOS demo tasks
 *
 * The demo tasks demonstrate basic use of FreeRTOS, with inter-task
 * communication using queues and mutexes.
 *
 * For details on how the demo works, see \ref appdoc_intro.
 *
 * For detailed information on the tasks, see:
 * - \ref main_task()
 * - \ref graph_task()
 * - \ref terminal_task()
 * - \ref about_task()
 * - \ref uart_task()
 *
 * The demo tasks depend on the following drivers:
 * - \ref oled1_xpro_io_group
 * - \ref edbg_cdc_rx_group
 * - \ref asfdoc_common2_gfx_mono
 *
 * @{
 */

typedef enum  
{
 P_STD=0x00,
 P_WEEK=0x01,
 P_WASH=0x02,
 P_DAY=0x03,	
 P_OFF=0xFF,
} xPType;

typedef enum {
	STOP,
	RUN,
	ALARM,
	REVISION,
} xMacState;

typedef enum {
	ALM_1,
	ALM_2,
	ALM_3,
} xAlm;

typedef enum {
	TS,
	TI,
	TFP,
	TP,
	TA,
	TIW,
	TW1,
	TW2
} xPRGSeq;
 
COMPILER_PACK_SET(1)
typedef struct {
	uint8_t		xProgramType;
	char		cPrgName[16];
	time_t		xStartTime;
	uint16_t	usTs;
	uint16_t	usTi; 
	uint16_t	usTfp;
	uint16_t	usTp;
	uint16_t	usTr;
	uint16_t	usTa;
	uint16_t	usTlp;
	uint8_t		ucTempMin;
	uint8_t		ucTempMax;
	uint8_t		ucHMin;
	uint8_t		ucHMax;	
	uint8_t		ucIMin;
	uint8_t		ucIMax;	
	uint8_t		ucSpare[16];
	} xProgram;
	
typedef struct {
	uint8_t		ucIPAddr[4];
	uint8_t		ucWEPKey[5];
	uint16_t	usRevTime;
	time_t		xDateTime;
	uint8_t		ucLanguage;
	uint32_t	ulPump1;
	uint32_t	ulPump2;
	uint8_t		ucSpare[20];
} xSetup;

typedef struct {
 uint8_t ucPrgIndex;
 time_t xDateTime;
 uint8_t ucLogPages;
 uint8_t ucSpare[4];
} xLogTb;

typedef struct {
	time_t xDateTime;
	xMacState xMachineState;
	uint16_t usTemp;
	uint16_t usH;
	uint16_t usWRKTime;
	uint8_t ucSpare[5];
} xLogData;

typedef struct {
	uint8_t ucTag[7];
	uint16_t usml;
} xTagData;

COMPILER_PACK_RESET()

bool xSDDet;
xProgram xProgramList[MAX_PRG];
xLogTb xLogTable[MAX_LOGS];
xSetup xMachineSetup;
xMacState xMachineState;
xAlm xAlarm;
xPRGSeq xProgramSequence;
xTagData xTags[MAX_TAGS];
uint8_t ucLogPos;

time_t xMasterTime;
time_t xProgramStartTime;
time_t xElapTime;

uint32_t ulProgramRunTime;
uint32_t ulMachineWorkTime;
uint32_t ulProgramTimeToEnd;

struct tm timeinfo;
struct rtc_calendar_time Machinetime;

FATFS fs;
FIL file_object;
//char test_file_name[] = "0:quick.txt";

char TempString[25];
extern char LCDlabels[30][MAXLSIZE];

FRESULT res;
FILINFO fno;
DIR dir;
#if _USE_LFN
    //static char lfn[_MAX_LFN + 1];   /* Buffer to store the LFN */
    ////fno.lfname = lfn;
    ////fno.lfsize = sizeof lfn;
#endif

extern struct rtc_module rtc_instance;



void tasks_init(void);
void usart_write_callback(struct usart_module * );
void usart_read_callback(struct usart_module * );
void tc_callback_timeout(struct tc_module * );

void usart_break_callback(struct usart_module * );
void usart_start_callback(struct usart_module * );
void usart_received_callback(struct usart_module * );


/** @} */

#endif // DEMO_TASKS_H
