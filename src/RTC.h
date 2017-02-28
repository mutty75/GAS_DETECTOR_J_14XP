/*
 * RTC.h
 *
 * Created: 10/2/2016 13:12:39
 *  Author: ROBERTOC
 */ 


#ifndef RTC_H_
#define RTC_H_

#define RTC_CS PIN_PB10

COMPILER_ALIGNED(8)
typedef struct {
	uint8_t xSec:4,
	        xSec10:3,
			xStart:1;
	uint8_t xMin:4,
	        xMin10:3;
	uint8_t xHour:4,
	        xHour10:2,
			x1224:1,
			xCalSign:1;
	uint8_t xDay:3,
	        xBatEN:1,
			xVBat:1,
			xOSCon:1;			
	uint8_t xDate:4,
	        xDate10:2;	
	uint8_t xMonth:4,
	        xMonth10:1,
			xLeap:1;
	uint8_t xYear:4,
	        xYear10:4;	
	uint8_t x;																							
} xRTCStruct;


void RTCRead(void);
void RTC_WriteRegister(uint8_t, uint8_t );
void RTC_WriteRegisters(xRTCStruct * );

#endif /* RTC_H_ */