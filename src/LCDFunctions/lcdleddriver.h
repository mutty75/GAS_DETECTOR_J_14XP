#include "Arial.h"
#include "NumBig.h"
#include "small.h"
#include "Bitmap.h"

#include "stdarg.h"
#include "stdio.h"
#include "st7565r.h"

///////////////////////////////////////////////////
//// LED MACROS
///////////////////////////////////////////////////
//
//#define vAllLedON    *(unsigned portCHAR *)&xIOLedRAM = 0xff;
//#define vAllLedOFF   *(unsigned portCHAR *)&xIOLedRAM = 0x00;

/////////////////////////////////////////////////

#ifndef LCD_DRV
#define LCD_DRV

typedef enum { B,BF } xFILL;
typedef enum { SET,RESET,TOGGLE } xPOINT;
typedef enum { SMOOTH,SQUARE } xBOXSTYLE;
typedef enum { LAYER0,LAYER1,INTRALAYER1,INTRAMESSAGE0,MESSAGE0,LAYERALL } xLCDLAYER;
typedef enum { NORMAL = 0x00, REVERSED = 0xFF } xMODE;
typedef enum { FULL,BORDER } xAREA;
typedef enum { CAPITAL,SMALL,SYMBOL } xCHARSET;
typedef enum { COMPLETE,UPPERHALF,LOWERHALF } xCIRCLE;
typedef enum { LEFT,CENTER,RIGHT } xALIGNMENT;

#endif

//////////////////////////////////////////////////

//extern unsigned char ucLCDRAM[1024];

//////////////////////////////////////////////////

/**
 * \internal
 * \brief String Print
 */
unsigned char ucPrintf (unsigned char, signed char, unsigned char, xMODE, Font *, xLCDLAYER, xALIGNMENT,const char *, ...) __attribute__ ((format(gnu_printf, 8, 9)));
unsigned char ucStringOUTLCD (unsigned char , signed char , unsigned char , xMODE , Font *, char *, xLCDLAYER, xALIGNMENT );
unsigned char ucStringLen (char *, Font * );
void vBoxSelect (unsigned char, unsigned char, unsigned char, unsigned char, xLCDLAYER );
void vEraseArea (unsigned char, unsigned char, unsigned char, unsigned char, xAREA, xLCDLAYER );
void vLoadBMP (unsigned portCHAR, unsigned portCHAR, Bitmap const *, xMODE, xLCDLAYER);

//void vStringScrollLCD (unsigned portCHAR , signed portCHAR, unsigned portCHAR, xMODE, Font __hugeflash *,portCHAR *, xLCDLAYER ,xALIGNMENT );
//void vCircle (unsigned portCHAR, unsigned portCHAR, unsigned portCHAR,
              //portFLOAT, unsigned portSHORT, unsigned portSHORT, xLCDLAYER);
//void vCircleFast (unsigned portCHAR, unsigned portCHAR, unsigned portCHAR,portFLOAT, xLCDLAYER,xCIRCLE);
//void vBox  (unsigned portCHAR, unsigned portCHAR, unsigned portCHAR, unsigned portCHAR, xFILL, xBOXSTYLE, xLCDLAYER);
//void vHVLine (unsigned portCHAR, unsigned portCHAR, unsigned portCHAR, unsigned portCHAR, xLCDLAYER);
//void vVLinePattern (unsigned portCHAR, unsigned portCHAR, unsigned portCHAR, unsigned portCHAR,
                    //unsigned portSHORT *, unsigned portCHAR, xLCDLAYER);
//void vLine (unsigned portCHAR, unsigned portCHAR, unsigned portCHAR, unsigned portCHAR, xLCDLAYER);
//void vPSet (unsigned portCHAR, unsigned portCHAR, xPOINT, xLCDLAYER);

//void vBoxSelect (unsigned portCHAR, unsigned portCHAR, unsigned portCHAR, unsigned portCHAR, xLCDLAYER );
//void vEraseArea (unsigned portCHAR, unsigned portCHAR, unsigned portCHAR, unsigned portCHAR, xAREA, xLCDLAYER);
//void HistogramTest (unsigned portCHAR ucX, unsigned portCHAR ucY, unsigned portCHAR ucXSize,
                    //unsigned portCHAR ucYSize, unsigned portCHAR ucXDepth, unsigned portCHAR ucYDepth,
                    //unsigned portCHAR ucHeight);

//unsigned portCHAR ucStringLen (portCHAR *,Font __hugeflash *);
//void vBackLightPWMInit();
//void vBackLightPWMSet(unsigned portCHAR );

/////////////////////////////////////////////////

//void LCDDisplayInit();
//void vContrastTrim (unsigned portCHAR );
//void LoadBTSRLogo();
//void vLedInit();
//void vClrAllDisplay(xLCDLAYER);

//void vStartLCDTask( unsigned portBASE_TYPE uxPriority, void *);

/////////////////////////////////////////////////
