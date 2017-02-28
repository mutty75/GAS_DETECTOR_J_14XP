#include "lcdleddriver.h"
#include "string.h"

////#include "RAMBusDriver\rambus.h"
////#include "MainDisplay\MainDisplay.h"
////#include "BoardScan\BoardScan.h"
////#include "Learn\LearnControl.h"
////#include "IOConfig\IOCheck.h"
////#include "ExternalIO\ExtIOHandler.h"
////#include "Learn\Learn.h"
////#include "MsgBox\MsgBox.h"
//#include "math.h"

//#include "compiler.h"
////#include <inavr.h>
//
////#include "ApplicationTask\applTask.h"
//
////#define  DEG (portFLOAT)(3.14159265/180)
////#define  LCD_RESET_TIME  2000
////#define ALARM_BLK_TIME 50;
////#define NXCY_BLK_TIME 50;
//
////unsigned portCHAR ucAlarmBlink;
////unsigned portCHAR ucNxCyBlink;
////unsigned portCHAR ucBlinkStatus;
///

char LCDBuffer[45];
//unsigned char ucLCDRAM[1024];

//////////////////////////////////////////////////////////////////////////////////
//
////void vContrastTrim (unsigned portCHAR ucContrast)
////{
  ////CSSelectLCD;
   ////ucLCDCommand = 0x81;         //Volume Control
   ////ucLCDCommand = ucContrast;   //Contrast 0-63
  ////CSSelectRAM0L;
////}
//
//////////////////////////////////////////////////////////////////////////////////
//
////void vLedInit()
////{unsigned portCHAR ucOutput;
////
////CSSelectRAM0L;               // Update Led
////ucOutput = 0xff;
////CSSelectLED;
////ucLCDData = ucOutput;
////CSSelectRAM0L;
////
////}
//
//////////////////////////////////////////////////////////////////////////////////
//
////void vBackLightPWMInit()
////{
 ////OCR0B = 0x00;
 ////TCCR0A = (1<<COM0B1)|(1<<WGM01)|(1<<WGM00);
 ////TCCR0B = (1<<CS01)|(1<<CS00);
////}
//
//////////////////////////////////////////////////////////////////////////////////
//
////void vBackLightPWMSet(unsigned portCHAR ucValue)
////{
 ////OCR0B = ucValue;
////}
//
//////////////////////////////////////////////////////////////////////////////////
//
////void LCDDisplayInit()
////{   unsigned portSHORT i,j;
////
    ////LCDReset;
////
    ////CSSelectLCD;
    ////ucLCDCommand = 0xe2;         //Software Reset
    ////ucLCDCommand = 0xa3;         //Set LCD Bias
    ////ucLCDCommand = 0xa1;         //Direction Select
    ////ucLCDCommand = 0xc0;         //Direction Select
    ////ucLCDCommand = 0x27;         //Resistor Ratio Set
    ////ucLCDCommand = 0x81;         //Volume Control
    ////ucLCDCommand = xDisplaySetup.ucContrast; //Contrast 0-63
    ////ucLCDCommand = 0x2f;         //Power Supply Mode
    ////ucLCDCommand = 0x40;         //Set Start Line
    ////ucLCDCommand = 0xb0;         //Set Page
    ////ucLCDCommand = 0x10;         //Set Column H
    ////ucLCDCommand = 0x00;         //Set Column L
    ////ucLCDCommand = 0xaf;         //Display ON
    ////ucLCDCommand = 0xa7;         //Normal Mode
    ////ucLCDCommand = 0xa4;         //Data From RAM
////
    ////for (i=0;i<8;i++)
     ////{ CSSelectLCD;
       ////ucLCDCommand = 0xb0 + i;
       ////ucLCDCommand = 0x10;
       ////ucLCDCommand = 0x04;
       ////for (j=0;j<0x80;j++)
        ////{ CSSelectRAM0L;
          ////ucVideoRAMLayer0[i*128+j] = 0xff;
          ////CSSelectLCD;
          ////ucLCDData = 0xff;
        ////}
      ////}
////
    ////CSSelectRAM0L;
    ////vAllLedOFF;
////}
//
//////////////////////////////////////////////////////////////////////////////////
//
////void LoadBTSRLogo()
////{ unsigned portSHORT usIndex;
////
  ////for (usIndex=0;usIndex<1024;usIndex++)
    ////ucVideoRAMLayer0[usIndex] = BTSRLogoBitmap[usIndex];
  ////vAllLedON;
////}
//
//////////////////////////////////////////////////////////////////////////////////
//
////void vClrAllDisplay(xLCDLAYER xLayer)
////{ unsigned portSHORT usIndex;
////
////switch (xLayer)
////{
////
 ////case LAYER0:
////
   ////for (usIndex=0;usIndex<1024;usIndex++)
    ////ucVideoRAMLayer0[usIndex] = 0xff;
 ////break;
////
 ////case LAYER1:
////
   ////for (usIndex=0;usIndex<1024;usIndex++)
    ////{
     ////ucVideoRAMLayer1[usIndex] = 0xff;
     //////ucIntraLayer1[usIndex] = 0xff;
     ////ucIntraLayer1[usIndex] = 0x00;
    ////}
 ////break;
////
 ////case MESSAGE0:
////
   ////for (usIndex=0;usIndex<1024;usIndex++)
    ////{
     ////ucVideoRAMMessage0[usIndex] = 0xff;
     //////ucIntraMessage0[usIndex] = 0xff;
     ////ucIntraMessage0[usIndex] = 0x00;
    ////}
 ////break;
////
 ////case LAYERALL:
////
   ////for (usIndex=0;usIndex<1024;usIndex++)
    ////{
     ////ucVideoRAMLayer0[usIndex] = 0xff;
     ////ucVideoRAMLayer1[usIndex] = 0xff;
     ////ucIntraLayer1[usIndex] = 0x00;
     ////ucVideoRAMMessage0[usIndex] = 0xff;
     ////ucIntraMessage0[usIndex] = 0x00;
    ////}
 ////break;
////}
////
////}
//
//////////////////////////////////////////////////////////////////////////////////
//
////unsigned portCHAR *LayerSelection (xLCDLAYER *xLayer)
////{
////
  ////switch (*xLayer)
////{
////
 ////case LAYER0:
   ////return ucVideoRAMLayer0;
////
 ////case LAYER1:
   ////return ucVideoRAMLayer1;
////
 ////case INTRALAYER1:
   ////return ucIntraLayer1;
////
 ////case MESSAGE0:
   ////return ucVideoRAMMessage0;
////
 ////case INTRAMESSAGE0:
   ////return ucIntraMessage0;
////
////}
////
////return NULL;
////
////}
//
//////////////////////////////////////////////////////////////////////////////////
//
////static portTASK_FUNCTION( vTaskLCD, pvParameters )
////{
////
////unsigned portCHAR ucOutput,ucj,LCDDivider;
////unsigned portSHORT usOffset,usDisplayReset;
////
////LCDDivider = 0x00;
////usDisplayReset = 0x00;
////
////for(;;)
////{   
  ////CSSelectLCD;
  ////
  ////if (!usDisplayReset)
  ////{
    ////ucLCDCommand = 0xe2;         //Software Reset
    ////ucLCDCommand = 0xa3;         //Set LCD Bias
    ////ucLCDCommand = 0xa1;         //Direction Select
    ////ucLCDCommand = 0xc0;         //Direction Select
    ////ucLCDCommand = 0x27;         //Resistor Ratio Set
    ////ucLCDCommand = 0x81;         //Volume Control
    ////ucLCDCommand = xDisplaySetup.ucContrast; //Contrast 0-63
    ////ucLCDCommand = 0x2f;         //Power Supply Mode
    ////ucLCDCommand = 0x40;         //Set Start Line
    ////ucLCDCommand = 0xb0;         //Set Page
    ////ucLCDCommand = 0x10;         //Set Column H
    ////ucLCDCommand = 0x00;         //Set Column L
    ////ucLCDCommand = 0xaf;         //Display ON
    ////ucLCDCommand = 0xa7;         //Normal Mode
    ////ucLCDCommand = 0xa4;         //Data From RAM*/
    ////usDisplayReset = LCD_RESET_TIME;
  ////}
  ////
  ////usDisplayReset--;
  ////ucLCDCommand = 0xb0 + LCDDivider;
  ////ucLCDCommand = 0x10;
  ////ucLCDCommand = 0x04;
  ////ResetCSSelectLCD;
  ////usOffset = (LCDDivider<<7)-1;
  ////for (ucj=0;ucj<0x80;ucj++)
  ////{ 
   ////usOffset++;
   ////CSSelectRAM0L;
   ////ucOutput = (((ucVideoRAMLayer0[usOffset] | ucIntraLayer1[usOffset]) & ucVideoRAMLayer1[usOffset])
              ////| ucIntraMessage0[usOffset]) & ucVideoRAMMessage0[usOffset];
   ////CSSelectLCD;
   ////ucLCDData = ucOutput;
   ////ResetCSSelectLCD;
  ////}
  ////LCDDivider = (LCDDivider + 0x01) & 0x07;
  ////
  ////CSSelectRAM0L;               // Update Led
  ////if (!ucAlarmBlink--) 
  ////{
    ////ucBlinkStatus = (ucBlinkStatus) ? 0x00 : 0xff;
    ////ucAlarmBlink = ALARM_BLK_TIME;
  ////}
  ////
  ////
  ////if (IOCheckMode)
  ////{
    ////*(unsigned portCHAR *)& xIOLedRAM = 0x00;
    ////if (PINE & (1<<4)) *(unsigned portCHAR *)&xIOLedRAM |= (1<<0);
    ////if (xConfigData.xPrx==NC) *(unsigned portCHAR *)&xIOLedRAM ^= (1<<0);
    ////if (PINE & (1<<5)) *(unsigned portCHAR *)&xIOLedRAM |= (1<<1);
    ////if (xConfigData.xZpx==NC) *(unsigned portCHAR *)&xIOLedRAM ^= (1<<1);
    ////if (PINE & (1<<6)) *(unsigned portCHAR *)&xIOLedRAM |= (1<<2);
    ////if (xConfigData.xStc==NC) *(unsigned portCHAR *)&xIOLedRAM ^= (1<<2);
    ////goto LedControlOVR;
  ////}
  ////
  ////if ((xMatrixInAlarm) || (xScanErrorStack.xDeviceStack[0x00].ucDevInError)) *(unsigned portCHAR *)&xIOLedRAM = ucBlinkStatus;
  ////
//////  if (ucStcBlink)
//////  {
//////    ucStcBlink--;
//////    *(unsigned portCHAR *)&xIOLedRAM = *(unsigned portCHAR *)&xIOLedRAM | (1<<(ucStcBlink>>4));
//////  }
  ////
////LedControlOVR:
  ////
  ////ucOutput = ~*(unsigned portCHAR *)&xIOLedRAM;
  ////CSSelectLED;
  ////ucLCDData = ucOutput;
  ////
  /////* Sospensione del task */
  ////vTaskDelay( configLCDLEDTaskTick );
////}
////}
//
//////////////////////////////////////////////////////////////////////
//
//void vStartLCDTask( unsigned portBASE_TYPE uxPriority, void *pvParameters)
//{
  //xTaskCreate( vTaskLCD, ( const signed portCHAR * const ) "LCDDrv",configMINIMAL_STACK_SIZE,NULL, uxPriority, ( xTaskHandle * ) NULL );
//}
//
//////////////////////////////////////////////////////////////////////
//
//void vBox (unsigned portCHAR ucX1, unsigned portCHAR ucY1, unsigned portCHAR ucX2, unsigned portCHAR ucY2, xFILL Fill,
           //xBOXSTYLE xBStyle, xLCDLAYER xLayer)
//{
//unsigned portCHAR ucYBit1,ucYBit2,ucIndex,ucIndexFill,*pucVideoRam;
//unsigned portSHORT usYByte1,usYByte2;
//
//pucVideoRam = LayerSelection(&xLayer);
//
//usYByte1 = (ucY1 / 0x08)<<7;                      //Trova il Byte Colonna di partenza Riga Start
//ucYBit1 = ~(0x01<<(ucY1 % 0x08));            //Trova il Bit  offset nel Byte Colonna Riga Start
//
//usYByte2 = (ucY2 / 0x08)<<7;                      //Trova il Byte Colonna di partenza Riga Stop
//ucYBit2 = ~(0x01<<(ucY2 % 0x08));            //Trova il Bit  offset nel Byte Colonna Riga Stop
//
//if (xBStyle==SMOOTH)
  //for (ucIndex=(ucX1+1);ucIndex<=(ucX2-1);ucIndex++)
  //{
   //pucVideoRam[usYByte1+ucIndex] &= ucYBit1;
   //pucVideoRam[usYByte2+ucIndex] &= ucYBit2;
  //}
  //else
  //for (ucIndex=ucX1;ucIndex<=ucX2;ucIndex++)
  //{
   //pucVideoRam[usYByte1+ucIndex] &= ucYBit1;
   //pucVideoRam[usYByte2+ucIndex] &= ucYBit2;
  //}
//
//
//for (ucIndex=(ucY1+1);ucIndex<(ucY2);ucIndex++)
//{
//usYByte1 = (ucIndex / 0x08)<<7;              //Tracciamento Righe verticali come righe orizzontali
//ucYBit1 = ~(0x01<<(ucIndex % 0x08));       //di 1 Pixel
//
//if (Fill==B)
    //{
     //pucVideoRam[usYByte1+ucX1] &= ucYBit1;   //Senza Riempimento
     //pucVideoRam[usYByte1+ucX2] &= ucYBit1;   //
    //}
    //else
    //{
      //for (ucIndexFill=ucX1;ucIndexFill<=ucX2;ucIndexFill++)  //Con Riempimento
      //pucVideoRam[usYByte1+ucIndexFill] &=  ucYBit1;
    //}
//
//}
//
//}
//
///////////////////////////////////////////////////////////////////////
//
//void vPSet (unsigned portCHAR ucX, unsigned portCHAR ucY, xPOINT Point, xLCDLAYER xLayer)
//{  unsigned portCHAR ucYBit,*pucVideoRam;
   //unsigned portSHORT usYByte;
//
   //pucVideoRam = LayerSelection(&xLayer);
//
   //usYByte = ucX + ((ucY / 0x08)<<7);
   //ucYBit =  ~(0x01<<(ucY % 0x08));
//
   //switch (Point)
   //{
   //case SET:
   //pucVideoRam[usYByte] &= ucYBit;
   //break;
//
   //case RESET:
   //pucVideoRam[usYByte] |= ~ucYBit;
   //break;
//
   //case TOGGLE:
   //pucVideoRam[usYByte] ^= (~ucYBit);
   //break;
   //}
//}
//
///////////////////////////////////////////////////////////////////////
//
//void vCircle (unsigned portCHAR CenterX, unsigned portCHAR CenterY,
              //unsigned portCHAR Radius, portFLOAT Ratio,
              //unsigned portSHORT StartAngle, unsigned portSHORT StopAngle, xLCDLAYER xLayer)
//{unsigned portCHAR ucX,ucY;
 //unsigned portSHORT usDegree;
//
 //for (usDegree=StartAngle;usDegree<=StopAngle;usDegree+=1)
 //{
 //ucY = (unsigned portCHAR)(CenterY + (((portFLOAT)Radius)*sin(0.01745*(portFLOAT)(usDegree))));
 //ucX = (unsigned portCHAR)(CenterX + (((portFLOAT)Radius)*cos(0.01745*(portFLOAT)(usDegree)))*Ratio);
 //vPSet (ucX,ucY,SET,xLayer);
 //}
//
//}
//
///////////////////////////////////////////////////////////////////////
//
///*void CirclePoints (signed portCHAR cX,signed portCHAR cY,
                   //unsigned portCHAR CenterX, unsigned portCHAR CenterY)
//{
 //vPSet (cX+CenterX,CenterY+cY,SET,LAYER0);
 //vPSet (cX+CenterX,CenterY-cY,SET,LAYER0);
//
 //vPSet (CenterX-cX,CenterY+cY,SET,LAYER0);
 //vPSet (CenterX-cX,CenterY-cY,SET,LAYER0);
//
 //vPSet (CenterX+cY,CenterY+cX,SET,LAYER0);
 //vPSet (CenterX+cY,CenterY-cX,SET,LAYER0);
//
 //vPSet (CenterX-cY,CenterY+cX,SET,LAYER0);
 //vPSet (CenterX-cY,CenterY-cX,SET,LAYER0);
//}*/
//
///////////////////////////////////////////////////////////////////////
//
////void vCircleFast (unsigned portCHAR CenterX, unsigned portCHAR CenterY,
////                  unsigned portCHAR Radius, portFLOAT Ratio, xLCDLAYER xLayer, xCIRCLE xCir)
////{unsigned portCHAR usDegree;
//// unsigned portCHAR  ucX,ucY;
//// /*signed portCHAR cY,cD,cX = 0;
////
////
//// cY = Radius;
//// cD = 1 - Radius;
//// CirclePoints (cX,cY,CenterX,CenterY);
//// while (cY > cX)
//// { if (cD < 0)
////        {
////         cD += 2*cX + 3;
////         cX++;
////        }
////       else
////       {
////        cD += 2*(cX-cY) + 5;
////        cX++; cY--;
////       }
////  CirclePoints (cX,cY,CenterX,CenterY);
//// }*/
////
//// switch (xCir)
//// {
////
//// case COMPLETE:
////
//// for (usDegree=0;usDegree<=89;usDegree+=1)
//// {
//// ucY = (unsigned portCHAR)(Radius*sin(0.01745*(usDegree)));
//// ucX = (unsigned portCHAR)(((Radius*cos(0.01745*(usDegree)))*Ratio);
////
//// vPSet (ucX+CenterX,CenterY+ucY,SET,xLayer);
//// vPSet (ucX+CenterX,CenterY-ucY,SET,xLayer);
////
//// vPSet (CenterX-ucX,CenterY+ucY,SET,xLayer);
//// vPSet (CenterX-ucX,CenterY-ucY,SET,xLayer);
////
////
//// //vPSet (CenterX+ucY,CenterY+ucX,SET);
//// //vPSet (CenterX+ucY,CenterY-ucX,SET);
////
//// //vPSet (CenterX-ucY,CenterY+ucX,SET);
//// //vPSet (CenterX-ucY,CenterY-ucX,SET);
////
//// }
////
//// break;
////
//// case UPPERHALF:
////
//// for (usDegree=0;usDegree<=89;usDegree+=1)
//// {
//// ucY = (Radius*sin(0.01745*(usDegree)));
//// ucX = (Radius*cos(0.01745*(usDegree)))*Ratio;
////
//// //vPSet (ucX+CenterX,CenterY+ucY,SET,xLayer);
//// vPSet (ucX+CenterX,CenterY-ucY,SET,xLayer);
////
//// //vPSet (CenterX-ucX,CenterY+ucY,SET,xLayer);
//// vPSet (CenterX-ucX,CenterY-ucY,SET,xLayer);
////
////
//// //vPSet (CenterX+ucY,CenterY+ucX,SET);
//// //vPSet (CenterX+ucY,CenterY-ucX,SET);
////
//// //vPSet (CenterX-ucY,CenterY+ucX,SET);
//// //vPSet (CenterX-ucY,CenterY-ucX,SET);
////
//// }
////
//// break;
////
//// case LOWERHALF:
////
//// for (usDegree=0;usDegree<=89;usDegree+=1)
//// {
//// ucY = (Radius*sin(0.01745*(usDegree)));
//// ucX = (Radius*cos(0.01745*(usDegree)))*Ratio;
////
//// vPSet (ucX+CenterX,CenterY+ucY,SET,xLayer);
//// //vPSet (ucX+CenterX,CenterY-ucY,SET,xLayer);
////
//// vPSet (CenterX-ucX,CenterY+ucY,SET,xLayer);
//// //vPSet (CenterX-ucX,CenterY-ucY,SET,xLayer);
////
////
//// //vPSet (CenterX+ucY,CenterY+ucX,SET);
//// //vPSet (CenterX+ucY,CenterY-ucX,SET);
////
//// //vPSet (CenterX-ucY,CenterY+ucX,SET);
//// //vPSet (CenterX-ucY,CenterY-ucX,SET);
////
//// }
////
//// break;
////
//// }
////
////}
//
//////////////////////////////////////////////////////////////////////
//
//void vHVLine (unsigned portCHAR ucX1, unsigned portCHAR ucY1, unsigned portCHAR ucX2, unsigned portCHAR ucY2, xLCDLAYER xLayer)
//{
//unsigned portCHAR ucYBit1,ucTemp,ucIndex,*pucVideoRam;
//unsigned portSHORT usYByte1;
//
//pucVideoRam = LayerSelection(&xLayer);
//
//if (ucX2<ucX1)
//{
  //ucTemp = ucX2;
  //ucX2 = ucX1;
  //ucX1 = ucTemp;
//}
//
//if (ucY2<ucY1)
//{
  //ucTemp = ucY2;
  //ucY2 = ucY1;
  //ucY1 = ucTemp;
//}
//
//
//if (ucX1==ucX2)                            //Retta verticale
//{
 //for (ucIndex=ucY1;ucIndex<=ucY2;ucIndex++)
 //{
   //usYByte1 = (ucIndex / 0x08)<<7;                 //Tracciamento Righe verticali come righe orizzontali
   //ucYBit1 = ~(0x01<<(ucIndex % 0x08));            //di 1 Pixel
   //pucVideoRam[usYByte1+ucX1] &= ucYBit1;
 //}
 //return;
//}
//
//if (ucY1==ucY2)                            //Retta Orizzontale
//{
 //usYByte1 = (ucY1 / 0x08)<<7;                  //Tracciamento Righe verticali come righe orizzontali
 //ucYBit1 =  ~(0x01<<(ucY1 % 0x08));           //di 1 Pixel
 //for (ucIndex=ucX1;ucIndex<=ucX2;ucIndex++)
  //pucVideoRam[usYByte1+ucIndex] &= ucYBit1;
//}
//
//}
//
//////////////////////////////////////////////////////////////////////
//
//void vVLinePattern (unsigned portCHAR ucX1, unsigned portCHAR ucY1, unsigned portCHAR ucX2, unsigned portCHAR ucY2,
                     //unsigned portSHORT *usPatternLineCounter, unsigned portCHAR ucPattern, xLCDLAYER xLayer)
//{
//unsigned portCHAR ucTempBuffer,ucYBit1,ucIndex,*pucVideoRam;
//unsigned portSHORT usYByte1;
//
//pucVideoRam = LayerSelection(&xLayer);
//
 //if (ucY2<ucY1)
 //{
  //ucTempBuffer=ucY1;
  //ucY1=ucY2;
  //ucY2=ucTempBuffer;
 //}
//
 //switch (ucY2-ucY1)
 //{
  //case 0x00:
  //case 0x01:
//
  //if (ucPattern)
  //{
   //(*usPatternLineCounter)++;
   //if (!(*usPatternLineCounter % ucPattern)) return;
  //}
  //vPSet (ucX1,ucY2,SET,xLayer);
  //return;
//
  //default:
//
  //for (ucIndex=ucY1;ucIndex<=ucY2;ucIndex++)
  //{
   //if (ucPattern)
   //{
    //(*usPatternLineCounter)++;
    //if (!((*usPatternLineCounter) % ucPattern)) continue;
   //}
   //usYByte1 = (ucIndex / 0x08)<<7;                 //Tracciamento Righe verticali come righe orizzontali
   //ucYBit1 = ~(0x01<<(ucIndex % 0x08));            //di 1 Pixel
   //pucVideoRam[usYByte1+ucX1] &= ucYBit1;
  //}
//
 //}
//
//}
//
//
//
//////////////////////////////////////////////////////////////////////
//
//void vLine (unsigned portCHAR ucX1, unsigned portCHAR ucY1, unsigned portCHAR ucX2, unsigned portCHAR ucY2, xLCDLAYER xLayer)
//{
//unsigned portCHAR ucYBit1,ucIndex,*pucVideoRam;
//unsigned portSHORT usYByte1;
//portFLOAT fm,fq;                          // costanti equazione retta y=mx+q
//
//pucVideoRam = LayerSelection(&xLayer);
//
//if (ucX1==ucX2)                            //Retta verticale
//{ if  (ucY2<ucY1)
                  //{
                    //ucIndex = ucY2;
                    //ucY2 = ucY1;
                    //ucY1 = ucIndex;
                  //}
 //for (ucIndex=ucY1;ucIndex<=ucY2;ucIndex++)
 //{
   //usYByte1 = (ucIndex / 0x08)<<7;              //Tracciamento Righe verticali come righe orizzontali
   //ucYBit1 = ~(0x01<<(ucIndex % 0x08));         //di 1 Pixel
   //*(pucVideoRam + usYByte1+ucX1) &= ucYBit1;
 //}
 //return;
//}
//
//fm = (portFLOAT)(ucY2-ucY1)/(portFLOAT)(ucX2-ucX1);
//fq = (portFLOAT)(ucY2) - fm*(portFLOAT)(ucX2);
//
//if ((fm<=1) && (fm>=-1))                           //Coeff. Angolare -1<=m<=1
  //{ if  (ucX2<ucX1)
                  //{
                    //ucIndex = ucX2;
                    //ucX2 = ucX1;
                    //ucX1 = ucIndex;
                  //}
    //for (ucIndex=ucX1;ucIndex<=ucX2;ucIndex++)
    //{
    //ucY1 = (portCHAR)(fm*(portFLOAT)ucIndex+fq);
    //usYByte1 = (ucY1 / 0x08)<<7;
    //ucYBit1 = ~(0x01<<(ucY1 % 0x08));
    //*(pucVideoRam + usYByte1+ucIndex) &= ucYBit1;
    //}
  //}
  //else                                            //Coeff. Angolare m>1 ; m<-1
  //{ if  (ucY2<ucY1)
                  //{
                    //ucIndex = ucY2;
                    //ucY2 = ucY1;
                    //ucY1 = ucIndex;
                  //}
    //for (ucIndex=ucY1;ucIndex<=ucY2;ucIndex++)
    //{
    //ucX1 = (portCHAR)(((portFLOAT)ucIndex-fq)/fm);
    //usYByte1 = (ucIndex / 0x08)<<7;
    //ucYBit1 = ~(0x01<<(ucIndex % 0x08));
    //*(pucVideoRam + usYByte1+ucX1) &= ucYBit1;
    //}
  //}
//}
//
///////////////////////////////////////////////////////////////////////////////////////////////////

unsigned char ucStringLen (char *pcString,Font *FontType)
{unsigned char ucTempLen = 0x00;

while (*pcString)
{
ucTempLen += FontType->IndexTable[*pcString - FontType->StartChar].ucCharWidth;
pcString++;
}
//ucTempLen = (ucBoxLen-ucTempLen)>>1;
return ucTempLen;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

unsigned char ucStringOUTLCD (unsigned char ucX, signed char ucY, unsigned char ucBoxLen,
                    xMODE Reversed, Font *FontType,char *pcString, xLCDLAYER xLayer,xALIGNMENT xAlign)
{unsigned short usString,usOffset,usIndex,usYPrec,usMaskYPrec;
 unsigned char  *pucVideoRam,*pucTmpVideoRamPointer,ucHeight,ucHIndex,ucWidth,ucPosition,ucYShift,ucStartSpace=0x00;
 unsigned char  ReverseMask[8]    = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
 unsigned char  PlacementMask[8]  = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

pucVideoRam = ucLCDRAM;
ucHeight = (FontType->FontHeight-1) / 8;
usString = 0x00;

ucYShift = ucY % 8;
ucY >>= 3;

for (ucHIndex=0x00;ucHIndex<ucHeight;ucHIndex++)
  PlacementMask[ucHIndex] = 0xff;
PlacementMask[ucHeight] = ~(0xff<<((FontType->FontHeight-1) % 8 + 1));
ucHeight++;

if (Reversed==REVERSED)
 for (ucHIndex=0x00;ucHIndex<8;ucHIndex++)
   ReverseMask[ucHIndex] = PlacementMask[ucHIndex];

if (xAlign==CENTER)
{
ucStartSpace = ucStringLen(pcString,FontType);
ucStartSpace = (ucBoxLen>ucStartSpace) ? ((ucBoxLen-ucStartSpace)>>1) : 0x00;
for (usIndex=0x00;usIndex<ucStartSpace;usIndex++)                     //Riempimento box Iniziale
{
 usYPrec = ~(-1<<ucYShift);
 usMaskYPrec  =0x00;
 pucTmpVideoRamPointer = pucVideoRam+usIndex+ucX+((ucY-1)<<7);

 for (ucHIndex=0x00;ucHIndex<ucHeight;ucHIndex++)
 {
  pucTmpVideoRamPointer += (1<<7);
  usMaskYPrec += PlacementMask[ucHIndex]<<ucYShift;
  usYPrec += (0xff ^ ReverseMask[ucHIndex])<<ucYShift;
  *pucTmpVideoRamPointer = (*pucTmpVideoRamPointer | usMaskYPrec) & usYPrec;
  usYPrec = 0x00ff & Swap16 ((signed short)usYPrec);
  usMaskYPrec = 0x00ff & Swap16 ((signed short)usMaskYPrec);
 }

  pucTmpVideoRamPointer += (1<<7);
  usMaskYPrec += PlacementMask[ucHeight]<<ucYShift;
  usYPrec += (0xff ^ ReverseMask[ucHeight])<<ucYShift;
  *pucTmpVideoRamPointer = (*pucTmpVideoRamPointer | usMaskYPrec) & usYPrec;
}
}

ucBoxLen += ucX;
ucX += ucStartSpace;

while (*(pcString+usString))                                    //Caricamento Caratteri
{
ucPosition = *(pcString+usString) - FontType->StartChar;
ucWidth = FontType->IndexTable[ucPosition].ucCharWidth;
usOffset = FontType->IndexTable[ucPosition].usOffset; //Caricare anche il secondo

for (usIndex=0x00;usIndex<ucWidth;usIndex++)
{
 usYPrec = ~(-1<<ucYShift);
 usMaskYPrec  =0x00;
 pucTmpVideoRamPointer = pucVideoRam+usIndex+ucX+((ucY-1)<<7);

 for (ucHIndex=0x00;ucHIndex<ucHeight;ucHIndex++)
 {
  pucTmpVideoRamPointer += (1<<7);
  usMaskYPrec += PlacementMask[ucHIndex]<<ucYShift;
  usYPrec += (FontType->FontData[usIndex+usOffset+ucHIndex*ucWidth] ^ ReverseMask[ucHIndex])<<ucYShift;
  *pucTmpVideoRamPointer = (*pucTmpVideoRamPointer | usMaskYPrec) & usYPrec;
  usYPrec = 0x00ff & Swap16 ((signed short)usYPrec);
  usMaskYPrec = 0x00ff & Swap16 ((signed short)usMaskYPrec);
 }

  pucTmpVideoRamPointer += (1<<7);
  usMaskYPrec += PlacementMask[ucHeight]<<ucYShift;
  usYPrec += (0xff ^ ReverseMask[ucHeight])<<ucYShift;
  *pucTmpVideoRamPointer = (*pucTmpVideoRamPointer | usMaskYPrec) & usYPrec;
}

ucX += ucWidth;
usString++;
}

for (usIndex=ucX;usIndex<ucBoxLen;usIndex++)                     //Riempimento box Finale
{
 usYPrec = ~(-1<<ucYShift);
 usMaskYPrec  =0x00;
 pucTmpVideoRamPointer = pucVideoRam+usIndex+((ucY-1)<<7);

 for (ucHIndex=0x00;ucHIndex<ucHeight;ucHIndex++)
 {
  pucTmpVideoRamPointer += (1<<7);
  usMaskYPrec += PlacementMask[ucHIndex]<<ucYShift;
  usYPrec += (0xff ^ ReverseMask[ucHIndex])<<ucYShift;
  *pucTmpVideoRamPointer = (*pucTmpVideoRamPointer | usMaskYPrec) & usYPrec;
  usYPrec = 0x00ff & Swap16 ((signed short)usYPrec);
  usMaskYPrec = 0x00ff & Swap16 ((signed short)usMaskYPrec);
 }

  pucTmpVideoRamPointer += (1<<7);
  usMaskYPrec += PlacementMask[ucHeight]<<ucYShift;
  usYPrec += (0xff ^ ReverseMask[ucHeight])<<ucYShift;
  *pucTmpVideoRamPointer = (*pucTmpVideoRamPointer | usMaskYPrec) & usYPrec;
}

ucWidth = ucX;
return ucWidth;

}

//////////////////////////////////////////////////////////////////////////////////////////////////
//void vBoxShiftLeft (unsigned portCHAR ucX, signed portCHAR ucY, unsigned portCHAR ucXSize, unsigned portCHAR ucYSyze,
                    //unsigned portCHAR ucXShift, xLCDLAYER xLayer)
//{unsigned portSHORT usIndex;
 //unsigned portCHAR  *pucVideoRam,*pucTmpVideoRamPointer,*pucTmpVideoRamPointer1,ucHeight,ucHIndex,ucYShift,ucTmp;
 //unsigned portCHAR  PlacementMask[8]  = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//
	//pucVideoRam = LayerSelection(&xLayer);
	//ucYShift = ucY % 8;
	//PlacementMask[0]=(0xff<<(ucYShift));	//Mette uno 1 in corrispondenza del box selezionato
	//
	//ucHeight = 1+(ucYSyze-(8-ucYShift)-1) / 8;
    	//
    //for (ucHIndex=1;ucHIndex<ucHeight;ucHIndex++)
  		//PlacementMask[ucHIndex] = 0xff;
	//PlacementMask[ucHeight] = ~(0xff<<((ucYSyze+ucY) & 0x07));
	//ucHeight++;
	//ucY >>= 3;
	//
	//for (usIndex=0;usIndex<(ucXSize-ucXShift);usIndex++)                     //Shift a sinistra
	//{	pucTmpVideoRamPointer = pucVideoRam+usIndex+ucX+((ucY-1)<<7);
	    //for (ucHIndex=0;ucHIndex<ucHeight;ucHIndex++)
		//{	pucTmpVideoRamPointer += (1<<7);
			//pucTmpVideoRamPointer1=pucTmpVideoRamPointer+ucXShift;
			//ucTmp = *pucTmpVideoRamPointer1 & PlacementMask[ucHIndex];
			//*pucTmpVideoRamPointer &= ~PlacementMask[ucHIndex];
			//*pucTmpVideoRamPointer |= ucTmp;
			//*pucTmpVideoRamPointer1 |= PlacementMask[ucHIndex];
		//}
	//}
//}
//

//////////////////////////////////////////////////////////////////////////////////////

//unsigned char ucPrintf (unsigned char, signed char, unsigned char, xMODE, Font *, xLCDLAYER, xALIGNMENT, char *, ...) __attribute__ ((format (gnu_printf, 2, 3)));

unsigned char ucPrintf (unsigned char ucX, signed char ucY, unsigned char ucBoxLen,
             xMODE xReversed, Font *FontType, xLCDLAYER xLayer,xALIGNMENT xAlign,const char *pcString, ...)
{ //portCHAR *LCDBuffer = " ";
  va_list ap;

  va_start(ap,*pcString);
  vsprintf (LCDBuffer,pcString,ap);
  va_end (ap);
  ucStringOUTLCD (ucX,ucY,ucBoxLen,xReversed,FontType,LCDBuffer,xLayer,xAlign);
  return strlen(LCDBuffer);

}

//////////////////////////////////////////////////////////////////////////////////////

void vLoadBMP (unsigned portCHAR ucX, unsigned portCHAR ucY, Bitmap const *xBmp, xMODE Reversed, xLCDLAYER xLayer)
{ unsigned portCHAR ucWidth,ucHeight,ucYShift,ucHIndex,*pucVideoRam,*pucTmpVideoRamPointer;
  unsigned portSHORT usIndex,usYPrec,usMaskYPrec;
  unsigned portCHAR  ReverseMask[8]    = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  unsigned portCHAR  PlacementMask[8]  = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

pucVideoRam = ucLCDRAM;

ucHeight = (xBmp->ucHeight-1) / 8;
ucYShift = ucY % 8;
ucY >>= 3;

for (ucHIndex=0x00;ucHIndex<ucHeight;ucHIndex++)
  PlacementMask[ucHIndex] = 0xff;
PlacementMask[ucHeight] = ~(0xff<<((xBmp->ucHeight-1) % 8 + 1));
ucHeight++;

if (Reversed==REVERSED)
 for (ucHIndex=0x00;ucHIndex<8;ucHIndex++)
   ReverseMask[ucHIndex] = PlacementMask[ucHIndex];

ucWidth = xBmp->ucWidth;                                                //Caricamento BitMap

for (usIndex=0x00;usIndex<ucWidth;usIndex++)
{
 usYPrec = ~(-1<<ucYShift);
 usMaskYPrec  =0x00;
 pucTmpVideoRamPointer = pucVideoRam+usIndex+ucX+((ucY-1)<<7);

 for (ucHIndex=0x00;ucHIndex<ucHeight;ucHIndex++)
 {
  pucTmpVideoRamPointer += (1<<7);
  usMaskYPrec += PlacementMask[ucHIndex]<<ucYShift;
  usYPrec += (xBmp->pucBitmapData[usIndex+ucHIndex*ucWidth] ^ ReverseMask[ucHIndex])<<ucYShift;
  *pucTmpVideoRamPointer = (*pucTmpVideoRamPointer | usMaskYPrec) & usYPrec;
  usYPrec = 0x00ff & Swap16 (usYPrec);
  usMaskYPrec = 0x00ff & Swap16 (usMaskYPrec);
 }

  pucTmpVideoRamPointer += (1<<7);
  usMaskYPrec += PlacementMask[ucHeight]<<ucYShift;
  usYPrec += (0xff ^ ReverseMask[ucHeight])<<ucYShift;
  *pucTmpVideoRamPointer = (*pucTmpVideoRamPointer | usMaskYPrec) & usYPrec;
}

}

///////////////////////////////////////////////////////////////////////////////////////

void vBoxSelect (unsigned char ucX, unsigned char ucY, unsigned char ucSize, unsigned char ucHeight, xLCDLAYER xLayer)
{ unsigned char ucYShift,*pucVideoRam,*pucTmpVideoRamPointer,ucTmpHeight,ucHIndex;
  unsigned short usIndex,usMaskYPrec;
  unsigned char  ReverseMask[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

pucVideoRam = ucLCDRAM;

ucTmpHeight = (ucHeight-1) / 8;

for (ucYShift=0x00;ucYShift<ucTmpHeight;ucYShift++)
  ReverseMask[ucYShift] = 0xff;
ReverseMask[ucTmpHeight] = ~(0xff<<((ucHeight-1) % 8 + 1));
ucTmpHeight++;

ucYShift = ucY % 8;
ucY >>= 3;

for (usIndex=0x00;usIndex<ucSize;usIndex++)
{
 usMaskYPrec  = 0x00;
 pucTmpVideoRamPointer = pucVideoRam+usIndex+ucX+((ucY-1)<<7);

 for (ucHIndex=0x00;ucHIndex<ucTmpHeight;ucHIndex++)
  {
    pucTmpVideoRamPointer += (1<<7);
    usMaskYPrec += ReverseMask[ucHIndex]<<ucYShift;
    *pucTmpVideoRamPointer ^= usMaskYPrec;
    usMaskYPrec = 0x00ff & Swap16(usMaskYPrec);
  }

  pucTmpVideoRamPointer += (1<<7);
  usMaskYPrec += ReverseMask[ucTmpHeight]<<ucYShift;
  *pucTmpVideoRamPointer ^= usMaskYPrec;
}

}

////////////////////////////////////////////////////////////////////////////////////////

void vEraseArea (unsigned char ucX1, unsigned char ucY1, unsigned char ucX2,
                 unsigned char ucY2, xAREA xArea, xLCDLAYER xLayer)
{unsigned char ucHeight,ucYShift,ucHIndex,*pucVideoRam,*pucTmpVideoRamPointer;
  unsigned short usIndex,usMaskYPrec;
  unsigned char  PlacementMask[8]  = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

/*pucVideoRam = LayerSelection(&xLayer);

ucHeight = ucY2-ucY1+1;
usIndex = ucHeight - 1;
ucHeight = ((ucHeight % 8)==0x00) ? (ucHeight / 8) : (ucHeight / 8) + 1;

for (ucHIndex=0x00;ucHIndex<(ucHeight-1);ucHIndex++)
  PlacementMask[ucHIndex] = 0xff;
PlacementMask[ucHeight-1] = ~(0xff<<(ucY2-ucY1+1 - ((ucHeight-1)<<3)));

ucYShift = ucY1 % 8;
ucY1 >>= 3;

if (xArea==BORDER)
{
  usMaskYPrec  =0x00;
  pucTmpVideoRamPointer = pucVideoRam+ucX1+((ucY1-1)<<7);

  for (ucHIndex=0x00;ucHIndex<ucHeight;ucHIndex++)
  {
    pucTmpVideoRamPointer += (1<<7);
    usMaskYPrec += PlacementMask[ucHIndex]<<ucYShift;
    *pucTmpVideoRamPointer |= usMaskYPrec;
    usMaskYPrec = 0x00ff & __reverse (usMaskYPrec);
  }

  pucTmpVideoRamPointer += (1<<7);
  usMaskYPrec += PlacementMask[ucHeight]<<ucYShift;
  *pucTmpVideoRamPointer |= usMaskYPrec;

  usMaskYPrec  =0x00;
  pucTmpVideoRamPointer = pucVideoRam+ucX2+((ucY1-1)<<7);

  for (ucHIndex=0x00;ucHIndex<ucHeight;ucHIndex++)
  {
    pucTmpVideoRamPointer += (1<<7);
    usMaskYPrec += PlacementMask[ucHIndex]<<ucYShift;
    *pucTmpVideoRamPointer |= usMaskYPrec;
    usMaskYPrec = 0x00ff & __reverse (usMaskYPrec);
  }

  pucTmpVideoRamPointer += (1<<7);
  usMaskYPrec += PlacementMask[ucHeight]<<ucYShift;
  *pucTmpVideoRamPointer |= usMaskYPrec;

  ucX1++;
  ucX2--;

  *(unsigned portLONG *)&PlacementMask[0] = 0x00000000;
  *(unsigned portLONG *)&PlacementMask[4] = 0x00000000;

  PlacementMask[0] = 0x01;
  PlacementMask[ucHeight-1] |= 0x01<<(usIndex - ((ucHeight-1)<<3));

}

for (usIndex=ucX1;usIndex<=ucX2;usIndex++)
{
  usMaskYPrec  =0x00;
  pucTmpVideoRamPointer = pucVideoRam+usIndex+((ucY1-1)<<7);

  for (ucHIndex=0x00;ucHIndex<ucHeight;ucHIndex++)
  {
    pucTmpVideoRamPointer += (1<<7);
    usMaskYPrec += PlacementMask[ucHIndex]<<ucYShift;
    *pucTmpVideoRamPointer |= usMaskYPrec;
    usMaskYPrec = 0x00ff & __reverse (usMaskYPrec);
  }

  pucTmpVideoRamPointer += (1<<7);
  usMaskYPrec += PlacementMask[ucHeight]<<ucYShift;
  *pucTmpVideoRamPointer |= usMaskYPrec;
}*/

pucVideoRam = ucLCDRAM;

ucHeight = ucY2-ucY1+1;
usIndex = ucHeight - 1;
ucHeight = (ucHeight-1) / 8;

for (ucHIndex=0x00;ucHIndex<ucHeight;ucHIndex++)
  PlacementMask[ucHIndex] = 0xff;
PlacementMask[ucHeight] = ~(0xff<<((ucY2-ucY1) % 8 + 1));
ucHeight++;

ucYShift = ucY1 % 8;
ucY1 >>= 3;

if (xArea==BORDER)
{
  usMaskYPrec  =0x00;
  pucTmpVideoRamPointer = pucVideoRam+ucX1+((ucY1-1)<<7);

  for (ucHIndex=0x00;ucHIndex<ucHeight;ucHIndex++)
  {
    pucTmpVideoRamPointer += (1<<7);
    usMaskYPrec += PlacementMask[ucHIndex]<<ucYShift;
    *pucTmpVideoRamPointer |= usMaskYPrec;
    usMaskYPrec = 0x00ff & Swap16 (usMaskYPrec);
  }

  pucTmpVideoRamPointer += (1<<7);
  usMaskYPrec += PlacementMask[ucHeight]<<ucYShift;
  *pucTmpVideoRamPointer |= usMaskYPrec;

  usMaskYPrec  =0x00;
  pucTmpVideoRamPointer = pucVideoRam+ucX2+((ucY1-1)<<7);

  for (ucHIndex=0x00;ucHIndex<ucHeight;ucHIndex++)
  {
    pucTmpVideoRamPointer += (1<<7);
    usMaskYPrec += PlacementMask[ucHIndex]<<ucYShift;
    *pucTmpVideoRamPointer |= usMaskYPrec;
    usMaskYPrec = 0x00ff & Swap16 (usMaskYPrec);
  }

  pucTmpVideoRamPointer += (1<<7);
  usMaskYPrec += PlacementMask[ucHeight]<<ucYShift;
  *pucTmpVideoRamPointer |= usMaskYPrec;

  ucX1++;
  ucX2--;

  //*(uint32_t *)&PlacementMask[0] = 0x00000000;
  //*(uint32_t *)&PlacementMask[4] = 0x00000000;
  memset(PlacementMask,0x00,sizeof(PlacementMask));

  PlacementMask[0] = 0x01;
  PlacementMask[ucHeight-1] |= 0x01<<(usIndex - ((ucHeight-1)<<3));

}

for (usIndex=ucX1;usIndex<=ucX2;usIndex++)
{
  usMaskYPrec  =0x00;
  pucTmpVideoRamPointer = pucVideoRam+usIndex+((ucY1-1)<<7);

  for (ucHIndex=0x00;ucHIndex<ucHeight;ucHIndex++)
  {
    pucTmpVideoRamPointer += (1<<7);
    usMaskYPrec += PlacementMask[ucHIndex]<<ucYShift;
    *pucTmpVideoRamPointer |= usMaskYPrec;
    usMaskYPrec = 0x00ff & Swap16 (usMaskYPrec);
  }

  pucTmpVideoRamPointer += (1<<7);
  usMaskYPrec += PlacementMask[ucHeight]<<ucYShift;
  *pucTmpVideoRamPointer |= usMaskYPrec;
}


}

////////////////////////////////////////////////////////////////////////////////////////
//
//void HistogramTest (unsigned portCHAR ucX, unsigned portCHAR ucY, unsigned portCHAR ucXSize,
                    //unsigned portCHAR ucYSize, unsigned portCHAR ucXDepth, unsigned portCHAR ucYDepth,
                    //unsigned portCHAR ucHeight)
//{ unsigned portCHAR ucIndex;
//
///*
      //vBox (0,15,10,60,B,SQUARE,LAYER0);
      //vBox (2,35,8,58,BF,SQUARE,LAYER0);
      //vHVLine (20,10,20,55,LAYER0);
      //vHVLine (10,10,20,10,LAYER0);
      //vLine (0,15,10,10,LAYER0);
      //vLine (10,15,20,10,LAYER0);
      //vLine (10,60,20,55,LAYER0);
//
      //for (usIndex=29;usIndex<52;usIndex++)
        //vLine (12,6+usIndex,18,3+usIndex,LAYER0);
//
      //vBox (K,25,10+K,60,B,SQUARE,LAYER0);
      //vBox (2+K,45,8+K,58,BF,SQUARE,LAYER0);
      //vHVLine (20+K,20,20+K,55,LAYER0);
      //vHVLine (10+K,20,20+K,20,LAYER0);
      //vLine (0+K,25,10+K,20,LAYER0);
      //vLine (10+K,25,20+K,20,LAYER0);
      //vLine (10+K,60,20+K,55,LAYER0);
//
      //for (usIndex=39;usIndex<52;usIndex++)
        //vLine (12+K,6+usIndex,18+K,3+usIndex,LAYER0);
//
//
      //vBox (2*K,5,10+2*K,60,B,SQUARE,LAYER0);
      //vBox (2+2*K,15,8+2*K,58,BF,SQUARE,LAYER0);
      //vHVLine (20+2*K,0,20+2*K,55,LAYER0);
      //vHVLine (10+2*K,0,20+2*K,0,LAYER0);
      //vLine (0+2*K,5,10+2*K,0,LAYER0);
      //vLine (10+2*K,5,20+2*K,0,LAYER0);
      //vLine (10+2*K,60,20+2*K,55,LAYER0);
//
      //for (usIndex=9;usIndex<52;usIndex++)
        //vLine (12+2*K,6+usIndex,18+2*K,3+usIndex,LAYER0);
//
//
      //vBox (73,3,125,19,B,SMOOTH,LAYER0);
      //vBox (73,23,125,39,B,SMOOTH,LAYER0);
      //vBox (73,43,125,59,B,SMOOTH,LAYER0);      */
//
//
      //vBox (ucX,ucY,ucX+ucXSize,ucY+ucYSize,B,SQUARE,LAYER0);                           //Box Esterno
      ////vBox (ucX+2,ucY+ucYSize-ucHeight,ucX+ucXSize-2,ucY+ucYSize-2,BF,SQUARE,LAYER0);   //Box Interno
      //vBox (ucX,ucY+ucYSize-ucHeight,ucX+ucXSize,ucY+ucYSize,BF,SQUARE,LAYER0);   //Box Interno
//
      //for (ucIndex=2;ucIndex<ucHeight;ucIndex++)
        //vLine (ucX+ucXSize,ucY+ucYSize-ucIndex,
               //ucX+ucXSize+ucXDepth,ucY+ucYSize-ucYDepth-ucIndex,LAYER0);
//
      ////vEraseArea (ucX+ucXSize+1,0,ucX+ucXSize+1,63,LAYER0);
      ////vEraseArea (ucX+ucXSize+ucXDepth-1,0,ucX+ucXSize+ucXDepth-1,63,LAYER0);
//
      //vHVLine (ucX+ucXSize+ucXDepth,ucY-ucYDepth,ucX+ucXSize+ucXDepth,ucY-ucYDepth+ucYSize,LAYER0);  //Ombra Verticale
      //vHVLine (ucX+ucXDepth,ucY-ucYDepth,ucX+ucXSize+ucXDepth,ucY-ucYDepth,LAYER0);  //Ombra Orizzontale
//
      //vLine (ucX,ucY,ucX+ucXDepth,ucY-ucYDepth,LAYER0);
      //vLine (ucX+ucXSize,ucY,ucX+ucXSize+ucXDepth,ucY-ucYDepth,LAYER0);
      //vLine (ucX+ucXSize,ucY+ucYSize,ucX+ucXSize+ucXDepth,ucY+ucYSize-ucYDepth,LAYER0);
//
      //vBox (73,3,125,19,B,SMOOTH,LAYER0);
      //vBox (73,23,125,39,B,SMOOTH,LAYER0);
      //vBox (73,43,125,59,B,SMOOTH,LAYER0);
//
      //ucPrintf (75,5,30,REVERSED,&mssans,LAYER0,LEFT,"Eff.A %d%% ",50);
      //ucPrintf (75,25,30,NORMAL,&mssans,LAYER0,LEFT,"Eff.B %d%% ",12);
      //ucPrintf (75,45,30,REVERSED,&mssans,LAYER0,LEFT,"Eff.C %d%% ",90);
//}
//
////////////////////////////////////////////////////////////////////////////////////////
//
//
