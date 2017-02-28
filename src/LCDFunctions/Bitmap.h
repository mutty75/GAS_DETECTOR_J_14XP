#include "FreeRTOS.h"

#ifndef BMP_H
#define BMP_H

typedef struct
{ unsigned portCHAR ucWidth;
  unsigned portCHAR ucHeight;
  unsigned portCHAR const *pucBitmapData;
} Bitmap;

#endif

//extern __hugeflash unsigned char BTSRLogoBitmap[];
//
//extern __hugeflash Bitmap Stop;
//extern __hugeflash unsigned char StopData[];
//
//extern __hugeflash Bitmap x;
//extern __hugeflash unsigned char xData[];
//
//extern __hugeflash Bitmap arrowdown;
//extern __hugeflash unsigned char arrowdownData[];
//
//extern __hugeflash Bitmap arrowup;
//extern __hugeflash unsigned char arrowupData[];
//
//extern __hugeflash Bitmap arrowleft;
//extern __hugeflash unsigned char arrowleftData[];
//
//extern __hugeflash Bitmap arrowright;
//extern __hugeflash unsigned char arrowrightData[];
//
//extern __hugeflash Bitmap warn;
//extern __hugeflash unsigned char warnData[];
//
//extern __hugeflash Bitmap glasswatch;
//extern __hugeflash unsigned char glasswatchData[];
//
//extern __hugeflash Bitmap arrowup_big;
//extern __hugeflash unsigned char arrowup_bigData[];
//
//extern __hugeflash Bitmap arrowdown_big;
//extern __hugeflash unsigned char arrowdown_bigData[];
//
//extern __hugeflash Bitmap enter;
//extern __hugeflash unsigned char enterData[];
//
//extern __hugeflash Bitmap b123;
//extern __hugeflash unsigned char b123Data[];
//
//extern __hugeflash Bitmap abc;
//extern __hugeflash unsigned char abcData[];
//
//extern __hugeflash Bitmap CapsABC;
//extern __hugeflash unsigned char CapsABCData[];
//
//extern __hugeflash Bitmap arrowleft;
//extern __hugeflash unsigned char arrowleftData[];
//
//extern __hugeflash Bitmap StartFeed;
//extern __hugeflash unsigned char StartFeedData[];
//
//extern __hugeflash Bitmap ProvaWrench;
//extern __hugeflash unsigned char ProvaWrenchData[];
//
//extern __hugeflash Bitmap brick;
//extern __hugeflash unsigned char brickData[];
//
//extern __hugeflash Bitmap brick1;
//extern __hugeflash unsigned char brick1Data[];
//
//extern __hugeflash Bitmap Istog;
//extern __hugeflash unsigned char IstogData[];
//
//extern __hugeflash Bitmap Graph;
//extern __hugeflash unsigned char GraphData[];
//
//extern __hugeflash Bitmap counters;
//extern __hugeflash unsigned char countersData[];
//
//extern __hugeflash Bitmap learn;
//extern __hugeflash unsigned char learnData[];
//
//extern __hugeflash Bitmap learn1;
//extern __hugeflash unsigned char learn1Data[];
//
//extern __hugeflash Bitmap reset;
//extern __hugeflash unsigned char resetData[];
//
//extern __hugeflash Bitmap nextcy ;
//extern __hugeflash unsigned char nextcyData[];
//
//extern __hugeflash Bitmap smdin;
//extern __hugeflash unsigned char smdinData[];
//
//extern __hugeflash Bitmap scrolldown;
//extern __hugeflash unsigned char scrolldownData[];
//
//extern __hugeflash Bitmap arrowsmallleft;
//extern __hugeflash unsigned char arrowsmallleftData[];
//
//extern __hugeflash Bitmap arrowsmallright;
//extern __hugeflash unsigned char arrowsmallrightData[];
//
//extern __hugeflash Bitmap exitdoor;
//extern __hugeflash unsigned char exitdoorData[];
//
//extern __hugeflash Bitmap Floppy;
//extern __hugeflash unsigned char FloppyData[];
//
//extern __hugeflash Bitmap FrecciaMsg;
//extern __hugeflash unsigned char FrecciaMsgData[];
//
//extern __hugeflash Bitmap Creel;
//extern __hugeflash unsigned char CreelData[];
//
//extern __hugeflash Bitmap Identify;
//extern __hugeflash unsigned char IdentifyData[];
//
//extern __hugeflash Bitmap IdentifyMap;
//extern __hugeflash unsigned char IdentifyMapData[];
//
//extern __hugeflash Bitmap updown;
//extern __hugeflash unsigned char updownData[];
//
//extern __hugeflash Bitmap leftright;
//extern __hugeflash unsigned char leftrightData[];
//
//extern __hugeflash Bitmap Attention;
//extern __hugeflash unsigned char AttentionData[];
//
//extern __hugeflash Bitmap updown1;
//extern __hugeflash unsigned char updown1Data[];
//
//extern __hugeflash Bitmap CreelStd;
//extern __hugeflash unsigned char CreelStdData[];
//
//extern __hugeflash Bitmap CreelAdv;
//extern __hugeflash unsigned char CreelAdvData[];
//
//extern __hugeflash Bitmap Logo;
//extern __hugeflash unsigned char LogoData[];
//
//extern __hugeflash Bitmap StyleTick;
//extern __hugeflash unsigned char StyleTickData[];
//
//extern __hugeflash Bitmap thickness1;
//extern __hugeflash unsigned char thickness1Data[];
//
//extern __hugeflash Bitmap thickness2;
//extern __hugeflash unsigned char thickness2Data[];
//
//extern __hugeflash Bitmap thickness3;
//extern __hugeflash unsigned char thickness3Data[];
//
//extern __hugeflash Bitmap MatrixOutLine;
//extern __hugeflash unsigned char MatrixOutLineData[];
//
//extern __hugeflash Bitmap ArrowMain;
//extern __hugeflash unsigned char ArrowMainData[];
//
//extern __hugeflash Bitmap ArrowEmpty;
//extern __hugeflash unsigned char ArrowEmptyData[];
//
//extern __hugeflash Bitmap Load;
//extern __hugeflash unsigned char LoadData[];
//
//extern __hugeflash Bitmap Contr;
//extern __hugeflash unsigned char ContrData[];
//
//extern __hugeflash Bitmap Bright;
//extern __hugeflash unsigned char BrightData[];
//
//extern __hugeflash Bitmap CheckedBox;
//extern __hugeflash unsigned char CheckedBoxData[];
//
//extern __hugeflash Bitmap FreeBox;
//extern __hugeflash unsigned char FreeBoxData[];
//
//extern __hugeflash Bitmap CreelIS3;
//extern __hugeflash unsigned char CreelIS3Data[];
//
//extern __hugeflash Bitmap TS5Bmp;
//extern __hugeflash unsigned char TS5Data[];
//
//extern __hugeflash Bitmap CreelTS;
//extern __hugeflash unsigned char CreelTSData[];
//
//extern __hugeflash Bitmap AddNew1;
//extern __hugeflash unsigned char AddNew1Data[];
//
//extern __hugeflash Bitmap Erase;
//extern __hugeflash unsigned char EraseData[];
//
//extern __hugeflash Bitmap Edit;
//extern __hugeflash unsigned char EditData[];
//
//extern __hugeflash Bitmap Lock;
//extern __hugeflash unsigned char LockData[];
//
//extern __hugeflash Bitmap Weight;
//extern __hugeflash unsigned char WeightData[];
//
//extern __hugeflash Bitmap Flash;
//extern __hugeflash unsigned char FlashData[];
//
//extern __hugeflash Bitmap Creel2;
//extern __hugeflash unsigned char Creel2Data[];
//
//extern __hugeflash Bitmap arrowsmallleft2;
//extern __hugeflash unsigned char arrowsmallleft2Data[];
//
//extern __hugeflash Bitmap arrowsmallright2;
//extern __hugeflash unsigned char arrowsmallright2Data[];
//
//extern __hugeflash Bitmap ClrCount;
//extern __hugeflash unsigned char ClrCountData[];
//
//extern __hugeflash Bitmap Details;
//extern __hugeflash unsigned char DetailsData[];
//
//extern __hugeflash Bitmap Menuup;
//extern __hugeflash unsigned char MenuupData[];
//
//extern __hugeflash Bitmap Eff;
//extern __hugeflash unsigned char EffData[];
//
//extern __hugeflash Bitmap UltraFeeder;
//extern __hugeflash unsigned char UltraFeederData[];
//
//extern __hugeflash Bitmap Bar;
//extern __hugeflash unsigned char BarData[];
//
//extern __hugeflash Bitmap arrowsplit;
//extern __hugeflash unsigned char arrowsplitData[];
//
//extern __hugeflash Bitmap arrowsplitright;
//extern __hugeflash unsigned char arrowsplitrightData[];
//
//extern __hugeflash Bitmap UltraSmall;
//extern __hugeflash unsigned char UltraSmallData[];
//
//extern __hugeflash Bitmap Modify;
//extern __hugeflash unsigned char ModifyData[];
//
//extern __hugeflash Bitmap map;
//extern __hugeflash unsigned char mapData[];
//
//extern __hugeflash Bitmap unload;
//extern __hugeflash unsigned char unloadData[];
//
//extern __hugeflash Bitmap map1;
//extern __hugeflash unsigned char map1Data[];
//
//extern __hugeflash Bitmap map2;
//extern __hugeflash unsigned char map2Data[];
//
//extern __hugeflash Bitmap Floppy3;
//extern __hugeflash unsigned char Floppy3Data[];
//
//extern __hugeflash Bitmap ZoomMeno;
//extern __hugeflash unsigned char ZoomMenoData[];
//
//extern __hugeflash Bitmap ZoomPiu;
//extern __hugeflash unsigned char ZoomPiuData[];
//
//extern __hugeflash Bitmap Setup;
//extern __hugeflash unsigned char SetupData[];
//
//extern __hugeflash Bitmap Mark;
//extern __hugeflash unsigned char MarkData[];
//
//extern __hugeflash Bitmap Event;
//extern __hugeflash unsigned char EventData[];
//
//extern __hugeflash Bitmap Mark1;
//extern __hugeflash unsigned char Mark1Data[];
//
//extern __hugeflash Bitmap Mark2;
//extern __hugeflash unsigned char Mark2Data[];
//
//extern __hugeflash Bitmap ArrowLFill;
//extern __hugeflash unsigned char ArrowLFillData[];
//
//extern __hugeflash Bitmap ArrowLEmpty;
//extern __hugeflash unsigned char ArrowLEmptyData[];
//
//extern __hugeflash Bitmap ArrowREmpty;
//extern __hugeflash unsigned char ArrowREmptyData[];
//
//extern __hugeflash Bitmap ArrowRFill;
//extern __hugeflash unsigned char ArrowRFillData[];
//
//extern __hugeflash Bitmap sveglia;
//extern __hugeflash unsigned char svegliaData[];
//
//extern __hugeflash Bitmap sv;
//extern __hugeflash unsigned char svData[];
//
//extern __hugeflash Bitmap MetRst;
//extern __hugeflash unsigned char MetRstData[];
//
//extern __hugeflash Bitmap MetRstAll;
//extern __hugeflash unsigned char MetRstAllData[];
//
//extern __hugeflash Bitmap ErrStat;
//extern __hugeflash unsigned char ErrStatData[];
//
//extern __hugeflash Bitmap ClrDev;
//extern __hugeflash unsigned char ClrDevData[];
//
//extern __hugeflash Bitmap ClrAll;
//extern __hugeflash unsigned char ClrAllData[];
//
//extern __hugeflash Bitmap arrowright;
//extern __hugeflash unsigned char arrowrightData[];
//
//extern __hugeflash Bitmap arrowleft1;
//extern __hugeflash unsigned char arrowleft1Data[];
//
//extern __hugeflash Bitmap arrowright1;
//extern __hugeflash unsigned char arrowright1Data[];
//
//extern __hugeflash Bitmap ErrStat1;
//extern __hugeflash unsigned char ErrStat1Data[];
//
//extern __hugeflash Bitmap ErrStat2;
//extern __hugeflash unsigned char ErrStat2Data[];
//
//extern __hugeflash Bitmap ErrStat3;
//extern __hugeflash unsigned char ErrStat3Data[];
//
//extern __hugeflash Bitmap FrecciaMsgL;
//extern __hugeflash unsigned char FrecciaMsgLData[];
//
//extern __hugeflash Bitmap FrecciaMsgE;
//extern __hugeflash unsigned char FrecciaMsgEData[];
//
//extern __hugeflash Bitmap FrecciaMsgLE;
//extern __hugeflash unsigned char FrecciaMsgLEData[];
//
//extern __hugeflash Bitmap FrecciaD;
//extern __hugeflash unsigned char FrecciaDData[];
//
//extern __hugeflash Bitmap FrecciaDE;
//extern __hugeflash unsigned char FrecciaDEData[];
//
//extern __hugeflash Bitmap FrecciaU;
//extern __hugeflash unsigned char FrecciaUData[];
//
//extern __hugeflash Bitmap FrecciaUE;
//extern __hugeflash unsigned char FrecciaUEData[];
//
//extern __hugeflash Bitmap FrecciaDSmall;
//extern __hugeflash unsigned char FrecciaDSmallData[];
//
//extern __hugeflash Bitmap FrecciaUSmall;
//extern __hugeflash unsigned char FrecciaUSmallData[];
//
//extern __hugeflash Bitmap nextcy1;
//extern __hugeflash unsigned char nextcy1Data[];
//
//extern __hugeflash Bitmap PiuLabel;
//extern __hugeflash unsigned char PiuLabelData[];
//
//extern __hugeflash Bitmap MenoLabel;
//extern __hugeflash unsigned char MenoLabelData[];
//
//extern __hugeflash Bitmap arrowsmallrightempty;
//extern __hugeflash unsigned char arrowsmallrightemptyData[];
//
//extern __hugeflash Bitmap arrowsmallleftempty;
//extern __hugeflash unsigned char arrowsmallLeftemptyData[];
//
//extern __hugeflash Bitmap arrowsmallRightFull;
//extern __hugeflash unsigned char arrowsmallRightFullData[];
//
//extern __hugeflash Bitmap arrowsmallLeftFull;
//extern __hugeflash unsigned char arrowsmallLeftFullData[];
//
//extern __hugeflash Bitmap CycleLoad;
//extern __hugeflash unsigned char CycleLoadData[];
//
//extern __hugeflash Bitmap Exclusion;
//extern __hugeflash unsigned char ExclusionData[];
//
//extern __hugeflash Bitmap Sig1;
//extern __hugeflash unsigned char Sig1Data[];
//
//extern __hugeflash Bitmap Sig2;
//extern __hugeflash unsigned char Sig2Data[];
//
//extern __hugeflash Bitmap Sig3;
//extern __hugeflash unsigned char Sig3Data[];
//
//extern __hugeflash Bitmap UltraSmallF;
//extern __hugeflash unsigned char UltraSmallFData[];
//
//extern __hugeflash Bitmap ArrDownSmall;
//extern __hugeflash unsigned char ArrDownSmallData[];
//
//extern __hugeflash Bitmap Prescaler;
//extern __hugeflash unsigned char PrescalerData[];
//
//extern __hugeflash Bitmap OffImg;
//extern __hugeflash unsigned char OffImgData[];
//
//extern __hugeflash Bitmap lbl10x;
//extern __hugeflash unsigned char lbl10xData[];
//
//extern __hugeflash Bitmap lbl100x;
//extern __hugeflash unsigned char lbl100xData[];
//
//extern __hugeflash Bitmap Roman;
//extern __hugeflash unsigned char RomanData[];
//
//extern __hugeflash Bitmap ESlot;
//extern __hugeflash unsigned char ESlotData[];
//
//extern __hugeflash Bitmap plearn;
//extern __hugeflash unsigned char plearnData[];
//
//extern __hugeflash Bitmap Calib;
//extern __hugeflash unsigned char CalibData[];
//
//extern __hugeflash Bitmap CalibFlat;
//extern __hugeflash unsigned char CalibFlatData[];
//
//extern __hugeflash Bitmap CalList;
//extern __hugeflash unsigned char CalListData[];
//
//extern __hugeflash Bitmap LFALock;
//extern __hugeflash unsigned char LFALockData[];
//
//extern __hugeflash Bitmap SLock;
//extern __hugeflash unsigned char SLockData[];
//
//extern __hugeflash Bitmap RFSmall;
//extern __hugeflash unsigned char RFSmallData[];
//
//extern __hugeflash Bitmap RFSmallLfa;
//extern __hugeflash unsigned char RFSmallLfaData[];
//
//extern __hugeflash Bitmap Nemo;
//extern __hugeflash unsigned char NemoData[];
//
//extern __hugeflash Bitmap Learn;
//extern __hugeflash unsigned char LearnData[];
//
//extern __hugeflash Bitmap ON;
//extern __hugeflash unsigned char ONData[];
//
//extern __hugeflash Bitmap OFF;
//extern __hugeflash unsigned char OFFData[];
//
//extern __hugeflash Bitmap Prod;
//extern __hugeflash unsigned char ProdData[];
//
//extern __hugeflash Bitmap Zero;
//extern __hugeflash unsigned char ZeroData[];
//
//extern __hugeflash Bitmap UNIFSmall;
//extern __hugeflash unsigned char UNIFSmallData[];
//
//extern __hugeflash Bitmap Circle1;
//extern __hugeflash unsigned char Circle1Data[];
//
//extern __hugeflash Bitmap Circle;
//extern __hugeflash unsigned char CircleData[];

const Bitmap Folder;
const unsigned char FolderData[8];


















