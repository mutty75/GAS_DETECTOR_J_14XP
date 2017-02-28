#include "FreeRTOS.h"

#ifndef FONTTYPEDEF
#define FONTTYPEDEF

typedef struct
{
  unsigned portCHAR  ucCharWidth;
  unsigned portSHORT usOffset;
} FontIndex;

typedef struct
{
  unsigned portCHAR            FontHeight;
  unsigned portCHAR            StartChar;
  FontIndex         const      *IndexTable;
  unsigned portCHAR const      *FontData;
} Font;

#endif


extern Font small;
extern const FontIndex small_Index[];
extern const unsigned char small_Data[];
