//#include "FreeRTOS.h"

#ifndef FONTTYPEDEF
#define FONTTYPEDEF

typedef struct
{
  unsigned char  ucCharWidth;
  unsigned short usOffset;
} FontIndex;

typedef struct
{
  unsigned char               FontHeight;
  unsigned char               StartChar;
  const FontIndex             *IndexTable;
  const unsigned char         *FontData;
} Font;

#endif


extern Font NumBig;
extern const FontIndex NumBig_Index[];
extern const unsigned char NumBig_Data[];
