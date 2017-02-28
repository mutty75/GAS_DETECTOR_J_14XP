#include <asf.h>

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

//typedef struct
//{
//unsigned char            FontHeight;
//unsigned char            StartChar;
//const FontIndex		   *IndexTable;
//flash_addr_t             *FontData;
//} Font;

#endif

extern Font Arial;
extern const FontIndex Arial_Index[];
extern const unsigned char Arial_Data[];
