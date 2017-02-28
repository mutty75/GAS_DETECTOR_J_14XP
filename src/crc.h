#include "FreeRTOS.h"
#include <stdbool.h>

#define updcrc(ch, crc) (uscrctab[ ( ((crc >> 8) & 0xFF) ^ ch )] ^ (crc << 8))

extern const unsigned short uscrctab[256];
extern unsigned short usCrc16(unsigned char *, unsigned short, unsigned short );

//extern bool xCheckApplicationCRC (unsigned long ulStartAddress, unsigned long ulEndAddress);




