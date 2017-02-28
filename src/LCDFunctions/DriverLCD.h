
#include <stdint.h>
#include <stdio.h>

char LCDData[2][17];

extern void lcd_init(void);
extern void lcd_putc(char );
extern void lcd_gotoxy(unsigned char, unsigned char );
extern void lcd_send_byte(unsigned char, unsigned char );
extern void vLCDUpdate(void);