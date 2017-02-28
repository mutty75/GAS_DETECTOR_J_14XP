///////////////////////////////////////////////////////////////////////////////
////                             LCD.C                                     ////
////                 Driver for common LCD modules                         ////
////                                                                       ////
////  lcd_init()   Must be called before any other function.               ////
////                                                                       ////
////  lcd_putc(c)  Will display c on the next position of the LCD.         ////
////                 \a  Set cursor position to upper left                 ////
////                 \f  Clear display, set cursor to upper left           ////
////                 \n  Go to start of second line                        ////
////                 \b  Move back one position                            ////
////              If LCD_EXTENDED_NEWLINE is defined, the \n character     ////
////              will erase all remanining characters on the current      ////
////              line, and move the cursor to the beginning of the next   ////
////              line.                                                    ////
////              If LCD_EXTENDED_NEWLINE is defined, the \r character     ////
////              will move the cursor to the start of the current         ////
////              line.                                                    ////
////                                                                       ////
////  lcd_gotoxy(x,y) Set write position on LCD (upper left is 1,1)        ////
////                                                                       ////
////  lcd_getc(x,y)   Returns character at position x,y on LCD             ////
////                                                                       ////
////  lcd_cursor_on(int1 on)   Turn the cursor on (on=TRUE) or off         ////
////              (on=FALSE).                                              ////
////                                                                       ////
////  lcd_set_cgram_char(w, *p)   Write a custom character to the CGRAM.   ////
////                                                                       ////
////                                                                       ////
////  CONFIGURATION                                                        ////
////  The LCD can be configured in one of two ways: a.) port access or     ////
////  b.) pin access.  Port access requires the entire 7 bit interface     ////
////  connected to one GPIO port, and the data bits (D4:D7 of the LCD)     ////
////  connected to sequential pins on the GPIO.  Pin access                ////
////  has no requirements, all 7 bits of the control interface can         ////
////  can be connected to any GPIO using several ports.                    ////
////                                                                       ////
////  To use port access, #define LCD_DATA_PORT to the SFR location of     ////
////  of the GPIO port that holds the interface, -AND- edit LCD_PIN_MAP    ////
////  of this file to configure the pin order.  If you are using a         ////
////  baseline PIC (PCB), then LCD_OUTPUT_MAP and LCD_INPUT_MAP also must  ////
////  be defined.                                                          ////
////                                                                       ////
////  Example of port access:                                              ////
////     #define LCD_DATA_PORT getenv("SFR:PORTD")                         ////
////                                                                       ////
////  To use pin access, the following pins must be defined:               ////
////     LCD_ENABLE_PIN                                                    ////
////     LCD_RS_PIN                                                        ////
////     LCD_RW_PIN                                                        ////
////     LCD_DATA4                                                         ////
////     LCD_DATA5                                                         ////
////     LCD_DATA6                                                         ////
////     LCD_DATA7                                                         ////
////                                                                       ////
////  Example of pin access:                                               ////
////     #define LCD_ENABLE_PIN  PIN_E0                                    ////
////     #define LCD_RS_PIN      PIN_E1                                    ////
////     #define LCD_RW_PIN      PIN_E2                                    ////
////     #define LCD_DATA4       PIN_D4                                    ////
////     #define LCD_DATA5       PIN_D5                                    ////
////     #define LCD_DATA6       PIN_D6                                    ////
////     #define LCD_DATA7       PIN_D7                                    ////
////                                                                       ////
///////////////////////////////////////////////////////////////////////////////
////        (C) Copyright 1996,2010 Custom Computer Services           ////
//// This source code may only be used by licensed users of the CCS C  ////
//// compiler.  This source code may only be distributed to other      ////
//// licensed users of the CCS C compiler.  No other use, reproduction ////
//// or distribution is permitted without written permission.          ////
//// Derivative programs created using this software in object code    ////
//// form are not restricted in any way.                               ////
///////////////////////////////////////////////////////////////////////////

#include "asf.h"
#include <LCDFunctions/DriverLCD.h>
#include <stdbool.h>

//#define _XTAL_FREQ 24000000

//#define LCD_ENABLE_PIN(x)  LATBbits.LB3=x	                                   ////
//#define LCD_RS_PIN(x)      LATAbits.LA2=x                                      ////
//#define LCD_RW_PIN(x)      LATAbits.LA3=x                                      ////
//#define LCD_DATA4(x)       LATDbits.LD0=x                                      ////
//#define LCD_DATA5(x)       LATDbits.LD1=x                                      ////
//#define LCD_DATA6(x)       LATDbits.LD2=x                                      ////
//#define LCD_DATA7(x)       LATDbits.LD3=x                                      ////

//#define TRIS_LCD_ENABLE_PIN(x)  TRISBbits.RB3=x                                ////
//#define TRIS_LCD_RS_PIN(x)      TRISAbits.RA2=x                                ////
//#define TRIS_LCD_RW_PIN(x)      TRISAbits.RA3=x                                ////
//#define TRIS_LCD_DATA4(x)       TRISDbits.RD0=x                                ////
//#define TRIS_LCD_DATA5(x)       TRISDbits.RD1=x                                ////
//#define TRIS_LCD_DATA6(x)       TRISDbits.RD2=x                                ////
//#define TRIS_LCD_DATA7(x)       TRISDbits.RD3=x                                ////

#define PORT_LCD_ENABLE_PIN  PIN_PA02                                         ////
#define PORT_LCD_RS_PIN      PIN_PA03                                         ////
#define PORT_LCD_RW_PIN      PIN_PA27	                                       ////
#define PORT_LCD_DATA4       PIN_PA12                                         ////
#define PORT_LCD_DATA5       PIN_PA13                                         ////
#define PORT_LCD_DATA6       PIN_PA15                                          ////
#define PORT_LCD_DATA7       PIN_PA09                                         ////

//#define LCD_ENABLE_PIN  PIN_D5                                    ////
//#define LCD_RS_PIN      PIN_B4                                    ////
//#define LCD_RW_PIN      PIN_D4                                    ////
//#define LCD_DATA4       PIN_D0                                    ////
//#define LCD_DATA5       PIN_D1                                    ////
//#define LCD_DATA6       PIN_D2                                    ////
//#define LCD_DATA7       PIN_D3

//#define LCD_ENABLE_PIN(x)  LATDbits.LATD5=x                                      ////
//#define LCD_RS_PIN(x)      LATBbits.LATB4=x                                      ////
//#define LCD_RW_PIN(x)      LATDbits.LATD4=x                                      ////
//#define LCD_DATA4(x)       LATDbits.LD0=x                                      ////
//#define LCD_DATA5(x)       LATDbits.LD1=x                                      ////
//#define LCD_DATA6(x)       LATDbits.LD2=x                                      ////
//#define LCD_DATA7(x)       LATDbits.LD3=x                                      ////
//
//#define TRIS_LCD_ENABLE_PIN(x)  TRISDbits.TRISD5=x                                ////
//#define TRIS_LCD_RS_PIN(x)      TRISBbits.TRISB4=x                                ////
//#define TRIS_LCD_RW_PIN(x)      TRISDbits.TRISD4=x                                ////
//#define TRIS_LCD_DATA4(x)       TRISDbits.TRISD0=x                                ////
//#define TRIS_LCD_DATA5(x)       TRISDbits.TRISD1=x                                ////
//#define TRIS_LCD_DATA6(x)       TRISDbits.TRISD2=x                                ////
//#define TRIS_LCD_DATA7(x)       TRISDbits.TRISD3=x                                ////
//
//
//#define PORT_LCD_ENABLE_PIN  PORTDbits.RD5                                     ////
//#define PORT_LCD_RS_PIN      PORTBbits.RB4                                     ////
//#define PORT_LCD_RW_PIN      PORTDbits.RD4                                     ////
//#define PORT_LCD_DATA4       PORTDbits.RD0                                     ////
//#define PORT_LCD_DATA5       PORTDbits.RD1                                     ////
//#define PORT_LCD_DATA6       PORTDbits.RD2                                     ////
//#define PORT_LCD_DATA7       PORTDbits.RD3

// define the pinout.
// only required if port access is being used.
//typedef struct
//{
//    unsigned char  enable:1,        // This structure is overlayed
//                   rs:1,            // on to an I/O port to gain
//                   rw:1,            // access to the LCD pins.
//                   unused:1,        // The bits are allocated from
//                   data : 4;        // low order up.  ENABLE will
//                                    // be LSB pin of that port.
////  #if defined(__PCD__)       // The port used will be LCD_DATA_PORT.
////   unsigned int    reserved: 8;
////  #endif
//} LCD_PIN_MAP;

// this is to improve compatability with previous LCD drivers that accepted
// a define labeled 'use_portb_lcd' that configured the LCD onto port B.
//#if ((defined(use_portb_lcd)) && (use_portb_lcd==TRUE))
// #define LCD_DATA_PORT getenv("SFR:PORTB")
//#endif

//#if defined(__PCB__)
//   // these definitions only need to be modified for baseline PICs.
//   // all other PICs use LCD_PIN_MAP or individual LCD_xxx pin definitions.
////                                    EN, RS,   RW,   UNUSED,  DATA
// const LCD_PIN_MAP LCD_OUTPUT_MAP =  {0,  0,    0,    0,       0};
// const LCD_PIN_MAP LCD_INPUT_MAP =   {0,  0,    0,    0,       0xF};
//#endif

////////////////////// END CONFIGURATION ///////////////////////////////////

//#ifndef LCD_ENABLE_PIN
   //#define lcd_output_enable(x) lcdlat.enable=x
   //#define lcd_enable_tris()   lcdtris.enable=0
//#else
   //#define lcd_output_enable(x) LCD_ENABLE_PIN(x)
   //#define lcd_enable_tris()  TRIS_LCD_ENABLE_PIN(0)
//#endif
//
//#ifndef LCD_RS_PIN
   //#define lcd_output_rs(x) lcdlat.rs=x
   //#define lcd_rs_tris()   lcdtris.rs=0
//#else
   //#define lcd_output_rs(x) LCD_RS_PIN(x)
   //#define lcd_rs_tris()  TRIS_LCD_RS_PIN(0)
//#endif
//
//#ifndef LCD_RW_PIN
   //#define lcd_output_rw(x) lcdlat.rw=x
   //#define lcd_rw_tris()   lcdtris.rw=0
//#else
   //#define lcd_output_rw(x) LCD_RW_PIN(x)
   //#define lcd_rw_tris()  TRIS_LCD_RW_PIN(0)
//#endif

// original version of this library incorrectly labeled LCD_DATA0 as LCD_DATA4,
// LCD_DATA1 as LCD_DATA5, and so on.  this block of code makes the driver
// compatible with any code written for the original library
//#if (defined(LCD_DATA0) && defined(LCD_DATA1) && defined(LCD_DATA2) && defined(LCD_DATA3) && !defined(LCD_DATA4) && !defined(LCD_DATA5) && !defined(LCD_DATA6) && !defined(LCD_DATA7))
//   #define  LCD_DATA4    LCD_DATA0
//   #define  LCD_DATA5    LCD_DATA1
//  #define  LCD_DATA6    LCD_DATA2
//   #define  LCD_DATA7    LCD_DATA3
//#endif

//#ifndef LCD_DATA4
//#ifndef LCD_DATA_PORT
//   #if defined(__PCB__)
//      #define LCD_DATA_PORT      0x06     //portb
//      #define set_tris_lcd(x)   set_tris_b(x)
//   #else
//     #if defined(PIN_D0)
//      #define LCD_DATA_PORT      getenv("SFR:PORTD")     //portd
//     #else
//      #define LCD_DATA_PORT      getenv("SFR:PORTB")     //portb
//     #endif
//   #endif
//#endif
//
//#if defined(__PCB__)
//   LCD_PIN_MAP lcd, lcdlat;
//   #byte lcd = LCD_DATA_PORT
//   #byte lcdlat = LCD_DATA_PORT
//#elif defined(__PCM__)
//   LCD_PIN_MAP lcd, lcdlat, lcdtris;
//   #byte lcd = LCD_DATA_PORT
//   #byte lcdlat = LCD_DATA_PORT
//   #byte lcdtris = LCD_DATA_PORT+0x80
//#elif defined(__PCH__)
//   LCD_PIN_MAP lcd, lcdlat, lcdtris;
//   #byte lcd = LCD_DATA_PORT
//   #byte lcdlat = LCD_DATA_PORT+9
//   #byte lcdtris = LCD_DATA_PORT+0x12
//#elif defined(__PCD__)
//   LCD_PIN_MAP lcd, lcdlat, lcdtris;
//   #word lcd = LCD_DATA_PORT
//   #word lcdlat = LCD_DATA_PORT+2
//   #word lcdtris = LCD_DATA_PORT-0x02
//#endif
//#endif   //LCD_DATA4 not defined

#ifndef LCD_TYPE
   #define LCD_TYPE 2           // 0=5x7, 1=5x10, 2=2 lines
#endif

#ifndef LCD_LINE_TWO
   #define LCD_LINE_TWO 0x40    // LCD RAM address for the second line
#endif

#ifndef LCD_LINE_LENGTH
   #define LCD_LINE_LENGTH 20
#endif

char const pt[6][8]={{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
                     {0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00},
                     {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00},
                     {0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x00},
                     {0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x00},
                     {0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x00}};

unsigned char const LCD_INIT_STRING[4] = {0x20 | (LCD_TYPE << 2), 0xc, 1, 6};
                             // These bytes need to be sent to the LCD
                             // to start it up.

void lcd_set_cgram_char(unsigned char, char const *);

unsigned char lcd_read_byte(void);
unsigned char lcd_read_nibble(void);

unsigned char lcd_read_byte(void)
{
   unsigned char low,high;
   struct port_config pin_conf;
   

   port_get_config_defaults(&pin_conf);
   pin_conf.direction  = PORT_PIN_DIR_INPUT;
   pin_conf.input_pull = PORT_PIN_PULL_UP;
   port_pin_set_config(PORT_LCD_DATA4, &pin_conf);
   port_pin_set_config(PORT_LCD_DATA5, &pin_conf);
   port_pin_set_config(PORT_LCD_DATA6, &pin_conf);
   port_pin_set_config(PORT_LCD_DATA7, &pin_conf);

   //TRIS_LCD_DATA4(1);
   //TRIS_LCD_DATA5(1);
   //TRIS_LCD_DATA6(1);
   //TRIS_LCD_DATA7(1);
   
   //lcd_output_rw(1);
   port_pin_set_output_level(PORT_LCD_RW_PIN,1);
   delay_us(1);
   //lcd_output_enable(1);
   port_pin_set_output_level(PORT_LCD_ENABLE_PIN,1);
   delay_us(1);
   high = lcd_read_nibble();

   //lcd_output_enable(0);
   port_pin_set_output_level(PORT_LCD_ENABLE_PIN,0);
   delay_us(1);
   //lcd_output_enable(1);
   port_pin_set_output_level(PORT_LCD_ENABLE_PIN,1);
   delay_us(1);
   low = lcd_read_nibble();

   //lcd_output_enable(0);
   port_pin_set_output_level(PORT_LCD_ENABLE_PIN,0);

   pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
   port_pin_set_config(PORT_LCD_DATA4, &pin_conf);
   port_pin_set_config(PORT_LCD_DATA5, &pin_conf);
   port_pin_set_config(PORT_LCD_DATA6, &pin_conf);
   port_pin_set_config(PORT_LCD_DATA7, &pin_conf);  
  
   //TRIS_LCD_DATA4(0);
   //TRIS_LCD_DATA5(0);
   //TRIS_LCD_DATA6(0);
   //TRIS_LCD_DATA7(0);

   return( (high<<4) | low);
}

unsigned char lcd_read_nibble(void)
{
  //#if (defined(LCD_DATA4) && defined(LCD_DATA5) && defined(LCD_DATA6) && defined(LCD_DATA7))
   unsigned char n = 0x00;

   /* Read the data port */
   n |= port_pin_get_input_level(PORT_LCD_DATA4);
   n |= port_pin_get_input_level(PORT_LCD_DATA5) << 1;
   n |= port_pin_get_input_level(PORT_LCD_DATA6) << 2;
   n |= port_pin_get_input_level(PORT_LCD_DATA7) << 3;

   return(n);
  //#else
   //return(lcd.data);
  //#endif
}

static void lcd_send_nibble(unsigned char n)
{
   /* Write to the data port */
   //output_bit(LCD_DATA4, bit_test(n, 0));
   //output_bit(LCD_DATA5, bit_test(n, 1));
   //output_bit(LCD_DATA6, bit_test(n, 2));
   //output_bit(LCD_DATA7, bit_test(n, 3));
   
   //LCD_DATA4((n & (1<<0)) ? 1 : 0);
   port_pin_set_output_level(PORT_LCD_DATA4,(n & (1<<0)) ? 1 : 0);
   //LCD_DATA5((n & (1<<1)) ? 1 : 0);
   port_pin_set_output_level(PORT_LCD_DATA5,(n & (1<<1)) ? 1 : 0);
   //LCD_DATA6((n & (1<<2)) ? 1 : 0);
   port_pin_set_output_level(PORT_LCD_DATA6,(n & (1<<2)) ? 1 : 0);
   //LCD_DATA7((n & (1<<3)) ? 1 : 0);
   port_pin_set_output_level(PORT_LCD_DATA7,(n & (1<<3)) ? 1 : 0);

   delay_us(1);
   //lcd_output_enable(1);
   port_pin_set_output_level(PORT_LCD_ENABLE_PIN,1);
   delay_us(2);
   //lcd_output_enable(0);
   port_pin_set_output_level(PORT_LCD_ENABLE_PIN,0);
}

void lcd_send_byte(unsigned char address, unsigned char n)
{
  //#if defined(__PCB__)
   //set_tris_lcd(LCD_OUTPUT_MAP);
  //#else
   //lcd_enable_tris();
   //lcd_rs_tris();
   //lcd_rw_tris();
  //#endif

   //lcd_output_rs(0);
   port_pin_set_output_level(PORT_LCD_RS_PIN,0);
   while (lcd_read_byte() & (1<<7)) ;
   //lcd_output_rs(address);
   port_pin_set_output_level(PORT_LCD_RS_PIN,address);
   delay_us(1);
   //lcd_output_rw(0);
   port_pin_set_output_level(PORT_LCD_RW_PIN,0);
   delay_us(1);
   //lcd_output_enable(0);
   port_pin_set_output_level(PORT_LCD_ENABLE_PIN,0);
   lcd_send_nibble(n >> 4);
   lcd_send_nibble(n & 0xf);
}

#if defined(LCD_EXTENDED_NEWLINE)
unsigned int8_t g_LcdX, g_LcdY;
#endif

void lcd_init(void)
{
   unsigned char i;

 //#if defined(__PCB__)
 //  set_tris_lcd(LCD_OUTPUT_MAP);
 //#else
 // #if (defined(LCD_DATA4) && defined(LCD_DATA5) && defined(LCD_DATA6) && defined(LCD_DATA7))
   //output_drive(LCD_DATA4);
   //output_drive(LCD_DATA5);
   //output_drive(LCD_DATA6);
   //output_drive(LCD_DATA7);
   //TRIS_LCD_DATA4(0);
   //TRIS_LCD_DATA5(0);
   //TRIS_LCD_DATA6(0);
   //TRIS_LCD_DATA7(0);
//  #else
//   lcdtris.data = 0x0;
//  #endif
   //lcd_enable_tris();
   //lcd_rs_tris();
   //lcd_rw_tris();
// #endif

   struct port_config pin_conf;
   port_get_config_defaults(&pin_conf);

	///* Configure LEDs as outputs, turn them off */
   pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
   port_pin_set_config(PORT_LCD_RS_PIN, &pin_conf);
   port_pin_set_config(PORT_LCD_RW_PIN, &pin_conf); 
   port_pin_set_config(PORT_LCD_ENABLE_PIN, &pin_conf);
   port_pin_set_config(PORT_LCD_DATA4, &pin_conf);
   port_pin_set_config(PORT_LCD_DATA5, &pin_conf);
   port_pin_set_config(PORT_LCD_DATA6, &pin_conf);
   port_pin_set_config(PORT_LCD_DATA7, &pin_conf);
	
   //lcd_output_rs(0);
   port_pin_set_output_level(PORT_LCD_RS_PIN,0);
   //lcd_output_rw(0);
   port_pin_set_output_level(PORT_LCD_RW_PIN,0);
   //lcd_output_enable(0);
   port_pin_set_output_level(PORT_LCD_ENABLE_PIN,0);
   
   delay_ms(15);
   for(i=1;i<=3;++i)
   {
       lcd_send_nibble(3);
       delay_ms(5);
   }

   lcd_send_nibble(2);
   delay_ms(5);
   for(i=0;i<=3;++i)
      lcd_send_byte(0,LCD_INIT_STRING[i]);

  #if defined(LCD_EXTENDED_NEWLINE)
   g_LcdX = 0;
   g_LcdY = 0;
  #endif
  
  lcd_set_cgram_char(0,pt[0]);
  lcd_set_cgram_char(1,pt[1]);
  lcd_set_cgram_char(2,pt[2]);
  lcd_set_cgram_char(3,pt[3]);
  lcd_set_cgram_char(4,pt[4]);
  lcd_set_cgram_char(5,pt[5]);
  
  memset(LCDData,0x00,sizeof(LCDData));
  
}

void lcd_gotoxy(unsigned char x, unsigned char y)
{
   unsigned char address;

   switch (y)
   {
    case 0x00: address=0x00; break;
    case 0x01: address=0x40; break;
    case 0x02: address=0x10; break;
    case 0x03: address=0x50; break;
   }

   //if(y!=1)
   //   address=LCD_LINE_TWO;
   //else
   //   address=0;

   address+=x-1;
   lcd_send_byte(0,0x80|address);

  #if defined(LCD_EXTENDED_NEWLINE)
   g_LcdX = x - 1;
   g_LcdY = y - 1;
  #endif
}

void lcd_putc(char c)
{
   switch (c)
   {
      case '\a'   :  lcd_gotoxy(1,1);     break;

      case '\f'   :  lcd_send_byte(0,1);
                     delay_ms(2);
                    #if defined(LCD_EXTENDED_NEWLINE)
                     g_LcdX = 0;
                     g_LcdY = 0;
                    #endif
                     break;

     #if defined(LCD_EXTENDED_NEWLINE)
      case '\r'   :  lcd_gotoxy(1, g_LcdY+1);   break;
      case '\n'   :
         while (g_LcdX++ < LCD_LINE_LENGTH)
         {
            lcd_send_byte(1, ' ');
         }
         lcd_gotoxy(1, g_LcdY+2);
         break;
     #else
      case '\n'   : lcd_gotoxy(1,2);        break;
     #endif

      case '\b'   : lcd_send_byte(0,0x10);  break;

     #if defined(LCD_EXTENDED_NEWLINE)
      default     :
         if (g_LcdX < LCD_LINE_LENGTH)
         {
            lcd_send_byte(1, c);
            g_LcdX++;
         }
         break;
     #else
      default     : lcd_send_byte(1,c);     break;
     #endif
   }
}

//static char lcd_getc(unsigned char x, unsigned char y)
//{
   //char value;
//
   //lcd_gotoxy(x,y);
   //while (lcd_read_byte() & (1<<7)); // wait until busy flag is low
   ////lcd_output_rs(1);
   //port_pin_set_output_level(PORT_LCD_RS_PIN,1);
   //value = lcd_read_byte();
   ////lcd_output_rs(0);
   //port_pin_set_output_level(PORT_LCD_RS_PIN,0);
//
   //return(value);
//}

// write a custom character to the ram
// which is 0-7 and specifies which character array we are modifying.
// ptr points to an array of 8 bytes, where each byte is the next row of
//    pixels.  only bits 0-4 are used.  the last row is the cursor row, and
//    usually you will want to leave this byte 0x00.

void lcd_set_cgram_char(unsigned char which, char const *ptr)
{
   unsigned int i;

   which <<= 3;
   which &= 0x38;

   lcd_send_byte(0, 0x40 | which);  //set cgram address

   for(i=0; i<8; i++)
   {
      lcd_send_byte(1, *ptr++);
   }

   #if defined(LCD_EXTENDED_NEWLINE)
    lcd_gotoxy(g_LcdX+1, g_LcdY+1);  //set ddram address
   #endif
}

//static void lcd_cursor_on(bool on)
//{
   //if (on)
   //{
      //lcd_send_byte(0,0x0F);           //turn LCD cursor ON
   //}
   //else
   //{
      //lcd_send_byte(0,0x0C);           //turn LCD cursor OFF
   //}
//}

void vLCDUpdate()
{
    //static uint8_t ucRow=0;

    lcd_gotoxy(1,0);
    for (uint8_t i=0;i<0x10;i++)
    {
        if (!LCDData[0][i]) LCDData[0][i]=0x20;
     lcd_send_byte (1,LCDData[0][i]);
    }
    lcd_gotoxy(1,1);
    for (uint8_t i=0;i<0x10;i++)
    {
        if (!LCDData[1][i]) LCDData[1][i]=0x20;
     lcd_send_byte (1,LCDData[1][i]);
    }  

}

//#endif



