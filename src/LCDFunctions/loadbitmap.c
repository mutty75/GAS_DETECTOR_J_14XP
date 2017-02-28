/***************************************************
  This is an example sketch for the Adafruit 2.2" SPI display.
  This library works with the Adafruit 2.2" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/1480
 
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "asf.h"
#include "loadbitmap.h"
#include "ili9341.h"

#define BUFFPIXEL 64
uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)

uint16_t read16(FIL *);
uint32_t read32(FIL *);

void bmpDraw(FIL *bmpFile, uint16_t x, uint16_t y) {

  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  //bool	   goodBmp = false;       // Set to true on valid header parse
  bool     flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0;
  unsigned int bread;
  
  

  if((x >= 320) || (y >= 240)) return;


  // Open requested file on SD card
  //bmpFile = SD.open(filename))
  f_lseek(bmpFile,0);

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    read32(bmpFile);
    read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    
    // Read DIB header
    read32(bmpFile);
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        //goodBmp = true; // Supported BMP format -- proceed!

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= 320)  w = 320  - x;
        if((y+h-1) >= 240) h = 240 - y;

        // Set TFT address window to clipped image bounds
        //tft.setAddrWindow(x, y, x+w-1, y+h-1);  //Questo forse va ripristinato

        for (row=0; row<h; row++) { // For each scanline...

          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if(f_tell(bmpFile) != pos) { // Need seek?
            //bmpFile.seek(pos);
			f_lseek(bmpFile,pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          for (col=0; col<w; col++) { // For each pixel...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              f_read(bmpFile,sdbuffer, sizeof(sdbuffer),&bread);
              buffidx = 0; // Set index to beginning
            }

            //sdbuffer[buffidx]=120;
            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            //tft.pushColor(tft.Color565(r,g,b));
			Adafruit_ILI9340_drawPixel(col,row,ILI9341_COLOR(r,g,b));
          } // end pixel
        } // end scanline
      } // end goodBmp
    }
  }

  f_close(bmpFile);
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(FIL *bmp) {
  uint16_t result;
  unsigned int bread;
  
  //((uint8_t *)&result)[0] = f_read(bmp,1,&bread); // LSB
  //((uint8_t *)&result)[1] = f_read(bmp,1,&bread); // MSB
  f_read(bmp,&result,2,&bread);
  return result;
}

uint32_t read32(FIL *bmp) {
  uint32_t result;
  unsigned int bread;
  
  //((uint8_t *)&result)[0] = f_read(bmp,1,&bread); // LSB
  //((uint8_t *)&result)[1] = f_read(bmp,1,&bread);
  //((uint8_t *)&result)[2] = f_read(bmp,1,&bread);
  //((uint8_t *)&result)[3] = f_read(bmp,1,&bread); // MSB
  f_read(bmp,&result,4,&bread);
  return result;
}
