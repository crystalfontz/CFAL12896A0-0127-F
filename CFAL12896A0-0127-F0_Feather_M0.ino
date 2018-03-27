//===========================================================================
//
//
//  CRYSTALFONTZ 
//
//  This code uses the 4-wire SPI MCU mode of the display.
//TODO: add link once created
//  https://www.crystalfontz.com/product/
//
//  The controller is a Solomon Systech SSD1351
//TODO: add link once created
//    https://www.crystalfontz.com/controllers/
//
//  Controlled via Adafruit Feather M0
//============================================================================
//
//
//
//===========================================================================
//This is free and unencumbered software released into the public domain.
//
//Anyone is free to copy, modify, publish, use, compile, sell, or
//distribute this software, either in source code form or as a compiled
//binary, for any purpose, commercial or non-commercial, and by any
//means.
//
//In jurisdictions that recognize copyright laws, the author or authors
//of this software dedicate any and all copyright interest in the
//software to the public domain. We make this dedication for the benefit
//of the public at large and to the detriment of our heirs and
//successors. We intend this dedication to be an overt act of
//relinquishment in perpetuity of all present and future rights to this
//software under copyright law.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
//OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
//ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
//OTHER DEALINGS IN THE SOFTWARE.
//
//For more information, please refer to <http://unlicense.org/>
//============================================================================


//============================================================================
// Pinnout for 4-wire SPI
//
//   Feather  | Port |            |  Function                   | Wire
//------------+------+------------+-----------------------------+------------
//  N/A	      |      | #2         |  POWER 12V                  | Red
//  GND	      |      | #1 #30     |  GROUND                     | Black
//  3.3V      |      | #4 #27     |  POWER 3.3V                 | Green
//  GND	      |      | #16        |  GROUND                     | Black
// -----------+------+------------+-----------------------------+------------
//  3.3V      |      | #15        |  Enable/Read       (E/RD)   | Orange
//  3.3V      |      | #16        |  Read/Write         (R/W)   | Green
//  A0        |      | #17        |  Interface Select 0 (BS0)   | White
//  A1        |      | #18        |  Interface Select 1 (BS1)   | Black
//  A4        |      | #19        |  Chip Enable Signal  (CS)   | Purple
//  A5        |      | #20        |  Data/Command        (DC)   | Yellow
//  A2        |      | #21        |  Reset            (RESET)   | Blue
// -----------+------+------------+-----------------------------+------------
//  24        |      | #14        |  LCD_D10 (DB0) SCLK         | Purple
//  23        |      | #13        |  LCD_D11 (DB1) SDIN         | Green
//  N/C       |      | #12        |  LCD_D12 (DB2)              | Orange
//  3.3V      |      | #11        |  LCD_D13 (DB3)              | Yellow
//  3.3V      |      | #10        |  LCD_D14 (DB4)              | Grey
//  3.3V      |      | #9         |  LCD_D15 (DB5)              | Brown
//  3.3V      |      | #8         |  LCD_D16 (DB6)              | White
//  3.3V      |      | #7         |  LCD_D17 (DB7)              | Black
// -----------+------+------------+-----------------------------+------------
//============================================================================
// The following components should also be soldered prior to powering:
//  1. A 1uF cap between VDD and VSS
//  2. A .1uF cap and a 4.7uF cap in parallel between VDDIO and VSS
//  3. A .1uF cap and a 4.7uF cap in parallel between VCI and VSS
//  4. A 560kOhm 
//  5. A 50Ohm resistor in series with a ~1.4V,.5W diode from VSL to VSS (cathode to ground)
//  6. A 4.7uF tantalum cap (cathode to ground)
//  7. A .1uF cap and a 10uF cap in parallel between VCC and VSS
//============================================================================
//  BS0,BS1 interface settings:
//  
//      Interface         | BS0 | BS1 
//  ----------------------+-----+-----
//    3-wire SPI          |  1  |  0  
//  >>4-wire SPI          |  0  |  0  
//    8-bit 68XX Parallel |  1  |  1  
//    8-bit 80XX Parallel |  0  |  1  
//
//  This code is demonstrated using 4-wire SPI
//============================================================================


#include <SD.h>
#include <avr/io.h>
#include <SPI.h>

//map the pins to the corresponding ports
#define CS    PORT_PA05 //A4
#define DC    PORT_PB02 //A5
#define RESET PORT_PB09 //A2
#define BS0   PORT_PA02 //A0
#define BS1   PORT_PB08 //A1
#define LED   PORT_PA17 //13

#define SD_CS 4 //define the SD CS pin


//define the pre-processor macros to set and clear pins
//these work the same as "digitalWrite" but are much faster
#define CLR_CS     PORT->Group[PORTA].OUTCLR.reg = CS 
#define SET_CS     PORT->Group[PORTA].OUTSET.reg = CS 
#define CLR_DC     PORT->Group[PORTB].OUTCLR.reg = DC 
#define SET_DC     PORT->Group[PORTB].OUTSET.reg = DC 
#define CLR_RESET  PORT->Group[PORTB].OUTCLR.reg = RESET 
#define SET_RESET  PORT->Group[PORTB].OUTSET.reg = RESET 

#define CLR_BS0    PORT->Group[PORTA].OUTCLR.reg = BS0 
#define SET_BS0    PORT->Group[PORTA].OUTSET.reg = BS0 
#define CLR_BS1    PORT->Group[PORTB].OUTCLR.reg = BS1 
#define SET_BS1    PORT->Group[PORTB].OUTSET.reg = BS1 

#define LED_ON     PORT->Group[PORTA].DIRSET.reg = PORT_PA17
#define LED_OFF    PORT->Group[PORTA].DIRCLR.reg = PORT_PA17


//The following are the interupt bits for SPI on the SERCOM4 register
/*!< bit:      0  Data Register Empty Interrupt Enable */
/*!< bit:      1  Transmit Complete Interrupt Enable   */
/*!< bit:      2  Receive Complete Interrupt Enable    */
/*!< bit:      3  Slave Select Low Interrupt Enable    */
/*!< bit:  4.. 6  Reserved                             */
/*!< bit:      7  Combined Error Interrupt Enable      */
//Use these bits to check for the interrupt
//wait for the data register to be empty
#define WAIT_DRE  while( 0 == REG_SERCOM4_SPI_INTFLAG & 0x1)
//wait for the transmit complete flag
#define WAIT_TXC  while (0 == REG_SERCOM4_SPI_INTFLAG & 0x2)
//wait for the receive complete flag
#define WAIT_RXC  while (0 == REG_SERCOM4_SPI_INTFLAG & 0x4)

//prior to sending data to RAM for it to be shown on the display, the following
//command must be sent
#define WRITE_RAM writeCommand(0x5C);


//================================================================================
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF
//================================================================================




#define SPI_4_WIRE

#ifdef SPI_4_WIRE
//============================================================================
void writeCommand(uint8_t command)
{
  // Select the LCD's command register
  CLR_DC;
  // Select the LCD controller
  CLR_CS;

  //write directly to the SPI register
  REG_SERCOM4_SPI_DATA = command;

  //once the data is in the SPI register, we have to wait for it to finish processing
  //before we can raise the CS pin. 
  while (0 == (REG_SERCOM4_SPI_INTFLAG & 0x2));

  //deselect the chip
  SET_CS;
}
//----------------------------------------------------------------------------
void writeData(uint8_t data)
{
  // Select the LCD's data register
  SET_DC;
  // Select the LCD controller
  CLR_CS;

  //write directly to the SPI register
  REG_SERCOM4_SPI_DATA = data;

  //once the data is in the SPI register, we have to wait for it to finish processing
  //before we can raise the CS pin. 
  while (0 == (REG_SERCOM4_SPI_INTFLAG & 0x2));
  
  //deselct the chip
  SET_CS;
}
void writeDataFast(uint8_t data)
{
  //this function can be used when there's no need to change the CS or DC pins
  //to save a little time
  while (0 == (REG_SERCOM4_SPI_INTFLAG & 0x2));
  REG_SERCOM4_SPI_DATA = data;
}

#endif


//================================================================================
void interface_Init()
{
  //set the direction of the ports to output
  //to set as input, change DIRSET to DIRCLR
  //these perform the same function as "pinout();"
  PORT->Group[PORTA].DIRSET.reg = CS;
  PORT->Group[PORTB].DIRSET.reg = DC;
  PORT->Group[PORTB].DIRSET.reg = RESET;
  PORT->Group[PORTA].DIRSET.reg = BS0;
  PORT->Group[PORTB].DIRSET.reg = BS1;
  PORT->Group[PORTA].DIRSET.reg = LED;

#ifdef SPI_4_WIRE
  //set to 4-wire SPI
  CLR_BS0;
  CLR_BS1;
#endif
}


//================================================================================
void init_8080()
{
  SET_CS;
  SET_DC;

  SET_RESET;
  delay(500);
  CLR_RESET;
  delay(500);
  SET_RESET;
  delay(500);



  // Initialization Sequence
  Serial.println("top of init");
  //Set Command Lock
  writeCommand(0xFD);  // set command lock
  writeData(0x12);
  //0001 0010
  //|||| ||||-- MCU protection status [reset = 12h]
  //              12h: Unlock OLED driver IC MCU interface from entering command [reset]
  //              16h: Lock OLED driver IC MCU interface from entering command
  //              B0h: Command A2, B1, B3, BB, BE, C1 inaccessible in both lock and unlock state [reset]
  //              B1h: Command A2, B1, B3, BB, BE, C1  accessible if in unlock state

  //Set Command Lock
  writeCommand(0xFD);
  writeData(0xB1);
  //1011 0001
  //|||| ||||-- MCU protection status [reset = 12h]
  //              12h: Unlock OLED driver IC MCU interface from entering command [reset]
  //              16h: Lock OLED driver IC MCU interface from entering command
  //              B0h: Command A2, B1, B3, BB, BE, C1 inaccessible in both lock and unlock state [reset]
  //              B1h: Command A2, B1, B3, BB, BE, C1  accessible if in unlock state

  //Set Sleep Mode: Sleep mode ON (Display Off)
  writeCommand(0xAE);

  //Front Clock Divider (DivSet) / Oscillator Frequency
  writeCommand(0xB3);
  writeData(0xF1);
  //1111 0001
  //|||| ||||-- MCU protection status [reset = 12h]
  //||||          0h: DIVSET = Divide by 1
  //||||          1h: DIVSET = Divide by 2
  //||||        >>2h: DIVSET = Divide by 4
  //||||          3h: DIVSET = Divide by 8
  //||||          4h: DIVSET = Divide by 16
  //||||          5h: DIVSET = Divide by 32
  //||||          6h: DIVSET = Divide by 64
  //||||          7h: DIVSET = Divide by 128
  //||||          8h: DIVSET = Divide by 256
  //||||          9h: DIVSET = Divide by 512
  //||||          Ah: DIVSET = Divide by 1024
  //||||          >=Bh: DIVSET = invalid
  //||||          
  //||||------- Oscillator frequency, frequency increases as level increases [reset = 1101b]
  //
  //              Note: This command is locked by Command FDh (Set Command Lock) by default
  //                    To unlock it, please refer to Command FDh

  //Set MUX Ratio
  writeCommand(0xCA);
  writeData(127);
  //0111 1111
  // ||| ||||-- MUX ratio 16MUX ~ 128MUX, [reset = 127]
  //              (range from 15 to 127)

  //Set Re-map / Color Depth (Display RAM to Panel)
  writeCommand(0xA0);
  writeData(0x74);
  //0111 0100
  //|||| ||||-- Address Increment
  //|||| |||    >>0: Horizontal address increment [reset]
  //|||| |||      1: Vertical address increment
  //|||| |||
  //|||| |||--- Column Address Mapping
  //|||| ||     >>0: Column address 0 is mapped to SEG0 [reset]
  //|||| ||       1: Column address 127 is mapped to SEG0
  //|||| ||
  //|||| ||---- Color Sequence
  //|||| |        0: Color sequence: A -> B -> C [reset]
  //|||| |      >>1: Color sequence: C -> B -> A
  //|||| |
  //|||| |----- Reserved
  //||||        >>0: Reserved
  //||||          1: Reserved
  //||||
  //||||------- Scan Direction
  //|||           0: Scan from COM0 to COM[N-1] [reset]. Where N is the Multiplex ratio.
  //|||         >>1: Scan from COM[N-1] to COM0. Where N is the Multiplex ratio.
  //|||
  //|||-------- COM Split Odd Even
  //||            0: Disable COM Split Odd Even
  //||          >>1: Enable COM Split Odd Even
  //||
  //||--------- Set Color Depth
  //            >>00 / 01: 65k color [reset]
  //              10: 262k color
  //              11: 262k color, 16-bit format 2

  //Set Column Address
  writeCommand(0x15);
  writeData(0x00); //Start Address [reset = 0]
  writeData(0x7F); //End Address [reset = 127]

  //Set Row Address
  writeCommand(0x75);
  writeData(0x00); //Start Address [reset = 0]
  writeData(0x7F); //End Address [reset = 127]

  //Set Display Start Line
  writeCommand(0xA1);
  writeData(0x00);
  //0000 0000
  // ||| ||||-- Set vertical scroll by RAM from 0~127 [reset = 00h]

  //Set Display Offset
  writeCommand(0xA2);
  writeData(0x60);
  //0110 0000
  // ||| ||||-- Set vertical scroll by Row from 0-127 [reset = 60h]
  //
  //              Note: This command is locked by Command FDh (Set Command Lock) by default
  //                    To unlock it, please refer to Command FDh

  //Set GPIO
  writeCommand(0xB5);
  writeData(0x00);
  //0000 0000
  //     ||||-- GPIO 0
  //     ||     >>00: Pin HiZ, Input disabled
  //     ||       01: Pin HiZ, Input enabled
  //     ||       10: Pin output LOW [reset]
  //     ||       11: Pin output HIGH
  //     ||       
  //     ||---- GPIO 1
  //            >>00: Pin HiZ, Input disabled
  //              01: Pin HiZ, Input enabled
  //              10: Pin output LOW [reset]
  //              11: Pin output HIGH

  //Function Selection
  writeCommand(0xAB);
  writeData(0x01);
  //0000 0001
  //||      |-- VDD Power Select
  //||            0: Select external Vdd
  //||          >>1: Select internal Vdd regulator [reset]
  //||
  //||--------- Interface Type
  //            >>00: 8-bit parallel interface [reset]
  //              01: 16-bit parallel interface
  //              11: 18-bit parallel interface

  //Set Reset (Phase 1) / Pre-charge (Phase 2) period
  writeCommand(0xB1);
  writeData(0x32);
  //0011 0010
  //|||| ||||-- Phase 1 period of 5~31 DCLK(s) clocks 
  //||||          0h: invalid
  //||||          1h: invalid
  //||||        >>2h: 5 DCLKs [reset]
  //||||          3h: 7 DCLKs
  //||||          .
  //||||          .
  //||||          Eh: 29 DCLKs
  //||||          Fh: 31 DCLKs
  //||||          80h: reset
  //||||
  //||||------- Phase 2 period of 5~31 DCLK(s) clocks 
  //              0h: invalid
  //              1h: invalid
  //              2h: invalid
  //            >>3h: 3 DCLKs
  //              4h: 4 DCLKs
  //              .
  //              .
  //              8h: 8 DCLKs [reset]
  //              .
  //              .
  //              Eh: 14 DCLKs
  //              Fh: 15 DCLKs
  //              80h: reset

  //Set VCOMH Voltage
  writeCommand(0xBE);
  writeData(0x05);
  //0000 0101
  //      |||-- Set COM deselect voltage level [reset = 05h]
  //              000: .72 x Vcc
  //              .
  //              .
  //            >>101: .82 x Vcc [reset]
  //              .
  //              .
  //              111: .86 x Vcc

  //Set Display Mode: Reset to normal display
  writeCommand(0xA6);

  //Set Contrast Current for Color A, B, C
  writeCommand(0xC1);
  writeData(0xC8);
  writeData(0x80);
  writeData(0xC8);

  //Master Contrast Current Control
  writeCommand(0xC7);
  writeData(0x0F);

  //Set Segment Low Voltage (VSL)
  writeCommand(0xB4);
  writeData(0xA0);
  writeData(0xB5);
  writeData(0x55);

  //Set Second Pre-charge period
  writeCommand(0xB6);
  writeData(0x01);

  //Set Sleep Mode: Sleep mode OFF (Display On)
  writeCommand(0xAF);
  delay(500);
}


//================================================================================
void returnHome()
{
  //to return to the start position, update the display window
  //since the display window is 128x96 pixles:

  //set column address
  writeCommand(0x15);
  //set display window to start at x=0
  writeData(0);
  //set display window to end at x=127
  writeData(127);

  //set row address
  writeCommand(0x75);
  //set display window to start at y=0
  writeData(0);
  //set display window to end at y=95
  writeData(95);
}

void setXY(uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY)
{
  //update the display window with the specified start and end positions

  //check for correct parameters
  if (startX > endX)
  {
    Serial.println("startX and endX are backwards");
    uint8_t temp = startX;
    startX = endX;
    endX = temp;
  }
  if (startY > endY)
  {
    Serial.println("startY and endY are backwards");
    uint8_t temp = startY;
    startY = endY;
    endY = temp;
  }
  if (endX > 127)
  {
    Serial.print("endX exceeds display window size: ");
    Serial.println(endX);
    endX = 127;
    if (startX > 127)
    {
      Serial.print("startX exceeds display window size: ");
      Serial.println(startX);
      startX = 127;
    }
  }
  if (endY > 95)
  {
    Serial.print("endY exceeds display window size: ");
    Serial.println(endY);
    endY = 95;
    if (startY > 95)
    {
      Serial.print("startY exceeds display window size: ");
      Serial.println(startY);
      startY = 95;
    }
  }

  //set column address
  writeCommand(0x15);
  //set display window to start at x=0
  writeData(startX);
  //set display window to end at x=127
  writeData(endX);

  //set row address
  writeCommand(0x75);
  //set display window to start at y=0
  writeData(startY);
  //set display window to end at y=95
  writeData(endY);
}

//================================================================================
void show_BMPs_in_root(void)
{
  File
    root_dir;
  root_dir = SD.open("/");
  if (0 == root_dir)
  {
    Serial.println("show_BMPs_in_root: Can't open \"root\"");
    return;
  }


  File
    bmp_file;

  while (1)
  {
    bmp_file = root_dir.openNextFile();
    if (0 == bmp_file)
    {
      // no more files, break out of while()
      // root_dir will be closed below.
      break;
    }
    //Skip directories (what about volume name?)
    if (0 == bmp_file.isDirectory())
    {
      //The file name must include ".BMP"
      if (0 != strstr(bmp_file.name(), ".BMP"))
      {
        //The BMP must be exactly 36918 long
        //(this is correct for182x96, 24-bit)
        uint32_t size = bmp_file.size();
        if (36918 == size)
        {
          //Jump over BMP header
          bmp_file.seek(54);

          //grab one row of pixels from the SD card at a time
          static uint8_t one_line[128 * 3];
          for (int line = 95; line >= 0; line--)
          {
            //Set the LCD to the left of this line. BMPs store data
            //to have the image drawn from the other end, uncomment the line below
            //setXY(0, line, 127, line);
            WRITE_RAM;

            //read a line from the SD card
            bmp_file.read(one_line, 128 * 3);

            //send the line to the display
            send_pixels(128 * 3, one_line);
          }
        }
      }
    }
    //Release the BMP file handle
    bmp_file.close();
    //Give a bit to let them see it
    delay(1000);
  }
  //Release the root directory file handle
  root_dir.close();
  SPI.beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0));
}


//================================================================================
void send_pixels(uint8_t byteCount, uint8_t *dataPtr)
{
  SET_DC;
  CLR_CS;
  uint8_t data;
  while (byteCount != 0)
  {
    //The data in the bitmap is stored as 24bits per pixel, 1byte per color, in a BGR format
    //Since the display is in a RGB format, we need to pull the blue first and store it in the
    //lower 5 bits of "pixel" and pull red last and store it in the upper 5 bits of "pixel
    //(r8)(r7)(r6)(r5) (r4)(g8)(g7)(g6) (g5)(g4)(g3)(b8) (b7)(b6)(b5)(b4)

    //Load the upper 5 bits of blue data in the lower 5 bits -> XXXX XXXX XXX(b8) (b7)(b6)(b5)(b4)
    uint16_t pixel = (*dataPtr >> 3) & 0x001F;
    //increase the pointer to the next byte
    dataPtr++;
    //reduce the count for the loop
    byteCount--;
    //Load the upper 6 bits of green data in the middle 6 bits-> XXXX X(g8)(g7)(g6) (g5)(g4)(g3)X XXXX
    pixel |= ((*dataPtr << 3) & 0x07E0);
    dataPtr++;
    byteCount--;
    //Load the upper 5 bits of red data in the upper 5 bits-> (r8)(r7)(r6)(r5) (r4)XXX XXXX XXXX
    pixel |= (*dataPtr << 8) & 0xF800;
    dataPtr++;
    byteCount--;
    writeData(pixel >> 8);
    writeData(pixel & 0xFF);
  }
  SET_CS;
}




void RGBBars()
{
  uint16_t color;

  //start at the top of the screen
  setXY(0, 0, 127, 95);
  WRITE_RAM;
  //loop through a third of the screen, 32 rows
  //the first color will be red, upper 5 bits of "color"
  for (int i = 0; i < 32; i++)
  {
    //set the color to black, 0x0000
    color = 0;
    for (int k = 0; k < 128; k++)
    {
      //write the 2 byte color to the screen, one byte at a time
      writeData(color >> 8);
      writeData(color & 0xFF);
      //since there are 5bits (0d-31d) associated with red and we need to span 128 pixels,
      //we will increment the color every 4th time through the loop (128/32 = 4)
      if ((k & 0x03) == 0x03)
      {
        color += 0x0800;
      }
    }
  }

  //start 32 pixels in, a third of the way from the top of the screen
  setXY(0, 32, 127, 95);
  WRITE_RAM;
  //loop through a third of the screen, 32 rows
  //the second color will be green, middle 6 bits of "color"
  for (int i = 0; i < 32; i++)
  {
    color = 0;
    for (int k = 0; k < 128; k++)
    {
      writeData(color >> 8);
      writeData(color & 0xFF);
      //since there are 6bits (0d-63d) associated with red and we need to span 128 pixels,
      //we will increment the color every 2nd time through the loop (128/64 = 2)
      if ((k & 0x01) == 0x01)
      {
        color += 0x0020;
      }
    }
  }

  //start 64 pixels in, two thirds of the way from the top of the screen
  setXY(0, 64, 127, 95);
  WRITE_RAM;
  //loop through a third of the screen, 32 rows
  //the third color will be blue, last 5 bits of "color"
  for (int i = 0; i < 32; i++)
  {
    color = 0;
    for (int k = 0; k < 128; k++)
    {
      writeData(color >> 8);
      writeData(color & 0xFF);
      //since there are 5bits (0d-31d) associated with blue and we need to span 128 pixels,
      //we will increment the color every 4th time through the loop (128/32 = 4)
      if ((k & 0x03) == 0x03)
      {
        color += 0x0001;
      }
    }
  }
}

void fill(uint16_t color)
{
  returnHome();
  WRITE_RAM;
  SET_DC;
  CLR_CS;
  for (uint16_t i = 0; i < 96 * 128; i++)
  {
    writeDataFast(color >> 8);
    writeDataFast(color & 0xFF);
  }
  SET_CS;
}

void writeColors(int delayTime)
{
  setXY(0, 0, 127, 95);
  WRITE_RAM;
  SET_DC;
  CLR_CS;
  for (uint16_t i = 0; i < 96 * 128; i++)
  {
    writeDataFast(BLACK >> 8);
    writeDataFast(BLACK & 0xFF);
  }
  delay(delayTime);
  for (uint16_t i = 0; i < 96 * 128; i++)
  {
    writeDataFast(BLUE >> 8);
    writeDataFast(BLUE & 0xFF);
  }
  delay(delayTime);
  for (uint16_t i = 0; i < 96 * 128; i++)
  {
    writeDataFast(RED >> 8);
    writeDataFast(RED & 0xFF);
  }
  delay(delayTime);
  for (uint16_t i = 0; i < 96 * 128; i++)
  {
    writeDataFast(GREEN >> 8);
    writeDataFast(GREEN & 0xFF);
  }
  delay(delayTime);
  for (uint16_t i = 0; i < 96 * 128; i++)
  {
    writeDataFast(CYAN >> 8);
    writeDataFast(CYAN & 0xFF);
  }
  delay(delayTime);
  for (uint16_t i = 0; i < 96 * 128; i++)
  {
    writeDataFast(MAGENTA >> 8);
    writeDataFast(MAGENTA & 0xFF);
  }
  delay(delayTime);
  for (uint16_t i = 0; i < 96 * 128; i++)
  {
    writeDataFast(YELLOW >> 8);
    writeDataFast(YELLOW & 0xFF);
  }
  delay(delayTime);

  SET_CS;
}


//============================================================================
void Put_Pixel(uint8_t x, uint8_t y, uint8_t p, uint16_t g)
{
  //x is locatino on x-axis
  //y is location on y-axis
  //p is size of the pixel (you can make squares with this feature)
  //g is the color
  uint8_t startCol = x;
  uint8_t endCol = (x + p) - 1;
  uint8_t startRow = y;
  uint8_t endRow = (y + p) - 1;
  uint8_t dr = endRow - startRow + 1;
  uint8_t dc = endCol - startCol + 1;



  setXY(startCol, startRow, endCol, endRow);

  int i = 0;
  int j = 0;
  WRITE_RAM;
  for (i = 0; i < dr; i++) {
    for (j = 0; j < dc; j++) {
      writeData(g >> 8);
      writeData(g & 0xFF);
    }
  }
}


//============================================================================
void OLED_Line(uint16_t x0, uint16_t y0,
  uint16_t x1, uint16_t y1,
  uint16_t g)
{
  int16_t
    dx;
  int16_t
    sx;
  int16_t
    dy;
  int16_t
    sy;
  int16_t
    err;
  int16_t
    e2;
  uint8_t
    p = 1;



  dx = abs((int16_t)x1 - (int16_t)x0);
  sx = x0 < x1 ? 1 : -1;
  dy = abs((int16_t)y1 - (int16_t)y0);
  sy = y0 < y1 ? 1 : -1;
  err = (dx > dy ? dx : -dy) / 2;
  for (;;)
  {
    //delay(10);
    Put_Pixel(x0, y0, p, g);
    if ((x0 == x1) && (y0 == y1))
      break;
    e2 = err;
    if (e2 > -dx)
    {
      err -= dy;
      x0 = (uint16_t)((int16_t)x0 + sx);
    }
    if (e2 < dy)
    {
      err += dx;
      y0 = (uint16_t)((int16_t)y0 + sy);
    }
  }

}


//================================================================================
void spanningLinesDemo() 
{
  //This demo draws consecutive lines to create a "spanning" effect

  uint16_t speed = 0;
  uint16_t g = 0xFFFF;
  uint8_t x;
  uint8_t y;
  uint8_t width = 127;
  uint8_t length = 95;
  
  x = 0;
  y = length;

  //start of span is (0,0)
  fill(0x00);
  
  while (x <= width )
  {

    OLED_Line(0, 0, x, length, g);
    x++;
    delay(speed);
  }
  while (y != 0)
  {
    y--;
    OLED_Line(0, 0, width, y, g);
    delay(speed);
  }
  g = ~g;

  //start of span is (63,0);
  while (y != length)
  {
    OLED_Line(width/2, 0, 0, y, g);
    y++;
    delay(speed);
  }
  x = 0;
  while (x <= width)
  {

    OLED_Line(width/2, 0, x, length, g);
    x++;
    delay(speed);
  }
  while (y != 0)
  {
    y--;
    OLED_Line(width/2, 0, width, y, g);
    delay(speed);
  }

  g = ~g;
  //start of span is (127,0)
  while (y != length)
  {
    OLED_Line(width, 0, 0, y, g);
    y++;
    delay(speed);
  }
  x = 0;
  while (x <= width)
  {

    OLED_Line(width, 0, x, length, g);
    x++;
    delay(speed);
  }
  //delay(5000);
  g = ~g;


  y = 0;
  x = width+1;
  //start of span is (127,95)
  while (x != 0)
  {
    x--;
    OLED_Line(width, length, x, 0, g);
    delay(speed);
  }
  while (y <= length)
  {
    OLED_Line(width, length, 0, y, g);
    y++;
    delay(speed);
  }
  //delay(5000);
  g = ~g;


  //start of span is (63,95);
  while (y != 0)
  {
    y--;
    OLED_Line(width/2, length, width, y, g);
    delay(speed);
  }
  x = width+1;
  while (x != 0)
  {
    x--;
    OLED_Line(width/2, length, x, 0, g);
    delay(speed);
  }
  while (y <= length)
  {
    OLED_Line(width/2, length, 0, y, g);
    y++;
    delay(speed);
  }
  g = ~g;


  //start of span is (0,95)
  while (y != 0)
  {
    y--;
    OLED_Line(0, length, width, y, g);
    delay(speed);
  }
  x = width+1;
  while (x != 0)
  {
    x--;
    OLED_Line(0, length, x, 0, g);
    delay(speed);
  }
}

//================================================================================
void setup()
{
  delay(500);
  //setup serial port for debugging
  Serial.begin(9600);
  Serial.println("started setup");
  //initialize SD card
  if (!SD.begin(SD_CS))
  {
    Serial.println("Card failed to initialize, or not present");
  }
  else
  {
    Serial.println("Card initialized.");
  }

  //initialize SPI settings, set to max speed = 12Mhz
  SPI.begin();
  SPI.beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0)); 

  //Setup feather and initialize display
  interface_Init();
  init_8080();
}

//================================================================================

void loop()
{  
  Serial.println("top of loop");

  RGBBars();
  delay(2000);

  for (int i = 0; i < 5; i++)
  {
  writeColors((4*i+5)*10);
  }

  show_BMPs_in_root();
  delay(2000);

  spanningLinesDemo();
  delay(2000);
}

