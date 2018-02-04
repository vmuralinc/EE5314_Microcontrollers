// Graphics LCD Example
// Jason Losh

//-----------------------------------------------------------------------------
// Objectives and notes             
//-----------------------------------------------------------------------------

// Using SPI bus
// Using graphics LCD

// Target Platform: 33FJ128MC802 Servo Demo Board
// Target uC:       33FJ128MC802
// Clock Rate:      80 MHz
// Devices used:    None
// Configuration:   JTAG disabled

// Hardware description:

// ST7565R-based 128x64 Graphics LCD 
//   ~CS connected to RA1
//   SI connects to RP7/SDO1 (output fn 7) (pin 16)
//   SCL connectes to RP6/SCLK1OUT (output fn 8) (pin 15)
//   A0 connects to RB5 (pin 14) 

//-----------------------------------------------------------------------------
// Device includes and assembler directives             
//-----------------------------------------------------------------------------

#include <p33FJ128MC802.h>
#define FCY 40000000UL                        // instruction cycle rate (ignore)
#include <libpic30.h>                        // __delay32
                                             // __delay_ms and __delay_us
                                             // note: use only small values
#include <stdio.h>
//#define USE_AND_OR                           // enable AND_OR mask setting
//#include <spi.h>
#include "gr_lcd.h"

//-----------------------------------------------------------------------------
// Subroutines                
//-----------------------------------------------------------------------------

// Initialize Hardware
void init_hw()
{
  LATBbits.LATB5 = 0;                        // write 0 to LCD A0 pin
  TRISBbits.TRISB5 = 0;                      // make A0 pin an output
  AD1PCFGLbits.PCFG1 = 1;                    // make CS pin digital
  LATAbits.LATA1 = 1;                        // write 1 to ~CS pin latch
  TRISAbits.TRISA1 = 0;                      // make ~CS pin an output
  RPOR3bits.RP6R = 8;                        // assign SCLK1OUT to RP6
  RPOR3bits.RP7R = 7;                        // assign SDO1 to RP7
}

//-----------------------------------------------------------------------------
// Main                
//-----------------------------------------------------------------------------

int main(void)
{
  int i;
  
  init_hw();
 
  gr_init();


  gr_set_txt_pos(84,3);
  gr_puts("Text");

 
  for (i = 0; i < 64; i++)
    gr_set_pixel(i,i, SET);
  for (i = 0; i < 64; i++)
    gr_set_pixel(64-i,i, INVERT);

  while (1)
  {
    gr_rectangle(84, 40, 24, 10, INVERT);
    __delay32(8000000);
  }

  while(1);
  return 0;
}
