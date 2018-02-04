// Frequency and Time Measurement Example
// Jason Losh

//-----------------------------------------------------------------------------
// Objectives and notes             
//-----------------------------------------------------------------------------

// Using interrupts, timers, and input capture module

// Target Platform: 33FJ128MC802 Serial Demo Board
// Target uC:       33FJ128MC802
// Clock Source:    8 MHz primary oscillator set in configuration bits
// Clock Rate:      80 MHz using prediv=2, plldiv=40, postdiv=2
// Devices used:    Timers, input capture

// Hardware description:
// Red LED
//   anode connected through 100ohm resistor to RB5 (pin 14), cathode grounded
// Green LED
//   anode connected through 100ohm resistor to RB4 (pin 11), cathode grounded
// ST7565R-based 128x64 Graphics LCD 
//   ~CS connected to RA1
//   SI connects to RP7/SDO1 (output fn 7) (pin 16)
//   SCL connectes to RP6/SCLK1OUT (output fn 8) (pin 15)
//   A0 connects to RB5 (pin 14) 
// Signal Input
//   RP3 (pin 7) is used as an input for the counter and input capture modules

//-----------------------------------------------------------------------------
// Device includes and assembler directives             
//-----------------------------------------------------------------------------

#include <p33FJ128MC802.h>
#define FCY 40000000UL                       // instruction cycle rate (ignore)
#include <libpic30.h>                        // __delay32
                                             // __delay_ms and __delay_us
                                             // note: use only small values
#include <stdio.h>
#include <string.h>
#include "gr_lcd.h"

//-----------------------------------------------------------------------------
// Globals             
//-----------------------------------------------------------------------------

unsigned long freq;
unsigned long period;

//-----------------------------------------------------------------------------
// Subroutines                
//-----------------------------------------------------------------------------

// Initialize Hardware
void init_hw()
{
  // LCD Setup
  LATBbits.LATB5 = 0;                        // write 0 to LCD A0 pin
  TRISBbits.TRISB5 = 0;                      // make A0 pin an output

  AD1PCFGLbits.PCFG1 = 1;                    // make ~CS pin digital
  LATAbits.LATA1 = 1;                        // write 1 to ~CS pin latch
  TRISAbits.TRISA1 = 0;                      // make ~CS pin an output

  RPOR3bits.RP6R = 8;                        // assign SCLK1OUT to RP6
  RPOR3bits.RP7R = 7;                        // assign SDO1 to RP7

  // Setup Timer 4 and Input Capture 1 Inputs
  AD1PCFGLbits.PCFG5 = 1;                    // make RP3 pin digital
  RPINR4bits.T4CKR = 3;                      // assign T4CK to RP3
  RPINR7bits.IC1R = 3;                       // assign IC1 to RP3

  // Setup Clock
  PLLFBDbits.PLLDIV = 38;                    // pll feedback divider = 40;
  CLKDIVbits.PLLPRE = 0;                     // pll pre divider = 2
  CLKDIVbits.PLLPOST = 0;                    // pll post divider = 2
}


// initialize frequency counter functionality
// integration time is given in units of 6.4us
// integration is from 6.4us to 419.3ms
void freq_init(unsigned int period)
{
  // Clear counter 5:4 (accumulator)
  TMR5HLD = 0;
  TMR4 = 0;
  // Clock timer 4 with T2CK external clock
  T4CONbits.TCS = 1;
  T4CONbits.TCKPS = 0;
  T4CONbits.T32 = 1;
  T4CONbits.TON = 1;
  // Clock timer 1 with internal 40/256 MHz clock
  // Set period as requested by period
  T1CONbits.TCS = 0;
  T1CONbits.TCKPS = 3;
  T1CONbits.TON = 1;
  PR1 = period;
  // Enable timer 1 interrupts
  IEC0bits.T1IE = 1;
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt (void)
{
  unsigned int lsw, msw;
  // quickly read counter 5:4
  lsw = TMR4;
  msw = TMR5HLD;
  // Clear counter 5:3
  TMR5HLD = 0;
  TMR4 = 0;  
  // combine LSW and MSW
  freq = msw;
  freq <<= 16;
  freq |= lsw;
  // clear IF
  IFS0bits.T1IF = 0;
 }

void timer_init()
{
  // Clock timer 2 at 40 MHz
  T2CONbits.TCS = 0;
  T2CONbits.TCKPS = 0;
  T2CONbits.TON = 1;
  // Use timer 2 as time base for input capture 1
  IC1CONbits.ICTMR = 1;
  // Trigger interrupt on every second capture event
  IC1CONbits.ICI = 1;
  IC1CONbits.ICM = 3;
  // Enable interrupts after clearing IF bit
  IFS0bits.IC1IF = 0;  
  IEC0bits.IC1IE = 1;
}

void __attribute__((interrupt, no_auto_psv)) _IC1Interrupt (void)
{
  // clear IF
  IFS0bits.IC1IF = 0;
  // read last two captures and compute difference
  period = IC1BUF;
  period = IC1BUF - period;
  // clear the timer so that the next difference pair will not wrap-around
  TMR2 = 0;
}

//-----------------------------------------------------------------------------
// Main                
//-----------------------------------------------------------------------------

int main(void)
{
  char str[17];
  unsigned long safe;

  // initialize hardware
  init_hw();                                 

  // set integration period to 1/10 second
  // 40 MHz / 256 / 10 Hz = 15625
  freq_init(15625);

  // start timer with 25ns clock rate
  timer_init();
  
  gr_init();

  gr_set_txt_pos(0,2);
  gr_puts("Frequency (Hz)");
  gr_set_txt_pos(0,4);
  gr_puts("Period (us)");

  // Update screen at a 4 Hz rate
  while(1)
  {
    // Display frequency
    // Frequency is in units of 0.1 Hz
    IEC0bits.T1IE = 0;
    safe = freq;
    IEC0bits.T1IE = 1;
    gr_set_txt_pos(0,3);
    sprintf(str, "%08lu", safe * 10);    
    gr_puts(str);

    // Display period
    // Period is in units of 25ns
    IEC0bits.IC1IE = 0;
    safe = period;
    IEC0bits.IC1IE = 1;
    gr_set_txt_pos(0,5);
    sprintf(str, "%08lu", safe / 40);    
    gr_puts(str);
    __delay_ms(250);
  }

  return 0;
}
