// Stop Go Example
// Jason Losh

//-----------------------------------------------------------------------------
// Objectives and notes             
//-----------------------------------------------------------------------------

// Basic digital I/O operations
// Programming hardware
// Debugging hardware

// Target Platform: 33FJ128MC802 Basic Demo Board
// Target uC:       33FJ128MC802
// Clock Rate:      Depends on oscillator type selected and frequency
// Devices used:    None

// Hardware description:
// Red LED
//   anode connected through 100ohm resistor to RB5 (pin 14), cathode grounded
// Green LED
//   anode connected through 100ohm resistor to RB4 (pin 11), cathode grounded
// Push Button
//  Push button connected between RB15 (pin 26) and ground

//-----------------------------------------------------------------------------
// Device includes and assembler directives             
//-----------------------------------------------------------------------------

#include <p33FJ128MC802.h>
#include <stdio.h>

//-----------------------------------------------------------------------------
// Subroutines                
//-----------------------------------------------------------------------------

// Initialize Hardware
void init_hw()
{
  LATBbits.LATB4 = 0;                        // write 0 into output latches
  LATBbits.LATB5 = 0;
  TRISBbits.TRISB4 = 0;                      // make green led pin an output
  TRISBbits.TRISB5 = 0;                      // make red led pin an output
  CNPU1bits.CN11PUE = 1;                     // enable pull-up for push button
}

// Wait for PB Press
void wait_pb()
{
  while (PORTBbits.RB15 == 1);               // wait until pb pressed
}
                                
//-----------------------------------------------------------------------------
// Main                
//-----------------------------------------------------------------------------

int main(void)
{
  init_hw();                                 // initialize hardware

  LATBbits.LATB5 = 1;                        // red led on

  wait_pb();                                 // wait for pb press

  LATBbits.LATB5 = 0;                        // green led on
  LATBbits.LATB4 = 1;  

  while(1);                                  // endless loop

  return 0;
}
