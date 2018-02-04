// Serial Example
// Jason Losh

//-----------------------------------------------------------------------------
// Objectives and notes             
//-----------------------------------------------------------------------------

// Using remappable pins
// Using uart engine

// Target Platform: 33FJ128MC802 Serial Demo Board
// Target uC:       33FJ128MC802
// Clock Source:    8 MHz primary oscillator set in configuration bits
// Clock Rate:      80 MHz using prediv=2, plldiv=40, postdiv=2
// Devices used:    UART

// Hardware description:
// Red LED
//   anode connected through 100ohm resistor to RB5 (pin 14), cathode grounded
// Green LED
//   anode connected through 100ohm resistor to RB4 (pin 11), cathode grounded
// Push Button
//  Push button connected between RB15 (pin 26) and ground
// SP3232E RS-232 Interface
//  T1IN connected to RP10 (pin 21)
//  R1OUT connected to RP11 (pin 22)

//-----------------------------------------------------------------------------
// Device includes and assembler directives             
//-----------------------------------------------------------------------------

#include <p33FJ128MC802.h>
#define FCY 40000000UL                       // instruction cycle rate (ignore)
#include <libpic30.h>                        // __delay32
                                             // __delay_ms and __delay_us
                                             // note: use only small values
#include <stdio.h>
#define USE_AND_OR                           // enable AND_OR mask setting
#include <string.h>

#define BAUD_19200 129                       // brg for low-speed, 40 MHz clock
                                             // round((40000000/16/19200)-1)

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
  RPINR18bits.U1RXR = 11;                    // assign U1RX to RP11
  RPOR5bits.RP10R = 3;                       // assign U1TX to RP10

  PLLFBDbits.PLLDIV = 38;                    // pll feedback divider = 40;
  CLKDIVbits.PLLPRE = 0;                     // pll pre divider = 2
  CLKDIVbits.PLLPOST = 0;                    // pll post divider = 2
}

void serial_init(int baud_rate)
{
  // set baud rate
  U1BRG = baud_rate;
  // enable uarts, 8N1, low speed brg
  U1MODE = 0x8000;
  // enable tx and rx
  U1STA = 0x0400;
}

void serial_puts(char str[])
{
  int i;
  for (i = 0; i < strlen(str); i++)
  {
    // make sure buffer is empty
    while(U1STAbits.UTXBF);
    // write character
    U1TXREG = str[i];
  }
}

char serial_getc()
{
  // clear out any overflow error condition
  if (U1STAbits.OERR == 1)
    U1STAbits.OERR = 0;
  // wait until character is ready
  while(!U1STAbits.URXDA);
  return U1RXREG;
}

// Wait for PB Press
void wait_pb()
{
  while (PORTBbits.RB15 == 1);               // wait until pb pressed
}
                               
//-----------------------------------------------------------------------------
// Main                
//-----------------------------------------------------------------------------

int c;

int main(void)
{
//  int c;
  init_hw();                                 // initialize hardware
  serial_init(BAUD_19200);                   // configure uart
  
  LATBbits.LATB4 = 1;                        // blink green LED for 500ms
  __delay32(20000000);
  LATBbits.LATB4 = 0;

  wait_pb();                                 // wait for pb press

  serial_puts("Enter \"1\" or \"0\"\r\n"); // print greeting

  while(1)
  {
    c =  serial_getc();
    LATBbits.LATB4 ^= 1;                  // toggle green LED for 500ms
    if (c == '1')                         // if '1' rx, turn on red led
      LATBbits.LATB5 = 1;  
    if (c == '0')                         // if '0' rx, turn off red led
      LATBbits.LATB5 = 0;  
  }
  return 0;
}
