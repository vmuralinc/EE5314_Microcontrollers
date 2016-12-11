// PROJECT
// VASUDEVAN MURALI -- 1000723391
// ARUNKUMAR KUPPUSWAMY -- 1000728397
// THAMIZH SELVAN ILAVARASU -- 1000726055

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
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

#define BAUD_19200 129                       // brg for low-speed, 40 MHz clock
                                             // round((40000000/16/19200)-1)
#define MAX_STR_LEN 80                       // max command line size
#define MAX_STR_PARAM 20                         // max command parameters
#define TRUE 1                               // true = 1
#define FALSE 0                              // false = 0
#define TYPE_DELIM 0                         // delim = 0
#define TYPE_ALPHA 1                         // alphabet = 1
#define TYPE_NUM 2                           // number = 2
#define TYPE_INEQ 3                          // inequality = 3
#define LO_MAC 0x1234567
#define HI_MAC 0x1234568
#define ADD_COND_RULE_SIZE 10
#define ARRAY_INIT_VAL 255

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

char sCommand[MAX_STR_LEN+1];
int aiOffset[MAX_STR_PARAM];
int aiType[MAX_STR_PARAM];
const int aiPinNo[12] = {30,31,19,20,21,0,1,18,4,23,28,29};
const int aiDevNo[12] = {0,1,2,3,4,5,6,7,8,9,10,11};
struct
{
  char acDevName[9];
  int  iDevNo;
}Devices[12];

struct
{
  char acCondName[9];
  int  iCondNo;
}Conditions[ADD_COND_RULE_SIZE];

struct
{
  char acRuleName[9];
  int  iRuleNo;
}Rules[ADD_COND_RULE_SIZE];

int aiHistory[12] = {1,1,0,0,0,0,0,0,0,0,0,0};
int aiBindArray[12][2] = {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},
						  {0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};
unsigned char aiCondArray[ADD_COND_RULE_SIZE][4];
unsigned char aiRuleArray[ADD_COND_RULE_SIZE][4];
unsigned long aiRuleArrayMsg[ADD_COND_RULE_SIZE];
const char acInequal[] = "<>=!";
int iParamCount = 0;
int iTemp;
unsigned long iDevMac = 0;
unsigned int iDevAdd = 0xFF;
unsigned long iMask =0;
unsigned long iAdd =0;
// assign 32x8word message buffers for ECAN1 in DMA RAM
unsigned int ecan1MsgBuf[32][8] __attribute__((space(dma), aligned(32*16)));
int Led3Count = -1;
int Led4Count = -1;
int Led5Count = -1;
int OpnSesnCount = -1;
unsigned char AddressMapShAdd[ADD_COND_RULE_SIZE];
unsigned long AddressMapMac[ADD_COND_RULE_SIZE];
unsigned int iSessionAdd;
int iCnt = 0;

//-----------------------------------------------------------------------------
// Init functions
//-----------------------------------------------------------------------------

// CAN Init Function
void can_init()
{
  // enter config mode
  C1CTRL1bits.REQOP = 4;
  while(C1CTRL1bits.OPMODE != 4);

  // bit rate to 125 kbps and NTQ to 20, so TQ = 1/2.5 MHz
  // phase segment 1 time is 8 TQ
  C1CFG2bits.SEG1PH = 7;
  // phase segment 2 time is set to be programmable
  C1CFG2bits.SEG2PHTS = 1;
  // phase segment 2 time is 6 TQ
  C1CFG2bits.SEG2PH = 5;
  // propagation segment time is 5 TQ
  C1CFG2bits.PRSEG = 4;
  // bus line is sampled three times at the sample point
  C1CFG2bits.SAM = 1;
  // Synchronization Jump Width set to 4 TQ
  C1CFG1bits.SJW = 3;
  // set baud rate prescaler
  // TQ = 2 * (BRP+1) / Fcyc
  // BRP = (TQ * Fcyc / 2) - 1
  // BRP = 7 for 125kbps, NTQ=20, Fcyc=40MHz
  C1CFG1bits.BRP = 7;

  // Enable Window to Access Acceptance Filter Registers */
  C1CTRL1bits.WIN=1;
  /* Select Acceptance Filter Mask 0 for Acceptance Filter 0 */
  C1FMSKSEL1bits.F0MSK=0;
  /* Configure Acceptance Filter Mask 0 register to mask SID<2:0>
	Mask Bits (11-bits) : 0b000 0000 0111 */
  C1RXM0SIDbits.SID = 0; // accept all
  C1RXM0SIDbits.EID = 0;
  C1RXM0EIDbits.EID = 0;
  /* Configure Acceptance Filter 0 to match standard identifier
	Filter Bits (11-bits): 0b011 1010 xxx */
  C1RXF0SIDbits.SID = 0; // who cares
  /* Acceptance Filter 0 to check for Standard Identifier */
  C1RXM0SIDbits.MIDE = 0;
  C1RXF0SIDbits.EXIDE= 0;
  /* Acceptance Filter 0 to use Message Buffer 1 to store message */
  C1BUFPNT1bits.F0BP = 1;
  /* Filter 0 enabled for Identifier match with incoming message */
  C1FEN1bits.FLTEN0=1;
  /* Clear Window Bit to Access ECAN Control Registers */
  C1CTRL1bits.WIN=0;

  // enter normal operating mode
  C1CTRL1bits.REQOP = 0;
  while(C1CTRL1bits.OPMODE != 0);

  // setup transmit and receive message buffers
  C1RXFUL1=C1RXFUL2=C1RXOVF1=C1RXOVF2=0x0000;
  // setup buffer 0 for tx buffer
  C1TR01CONbits.TXEN0=1;
  C1TR01CONbits.TX0PRI=0b11;
  // setup buffer 1 for rx buffer
  C1TR01CONbits.TXEN1=0;
  C1TR01CONbits.TX1PRI=0b11;

  // setup dma channel 0 for CAN transmission
  // word width
  DMA0CONbits.SIZE = 0;
  // direction is from DMA RAM to CAN tx
  DMA0CONbits.DIR = 1;
  // dma addressing mode is peripheral indirect addressing mode
  DMA0CONbits.AMODE = 2;
  // continuous op mode, w/o ping-pong buffering
  DMA0CONbits.MODE = 0;
  // DMA channel associated with ECAN1 tx event
  DMA0REQ = 70;
  // 8 word transfer
  DMA0CNT = 7;
  // peripheral address somes from ECAN1 tx register
  DMA0PAD = (unsigned int)&C1TXD;
  /* Start Address Offset for ECAN1 Message Buffer */
  DMA0STA = __builtin_dmaoffset(ecan1MsgBuf);
  /* Channel Enable: Enable DMA Channel 0 */
  DMA0CONbits.CHEN = 1;
  /* Channel Interrupt Enable: Enable DMA Channel 0 Interrupt */
  IEC0bits.DMA0IE = 1;

  /* Data Transfer Size: Word Transfer Mode */
  DMA1CONbits.SIZE = 0;
  /* Data Transfer Direction: Peripheral to DMA RAM */
  DMA1CONbits.DIR = 0;
  /* DMA Addressing Mode: Peripheral Indirect Addressing mode */
  DMA1CONbits.AMODE = 2;
  /* Operating Mode: Continuous, Ping-Pong modes disabled */
  DMA1CONbits.MODE = 0;
  /* Assign ECAN1 Receive event for DMA Channel 1 */
  DMA1REQ = 34;
  /* Set Number of DMA Transfer per ECAN message to 8 words */
  DMA1CNT = 7;
  /* Peripheral Address: ECAN1 Receive Register */
  DMA1PAD = (unsigned int)&C1RXD;
  /* Start Address Offset for ECAN1 Message Buffer */
  DMA1STA = __builtin_dmaoffset(ecan1MsgBuf);
  /* Channel Enable: Enable DMA Channel 1 */
  DMA1CONbits.CHEN = 1;
  /* Channel Interrupt Enable: Enable DMA Channel 1 Interrupt */
  IEC0bits.DMA1IE = 1;

  IEC2bits.C1IE = 1;
  C1INTEbits.TBIE = 1;
  C1INTEbits.RBIE = 1;
}

//Initialize clock timer 1
void timer1_init()
{
  // Clock timer 1 with internal 40/64 MHz clock
  // Set period as requested by period
  T1CONbits.TCS = 0;
  T1CONbits.TCKPS = 2;
  T1CONbits.TON = 1;
  // set integration period to 1/100 second
  // 40 MHz / 64 / 100 Hz = 6250
  PR1 = 6250;
  // Enable timer 1 interrupts
  IEC0bits.T1IE = 1;
}


// Initialize Serial hardware
void fSerialInit(int baud_rate)
{
  // set baud rate
  U1BRG = baud_rate;
  // enable uarts, 8N1, low speed brg
  U1MODE = 0x8000;
  // enable tx and rx
  U1STA = 0x0400;
  // UART Interrupt
  IEC0bits.U1RXIE = 1;
}

void fPwmInit()
{
  AD1PCFGLbits.PCFG3 = 1;
  TRISBbits.TRISB3 = 0;
  RPOR1bits.RP3R = 18;

  T2CONbits.TCS = 0;
  T2CONbits.TCKPS = 0;
  T2CONbits.TON = 1;
  PR2 = 0xFF;

  OC1CON = 0;
  OC1R = 0;
  OC1RS = 0;
  OC1CONbits.OCTSEL = 0;
  OC1CONbits.OCM = 6;
}

// Initialize Hardware
void fInitHw()
{
  LATBbits.LATB3 = 0;                        // write 0 into output latches
  LATBbits.LATB4 = 0;
  LATBbits.LATB5 = 0;
  TRISBbits.TRISB3 = 0;                      // make orange led pin an output
  TRISBbits.TRISB4 = 0;                      // make green led pin an output
  TRISBbits.TRISB5 = 0;                      // make red led pin an output
  CNPU1bits.CN11PUE = 1;                    // enable pull-up for push button
  CNPU1bits.CN12PUE = 1;                    // enable pull-up for push button
  RPINR18bits.U1RXR = 11;                    // assign U1RX to RP11
  RPOR5bits.RP10R = 3;                       // assign U1TX to RP10

  PLLFBDbits.PLLDIV = 38;                    // pll feedback divider = 40;
  CLKDIVbits.PLLPRE = 0;                     // pll pre divider = 2
  CLKDIVbits.PLLPOST = 0;                    // pll post divider = 2

  RPINR26bits.C1RXR = 8;                     // assign C1RX to RP8
  RPOR4bits.RP9R = 16;                       // assign C1TX to RP9

  TRISBbits.TRISB6 = 1;                      // make RP6 pin an input
  if(PORTBbits.RB6 == 1)
      iDevMac = HI_MAC;
  else
      iDevMac = LO_MAC;

  can_init();
  timer1_init();

}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Wait for PB Press
void fWaitPB()
{
  while (PORTBbits.RB15 == 1);               // wait until pb pressed
}

void fWaitPB1()
{
  while (PORTBbits.RB14 == 1);               // wait until pb1 pressed
}

// Serial put character
void fSerialPutC(char cChar)
{
  // make sure buffer is empty
  while(U1STAbits.UTXBF);
  // write character
  U1TXREG = cChar;
}

// Serial put string
void fSerialPutS(char str[])
{
  int iCount;
  for (iCount = 0; iCount < strlen(str); iCount++)
  {
    fSerialPutC(str[iCount]);
  }
}

// Serial get character
char fSerialGetC()
{
  // clear out any overflow error condition
  if (U1STAbits.OERR == TRUE)
      U1STAbits.OERR = FALSE;
  // wait until character is ready
  while(!U1STAbits.URXDA);
  return U1RXREG;
}

// Get the serial string
void fSerialGetS()
{
  int iCount;
  char cChar;
  iCount = 0;
  while(iCount<MAX_STR_LEN)
  {
    cChar = fSerialGetC();
    //fSerialPutC(cChar);
    if (cChar == '\b' && iCount > 0)
    {
      fSerialPutC(' ');
      fSerialPutC('\b');
      iCount--;
    }
    else if(cChar == '\n' || cChar == '\r')
    {
      break;
    }
    else if(cChar >= ' ')
    {
      sCommand[iCount] = tolower(cChar);
      iCount++;
    }
  }
  sCommand[iCount] = '\0';
  fSerialPutS("\n");
}

int fIsInequal(char cChar)
{
  int iCount;
  for(iCount = 0;iCount < strlen(acInequal);iCount++)
  {
    if(cChar == acInequal[iCount])
        return TRUE;
  }
  return FALSE;
}

// parse the type of the parameter
int fParseType(char cChar)
{
  if(isalpha(cChar) || cChar == '_')
  {
    return TYPE_ALPHA;
  }
  else if(isdigit(cChar))
  {
    return TYPE_NUM;
  }
  else if(fIsInequal(cChar))
  {
    return TYPE_INEQ;
  }
  else
  {
    return TYPE_DELIM;
  }
}

int Test(char strVerb[],int iMinParams)
{
  if(strcmp(&sCommand[aiOffset[0]],strVerb) == 0 && iParamCount >= iMinParams)
    return TRUE;
  return FALSE;
}

// Get Number function
unsigned long fGetNumber(int iParam)
{
  char *strNumber = &sCommand[aiOffset[iParam]];
  char **endPtr = '\0';
  unsigned long iReturn = 0;
  if(*strNumber == '0' && *(strNumber+1) == 'x')
  {
    iReturn = strtol(&sCommand[aiOffset[iParam]+2],endPtr,16);
  }
  else if(*strNumber == '0' && *(strNumber+1) == 'b')
  {
    iReturn = strtol(&sCommand[aiOffset[iParam]+2],endPtr,2);
  }
  else
  {
    iReturn = strtol(&sCommand[aiOffset[iParam]],endPtr,10);
  }
  return iReturn;
}

void fArrayInit()
{
  int iCount,iCount1;
  for(iCount = 0; iCount < ADD_COND_RULE_SIZE ; iCount++)
  {
    AddressMapShAdd[iCount] = 0;
    AddressMapMac[iCount] = 0;
    aiRuleArrayMsg[iCount] = 0;
    for(iCount1 = 0; iCount1 < 4; iCount1++)
    {
      aiCondArray[iCount][iCount1] = ARRAY_INIT_VAL;
      aiRuleArray[iCount][iCount1] = ARRAY_INIT_VAL;
    }
    strcpy(Conditions[iCount].acCondName,"\0");
    Conditions[iCount].iCondNo = ARRAY_INIT_VAL;
    strcpy(Rules[iCount].acRuleName,"\0");
    Rules[iCount].iRuleNo = ARRAY_INIT_VAL;
  }
  for(iCount = 0; iCount < 12; iCount++)
  {
    strcpy(Devices[iCount].acDevName,"\0");
    Devices[iCount].iDevNo = ARRAY_INIT_VAL;
  }
}

void fAddAddressMap(unsigned long ide,unsigned int add)
{
  int iCount;
  for(iCount = 0; iCount < ADD_COND_RULE_SIZE ; iCount++)
  {
    if((AddressMapMac[iCount] == ide) || (AddressMapMac[iCount] == 0))
    {
      AddressMapMac[iCount] = ide;
      AddressMapShAdd[iCount] = add;
      break;
    }
  }
}

void can_tx_29(unsigned long id, int rtr, int len, unsigned char data[])
{
  unsigned int temp = 0,iCount,iBufCount = 3;
  temp = id >> 16;
  temp = temp | 0x0003;
  ecan1MsgBuf[0][0] = temp;
  temp = id >> 6;
  temp = temp & 0x0FFF;
  ecan1MsgBuf[0][1] = temp;
  temp = id << 10;
  temp = temp & 0xFC00;
  if(rtr == 1)
      temp = temp | 0x0200;
  temp = temp | len;
  ecan1MsgBuf[0][2] = temp;
  if(len > 0)
  {
    ecan1MsgBuf[0][iBufCount] = 0;
    for(iCount = 0; iCount < len; iCount++)
    {
      if(iCount%2 != 0)
      {
        temp = data[iCount];
        temp = temp << 8;
        ecan1MsgBuf[0][iBufCount] = ecan1MsgBuf[0][iBufCount] | temp;
        iBufCount++;
      }
      else
        ecan1MsgBuf[0][iBufCount] = data[iCount];
    }
  }
  // request transmission
  C1TR01CONbits.TXREQ0 = 1;
}

void fSend(unsigned long iMsgId)
{
  iMask = 0x04000000;
  iMsgId = iMask | iMsgId;
  can_tx_29(iMsgId,0,0,0);
}

void fOpenSession(unsigned long iAdd)
{
  iMask = 0x06000000;
  iSessionAdd = iAdd;
  iAdd = iMask | (iAdd << 16);
  can_tx_29(iAdd,1,0,0);
  OpnSesnCount = 100;
}

void fResetSession()
{
  iMask = 0x06000002;
  iAdd = iSessionAdd;
  iAdd = iMask | (iAdd << 16);
  can_tx_29(iAdd,1,0,0);
}

void fReadSession(unsigned long iRegAdd)
{
  iMask = 0x06000003;
  iAdd = iSessionAdd;
  unsigned char data[2];
  iAdd = iMask | (iAdd << 16);
  data[0] = iRegAdd;
  data[1] = iRegAdd >> 8;
  can_tx_29(iAdd,0,2,data);
}

void fWriteSession(unsigned long iRegAdd,unsigned long iRegVal)
{
  iMask = 0x06000005;
  iAdd = iSessionAdd;
  unsigned char data[4];
  iAdd = iMask | (iAdd << 16);
  data[0] = iRegAdd;
  data[1] = iRegAdd >> 8;
  data[2] = iRegVal;
  data[3] = iRegVal >> 8;
  can_tx_29(iAdd,0,4,data);
}

void fGetSessionMac()
{
  iMask = 0x06000008;
  iAdd = iSessionAdd;
  iAdd = iMask | (iAdd << 16);
  can_tx_29(iAdd,1,0,0);
}

void fSetbit(int iPinNo)
{
  if(iPinNo > 15)
  {
    iTemp = iPinNo-16;
    iTemp = 1 << iTemp;
    LATB = LATB | iTemp;
  }
  else
  {
    iTemp = 1 << iPinNo;
    LATA = LATA | iTemp;
  }
}

void fClearbit(int iPinNo)
{
  if(iPinNo>15)
  {
    iTemp = iPinNo-16;
    iTemp = 1 << iTemp;
    iTemp = ~iTemp;
    LATB = LATB & iTemp;
  }
  else
  {
    iTemp = iPinNo;
    iTemp = 1 << iTemp;
    iTemp = ~iTemp;
    LATA = LATA & iTemp;
  }
}

void fTogglebit(int iPinNo)
{
  int iCheck;
  int iTemp1;
  if(iPinNo > 15)
  {
    iTemp = iPinNo-16;
    iCheck = LATB >> iTemp;
    iCheck = iCheck & 0x0001;
    if(iCheck == 0)
    {
      iTemp1 = 1 << iTemp;
      LATB = LATB | iTemp1;
    }
    else if(iCheck == 1)
    {
      iTemp1 = 1 << iTemp;
      iTemp1 = ~iTemp1;
      LATB = LATB & iTemp1;
    }
  }
  else
  {
    iTemp = iPinNo;
    iCheck = LATA >> iTemp;
    iCheck = iCheck & 0x0001;
    if(iCheck == 0)
    {
      iTemp1 = 1 << iTemp;
      LATA = LATA | iTemp1;
    }
    else if(iCheck == 1)
    {
      iTemp1 = 1 << iTemp;
      iTemp1 = ~iTemp1;
      LATA = LATA & iTemp1;
    }
  }
}

void fPwm(unsigned int iVal)
{
  LATBbits.LATB3 = 1;
  OC1RS = iVal;
}

void fBindSession(int iDeviceNo,int iInOutNo, int iPinNo)
{
  iMask= 0x08000001, iAdd = iSessionAdd;
  unsigned char data[6];
  iMask = iMask | (iAdd << 16);
  iAdd = iMask | (iDeviceNo << 8);
  data[0]=iInOutNo;
  data[1]=iPinNo;
  data[2]=0;
  data[3]=0;
  data[4]=0;
  data[5]=0;
  can_tx_29(iAdd,0,6,data);
}

void fEraseBindSession(int iDevNo)
{
  iMask= 0x08000001, iAdd = iSessionAdd;
  iMask = iMask | (iAdd << 16);
  iAdd = iMask | (iDevNo << 8);
  can_tx_29(iAdd,0,0,0);
}

void fCondSession(int iCondNo,int iDevNo,int iIeqNo,unsigned long iVal)
{
  iMask = 0x0A000001;
  iAdd = iSessionAdd;
  unsigned char data[4];
  iMask = iMask | (iAdd << 16);
  iAdd = iMask | (iCondNo << 8);
  if((iCondNo <= 254) && (iVal <= 65535))
  {
    data[0] = iDevNo;
    data[1] = iIeqNo;
    data[2] = iVal;
    data[3] = (iVal >> 8);
    can_tx_29(iAdd,0,4,data);
  }
  else
  {
    fSerialPutS("Condition Invalid\r\n");
  }
}

void fEraseCondSession(int iCondNo)
{
  iMask = 0x0A000001;
  iAdd = iSessionAdd;
  iMask = iMask | (iAdd << 16);
  iAdd = iMask | (iCondNo << 8);
  can_tx_29(iAdd,0,0,0);
}

void fMsgRuleSession(int iRuleNo,unsigned long iMsg,int iDevNo,int iVerbNo,int iVal)
{
  iMask = 0x0E000002;
  iAdd = iSessionAdd;
  unsigned char data[8];
  iMask = iMask | (iAdd << 16);
  iAdd = iMask | (iRuleNo << 8);
  data[0] = 0x12;
  data[1] = iMsg;
  data[2] = iMsg >> 8;
  data[3] = iMsg >> 16;
  data[4] = 0;
  data[5] = iDevNo;
  data[6] = iVerbNo;
  data[7] = iVal;
  can_tx_29(iAdd,0,8,data);
}

void fCondRuleSession(int iRuleNo, int iCondNo, unsigned long iMsg)
{
  iMask = 0x0E000002 , iAdd = iSessionAdd;
  unsigned char data[8];
  iMask = iMask | (iAdd << 16);
  iAdd = iMask | (iRuleNo << 8);
  data[0] = 0x21;
  data[1] = iMsg;
  data[2] = iMsg >> 8;
  data[3] = iMsg >> 16;
  data[4] = iCondNo;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;
  can_tx_29(iAdd,0,8,data);
}

void fEraseRuleSession(int iRuleNo)
{
  iMask = 0x0E000002 , iAdd = iSessionAdd;
  iMask = iMask | (iAdd << 16);
  iAdd = iMask | (iRuleNo << 8);
  can_tx_29(iAdd,0,0,0);
}

void fMsgRuleSetting(unsigned long iMsg, int iDevNo, int iVerb,int iVal,int iRuleNo)
{
  int iCount;
  for( iCount = 0; iCount < ADD_COND_RULE_SIZE; iCount++)
  {
    if( aiRuleArray[iCount][3] == iRuleNo
		|| aiRuleArray[iCount][3] == ARRAY_INIT_VAL )
    {
      aiRuleArrayMsg[iCount] = iMsg;
      aiRuleArray[iCount][0] = iDevNo;
      aiRuleArray[iCount][1] = iVerb;
      if(iVerb == 4)
        aiRuleArray[iCount][2] = iVal;
      aiRuleArray[iCount][3] = iRuleNo;
      break;
    }
  }
}

void fCondRuleSetting(unsigned long iMsg, int iCondNo,int iRuleNo)
{
  int iCount;
  for( iCount = 0; iCount < ADD_COND_RULE_SIZE; iCount++)
  {
    if( aiRuleArray[iCount][3] == iRuleNo
		|| aiRuleArray[iCount][3] == ARRAY_INIT_VAL )
    {
      aiRuleArrayMsg[iCount] = iMsg;
      aiRuleArray[iCount][2] = iCondNo;
      aiRuleArray[iCount][3] = iRuleNo;
      break;
    }
  }
}

void fAddress(unsigned long iMsgId,unsigned long iAdd)
{
  iMask = 0x10000000;
  unsigned char cAdd;
  cAdd = iAdd;
  iMsgId = iMask | iMsgId;
  can_tx_29(iMsgId,0,1,&cAdd);
}

void fHandleRecMsg(unsigned long id, int rtr, int len, unsigned char *data)
{
  unsigned long ide,iMsg;
  int mac_bit,type, iRegAdd, *iRegPtr, iRegVal,iSessionType;
  int iAdd,iDevNo,iPinNo,iIO,iTemp,iCondNo,iInEqNo,iVal,iRuleNo;
  int iVerb,iCount,jCount;
  char sMacMessage[MAX_STR_LEN+1] = "";

  mac_bit = (id & 0x10000000) >> 28;

  if(mac_bit == 1)
  {
    ide = id & 0x0FFFFFFF;
    if(len == 0)
      sprintf(sMacMessage,"MAC 0x%lx present\r\n",ide);
    else if(len == 1)
    {
      if(ide == iDevMac)
      iDevAdd = data[0];
      iTemp = ide & 0x0000001;

      if(iTemp == 0)
      sprintf(sMacMessage,"MAC 0x%lx(%d) present\r\n",ide,data[0]);
      else
      {
        data[2] = 0;
        can_tx_29(id,0,2,data);
      }
    }
    else if(len == 2)
      sprintf(sMacMessage,"MAC 0x%lx(%d) present\r\n",ide,data[0]);

    fAddAddressMap(ide,data[0]);
    fSerialPutS(sMacMessage);
  }
  else
  {
    type = id >> 24;
    ide = id & 0x00FFFFFF;

    // temp code
    if(type == 4 && ide == 1)
      LATBbits.LATB3 = 1;
    if(type == 4 && ide == 2)
      LATBbits.LATB3 = 0;
    //

    switch(type)
    {
      case 4: sprintf(sMacMessage,"Messsage %ld received\r\n",ide);
          fSerialPutS(sMacMessage);
          iVerb = 0;
          for(iCount = 0; iCount < ADD_COND_RULE_SIZE; iCount++)
          {
            if(ide == aiRuleArrayMsg[iCount])
            {
              iTemp = aiRuleArray[iCount][0];
              iPinNo = aiBindArray[iTemp][0];
              iVerb = aiRuleArray[iCount][1];
              if(iVerb == 4)
                iVal = aiRuleArray[iCount][2];
              break;
            }
          }
          switch(iVerb)
          {
            case 1: fClearbit(iPinNo);
                break;
            case 2: fSetbit(iPinNo);
                break;
            case 3: fTogglebit(iPinNo);
                break;
            case 4: fPwm(iVal);
            default:break;
          }
          break;
      case 6: iSessionType = (ide & 0x000000FF);
          iAdd = (ide >> 16);
          switch(rtr)
          {
            case 1:
              if(iSessionType == 0 && iAdd == iDevAdd)
              {
                iSessionAdd = iDevAdd;
                can_tx_29(id,0,0,0);
              }
              else if(iSessionType == 2 && iSessionAdd == iAdd)
              {
                can_tx_29(id,0,0,0);
                asm("RESET");
              }
              else if(iSessionType == 8 && iSessionAdd == iAdd)
              {
                data[0] = iDevMac;
                data[1] = iDevMac >> 8;
                data[2] = iDevMac >> 16;
                data[3] = iDevMac >> 24;
                can_tx_29(id,0,4,data);
              }
              break;
            case 0:
              if(iSessionType == 0)
              {
				OpnSesnCount = -1;
                fSerialPutS("Open response received\r\n");
              }
              else if(iSessionType == 3 && iSessionAdd == iAdd && len == 2)
              {
                iRegAdd = data[1];
                iRegAdd = iRegAdd << 8;
                iRegAdd = iRegAdd | data[0];
                iRegPtr = iRegAdd;
                iRegVal = *iRegPtr;
                data[2] = iRegVal;
                data[3] = iRegVal >> 8;
                can_tx_29(((id & 0xFFFFFF00) | 0x00000004),0,4,data);
              }
              else if(iSessionType == 4 && iSessionAdd == iAdd && len == 4)
              {
                iRegVal = data[3];
                iRegVal = iRegVal << 8;
                iRegVal = iRegVal | data[2];
                iTemp = (data[1] << 8) | data[0];
                sprintf(sMacMessage,"Value in the register 0x%x is %d\r\n",
						iTemp,iRegVal);
                fSerialPutS(sMacMessage);
              }
              else if(iSessionType == 5 && iSessionAdd == iAdd && len == 4)
              {
                iRegAdd = data[1];
                iRegAdd = iRegAdd << 8;
                iRegAdd = iRegAdd | data[0];
                iRegVal = data[3];
                iRegVal = iRegVal << 8;
                iRegVal = iRegVal | data[2];
                iRegPtr = iRegAdd;
                *iRegPtr = iRegVal;
                can_tx_29(((id & 0xFFFFFF00) | 0x00000006),0,2,data);
              }
              else if(iSessionType == 6 && iSessionAdd == iAdd && len == 2)
              {
                iTemp = (data[1] << 8) | data[0];
                sprintf(sMacMessage,"Value written to the register 0x%x\r\n",
									iTemp);
                fSerialPutS(sMacMessage);
              }
              else if(iSessionType == 8 && iSessionAdd == iAdd && len == 4)
              {
               sprintf(sMacMessage,"MAC of destination node is 0x%x%x%x%x\r\n",
										data[3],data[2],data[1],data[0]);
               fSerialPutS(sMacMessage);
              }
              break;
          }
          break;
      case 8: iSessionType = (ide & 0x000000FF);
          iAdd = (ide >> 16);
          iDevNo = (ide & 0x00FF00) >> 8;
          if(iSessionType == 1 && iSessionAdd == iAdd && len == 6)
          {
            iPinNo = data[1];
            iIO = data[0];
            aiBindArray[iDevNo][0] = iPinNo;
            aiBindArray[iDevNo][1] = iIO;
            if(iPinNo == aiPinNo[iDevNo])
              {
                switch(iIO)
                {
                  case 4:
                      if(iPinNo > 15)
                      {
                      iTemp = iPinNo - 16;
                      iTemp = 1 << iTemp;
                      TRISB = TRISB | iTemp;
                      iTemp = iPinNo - 16;
                      iTemp = (PORTB >> iTemp) & 1;
                    }
                    else
                    {
                      iTemp = 1 << iPinNo;
                      TRISA = TRISA | iTemp;
                      iTemp = (PORTA >> iPinNo) & 1;
                    }
                    break;
                  case 1:
                    if(iPinNo>15)
                    {
                      iTemp = iPinNo - 16;
                      iTemp = 1 << iTemp;
                      iTemp = ~iTemp;
                      TRISB = TRISB & iTemp;
                      iTemp = iPinNo - 16;
                      iTemp = (PORTB >> iTemp) & 1;
                    }
                    else
                    {
                      iTemp = iPinNo;
                      iTemp = 1 << iTemp;
                      iTemp = ~iTemp;
                      TRISA = TRISA & iTemp;
                      iTemp = (PORTA >> iPinNo) & 1;
                    }
                      break;
                  case 2:
                      fPwmInit();
                      break;

                }
              aiHistory[iDevNo] = iTemp;
                can_tx_29((id & 0xFFFFFFF0),0,2,data);
            }
              else
                fSerialPutS("Pin Number Mismatch\r\n");
          }
          else if(iSessionType == 1 && iSessionAdd == iAdd && len == 0)
          {
            aiBindArray[iDevNo][0] = 0;
            aiBindArray[iDevNo][1] = 0;
            can_tx_29((id & 0xFFFFFFF0),0,0,0);
          }
          else if((iSessionType == 0) && (iAdd == iSessionAdd))
            {
            if(len == 2)
            {
              iIO=data[0];
              iPinNo = data[1];
              switch(iIO)
              {
                case 4:
					sprintf(sMacMessage,"Binding response Pin%d is now input\r\n",iPinNo);
                    fSerialPutS(sMacMessage);
                    break;
                case 1:
                    sprintf(sMacMessage,"Binding response Pin%d is now output\r\n",iPinNo);
                    fSerialPutS(sMacMessage);
                    break;
                case 2:
                    sprintf(sMacMessage,"Binding response Pin%d is now pwm\r\n",iPinNo);
                    fSerialPutS(sMacMessage);
                    break;
              }
            }
            else
              fSerialPutS("Binding Erased\r\n");
            }
          break;
        case 10: iSessionType = (ide & 0x000000FF);
          iAdd = (ide >> 16);
          if((iSessionType == 1) && (iSessionAdd == iAdd))
          {
            iCondNo = (ide & 0x00FF00) >> 8;
            if(len == 4)
            {
              iInEqNo = data[1];
              iTemp = (data[3] << 8);
              iVal = data[2] | iTemp;
              iDevNo = data[0];
              for( iCount = 0; iCount < ADD_COND_RULE_SIZE; iCount++)
              {
                if( aiCondArray[iCount][0] == iCondNo
					|| aiCondArray[iCount][0] == ARRAY_INIT_VAL )
                {
                  aiCondArray[iCount][0] = iCondNo;
                  aiCondArray[iCount][1] = iInEqNo;
                  aiCondArray[iCount][2] = iVal;
                  aiCondArray[iCount][3] = iDevNo;
                  break;
                }
              }
              can_tx_29((id & 0xFFFFFFF0),0,4,data);
            }
            else if(len == 0)
            {
              for( iCount = 0; iCount < ADD_COND_RULE_SIZE; iCount++)
              {
                if( aiCondArray[iCount][0] == iCondNo )
                {
                  for(jCount = iCount + 1;jCount <  ADD_COND_RULE_SIZE; jCount++)
                  {
                    aiCondArray[jCount-1][0] = aiCondArray[jCount][0];
                    aiCondArray[jCount-1][1] = aiCondArray[jCount][1];
                    aiCondArray[jCount-1][2] = aiCondArray[jCount][2];
                    aiCondArray[jCount-1][3] = aiCondArray[jCount][3];
                    if(aiCondArray[jCount][0] == ARRAY_INIT_VAL)
                      break;
                  }
                  break;
                }
                else if(aiCondArray[iCount][0] == ARRAY_INIT_VAL )
                  break;
              }
              can_tx_29((id & 0xFFFFFFF0),0,0,0);
            }
          }
          else if((iSessionType == 0) && (iSessionAdd == iAdd))
          {
            if(len == 0)
            {
              fSerialPutS("Condition Erased\r\n");
            }
              else
              {
                iCondNo = (ide & 0x00FF00) >> 8;
              fSerialPutS("Condition Set Successfully\r\n");
            }
          }
          break;
      case 14: iSessionType = (ide & 0x000000FF);
          iAdd = (ide >> 16);
          iRuleNo = (ide & 0x00FF00) >> 8;
          if((iSessionType == 2) && (iSessionAdd == iAdd) && len == 8)
          {
            iTemp = data[3] << 8;
            iMsg = (iTemp | data[2]) << 8;
            iMsg = iMsg | data[1];
            iTemp = data[0];
            if(iTemp == 18)
            {
                iVerb = data[6];
              iDevNo = data[5];
              iVal = data[7];
              fMsgRuleSetting(iMsg,iDevNo,iVerb,iVal,iRuleNo);
              can_tx_29((id & 0xFFFFFFF0),0,8,data);
            }
            else
            {
              iCondNo = data[4];
              fCondRuleSetting(iMsg,iCondNo,iRuleNo);
              data[0] = 0x21;
              data[5] = 0;
              data[4] = iCondNo;
              data[6] = 0;
              can_tx_29((id & 0xFFFFFFF0),0,8,data);
            }
          }
          else if((iSessionType == 2) && (iSessionAdd == iAdd) && len == 0)
          {
            for( iCount = 0; iCount < ADD_COND_RULE_SIZE; iCount++)
            {
              if( aiRuleArray[iCount][3] == iRuleNo )
              {
                for(jCount = iCount + 1;jCount <  ADD_COND_RULE_SIZE; jCount++)
                {
                  aiRuleArrayMsg[jCount-1] = aiRuleArrayMsg[jCount];
                  aiRuleArray[jCount-1][0] = aiRuleArray[jCount][0];
                  aiRuleArray[jCount-1][1] = aiRuleArray[jCount][1];
                  aiRuleArray[jCount-1][2] = aiRuleArray[jCount][2];
                  aiRuleArray[jCount-1][3] = aiRuleArray[jCount][3];
                  if(aiRuleArray[jCount][3] == ARRAY_INIT_VAL)
                    break;
                }
                break;
              }
              else if(aiRuleArray[iCount][3] == ARRAY_INIT_VAL )
                break;
            }
            can_tx_29((id & 0xFFFFFFF0),0,0,0);
          }
          else if((iSessionType == 0) && (iSessionAdd == iAdd))
          {
            if(len == 8)
            {
              iCondNo = data[4];
              iTemp = data[0];
              if(iTemp == 18)
              {
                iRuleNo = (ide & 0x00FF00) >> 8;
                fSerialPutS("Message Rule Set Successfully\r\n");
              }
              else
              {
                iRuleNo = (ide & 0x00FF00) >> 8;
                fSerialPutS("Condition Rule Set Successfully\r\n");
              }
            }
            else
            fSerialPutS("Rule Erased\r\n");
          }
          break;
      }
  }
}

// parse the command from string
void fParseCommand()
{
  int iCount, iType, iSize = strlen(sCommand);
  iParamCount = 0;
  int iIneqNo, iVerb, iIOvalue;
  int jCount, iDevNo, iCondNo;
  for(iCount = 0; iCount < iSize; iCount++)
  {
    iType = fParseType(sCommand[iCount]);
    if(iType == TYPE_DELIM)
    {
      sCommand[iCount] = '\0';
    }
    else
    {
      aiOffset[iParamCount] = iCount;
      aiType[iParamCount] = iType;
      iCount++;

      while(fParseType(sCommand[iCount]) != TYPE_DELIM)
          iCount++;

      iCount--;
      iParamCount++;
    }
  }
  iParamCount--;

  if(Test("send",1))
  {
    fSend(fGetNumber(1));
  }
  else if(Test("address",2))
  {
    fAddress(fGetNumber(1),fGetNumber(2));
  }
  else if(Test("loopback",1))
  {
    if(strcmp(&sCommand[aiOffset[1]],"on") == 0)
      C1CTRL1bits.REQOP = 2;
    else if(strcmp(&sCommand[aiOffset[1]],"off") == 0)
      C1CTRL1bits.REQOP = 0;
  }
  else if(Test("open",1))
  {
    fOpenSession(fGetNumber(1));
  }
  else if(Test("reset",0))
  {
    fResetSession();
  }
  else if(Test("read",1))
  {
    fReadSession(fGetNumber(1));
  }
  else if(Test("write",2))
  {
    fWriteSession(fGetNumber(1),fGetNumber(2));
  }
  else if(Test("mac",0))
  {
    fGetSessionMac();
  }
  else if(Test("bind",3))
  {
    int iPinNo = fGetNumber(3);

    for(iCount = 0; iCount < 12; iCount++)
    {
      if(strcmp(Devices[iCount].acDevName,&sCommand[aiOffset[1]]) == 0
		|| strcmp(Devices[iCount].acDevName,"\0") == 0)
      {
        strcpy(Devices[iCount].acDevName,&sCommand[aiOffset[1]]);
        for(jCount = 0; jCount < 12; jCount++)
        {
          if(aiPinNo[jCount] == iPinNo)
          {
            iDevNo = jCount;
            Devices[iCount].iDevNo = iDevNo;
            break;
          }
        }
        break;
      }
    }

    if(strcmp(&sCommand[aiOffset[2]],"in") == 0)
      iIOvalue = 4;
    else if(strcmp(&sCommand[aiOffset[2]],"out") == 0)
      iIOvalue = 1;
    else if(strcmp(&sCommand[aiOffset[2]],"pwm") == 0)
      iIOvalue = 2;

    fBindSession(iDevNo,iIOvalue,iPinNo);
  }
  else if(Test("bind",2) && strcmp(&sCommand[aiOffset[2]],"erase") == 0)
  {
    iDevNo = ARRAY_INIT_VAL;
    for(iCount = 0; iCount < 12; iCount++)
    {
      if(strcmp(Devices[iCount].acDevName,&sCommand[aiOffset[1]]) == 0)
      {
        iDevNo = Devices[iCount].iDevNo;
        for(jCount = iCount+1; jCount < 12; jCount++)
        {
          strcpy(Devices[jCount-1].acDevName,Devices[jCount].acDevName);
          Devices[jCount-1].iDevNo = Devices[jCount].iDevNo;
          if(Devices[jCount].iDevNo == ARRAY_INIT_VAL)
            break;
        }
        break;
      }
      else if(strcmp(Devices[iCount].acDevName,"\0") == 0)
        break;
    }
    if(iDevNo != ARRAY_INIT_VAL)
      fEraseBindSession(iDevNo);
  }
  else if(Test("cond",4))
  {
    iDevNo = ARRAY_INIT_VAL, iCondNo = ARRAY_INIT_VAL;
    for(iCount = 0; iCount < ADD_COND_RULE_SIZE; iCount++)
    {
      if(strcmp(Conditions[iCount].acCondName,&sCommand[aiOffset[1]]) == 0 )
      {
        iCondNo = Conditions[iCount].iCondNo;
        break;
      }
      else if(strcmp(Conditions[iCount].acCondName,"\0") == 0)
      {
        strcpy(Conditions[iCount].acCondName,&sCommand[aiOffset[1]]);
        if(iCount == 0)
          iCondNo = 0;
        else
          iCondNo = Conditions[iCount-1].iCondNo + 1;
        Conditions[iCount].iCondNo = iCondNo;
        break;
      }
    }

    if(iCondNo == ARRAY_INIT_VAL)
    {
      fSerialPutS("Condition could not be set\r\n");
    }
    else
    {
      for(iCount = 0; iCount < 12; iCount++)
      {
        if(strcmp(Devices[iCount].acDevName,&sCommand[aiOffset[2]]) == 0 )
        {
          iDevNo = Devices[iCount].iDevNo;
          break;
        }
      }

      if(iDevNo == ARRAY_INIT_VAL)
      {
        fSerialPutS("Device name invalid\r\n");
      }
      else
      {
        if(strcmp(&sCommand[aiOffset[3]],"=")==0)
          iIneqNo = 1;
        else if(strcmp(&sCommand[aiOffset[3]],"!=")==0)
          iIneqNo = 2;
        else if(strcmp(&sCommand[aiOffset[3]],"<")==0)
          iIneqNo = 3;
        else if(strcmp(&sCommand[aiOffset[3]],"<=")==0)
          iIneqNo = 4;
        else if(strcmp(&sCommand[aiOffset[3]],">=")==0)
          iIneqNo = 5;
        else if(strcmp(&sCommand[aiOffset[3]],">")==0)
          iIneqNo = 6;
        fCondSession(iCondNo,iDevNo,iIneqNo,fGetNumber(4));
      }
    }
  }
  else if(Test("cond",2) && strcmp(&sCommand[aiOffset[2]],"erase") == 0)
  {
    iCondNo = ARRAY_INIT_VAL;
    for(iCount = 0; iCount < ADD_COND_RULE_SIZE; iCount++)
    {
      if(strcmp(Conditions[iCount].acCondName,&sCommand[aiOffset[1]]) == 0 )
      {
        iCondNo = Conditions[iCount].iCondNo;
        for(jCount = iCount+1; jCount < ADD_COND_RULE_SIZE; jCount++)
        {
          strcpy(Conditions[jCount-1].acCondName,Conditions[jCount].acCondName);
          Conditions[jCount-1].iCondNo = Conditions[jCount].iCondNo;
          if(Conditions[jCount].iCondNo == ARRAY_INIT_VAL)
          {
            break;
          }
        }
        break;
      }
      else if(strcmp(Conditions[iCount].acCondName,"\0") == 0)
      {
        break;
      }
    }

    if(iCondNo == ARRAY_INIT_VAL)
    {
      fSerialPutS("Condition name Invalid\r\n");
    }
    else
    {
      fEraseCondSession(iCondNo);
    }
  }
  else if(Test("rule",6))
  {
    iDevNo = ARRAY_INIT_VAL, iCondNo = ARRAY_INIT_VAL;
    int iRuleNo = ARRAY_INIT_VAL, iVal = 0;
    if(strcmp(&sCommand[aiOffset[2]],"on") == 0)
    {
      for(iCount = 0; iCount < ADD_COND_RULE_SIZE; iCount++)
      {
        if(strcmp(Rules[iCount].acRuleName,&sCommand[aiOffset[1]]) == 0 )
        {
          iRuleNo = Rules[iCount].iRuleNo;
          break;
        }
        else if(strcmp(Rules[iCount].acRuleName,"\0") == 0)
        {
          strcpy(Rules[iCount].acRuleName,&sCommand[aiOffset[1]]);
          if(iCount == 0)
            iRuleNo = 0;
          else
            iRuleNo = Rules[iCount-1].iRuleNo + 1;
          Rules[iCount].iRuleNo = iRuleNo;
          break;
        }
      }

      if(iRuleNo == ARRAY_INIT_VAL)
      {
        fSerialPutS("Rule could not be set\r\n");
      }
      else
      {
        if(strcmp(&sCommand[aiOffset[3]],"cond")== 0)
        {
          for(iCount = 0; iCount < ADD_COND_RULE_SIZE; iCount++)
          {
            if(strcmp(Conditions[iCount].acCondName,&sCommand[aiOffset[4]])==0)
            {
              iCondNo = Conditions[iCount].iCondNo;
              break;
            }
          }

          if(iCondNo == ARRAY_INIT_VAL)
          {
            fSerialPutS("Condition name invalid\r\n");
          }
          else
          {
            fCondRuleSession(iRuleNo,iCondNo,fGetNumber(6));
          }
        }
        else if(strcmp(&sCommand[aiOffset[3]],"msg") == 0)
        {
          for(iCount = 0; iCount < 12; iCount++)
          {
            if(strcmp(Devices[iCount].acDevName,&sCommand[aiOffset[5]]) == 0 )
            {
              iDevNo = Devices[iCount].iDevNo;
              break;
            }
          }

          if(iDevNo == ARRAY_INIT_VAL)
          {
            fSerialPutS("Device name invalid\r\n");
          }
          else
          {
            if(strcmp(&sCommand[aiOffset[6]],"off") == 0)
              iVerb = 1;
            else if(strcmp(&sCommand[aiOffset[6]],"on") == 0)
              iVerb = 2;
            else if(strcmp(&sCommand[aiOffset[6]],"toggle") == 0)
              iVerb = 3;
            else if(strcmp(&sCommand[aiOffset[6]],"set") == 0)
            {
              iVerb = 4;
              iVal = fGetNumber(7);
            }
            fMsgRuleSession(iRuleNo,fGetNumber(4),iDevNo,iVerb,iVal);
          }
        }
      }
    }
  }
  else if(Test("rule",2) || strcmp(&sCommand[aiOffset[2]],"erase") == 0)
  {
    int iRuleNo = ARRAY_INIT_VAL;
    for(iCount = 0; iCount < ADD_COND_RULE_SIZE; iCount++)
    {
      if(strcmp(Rules[iCount].acRuleName,&sCommand[aiOffset[1]]) == 0 )
      {
        iRuleNo = Rules[iCount].iRuleNo;
        for(jCount = iCount+1; jCount < ADD_COND_RULE_SIZE; jCount++)
        {
          strcpy(Rules[jCount-1].acRuleName,Rules[jCount].acRuleName);
          Rules[jCount-1].iRuleNo = Rules[jCount].iRuleNo;
          if(Rules[jCount].iRuleNo == ARRAY_INIT_VAL)
            break;
        }
        break;
      }
      else if(strcmp(Rules[iCount].acRuleName,"\0") == 0)
      {
        break;
      }
    }

    if(iRuleNo == ARRAY_INIT_VAL)
    {
      fSerialPutS("Rule name Invalid\r\n");
    }
    else
    {
      fEraseRuleSession(iRuleNo);
    }
  }
  iCnt = 0;
}
//-----------------------------------------------------------------------------
// Interrupt functions
//-----------------------------------------------------------------------------
void __attribute__((interrupt, no_auto_psv))_C1Interrupt(void)
{
  unsigned long id = 0;
  unsigned int temp;
  int rtr, len, iCount, ibufIndex = 3;
  unsigned char data[8];

  IFS2bits.C1IF = 0;        // clear interrupt flag
  if(C1INTFbits.TBIF)
  {
    LATBbits.LATB5 = 1;
    Led5Count = 25;
    C1INTFbits.TBIF = 0;
  }
  // handle receive interrupt
  if(C1INTFbits.RBIF)
  {
    LATBbits.LATB4 = 1;
    Led4Count = 25;
    // handle overflow
    if(C1RXFUL1bits.RXFUL1==1)
      C1RXFUL1bits.RXFUL1=0;
    // extract fields, call message handler
    // find out if the message is MAC or otherwise
    temp = ecan1MsgBuf[1][0];
    temp = temp << 3;
    temp = temp & 0xFFE0;
    id = id | temp;
    id = id << 7;
    temp = ecan1MsgBuf[1][1];
    id = id | temp;
    id = id << 6;
    temp = ecan1MsgBuf[1][2];
    id = id | (temp >> 10);
    rtr = (temp & 0x0200) >> 9;
    len = (temp & 0x000F);
    if(len > 0)
    {
      for(iCount = 0; iCount < len; iCount++)
      {
        if(iCount%2 == 0)
        {
          data[iCount] = ecan1MsgBuf[1][ibufIndex];
        }
        else
        {
          data[iCount] = ecan1MsgBuf[1][ibufIndex] >> 8;
          ibufIndex++;
        }
      }
    }

    fHandleRecMsg(id,rtr,len,data);

    C1INTFbits.RBIF = 0;
  }
}

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
  IFS0bits.DMA0IF = 0;          // Clear the DMA0 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void)
{
  IFS0bits.DMA1IF = 0;          // Clear the DMA1 Interrupt Flag;
}

// T1 Interrupt
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt (void)
{

  if(Led3Count >= 0)
    Led3Count--;
  if(Led3Count == 0)
    LATBbits.LATB3 = 0;

  if(Led4Count >= 0)
    Led4Count--;
  if(Led4Count == 0)
    LATBbits.LATB4 = 0;

  if(Led5Count >= 0)
    Led5Count--;
  if(Led5Count == 0)
    LATBbits.LATB5 = 0;

  if(OpnSesnCount >= 0)
    OpnSesnCount--;
  if(OpnSesnCount == 0)
    fSerialPutS("failed to open");

  IFS0bits.T1IF = 0;
}

// UART receive Interrupt routine
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt (void)
{
  char cChar;
  if(iCnt<MAX_STR_LEN)
  {
    cChar = U1RXREG;
    if (cChar == '\b' && iCnt > 0)
    {
      fSerialPutC(' ');
      fSerialPutC('\b');
      iCnt--;
    }
    else if(cChar == '\n' || cChar == '\r')
    {
      sCommand[iCnt] = '\0';
        fSerialPutS("\n");
      fParseCommand();
    }
    else if(cChar >= ' ')
    {
      sCommand[iCnt] = tolower(cChar);
      iCnt++;
    }
  }
  else
  {
    sCommand[iCnt] = '\0';
    fSerialPutS("\n");
    fParseCommand();
  }

  IFS0bits.U1RXIF = 0;
}


//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------


int main(void)
{
  char sMacMessage[MAX_STR_LEN + 1] = "";
  int iCount, jCount, iCondNo, iDevNo, iPinNo, iStatus, iFlag = 0;

  fInitHw();                                 // initialize hardware
  fSerialInit(BAUD_19200);                   // configure uart
  fArrayInit();

  LATBbits.LATB4 = 1;                        // blink green LED for 500ms
  Led4Count = 50;

//fWaitPB();

  fSerialPutS("Menu\r\n"); // print greeting

  // send and siplay MAC on powerup
  can_tx_29(iDevMac | 0x10000000,0,0,0);
  sprintf(sMacMessage,"MAC 0x%lx present(self)\r\n",iDevMac);
  fSerialPutS(sMacMessage);
  if(iDevMac == HI_MAC)
  {
    fAddAddressMap(iDevMac,0);
  }

  fSerialPutS("Enter command\r\n"); // print greeting

  while(TRUE)
  {
    for(iCount = 0; iCount < ADD_COND_RULE_SIZE; iCount++)
    {
      if(aiCondArray[iCount][0] == ARRAY_INIT_VAL)
      {
        break;
      }
      else
      {
        iCondNo = aiCondArray[iCount][0];
        iDevNo = aiCondArray[iCount][3];
        if(aiBindArray[iDevNo][1] == 4)
        {
          iPinNo = aiBindArray[iDevNo][0];
          if(iPinNo > 15)
          {
            iStatus = iPinNo - 16;
            iStatus = (PORTB >> iStatus) & 1;
          }
          else
          {
            iStatus = (PORTA >> iPinNo) & 1;
          }

          switch(aiCondArray[iCount][1])
          {
            case 1: if(aiCondArray[iCount][2] == iStatus
						&& iStatus != aiHistory[iDevNo])
                    {
                      iFlag = 1;
                    }
                    break;
            case 2: if(aiCondArray[iCount][2] != iStatus
						&& iStatus != aiHistory[iDevNo])
                    {
                      iFlag = 1;
                    }
                    break;
          }
          aiHistory[iDevNo] = iStatus;
          if(iFlag == 1)
          {
            for(jCount = 0; jCount < ADD_COND_RULE_SIZE; jCount++)
            {
              if( aiRuleArray[jCount][3] == ARRAY_INIT_VAL)
              {
                break;
              }
              else if( aiRuleArray[jCount][2] == iCondNo
				&& aiRuleArray[jCount][0] == ARRAY_INIT_VAL
              	&& aiRuleArray[jCount][3] != ARRAY_INIT_VAL)
              {
                fSend(aiRuleArrayMsg[jCount]);
              }
            }
          }
        }
        iFlag = 0;
      }
    }
  }
    return 0;
}
