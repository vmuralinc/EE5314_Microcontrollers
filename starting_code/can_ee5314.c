
//-----------------------------------------------------------------------------
// Global variables and defines   
//-----------------------------------------------------------------------------

// assign 32x8word message buffers for ECAN1 in DMA RAM
unsigned int ecan1MsgBuf[32][8] __attribute__((space(dma), aligned(32*16)));

//-----------------------------------------------------------------------------
// CAN TX Functions
//-----------------------------------------------------------------------------
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

//-----------------------------------------------------------------------------
// CAN TX Functions
//-----------------------------------------------------------------------------

void can_tx_29(unsigned long id, int rtr, int len, unsigned char data[])
{
  // TO DO: write up to 8 words in msg bugger
  ecan1MsgBuf[0][0] = ___;
  ecan1MsgBuf[0][1] = ___;
  ecan1MsgBuf[0][2] = ___;
  // request transmission
  C1TR01CONbits.TXREQ0 = 1;
}

//-----------------------------------------------------------------------------

void __attribute__((interrupt, no_auto_psv))_C1Interrupt(void)  
{    
  unsigned long temp, id;
  int ide, rtr, len;
  unsigned char* ptr;      
  IFS2bits.C1IF = 0;        // clear interrupt flag
  if(C1INTFbits.TBIF)
  { 
    C1INTFbits.TBIF = 0;
  }  
  // handle receive interrupt
  if(C1INTFbits.RBIF)
  {      
    // handle overflow
    if(C1RXFUL1bits.RXFUL1==1)
      C1RXFUL1bits.RXFUL1=0;
    // TO DO: extract fields, call message handler
    // x = ecan1MsgBuf[1][0];
    // y = ecan1MsgBuf[1][1];
    C1INTFbits.RBIF = 0;
  }
}

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
   LATBbits.LATB5 = 1;   
   IFS0bits.DMA0IF = 0;          // Clear the DMA0 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void)
{
  LATBbits.LATB4 = 1;   
  IFS0bits.DMA1IF = 0;          // Clear the DMA1 Interrupt Flag;
}
