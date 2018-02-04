// ST7565R Graphics LCD Library
// speed set for Fcyc = 40 MHz

// Hardware description:
// ST7565R-based 128x64 Graphics LCD 
//    ~CS connected to RA1
//    SI connects to RP7/SDO1 (output fn 7) (pin 16)
//    SCL connectes to RP6/SCLK1OUT (output fn 8) (pin 15)
//    A0 connects to  to RB5 (pin 14) 

//------------------------------------------------------------------------------
// Defines              
//------------------------------------------------------------------------------

// Set interface pin locations as appropriate
#define CS_PIN LATAbits.LATA1
#define A0_PIN LATBbits.LATB5

// Set viewing angle depending on how the display is mounted
#define VIEW_600 1
//#define VIEW_1200 1

// Set Pixel arguments
#define CLEAR  0
#define SET    1
#define INVERT 2

//------------------------------------------------------------------------------
// Data memory              
//------------------------------------------------------------------------------

unsigned char pixel_map[1024];
int txt_index = 0;

//------------------------------------------------------------------------------
// Flash memory constants            
//------------------------------------------------------------------------------

// CONST will place this in program memory and use PSV to map into data memory

const char char_gen[100][5] = {
// 96 character 5x7 bitmaps based on ISO-646 (BCT IRV extensions)
// Codes 32-127
// Space ! " % $ % & ' ( ) * + , - . /
  {0b0000000, 0b0000000, 0b0000000, 0b0000000, 0b0000000}, 
  {0b0000000, 0b0000000, 0b1001111, 0b0000000, 0b0000000},
  {0b0000000, 0b0000111, 0b0000000, 0b0000111, 0b0000000},
  {0b0010100, 0b1111111, 0b0010100, 0b1111111, 0b0010100},
  {0b0100100, 0b0101010, 0b1111111, 0b0101010, 0b0010010},
  {0b0100011, 0b0010011, 0b0001000, 0b1100100, 0b1100010},
  {0b0110110, 0b1001001, 0b1010101, 0b0100010, 0b1000000},
  {0b0000000, 0b0000101, 0b0000011, 0b0000000, 0b0000000},
  {0b0000000, 0b0011100, 0b0100010, 0b1000001, 0b0000000},
  {0b0000000, 0b1000001, 0b0100010, 0b0011100, 0b0000000},
  {0b0010100, 0b0001000, 0b0111110, 0b0001000, 0b0010100},
  {0b0001000, 0b0001000, 0b0111110, 0b0001000, 0b0001000},
  {0b0000000, 0b1010000, 0b0110000, 0b0000000, 0b0000000},
  {0b0001000, 0b0001000, 0b0001000, 0b0001000, 0b0001000},
  {0b0000000, 0b1100000, 0b1100000, 0b0000000, 0b0000000},
  {0b0100000, 0b0010000, 0b0001000, 0b0000100, 0b0000010},
// 0-9
  {0b0111110, 0b1010001, 0b1001001, 0b1000101, 0b0111110},
  {0b0000000, 0b1000010, 0b1111111, 0b1000000, 0b0000000},
  {0b1000010, 0b1100001, 0b1010001, 0b1001001, 0b1000110},
  {0b0100001, 0b1000001, 0b1000101, 0b1001011, 0b0110001},
  {0b0011000, 0b0010100, 0b0010010, 0b1111111, 0b0010000},
  {0b0100111, 0b1000101, 0b1000101, 0b1000101, 0b0111001},
  {0b0111100, 0b1001010, 0b1001001, 0b1001001, 0b0110000},
  {0b0000001, 0b1110001, 0b0001001, 0b0000101, 0b0000011},
  {0b0110110, 0b1001001, 0b1001001, 0b1001001, 0b0110110},
  {0b0000110, 0b1001001, 0b1001001, 0b0101001, 0b0011110},
// : ; < = > ? @
  {0b0000000, 0b0110110, 0b0110110, 0b0000000, 0b0000000},
  {0b0000000, 0b1010110, 0b0110110, 0b0000000, 0b0000000},
  {0b0001000, 0b0010100, 0b0100010, 0b1000001, 0b0000000},
  {0b0010100, 0b0010100, 0b0010100, 0b0010100, 0b0010100},
  {0b0000000, 0b1000001, 0b0100010, 0b0010100, 0b0001000},
  {0b0000010, 0b0000001, 0b1010001, 0b0001001, 0b0111110},
  {0b0110010, 0b1001001, 0b1111001, 0b1000001, 0b0111110},
// A-Z
  {0b1111110, 0b0010001, 0b0010001, 0b0010001, 0b1111110},
  {0b1111111, 0b1001001, 0b1001001, 0b1001001, 0b0110110},
  {0b0111110, 0b1000001, 0b1000001, 0b1000001, 0b0100010},
  {0b1111111, 0b1000001, 0b1000001, 0b0100010, 0b0011100},
  {0b1111111, 0b1001001, 0b1001001, 0b1001001, 0b1000001},
  {0b1111111, 0b0001001, 0b0001001, 0b0001001, 0b0000001},
  {0b0111110, 0b1000001, 0b1001001, 0b1001001, 0b0111010},
  {0b1111111, 0b0001000, 0b0001000, 0b0001000, 0b1111111},
  {0b0000000, 0b1000001, 0b1111111, 0b1000001, 0b0000000},
  {0b0100000, 0b1000000, 0b1000001, 0b0111111, 0b0000001},
  {0b1111111, 0b0001000, 0b0010100, 0b0100010, 0b1000001},
  {0b1111111, 0b1000000, 0b1000000, 0b1000000, 0b1000000},
  {0b1111111, 0b0000010, 0b0001100, 0b0000010, 0b1111111},
  {0b1111111, 0b0000100, 0b0001000, 0b0010000, 0b1111111},
  {0b0111110, 0b1000001, 0b1000001, 0b1000001, 0b0111110},
  {0b1111111, 0b0001001, 0b0001001, 0b0001001, 0b0000110},
  {0b0111110, 0b1000001, 0b1010001, 0b0100001, 0b1011110},
  {0b1111111, 0b0001001, 0b0011001, 0b0101001, 0b1000110},
  {0b1000110, 0b1001001, 0b1001001, 0b1001001, 0b0110001},
  {0b0000001, 0b0000001, 0b1111111, 0b0000001, 0b0000001},
  {0b0111111, 0b1000000, 0b1000000, 0b1000000, 0b0111111},
  {0b0011111, 0b0100000, 0b1000000, 0b0100000, 0b0011111},
  {0b0111111, 0b1000000, 0b1110000, 0b1000000, 0b0111111},
  {0b1100011, 0b0010100, 0b0001000, 0b0010100, 0b1100011},
  {0b0000111, 0b0001000, 0b1110000, 0b0001000, 0b0000111},
  {0b1100001, 0b1010001, 0b1001001, 0b1000101, 0b1000011},
// [ \ ] ^ _ `
  {0b0000000, 0b1111111, 0b1000001, 0b1000001, 0b0000000},
  {0b0000010, 0b0000100, 0b0001000, 0b0010000, 0b0100000},
  {0b0000000, 0b1000001, 0b1000001, 0b1111111, 0b0000000},
  {0b0000100, 0b0000010, 0b0000001, 0b0000010, 0b0000100},
  {0b1000000, 0b1000000, 0b1000000, 0b1000000, 0b1000000},
  {0b0000000, 0b0000001, 0b0000010, 0b0000100, 0b0000000},
// a-z
  {0b0100000, 0b1010100, 0b1010100, 0b1010100, 0b1111000},
  {0b1111111, 0b1000100, 0b1000100, 0b1000100, 0b0111000},
  {0b0111000, 0b1000100, 0b1000100, 0b1000100, 0b0100000},
  {0b0111000, 0b1000100, 0b1000100, 0b1001000, 0b1111111},
  {0b0111000, 0b1010100, 0b1010100, 0b1010100, 0b0011000},
  {0b0001000, 0b1111110, 0b0001001, 0b0000001, 0b0000010},
  {0b0001100, 0b1010010, 0b1010010, 0b1010010, 0b0111110},
  {0b1111111, 0b0001000, 0b0000100, 0b0000100, 0b1111000},
  {0b0000000, 0b1000100, 0b1111101, 0b1000000, 0b0000000},
  {0b0100000, 0b1000000, 0b1000100, 0b0111101, 0b0000000},
  {0b1111111, 0b0010000, 0b0101000, 0b1000100, 0b0000000},
  {0b0000000, 0b1000001, 0b1111111, 0b1000000, 0b0000000},
  {0b1111100, 0b0000100, 0b0011000, 0b0000100, 0b1111000},
  {0b1111100, 0b0001000, 0b0000100, 0b0000100, 0b1111000},
  {0b0111000, 0b1000100, 0b1000100, 0b1000100, 0b0111000},
  {0b1111100, 0b0010100, 0b0010100, 0b0010100, 0b0001000},
  {0b0001000, 0b0010100, 0b0010100, 0b0011000, 0b1111100},
  {0b1111100, 0b0001000, 0b0000100, 0b0000100, 0b0001000},
  {0b1001000, 0b1010100, 0b1010100, 0b1010100, 0b0100000},
  {0b0000100, 0b0111111, 0b1000100, 0b1000000, 0b0100000},
  {0b0111100, 0b1000000, 0b1000000, 0b0100000, 0b1111100},
  {0b0011100, 0b0100000, 0b1000000, 0b0100000, 0b0011100},
  {0b0111100, 0b1000000, 0b0100000, 0b1000000, 0b0111100},
  {0b1000100, 0b0101000, 0b0010000, 0b0101000, 0b1000100},
  {0b0001100, 0b1010000, 0b1010000, 0b1010000, 0b0111100},
  {0b1000100, 0b1100100, 0b1010100, 0b1001100, 0b1000100},
// { | } ~ cc
  {0b0000000, 0b0001000, 0b0110110, 0b1000001, 0b0000000},
  {0b0000000, 0b0000000, 0b1111111, 0b0000000, 0b0000000},
  {0b0000000, 0b1000001, 0b0110110, 0b0001000, 0b0000000},
  {0b0001100, 0b0000100, 0b0011100, 0b0010000, 0b0011000},
  {0b0000000, 0b0000000, 0b0000000, 0b0000000, 0b0000000},
// Assignments beyond ISO646
// Codes 128+: right arrow, left arrow, deg
  {0b0001000, 0b0001000, 0b0101010, 0b0011100, 0b0001000},
  {0b0001000, 0b0011100, 0b0101010, 0b0001000, 0b0001000},
  {0b0000111, 0b0000101, 0b0000111, 0b0000000, 0b0000000},
};

// SPI functions
void spi_write(unsigned char data)
{
  SPI1BUF = data;
}

unsigned char spi_read()
{
  while(!SPI1STATbits.SPIRBF);
  return SPI1BUF;
}

void gr_cs_on()
{
  CS_PIN = 0;
  __delay32(100);  
}

void gr_cs_off()
{
  CS_PIN = 1;
}

void gr_spi_init()
{
  // disable spi in case it is already running
  // to allow changes in configuration
  SPI1STATbits.SPIEN = 0;

  // set to highest rate <= 1 Mbps
  // 40 MHz / 16 / 3 = 833 kbps
  // 16:1 primary prescale, 1:3 secondary prescale
  SPI1CON1bits.PPRE = 1;
  SPI1CON1bits.SPRE = 5;
  // 8 bit
  SPI1CON1bits.MODE16 = 0;
  // idle state of clock is high for ST7565R controller
  SPI1CON1bits.CKP = 1;
  // data out updates on idle-to-active edge (falling) for ST7565R
  SPI1CON1bits.CKE = 0;
  // master mode
  SPI1CON1bits.MSTEN = 1;

  // frame and enhanced buffer disabled
  SPI1CON2 = 0;

  // enable spi, continue spi on idle, clr overflow
  SPI1STATbits.SPIROV = 0;
  SPI1STATbits.SPISIDL = 0;
  SPI1STATbits.SPIEN = 1;

  // asset cs for lcd display
  gr_cs_on();
}

void gr_command(int cmd)
{
  // A0 low for command
  A0_PIN = 0;
 
  // let line settle 
  __delay32(100);

  spi_write(cmd);
  spi_read();  
}

void gr_data(int data)
{
  // A0 high for data
  A0_PIN = 1;
 
  // let line settle 
  __delay32(100);

  spi_write(data);
  spi_read();  
}

void gr_set_page(int page)
{
  gr_command(0xB0 | page);
}

void gr_set_column(int x)
{
  #if VIEW_600
  gr_command(0x10 | ((x >> 4) & 0x0F));
  gr_command(0x00 | (x & 0x0F));
  #endif
  #if VIEW_1200
  gr_command(0x10 | (((x+4) >> 4) & 0x0F));
  gr_command(0x00 | ((x+4) & 0x0F));
  #endif
}

void gr_refresh_screen()
{
  int x, page, i = 0;
  for (page = 0; page < 8; page ++)
  {
    gr_set_page(page);
    gr_set_column(0);
    for (x = 0; x < 128; x++)
      gr_data(pixel_map[i++]);
  }    
}

void gr_clear()
{
  int i;
  // clear data memory pixel map
  for (i = 0; i < 1024; i++)
    pixel_map[i] = 0;
  // copy to display
  gr_refresh_screen();
}

void gr_init()
{
  // wait for graphics controller to leave reset
  __delay_ms(10);

  // configure spi
  gr_spi_init(); 

  gr_command(0x40); // set start line to 0
  #if VIEW_600
  gr_command(0xA1); // reverse horizontal order
  gr_command(0xC0); // normal vertical order
  #endif
  #if VIEW_1200
  gr_command(0xA0); // normal horizontal order
  gr_command(0xC8); // reverse vertical order
  #endif
  gr_command(0xA6); // normal pixel polarity
  gr_command(0xA2); // set led bias to 1/9
  gr_command(0x2F); // turn on voltage booster and regulator
  gr_command(0xF8); // set internal volt booster to 4x Vdd
  gr_command(0x00);
  gr_command(0x27); // set contrast
  gr_command(0x81); // set LCD drive voltage
  gr_command(0x16);
  gr_command(0xAC); // no flashing indicator
  gr_command(0x04);
  gr_clear();       // clear pixels
  gr_command(0xAF); // display on
}

void gr_set_pixel(int x, int y, int op)
{
  unsigned char data;
  int mask, page, index;
  // determine pixel map entry
  page = y >> 3;
  // determine pixel map index
  index = page << 7 | x;
  // generate mask 
  mask = 1 << (y & 7);  
  // read pixel map
  data = pixel_map[index];
  // apply operator (0 = clear, 1 = set, 2 = xor)
  switch(op)
  {
    case 0: data &= ~mask; break;
    case 1: data |= mask; break;
    case 2: data ^= mask; break;
  }
  // write to pixel map
  pixel_map[index] = data;
  // write to display
  gr_set_page(page);
  gr_set_column(x);
  gr_data(data);
}

void gr_rectangle(int xul, int yul, int dx, int dy, int op)
{
  int page, page_start, page_stop;
  int bit_index, bit_start, bit_stop;
  unsigned char mask, data;  
  int index;
  int x;
  // determine pages for rectangle
  page_start = yul >> 3;
  page_stop = (yul + dy - 1) >> 3;
  // draw in pages from top to bottom within extent
  for (page = page_start; page <= page_stop; page++)
  {
    // calculate mask for this page
    if (page > page_start)
      bit_start = 0;
    else
      bit_start = yul & 7;
    if (page < page_stop)
      bit_stop = 7;
    else
      bit_stop = (yul + dy - 1) & 7;
    mask = 0;
    for (bit_index = bit_start; bit_index <= bit_stop; bit_index++)
       mask |= 1 << bit_index;
    // write page
    gr_set_page(page);
    gr_set_column(xul);
    index = (page << 7) | xul;
    for (x = 0; x < dx; x++)
    {
      // read pixel map
      data = pixel_map[index];
      // apply operator (0 = clear, 1 = set, 2 = xor)
      switch(op)
      {
        case 0: data &= ~mask; break;
        case 1: data |= mask; break;
        case 2: data ^= mask; break;
      }
      // write to pixel map
      pixel_map[index++] = data;
      // write to display
      gr_data(data);
    }
  }
}

void gr_set_txt_pos(int x, int page)
{
  txt_index = (page << 7) + x;
  gr_set_page(page);
  gr_set_column(x);
}

void gr_putc(char c) 
{
  int i, val;
  unsigned char uc;
  // convert to unsigned to access characters > 127
  uc = (unsigned char) c;
  for (i = 0; i < 5; i++)
  {
    val = char_gen[uc-' '][i];    
    pixel_map[txt_index++] = val;
    gr_data(val);
  }
    pixel_map[txt_index++] = 0;
    gr_data(0);
}

void gr_puts(char str[])
{
  int i = 0;
  while (str[i] != 0)
    gr_putc(str[i++]);
}

