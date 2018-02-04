;Serial Example
;Jason Losh

;------------------------------------------------------------------------------
; Objectives and notes             
;------------------------------------------------------------------------------

;Using remappable pins
;Using uart engine
;Basic string functions

;Target Platform: 33FJ128MC802 Serial Demo Board
;Target uC:       33FJ128MC802
;Clock Source:    8 MHz primary oscillator set in configuration bits
;Clock Rate:      80 MHz using prediv=2, plldiv=40, postdiv=2
;Devices used:    UART

;Hardware description:
;Red LED
;  anode connected through 100ohm resistor to RB5 (pin 14), cathode grounded
;Green LED
;  anode connected through 100ohm resistor to RB4 (pin 11), cathode grounded
;Push Button
; Push button connected between RB15 (pin 26) and ground
;SP3232E RS-232 Interface
; T1IN connected to RP10 (pin 21)
; R1OUT connected to RP11 (pin 22)

;------------------------------------------------------------------------------
; Device includes and assembler directives             
;------------------------------------------------------------------------------

           .include "p33FJ128MC802.inc"
           .global __reset

;------------------------------------------------------------------------------
; Uninitialized variable space (in data memory)               
;------------------------------------------------------------------------------

           .bss
__SP:      .space   4*16                      ;allocate 16-level deep stack
__SP_LIM:  .space   4*2                       ;add pad after limit for debug

;------------------------------------------------------------------------------
; Code starts here               
;------------------------------------------------------------------------------

           .text
greeting:  .asciz "Enter \"1\" or \"0\"\r\n"

;------------------------------------------------------------------------------
; Subroutines                
;------------------------------------------------------------------------------

;Initialize Hardware
;  Parameters: nothing
;  Returns:    nothing
init_hw:   bclr     LATB, #4                  ;write 0 into output latches
           bclr     LATB, #5                  
           bclr     TRISB, #4                 ;make green led pin an output
           bclr     TRISB, #5                 ;make red led pin an output
           bset     CNPU1, #CN11PUE           ;enable pull-up for push button

           mov      #0x0003, W0               ;assign U1TX to RP10
           mov      W0, RPOR5
           mov      #0x1F0B, W0               ;assign U1RX to RP11
           mov      W0, RPINR18

                                              ;with 8 MHz input, set Fosc=80MHz
           mov      #38, W0                   ;set plldiv=40
           mov      W0, PLLFBD                ;Fosc = 8 / 2 * 40 / 4 = 40 MHz
           mov      #0x3000, W0               ;set pllpostdiv=2, pllprediv=2
           mov      W0, CLKDIV                ;Fosc = 8 / 2 * 40 / 2 = 80 MHz
           return

;Serial port initialize
;  Parameters: low-speed brg value
;  Returns:    nothing
           .equ     BAUD_19200, 129           ;round((40000000/16/19200)-1)

serial_init:
           mov      W0, U1BRG                 ;set low-speed baud rate
           mov      #0x8000, W0               ;enable 8n, no parity, 1 stop bit  
           mov      W0, U1MODE                ;(8N1), low-speed brg
           mov      #0x0400, W0               ;enable rx and rx, set int flag 
           mov      W0, U1STA                 ; after each char rx or tx
           return

;Wait_10us (approx)
;  Parameters: time(10us units) in WREG0
;  Returns:    nothing
wait_10us:                                   ;need 10us x 40 MHz instr cycle 
                                             ;= 400 ticks
                                             ;make N = WREG0 when fn called
                                             ;calculate 400 ticks if N = 1
                                             ; 2 ticks to get here (call)
           repeat   #390                     ; 1 tick
           nop                               ; 400 - 9 = 391 ticks (calculated)

w10_loop:  dec      W0, W0                    ; N ticks  
           bra      Z, w10_end                ; 2 + (N-1) ticks 
 
           repeat   #394                      ; (N-1) ticks
           nop                                ; 395(N-1) ticks
           bra      w10_loop                  ; 2(N-1) ticks 

w10_end:   return                             ; 3 ticks
                                       
;Wait_ms (approx)
;  Parameters: time(1ms units) in W
;  Returns:    nothing
wait_ms:   mov      W0, W1                    ;store time in 1ms resolution
w1_loop:   mov      #100, W0                  ;wait approx 1ms
           call     wait_10us       
           dec      W1, W1      
           bra      NZ, w1_loop    
           return                   

;Wait for PB Press
;  Parameters: nothing
;  Returns:    nothing
wait_pb:   btsc     PORTB, #15                ;is pb pressed?
           bra      wait_pb                   ;no, try again
           return

;Serial puts
;  Parameters: pointer to string in W0
;  Returns:    nothing
serial_puts: 
           mov.b    [W0++], W1                ;get char
           cp0.b    W1                        ;compare W1 with zero
           bra      Z, sps_done               ;exit loop if null found
sps_wait:  btsc     U1STA, #UTXBF             ;wait until tx char needed
           bra      sps_wait
           mov      W1, U1TXREG               ;tx char
           bra      serial_puts         
sps_done:  return

;Serial getc
;  Parameters: nothing
;  Returns:    character RX in W0
serial_getc: 
           btsc     U1STA, #OERR              ;clear overflow condition
           bclr     U1STA, #OERR
sgc_loop:  btss     U1STA, #URXDA             ;wait until rx char ready
           bra      sgc_loop
           mov      U1RXREG, W0               ;read char
           return 

;------------------------------------------------------------------------------
; Main                
;------------------------------------------------------------------------------

__reset:   mov      #__SP, W15                ;setup stack
           mov      #__SP_LIM, W0
           mov      W0, SPLIM
 
           call     init_hw                   ;initialize hardware
 
           mov      #BAUD_19200, W0           ;initialize uart1
           call     serial_init

           bset     LATB, #5                  ;blink red led for 1/2 sec              
           mov      #500, W0
           call     wait_ms
           bclr     LATB, #5               

           call     wait_pb                   ;wait for pb press

           bset     CORCON, #PSV              ;turn-on PSV
           mov      #psvpage(greeting), W0    ;set psv window to see greeting
           mov      W0, PSVPAG
           mov      #psvoffset(greeting), W0  ;make W0 point to greeting
           call     serial_puts

rx_loop:   call     serial_getc               ;get char
           btg      LATB, #4                  ;toggle green led                
           mov.b    #'1', W1                  ;is equal to '1'?
           cp.b     W0, W1                    
           bra      NZ, try_zero              ;no
           bset     LATB, #5                  ;yes, set red
           bra      rx_loop           
try_zero:  mov.b    #'0', W1                  ;is equal to '0'?
           cp.b     W0, W1                    
           bra      NZ, rx_loop               ;no
           bclr     LATB, #5                  ;yes, clear red
           bra      rx_loop

           .end

           