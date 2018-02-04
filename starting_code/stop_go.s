;Stop Go Example
;Jason Losh

;------------------------------------------------------------------------------
; Objectives and notes             
;------------------------------------------------------------------------------

;Using remappable pins

;Target Platform: 33FJ128MC802 Basic Demo Board
;Target uC:       33FJ128MC802
;Clock Rate:      Depends on oscillator type selected and frequency

;Hardware description:
;Red LED
;  anode connected through 470ohm resistor to RB5 (pin 14), cathode grounded
;Green LED
;  anode connected through 470ohm resistor to RB4 (pin 11), cathode grounded
;Push Button
; Push button connected between RB15 (pin 26) and ground

;------------------------------------------------------------------------------
; Device includes and assembler directives             
;------------------------------------------------------------------------------

         .include "p33FJ128MC802.inc"
         .global __reset

;------------------------------------------------------------------------------
; Uninitialized variable space (in data memory)               
;------------------------------------------------------------------------------

          .bss
__SP:     .space   4*16                      ;allocate 16-level deep stack
__SP_LIM: .space   4*2                       ;add pad after limit for debug


;------------------------------------------------------------------------------
; Code starts here
;------------------------------------------------------------------------------

         .text

;------------------------------------------------------------------------------
; Subroutines                
;------------------------------------------------------------------------------

;Initialize Hardware
;  Parameters: nothing
;  Returns:    nothing
init_hw:  bclr     LATB, #4                  ;write 0 into output latches
          bclr     LATB, #5                  
          bclr     TRISB, #4                 ;make green led pin an output
          bclr     TRISB, #5                 ;make red led pin an output
          bset     CNPU1, #CN11PUE           ;enable pull-up for push button
          return

;Wait for PB Press
;  Parameters: nothing
;  Returns:    nothing
wait_pb:  btsc     PORTB, #15                ;is pb pressed?
          bra      wait_pb                   ;no, try again
          return
                                
;------------------------------------------------------------------------------
; Main                
;------------------------------------------------------------------------------

__reset:  mov      #__SP, W15                ;setup stack
          mov      #__SP_LIM, W0
          mov      W0, SPLIM

          call     init_hw                   ;initialize hardware
 
          bset     LATB, #5                  ;turn-on red led                

          call     wait_pb                   ;wait for pb press

          bclr     LATB, #5                  ;turn-off red led
          bset     LATB, #4                  ;turn-on green led                

;--------------
; Endless loop
;--------------
        
eop:      nop
          bra      eop

         .end

           