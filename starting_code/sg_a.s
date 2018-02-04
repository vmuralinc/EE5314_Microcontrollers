;Stop Go Example
;Jason Losh

;------------------------------------------------------------------------------
; Objectives and notes             
;------------------------------------------------------------------------------

; C using inline ASM code
; C calling an external ASM function

; Target Platform: 33FJ128MC802 Basic Demo Board
; Target uC:       33FJ128MC802
; Clock Rate:      Depends on oscillator type selected and frequency
; Devices used:    None

;Hardware description:
; Red LED
;   anode connected through 100ohm resistor to RB5 (pin 14), cathode grounded
; Green LED
;  anode connected through 100ohm resistor to RB4 (pin 11), cathode grounded
; Push Button
;  Push button connected between RB15 (pin 26) and ground

;------------------------------------------------------------------------------
; Device includes and assembler directives             
;------------------------------------------------------------------------------

         .include "p33FJ128MC802.inc"
         .global _wait_pb

;------------------------------------------------------------------------------
; Code starts here               
;------------------------------------------------------------------------------

         .text

;------------------------------------------------------------------------------
; Subroutines                
;------------------------------------------------------------------------------

;Wait for PB Press
;  Parameters: nothing
;  Returns:    nothing
_wait_pb: btsc     PORTB, #15                ;is pb pressed?
          bra      _wait_pb                  ;no, try again
          return

         .end

           