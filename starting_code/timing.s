;Timing Example
;Jason Losh

;------------------------------------------------------------------------------
; Objectives and notes             
;------------------------------------------------------------------------------

;Basic timing
;Stopwatch and debugging

;Target Platform: 33FJ128MC802 Basic Demo Board
;Target uC:       33FJ128MC802
;Clock Source:    8 MHz primary oscillator set in configuration bits
;Clock Rate:      80 MHz using prediv=2, plldiv=40, postdiv=2
;Devices used:    None

;Hardware description:
;Red LED
;  anode connected through 470ohm resistor to RB5 (pin 14), cathode grounded
;Green LED
;  anode connected through 470ohm resistor to RB4 (pin 11), cathode grounded
;Speaker
;  4.7kohm from RB2 (pin 6) to base of 2N2222A, emitter grounded, collector
;  connected through 100ohm resistor to 3.3V.  Collector is ac coupled with
;  47 uF cap to grounded speaker.

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

;------------------------------------------------------------------------------
; Subroutines                
;------------------------------------------------------------------------------

;Initialize Hardware
;  Parameters: nothing
;  Returns:    nothing
init_hw:   bclr     LATB, #2                  ;write 0 into output latch
           bclr     LATB, #4
           bclr     LATB, #5                  
           bset     AD1PCFGL, #PCFG4          ;make speaker digital
           bclr     TRISB, #4                 ;make green led pin an output
           bclr     TRISB, #5                 ;make red led pin an output
           bclr     TRISB, #2                 ;make speaker driver an output
           bset     CNPU1, #CN11PUE           ;enable pull-up for push button

                                              ;with 8 MHz input, set Fosc=80MHz
           mov      #38, W0                   ;set plldiv=40
           mov      W0, PLLFBD                ;Fosc = 8 / 2 * 40 / 4 = 40 MHz
           mov      #0x3000, W0               ;set pllpostdiv=2, pllprediv=2
           mov      W0, CLKDIV                ;Fosc = 8 / 2 * 40 / 2 = 80 MHz
           return

;Wait for PB Press
;  Parameters: nothing
;  Returns:    nothing
wait_pb:   btsc     PORTB, #15                ;is pb pressed?
           bra      wait_pb                   ;no, try again
           return

;Wait_us (exactly including W0 load, call, and return)
;  Parameters: time in us in W0
;  Returns:    nothing
wait_us:                                      ;need 1us x 40 MHz instr cycle 
                                              ;= 40 ticks / us
                                              ;make N = W0 when fn called
                                              ;calculate 40 ticks if N = 1
                                              ; 1 tick for mov #__, W0
                                              ; 2 ticks to get here (call)
           repeat   #29                       ; 1 tick
           nop                                ; 40 - 10 = 30 ticks (calculated)

wu_loop:   dec      W0, W0                    ; N ticks  
           bra      Z, w10_end                ; 2 + (N-1) ticks 
 
           repeat   #34                       ; (N-1) ticks
           nop                                ; 35(N-1) ticks
           bra      wu_loop                   ; 2(N-1) ticks 

w10_end:   return                             ; 3 ticks
                                       
;Wait_ms (approx)
;  Parameters: time in ms in W
;  Returns:    nothing
wait_ms:   mov      W0, W1                    ;store time in 1ms resolution
wm_loop:   mov      #1000, W0                 ;wait approx 1ms
           call     wait_us       
           dec      W1, W1      
           bra      NZ, wm_loop    
           return                   
                           
;------------------------------------------------------------------------------
; Main                
;------------------------------------------------------------------------------

__reset:   mov      #__SP, W15                ;setup stack
           mov      #__SP_LIM, W0
           mov      W0, SPLIM

           call     init_hw                   ;initialize hardware
 
           bset     LATB, #5                  ;power-on flash (250ms)
           mov      #250, W0
           call     wait_ms
           bclr     LATB, #5

           bset     LATB, #4                  ;turn-on green led for approx 5s
           mov      #5000, W0
           call     wait_ms
           bclr     LATB, #4 

speaker:   bset     LATB, #2                  ;drive speaker on
           mov      #1136, W0                 ;1.136 ms (approx 1/(2*440 Hz))
           call     wait_us                   ;wait 1136 * 1us
           bclr     LATB, #2                  ;drive speaker off 
           mov      #1136, W0                 ;1.136 ms (approx 1/(2*440 Hz))
           call     wait_us                   ;wait 1136 * 1us
           bra      speaker                   ;loop forever

          .end

           