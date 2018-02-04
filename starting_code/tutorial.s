;Basic Tutorial
;Jason Losh

;------------------------------------------------------------------------------
; Objectives and notes             
;------------------------------------------------------------------------------

;Basic ASM program organization

;Basic sections:
; DATA section used to store initialized variables in data memory
; BSS section used to store uninitialized variables in data memory
; TEXT section used to store constant data and code in program memory

;Setting up a software stack

;Simple examples of literal, direct, and indirect memory addressing
;Accessing arrays stored in data memory and program memory

;Examples of PSV and table access to program memory

;Writing a subroutine (dinit) that initializes the DATA section
;(if this was a C program, the C init function would do this task) 


;Target Platform: 33FJ128MC802 simulator
;Target uC:       33FJ128MC802
;Clock Rate:      Varies
;Devices used:    None

;Hardware description:
;None

;------------------------------------------------------------------------------
; Device includes and assembler directives             
;------------------------------------------------------------------------------

         .include "p33FJ128MC802.inc"
         .global __reset

;------------------------------------------------------------------------------
; Initialized variable space (in data memory)             
; Must call dinit at reset to initialize the space
;------------------------------------------------------------------------------

          .data
array1:   .word    1, 2, 3, 4, 5, 6, 7, 8
size1:    .word    8
a:        .byte    0x12
          .align   2
b:        .word    0x1234
c:        .long    0x12345678

;------------------------------------------------------------------------------
; Uninitialized variable space (in data memory)               
;------------------------------------------------------------------------------

          .bss
__SP:     .space   4*16                      ;allocate 16-level deep stack
__SP_LIM: .space   4*2                       ;add pad after limit for debug
total:    .space   2
array2:   .space   2*8
size2:    .space   2
x:        .space   2
y:        .space   2
z:        .space   2

;------------------------------------------------------------------------------
; Constant variable space (in program memory)               
;------------------------------------------------------------------------------

          .text
string:   .ascii   "ABCD"                    ;String w/o null terminator
          .asciz   "EFGH"                    ;String w/ null terminator
array3:   .word    1, 2, 3, 4, 5, 6, 7, 8    ;Array of constants
size3:    .byte    8                         ;Size of array above

;------------------------------------------------------------------------------
; Code starts here               
;------------------------------------------------------------------------------

         .text

;------------------------------------------------------------------------------
; Subroutines                
;------------------------------------------------------------------------------

; Initialize data section by copying data_init table in .dinit in program 
; memory to .data space (normally initialized in C)
;
; Table format is discussed in 10.8.2 of the ASM30/LINK30 users guide
;
; data init record format is:
; struct data_record {
; char *dst; /* destination address */
; int len; /* length in bytes */
; int format; /* format code */
; char dat[0]; /* variable length data */
; };

; where the format code is:
; 0 Fill the output section with zeros (unsupported)
; 1 Copy 2 bytes of data from each instruction word in the data array
;   --data-init --no-pack-data options force this 2 bytes/program word packing
; 2 Copy 3 bytes of data from each instruction word in the data array
;   --data-init --pack data) options force this 3 bytes/program word format

; Default is --data-init and --pack-data

dinit:    mov      #tblpage(.startof.(.dinit)), W0   ;set tbl window to .dinit
          mov      W0, TBLPAG                                                   
          mov      #tbloffset(.startof.(.dinit)), W1 ;make W1 point to .dinit
          tblrdl   [W1++], W2                        ;make W2 equal to *dst
          tblrdl   [W1++], W3                        ;make W3 equal to len
          tblrdl   [W1++], W4                        ;make W4 equal to format

          cp       W4, #1                    ;is type 1?
          bra      Z, d_type1                ;yes
          cp       W4, #2                    ;is type 2?
          bra      Z, d_type2                ;yes
          bra      d_end                     ;not handled type

d_type1:  cp       W3, #0                    ;read LSB of LSW
          bra      Z, d_end                  ;(note LSb of W1 = 0 to read LSB)
          dec      W3, W3
          tblrdl.b [W1++], [W2++]            
          cp       W3, #0                    ;read MSB of LSW
          bra      Z, d_end                  ;(note LSb of W1 = 0 to read MSB)
          dec      W3, W3
          tblrdl.b [W1++], [W2++]                
          bra      d_type1          

d_type2:  cp       W3, #0                    ;read LSB of LSW
          bra      Z, d_end                  ;(note LSb of W1 = 0 to read LSB)
          dec      W3, W3
          tblrdl.b [W1++], [W2++]             
          cp       W3, #0                    ;read MSB of LSW
          bra      Z, d_end                  ;(note LSb of W1 = 1 to read MSB)
          dec      W3, W3
          tblrdl.b [W1--], [W2++]                
          cp       W3, #0                    ;read LSB of MSW
          bra      Z, d_end                  ;(note LSb of W1 = 0 to read LSB)
          dec      W3, W3
          tblrdh.b [W1++], [W2++]                
          inc      W1, W1                    ;skip MSB of MSW
          bra      d_type2          

d_end:    return

;------------------------------------------------------------------------------
; Main                
;------------------------------------------------------------------------------

__reset:  mov      #__SP, W15                ;Setup stack
          mov      #__SP_LIM, W0
          mov      W0, SPLIM
          call     dinit                     ;Initialize .data section

;---------------------------------------
; Literal and register-to-register move
; Init W0, W1; Compute W2 = W0 + W1
;---------------------------------------

          mov      #1, W0                    ;Init registers
          mov      #2, W1
          add      W0, W1, W2                ;W2 = W0 + W1

;------------------------------------
; Bit instructions
; Init W0; Set bit 2 and clear bit 3 in W0
;------------------------------------

          mov.b    #0b00101011, W0          ;Init register
          bset     W0, #2                   ;Set bit 2, clear bit 3
          bclr     W0, #3

;------------------
; Direct move
; Z = X + Y, X = A
;------------------

          mov      #4, W0                    ;Init memory values
          mov      W0, x
          mov      #5, W0
          mov      W0, y
          mov      x, W0                     ;Z = X + Y
          mov      y, W1
          add      W0, W1, W2
          mov      W2, z

          mov.b    a, WREG                   ;W0 = a
          mov      b, W4                     ;W4 = b
          mov      #c, W5                    ;W5 = &c
          mov.d    [W5], W6                  ;W7:6 = c      

;---------------
; Indirect move
; Z = X - Y
;---------------

          mov      #x, W0                     ;Z = X - Y
          mov      #y, W1
          mov      [W0], W2
          sub      W2, [W1], W2
          mov      W2, z

;---------------------------------------------------
; Totalize array1[size1] in data and store in total
;---------------------------------------------------

          mov      #array1, W1               ;make W1 point to array1[0]
          mov      size1, W2                 ;store number of terms (size1) in W2        

          mov      #0, W0                    ;zero total stored in W0
          dec      W2, W2                    ;iterate "add" line "size" times
          repeat   W2                      
          add      W0, [W1++], W0            ;add element, increment add by 2
        
          mov      W0, total                 ;store total in bss

;------------------------------------------------------------------
; Initialize array2[8] in bss and fill with 1, 2, 3, 4, 5, 6, 7, 8
;------------------------------------------------------------------

          mov      #array2, W1               ;make W1 point to array2[0]
          mov      #8, W2                    ;store number of terms in W2
          mov      W2, size2                 ;store number of terms in size2
          mov      #1, W0                    ;initialize W0 to 1
loop:     mov      W0, [W1++]                ;initialize array2[] element
          inc      W0, W0                    ;increment write value
          dec      W2, W2                    ;decrement loop counter
          bra      NZ, loop                  ;repeat if not done 

;--------------------------------------------------
; Totalize array2[size2] in bss and store in total
;--------------------------------------------------

          mov      #array2, W1               ;make W1 point to array2[0]
          mov      size2, W2                 ;store number of terms (size2) in W2        

          mov      #0, W0                    ;zero total stored in W0
          dec      W2, W2                    ;iterate "add" line "size" times
          repeat   W2                      
          add      W0, [W1++], W0            ;add element, increment add by 2
        
          mov      W0, total                 ;store total in bss

;--------------------------------------------------
; Totalize array3[size3] in psv and store in total
;--------------------------------------------------
 
          bset     CORCON, #PSV              ;turn-on PSV
 
          mov      #psvpage(size3), W0       ;set psv window to see size3
          mov      W0, PSVPAG
          mov      #psvoffset(size3), W0     ;make W0 point to size3
          mov      [W0], W2                  ;store number of terms (size3) in W2

          mov      #psvpage(array3), W0      ;set psv window to see array3
          mov      W0, PSVPAG
          mov      #psvoffset(array3), W1    ;make W1 point to array3[0]

          mov      #0, W0                    ;zero total stored in W0
          dec      W2, W2                    ;iterate "add" line "size" times
          repeat   W2                       
          add      W0, [W1++], W0            ;add element, increment add by 2
        
          mov      W0, total                 ;store total in bss

;--------------------------------------------
; Read LSW at program memory address 0x00294
;--------------------------------------------

          mov      #0, W0
          mov      W0, PSVPAG
          mov      #0x8294, W1
          mov      [W1], W0

;---------------------------------------------------
; Read MSB of LSW at program memory address 0x00294
;---------------------------------------------------

          mov      #0, W0
          mov      W0, PSVPAG
          mov      #0x8295, W1
          mov.b    [W1], W0

;--------------------------------------------
; Read LSW at program memory address 0x00294
;--------------------------------------------

          mov      #0, W0
          mov      W0, TBLPAG
          mov      #0x0294, W1
          tblrdl   [W1], W0

;--------------------------------------------
; Read MSW at program memory address 0x00294
;--------------------------------------------

          mov      #0, W0
          mov      W0, TBLPAG
          mov      #0x0294, W1
          tblrdh   [W1], W0

;---------------------------------------------------
; Read MSB of LSW at program memory address 0x00294
;---------------------------------------------------

          mov      #0, W0
          mov      W0,TBLPAG
          mov      #0x0295, W1
          tblrdl.b [W1], W0

;--------------
; Endless loop
;--------------
     
eop:      nop
          bra      eop

         .end

           