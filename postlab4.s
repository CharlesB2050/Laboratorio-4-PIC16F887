; Archivo:	postlab4.s
; Dispositivo:	PIC16F887
; Autor:	Carlos Estuardo Búcaro Pape
; Compilador:	pic-as (v2.35), MPLABX V6.00
;                
; Programa:	Contador en PORTD con incrementos y decrementos con botones
;		en PORTB con pullups internos
; Hardware:	LEDs en el PORTD, pushbuttons en el PORTB		
;
; Creado:	16 feb 2022
; Última modificación: 19 feb 2022
    
PROCESSOR 16F887
    
; PIC16F887 Configuration Bit Settings

; Assembly source line config statements

; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = ON            ; Power-up Timer Enable bit (PWRT enabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
  CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
  CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

// config statements should precede project file includes.
#include <xc.inc>
  
UP   EQU 0
DOWN EQU 7 
 
 RESET_TMR0 MACRO TMR_VAR
    BANKSEL TMR0	    ; cambiamos de banco
    MOVLW   TMR_VAR
    MOVWF   TMR0	    ; configuramos tiempo de retardo
    BCF	    T0IF	    ; limpiamos bandera de interrupción
    ENDM    
; ------- VARIABLES EN MEMORIA --------
PSECT udata_shr		    ; Memoria compartida
    W_TEMP:		DS 1
    STATUS_TEMP:	DS 1
    SUMA:		DS 1
    cont:		DS 2
    SEGS:		DS 1
    LIMS:		DS 1
    DECS:		DS 1
    LIMD:		DS 1

PSECT resVect, class=CODE, abs, delta=2
ORG 00h			    ; posición 0000h para el reset
;------------ VECTOR RESET --------------
resetVec:
    PAGESEL MAIN	    ; Cambio de pagina
    GOTO    MAIN
    
PSECT intVect, class=CODE, abs, delta=2
ORG 04h			    ; posición 0004h para interrupciones
;------- VECTOR INTERRUPCIONES ----------
PUSH:
    MOVWF   W_TEMP	    ; Guardamos W
    SWAPF   STATUS, W
    MOVWF   STATUS_TEMP	    ; Guardamos STATUS
    
ISR:
   BTFSC    RBIF
   CALL	    INT_IOCB
   
   BTFSC   T0IF	    
   CALL    INT_TMR0	   
   
POP:
    SWAPF   STATUS_TEMP, W  
    MOVWF   STATUS	    ; Recuperamos el valor de reg STATUS
    SWAPF   W_TEMP, F	    
    SWAPF   W_TEMP, W	    ; Recuperamos valor de W
    RETFIE		    ; Regresamos a ciclo principal
    
    
PSECT code, delta=2, abs
ORG 100h		    ; posición 100h para el codigo
;------------- CONFIGURACION ------------
ORG 200h
TABLA: 
    clrf    PCLATH		; Limpiamos registro PCLATH
    bsf	    PCLATH, 1		; Posicionamos el PC en dirección 02xxh
    andlw   0x0F		; no saltar más del tamaño de la tabla
    addwf   PCL			; Apuntamos el PC a caracter en ASCII de CONT
    retlw   00111111B			; 0 hexadecimal
    retlw   00000110B			; 1 hexadecimal
    retlw   01011011B			; 2 hexadecimal
    retlw   01001111B			; 3 hexadecimal
    retlw   01100110B			; 4 hexadecimal
    retlw   01101101B			; 5 hexadecimal
    retlw   01111101B			; 6 hexadecimal
    retlw   00000111B			; 7 hexadecimal
    retlw   01111111B			; 8 hexadecimal
    retlw   01100111B			; 9 hexadeciaml
MAIN:
    movlw   6
    movwf   LIMD
    movlw   10
    movwf   LIMS
    movlw   50
    movwf   cont
    CALL    CONFIG_IO	    ; Configuración de I/O
    CALL    CONFIG_RELOJ    ; Configuración de Oscilador
    CALL    CONFIG_IOCRB    ; Configuración de interrupción para el boton en B
    CALL    CONFIG_TMR0
    CALL    CONFIG_INT_RB	    ; Configuración de interrupciones
    CALL    CONFIG_INT_TMR0
    BANKSEL PORTD	    ; Cambio a banco 00
    
LOOP:
    ; Código que se va a estar ejecutando mientras no hayan interrupciones
    CALL    LIMITE1
    ;CALL    LIMITE2
    CALL    DISPLAY
    GOTO    LOOP	    
    
;------------- SUBRUTINAS ---------------
LIMDECS:
    INCF    DECS
    DECFSZ	LIMD
    RETURN
    CLRF	PORTA
    CLRF	DECS
    MOVLW	6
    MOVWF	LIMD
    RETURN
    
LIMSEGS:
    DECFSZ	LIMS
    RETURN
    CALL	LIMDECS
    CLRF	PORTC
    CLRF	SEGS
    MOVLW	10
    MOVWF	LIMS
    RETURN

DISPLAY:
    MOVF    SEGS, 0
    CALL    TABLA
    MOVWF   PORTC
    
    MOVF    DECS, 0
    CALL    TABLA
    MOVWF   PORTA
    RETURN

INT_IOCB:
    BANKSEL PORTB
    BTFSS   PORTB, UP
    INCF    PORTD
    BTFSS   PORTB, DOWN
    DECF    PORTD
    BCF	    RBIF
    RETURN
INT_TMR0:
    RESET_TMR0 178	    ; Reiniciamos TMR0 para 50ms
    ;INCF    PORTC
    ;MOVF    cont, 0
    DECFSZ  cont    
    RETURN
    ;BTFSC   STATUS, 2
    movlw    50
    movwf    cont
    incf    SEGS
    CALL    LIMSEGS
    RETURN
    
    
CONFIG_RELOJ:
    BANKSEL OSCCON	    ; cambiamos a banco 1
    BSF	    OSCCON, 0	    ; SCS -> 1, Usamos reloj interno
    BSF	    OSCCON, 6
    BSF	    OSCCON, 5
    BCF	    OSCCON, 4	    ; IRCF<2:0> -> 110 4MHz
    RETURN
 
CONFIG_IOCRB:
    BANKSEL TRISD
    BSF	    IOCB, UP
    BSF	    IOCB, DOWN
    
    BANKSEL PORTD
    MOVF    PORTB, W	    ; al leer termina la condición mismatch
    BCF	    RBIF 
    RETURN
    
 CONFIG_IO:
    BANKSEL ANSEL
    CLRF    ANSEL
    CLRF    ANSELH	    ; I/O digitales
    BANKSEL TRISD
    CLRF    TRISD	    ; PORTD como salida
    BANKSEL PORTD
    BANKSEL TRISC
    CLRF    TRISC	    ; PORTC como salida
    BANKSEL PORTC
    BANKSEL TRISA
    CLRF    TRISA
    BANKSEL PORTA
    CLRF    PORTA
    CLRF    PORTC
    CLRF    PORTD	    ; Apagamos PORTD
    BSF     TRISB, UP
    BSF     TRISB, DOWN
    BANKSEL OPTION_REG
    BCF     OPTION_REG, 7   ; activar las entradas con pull-ups interno
    BSF	    WPUB, UP
    BSF	    WPUB, DOWN
    RETURN
CONFIG_TMR0:
    BANKSEL OPTION_REG	    ; cambiamos de banco
    BCF	    T0CS	    ; TMR0 como temporizador
    BCF	    PSA		    ; prescaler a TMR0
    BSF	    PS2
    BSF	    PS1
    BSF	    PS0		    ; PS<2:0> -> 111 prescaler 1 : 256
    
    BANKSEL TMR0	    ; cambiamos de banco
    MOVLW   178
    MOVWF   TMR0	    ; 50ms retardo
    BCF	    T0IF	    ; limpiamos bandera de interrupción
    RETURN 
    
CONFIG_INT_RB:
    BANKSEL INTCON
    BSF	    GIE		    ; Habilitamos interrupciones
    BSF	    RBIE	    ; Habilitamos interrupcion del puerto B
    BCF	    RBIF	    ; Limpiamos bandera de interrupción en B
    RETURN
CONFIG_INT_TMR0:
    BANKSEL INTCON
    BSF	    GIE		    ; Habilitamos interrupciones
    BSF	    T0IE	    ; Habilitamos interrupcion TMR0
    BCF	    T0IF	    ; Limpiamos bandera de TMR0
    RETURN
LIMITE1:
    BTFSC   PORTD, 4
    CLRF    PORTD
    BTFSC   PORTD, 7
    CALL    TOTAL
    RETURN
;LIMITE2:
    ;BTFSC   PORTC, 4
    ;CLRF    PORTC
    ;RETURN
TOTAL:
    CLRF PORTD
    MOVF PORTD, 0
    ADDLW 15
    MOVWF   PORTD
    RETURN
END
    



