;Archivo: Lab04.as
;Dispositivo: PIC16F887
;Autor: Rodrigo García
;Carné: 20178
;Programa: contador
;Hardware: LEDs, Display 
;
;Creado: 8 febrero, 2023


processor 16F887
#include <xc.inc>
 
; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF             ; Watchdog Timer Enable bit (WDT enabled)
  CONFIG  PWRTE = OFF           ; Power-up Timer Enable bit (PWRT disabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF            ; Brown Out Reset Selection bits (BOR enabled)
  CONFIG  IESO = OFF             ; Internal External Switchover bit (Internal/External Switchover mode is enabled)
  CONFIG  FCMEN = OFF            ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
  CONFIG  LVP = OFF              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)
    
;---------------------------------------------------------

PSECT udata_bank0
  Cont:	    DS 2
  Cont_Seg: DS 1
  Cont_Decenas: DS 1
      
//----------------------------------MACROS---------------------------------------
RESET_TMR0 MACRO TMR_VAR
    BANKSEL TMR0		; seleccionar el banco 2 para timer0
    movlw   TMR_VAR		; cargar el valor a w
    movwf   TMR0		; cargarlo al registro del timer0
    bcf	    T0IF		; limpiar bandera del timer0
    endm  
  
//--------------------------VARIABLES EN MEMORIA--------------------------------
PSECT udata_shr			
    W_TEMP:		DS 1	
    STATUS_TEMP:	DS 1	
  
  
 //-----------------------------Vector reset------------------------------------
 PSECT resVect, class = CODE, abs, delta = 2;
 ORG 00h			; Posición 0000h RESET
 resetVec:			; Etiqueta para el vector de reset
    PAGESEL main
    goto main
  
 PSECT intVect, class = CODE, abs, delta = 2, abs
 ORG 04h			; Posición de la interrupción
 
//--------------------------INTERRUPCIONES------------------------------- 
PUSH:
    movwf   W_TEMP		; 
    swapf   STATUS, W		; intercambiar status con registro w
    movwf   STATUS_TEMP		; cargar valor a la variable temporal
    
ISR: 
    btfsc   RBIF		; interrupción PORTB, SI=1 NO=0
    call    INT_IOCB		; si es igual a 1, ejecutar interrupción
    
    btfsc   T0IF		; interrupción TMR0, SI=1 NO=0
    call    INT_TMR0		; si es igual a 1, ejecutar interrupción
    
POP:
    swapf   STATUS_TEMP, W	
    movwf   STATUS		
    swapf   W_TEMP, F		
    swapf   W_TEMP, W		
    retfie			

//----------------------------INTERRUPCIONES SUBRUTINAS------------------------------------    
INT_IOCB:			; interrupción del puerto B
    BANKSEL PORTA		; banco 00
    btfss   PORTB, 0		; revisar si el primer botón ha cambiado a 0
    incf    PORTA		; si cambió a 0, incrementar el valor del puerto A
    btfss   PORTB, 1		; revisar si el segundo botón ha cambiado a 0
    decf    PORTA		; si cambió a 0, disminuir el valor del puerto A
    bcf	    RBIF		; limpiar bandera del puerto B
    return 

INT_TMR0:
    RESET_TMR0 178		; llamar a la función macro del reseteo del timer0 para cargarle un valor
    incf    Cont		; incrementar la variable de milisegundos
    movf    Cont, W		; cargar el valor a w
    sublw   50			; restar 50 de w para revisar si ha llegado a los 1000ms
    btfsc   ZERO		; revisar si la bandera del 0 no esta activa
    goto    INC_SEG		; sí esta activa la bandera, ir a la siguiente subrutina
    return
    
INC_SEG:
    incf    Cont_Seg		; incrementar la variable de segundos
    movf    Cont_Seg, W		; cargar el valor a w
    clrf    Cont		; limpiar la variable de milisegundos
    sublw   10			; restar 10 de w cuando haya llegado a 10 segundos
    btfsc   ZERO		; revisar si la bandera del 0 no esta activa
    goto    INC_DECENAS		; sí esta activa la bandera, ir a la siguiente subrutina
    return
    
INC_DECENAS:
    incf    Cont_Decenas	; incrementar la valriable de decenas
    movf    Cont_Decenas, W	; cargarvalor a w
    clrf    Cont_Seg		; limpiar la variable de segundos
    sublw   6			; restar 6 de w cuando haya llegado a 60 segundos
    btfsc   ZERO		; revisar si la bandera del 0 no esta activa
    goto    REINICIO		; si esta activa la bandera, ir a la siguiente subrutina
    return
    
REINICIO:
    clrf    Cont_Decenas	; limpiar variable decenas
    clrf    Cont_Seg		; limpiar variable segundos
    return			
    
PSECT code, delta=2, abs
ORG 100h			; posición 100h para el codigo    
    
//------------------------------MAIN-------------------------------------
main:
    call    IO_CONFIG		; configuración de pines
    call    CLK_CONFIG   	; configuración de reloj
    call    TMR0_CONFIG
    call    IOCRB_CONFIG	; configuración del puerto b
    call    INT_CONFIG		; configuración de interrupciones
    BANKSEL PORTA

 //------------------------------LOOP-------------------------------------   
loop:				 
    movf    Cont_Seg, W		; mover valor de la variable segundos a w
    call    Tabla		; llamar a la tabla del display
    movwf   PORTD	        ; cargar el valor al puerto D
    movf    Cont_Decenas, W	; mover el valor de la variable decenas a w
    call    Tabla		; llamar a la tabla del display
    movwf   PORTC		; cargar el valor al puerto C
    goto    loop		
 //---------------------------TABLA DEL DISPLAY--------------------------------
PSECT Tabla, class = CODE, abs, delta = 2
ORG 200h			; Posición de la tabla

Tabla:
    clrf PCLATH
    bsf PCLATH, 1	
    andlw 0x0F
    addwf PCL			
    retlw 00111111B		; 0
    retlw 00000110B		; 1
    retlw 01011011B		; 2
    retlw 01001111B		; 3
    retlw 01100110B		; 4
    retlw 01101101B		; 5
    retlw 01111101B		; 6
    retlw 00000111B		; 7
    retlw 01111111B		; 8 
    retlw 01101111B		; 9
    retlw 01110111B		; A
    retlw 01111100B		; b
    retlw 00111001B		; C
    retlw 01011110B		; D
    retlw 01111001B		; C
    retlw 01110001B		; F
    
//------------------------------SUBRUTINAS--------------------------------------
IOCRB_CONFIG:
    BANKSEL IOCB		; seleccionar banco del IOCB
    bsf	    IOCB, 0		; activar IOCB para el primer pulsador
    bsf	    IOCB, 1		; activar IOCB para el segundo pulsador
    
    BANKSEL PORTA		; seleccionar banco 00
    movf    PORTB, W		; cargar el valor del puerto B a w
    bcf	    RBIF		; limpiar bandera de interrupción del puerto B
    return

IO_CONFIG:
    BANKSEL ANSEL		; selecionar el banco 10
    clrf    ANSEL		
    clrf    ANSELH		; colocar puertos en digital
    
    BANKSEL TRISA		; seleccionar banco 01
    bcf     PORTA, 0		; colocar pines del puerto A como salida 
    bcf     PORTA, 1
    bcf     PORTA, 2
    bcf     PORTA, 3
    
    bsf	    TRISB, 0		; colocar pines del puerto B como entrada
    bsf	    TRISB, 1		
    
    clrf    TRISC		; colocar puerto C como salida
    clrf    TRISD		; colocar puerto D como salida
    
    bcf	    OPTION_REG, 7	; limpiar RBPU para colocar Pull-Up en el puerto B
    bsf	    WPUB, 0		; setear pin 0 del puerto B como Weak Pull-Up
    bsf	    WPUB, 1		; setear pin 1 del puerto B como Weak Pull-Up
    
    BANKSEL PORTA		; banco 00
    clrf    PORTA		; limpiar puerto A
    clrf    PORTD		; limpiar puerto D
    clrf    PORTC		; limpiar puerto C
    return		    
    
CLK_CONFIG:
    BANKSEL OSCCON		; configuración de oscilador
    bsf	    SCS			; usar oscilador interno
    bcf	    IRCF0		
    bsf	    IRCF1		
    bsf	    IRCF2		;4MHz oscilador interno
    return

TMR0_CONFIG:
    BANKSEL OPTION_REG		
    bcf	    T0CS		; utilizar ciclo interno
    bcf	    PSA			
    bsf	    PS2			
    bsf	    PS1			
    bsf	    PS0			; prescaler en 1:256
    
    BANKSEL TMR0		; banco timer0
    //N = 256-[(20mS*4MHz)/(4*256)]
    movlw   178			; cargar valor a W
    movwf   TMR0		; cargar valor N al TMR0
    bcf	    T0IF		; limpiar bandera del tmr0
    return   
    
INT_CONFIG:
    BANKSEL INTCON
    bsf GIE			; activar interrupciones
    bsf RBIE			; activar cambio de interrupciones en portB
    bcf RBIF			; limpiar bandera de cambio del portB
    bsf	T0IE			; activar interrupciones en timer0
    bcf	T0IF			; Limpiar bandera del timer0
    return
    

   

end 