/*
 * File:   newmain.c
 * Author: DELL
 *
 * Created on 2020. január 24., 23:13
 */


// PIC16F18426 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32// Power-up default value for COSC bits (HFINTOSC with 2x PLL, with OSCFRQ = 16 MHz and CDIV = 1:1 (FOSC = 32 MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTS = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = OFF      // Brown-out reset enable bits (Brown-out reset disabled)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 2.45V)
#pragma config ZCDDIS = OFF     // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = OFF    // Peripheral Pin Select one-way control (The PPSLOCK bit can be set and cleared repeatedly by software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config BBSIZE = BB512   // Boot Block Size Selection bits (512 words boot block size)
#pragma config BBEN = OFF       // Boot Block Enable bit (Boot Block disabled)
#pragma config SAFEN = OFF      // SAF Enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block Write Protection bit (Application Block not write protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not write protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration Register not write protected)
#pragma config WRTD = OFF       // Data EEPROM write protection bit (Data EEPROM NOT write protected)
#pragma config WRTSAF = OFF     // Storage Area Flash Write Protection bit (SAF not write protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (UserNVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#define _XTAL_FREQ 32000000

unsigned int levels[8] = {1360,940,670,530,430,360,305,250};//-25,-30,-35,-40,-45,-50,-55,-60
unsigned int times[8] = {6,60,90,120,150,180,210,240};

void adc_init(void);
int adc_read(void);
void pwm_init(void);
void pwm_set(unsigned int value);
int result;
int millisec;
int reading;
int jumpers;
int offtimer;
int ontimer;

void main(void) {
    TRISBbits.TRISB4 = 0;//alarm relay
    TRISBbits.TRISB6 = 0;//alarm led
    TRISBbits.TRISB5 = 0;//underlevel led
    TRISC = 0b11111111;
    ANSELC = 0b00000000;
    WPUC = 0b11111111;//weak pullup on c port
    OSCCON1bits.NOSC = 0b110;//intosc,32mhz
    OSCCON1bits.NDIV = 0;//divide the osc output by 1
    OSCFRQbits.HFFRQ = 0b110;//intosc freq setting:32mhz
    INTCONbits.GIE= 1;//global interrupts are enabled
    INTCONbits.PEIE = 1;//peripherial interrupts are enabled
    T0CON0bits.EN = 1;//timer0 enabled
    T0CON1bits.CS = 0b010;//clock source is fosc/4
    T0CON1bits.T0CKPS = 0b0101;//prescaler = 32
    PIE0bits.TMR0IE = 1;
    
    pwm_init();
    adc_init();
    
    while(1)
    {
       pwm_set( (adc_read()>>2) );//level led pwm  
       if(millisec >= 1000)
       {
           reading = adc_read();
           jumpers = ~PORTC;
           if(reading <levels[(jumpers&0b111)])
           {
              offtimer++; 
              ontimer = 0;
              LATBbits.LATB5 = 1;
              if (offtimer >=times[((jumpers&0b11100000)>>5)])
           {
               LATBbits.LATB4 = 1;
               LATBbits.LATB6 = 1;
               offtimer = times[((jumpers&0b11100000)>>5)];
           }
           }
           
           if(reading >levels[(jumpers&0b111)])
           {
              ontimer++; 
              offtimer = 0;
              LATBbits.LATB5 = 0;
           if (ontimer >=3)
           {
               LATBbits.LATB4 = 0;
               LATBbits.LATB6 = 0;
               ontimer = 3;
               
           }
           }

           millisec = 0;
       }
       
       
       
    }
    
    return;
}

void adc_init(void)
{
    ADCON0bits.ADFM = 1;
    TRISAbits.TRISA4 = 1;
    ADPCHbits.ADPCH = 0b000100;
    ADCON0bits.ON = 1;
}
int adc_read(void)
{
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO_nDONE);
    result = ADRESH;
    result = result <<8;
    result |= ADRESL;
    return result;
}
void pwm_init(void)
{
    RA5PPS = 0x0D;
    TRISAbits.TRISA5 = 1;
    T2CONbits.CKPS = 0;
    T2CONbits.T2OUTPS = 0;
    T2HLTbits.MODE = 0;
    T2CLKCONbits.CS = 1;
    T2CONbits.ON = 1;
    PWM6CONbits.EN = 1;
    TRISAbits.TRISA5 = 0;
}
void pwm_set(unsigned int value)
{
    value = value <<6;
    PWM6DC = value;
    
}


void __interrupt () my_isr_routine (void) 
{
if(TMR0IE&&TMR0IF)
{ 
    TMR0L = 6;
    millisec++;
    TMR0IF = 0;
}    
}