/*
 * File:   freq_to_uart.c
 * Author: lesfo
 *
 * Created on June 30, 2024, 8:49 PM
 */

// PIC16LF18324 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with 2x PLL (32MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = LOW       // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 32000000     //define crystal frequency to 32MHz
#define INSTR_FREQ  8000000     // Fosc / 4 is instruction clock frequency

// Sending back a numeric value that overwrites the blanks
static unsigned char TX_MSG[] = {
  //01234567890123456 7 8
    "          :     %\r\n"
};
const static uint8_t TX_MSG_SIZE = (sizeof (TX_MSG) / sizeof (TX_MSG[0])) - sizeof(TX_MSG[0]);
const static uint8_t TX_BLANK_SIZE = TX_MSG_SIZE - 3;

static volatile uint16_t duty_period = 0;
static volatile uint16_t wave_period = 0;
static volatile uint16_t prev_duty_period = 0;
static volatile uint16_t prev_wave_period = 0;

// -----------------------SETUP
/**
 * See Data Sheet:
 * https://ww1.microchip.com/downloads/en/DeviceDoc/PIC16-L-F18324-18344-Data-Sheet-40001800D.pdf
 * Chapter 31, Section 31.1 "USART Asynchronous Mode" for the full settings
 */
static void setup_uart_tx(void) {
    // Using PIN RC4 for transmit (TX)
    // Setup the I/O pin
    ANSELCbits.ANSC4 = 0;      // Pin C4 is not an analog input
    RC4PPS = 0b10100;          // Pin C4 is the Transmit pin of USART

    // When SYNC = 0, BRGH = 1, BRG16 = 1 or SYNC = 1, BRG16 = 1,
    // these settings affect the baud rate.
    TX1STAbits.BRGH = 1;
    BAUD1CONbits.BRG16 = 1;
    TX1STAbits.SYNC = 0;

    // Set the BAUD rate
    // See table 31-4 of Data Sheet.
    SPBRGH = 0;
    SPBRGL = 68;

    // Enable the USART functions
    RC1STAbits.SPEN = 1;        // Serial port is enabled
    TX1STAbits.TXEN = 1;        // Transmit is enabled
}

/**
 * Frequency detect off the RA2 pin
 */
static void setup_timer_capture() {
    TRISAbits.TRISA2 = 1;       // Not output
    ANSELAbits.ANSA2 = 0;       // Not analog input
    CCP1PPS = 00010;
    // Timer backing
    T1CONbits.TMR1CS = 00;       // Fosc/4
    T1CONbits.T1CKPS = 00;       // Prescale 1:1
    T1CONbits.TMR1ON = 1;        // Enable timer
    // Capture hardware
    CCP1CONbits.CCP1MODE = 0b0011; // Edge triggering
    CCP1CONbits.CCP1EN = 1;      // Enable
    // Interrupt-on-change: IOC
    IOCAPbits.IOCAP2 = 1;        // Interrupt-on-RA2 pin positive
    IOCANbits.IOCAN2 = 1;        // Interrupt-on-RA2 pin negative
    CCP1CAPbits.CCP1CTS = 0b0100;// Capture source IOC interrupt
    PIE0bits.IOCIE = 1;          // Enable IOC
    PORTAbits.RA2 = 0;           // Preclear the pin
    
    INTCONbits.PEIE = 1;         // Enable 
    INTCONbits.GIE = 1;          // Enable general interrupts    
}

static void prep_num_content(
    float val, float max_pow_10, uint8_t int_digits, uint8_t offs) {

    // Fill in the value
    float decumulator = val * 100.0f;     // Past fractional part
    // 10000, 1000, 100, 10, 1
    float pow10 = max_pow_10 * 100.0f;    // Past fractional part
    uint8_t out_pos = offs;
    // Get the integer part of the value
    for (uint8_t digit = 0; digit < int_digits + 2; digit ++) {
        if (digit == int_digits) {
            TX_MSG[out_pos++] = '.';
        }
        uint8_t next_digit = (uint8_t)(decumulator / pow10);
        TX_MSG[out_pos++] = '0' + next_digit;
        decumulator -= next_digit * pow10;
        pow10 /= 10.0f;
    }
}

static void prep_msg(uint16_t wave_ticks, uint16_t duty_ticks) {
    // Reset all blanks
    for (uint8_t i = 0; i < TX_BLANK_SIZE; i ++) {
        TX_MSG[i] = ' ';
    }

    prep_num_content(INSTR_FREQ / (float)wave_ticks, 1000000.0, 7, 0);
    prep_num_content(100.0f * (float)duty_ticks/(float)wave_ticks, 10.0, 2, 11);
}
// -----------------------UART Interactions
/*
 * Loop over the message, character-by-character.
 */
void transmit() {
    // Write characters
    for (unsigned int i = 0; i < TX_MSG_SIZE; i++) {
        // Shovel the character into the fast shift buffer.
        TX1REG = TX_MSG[i];
        // Now we wait...
        // Polling immediately will return invalid results.
        asm("NOP");
        // This is only set when this USART1 is transmit enabled and no character
        // is waiting.
        while (PIR1bits.TXIF == 0) {
            // ...
        }
    }
}

// Timing
void setup_delay_clock() {
    T0CON0bits.T016BIT = 1; // 16-bit mode.
    T0CON1bits.T0CS = 0b011;// select HFINTOSC and disable TMR0 sync
    T0CON1bits.T0ASYNC = 1;
    /*
     * 0000 = 1:1
     * 0001 = 1:2
     * 0010 = 1:4   <- saw some success against VCR combo
     * 0011 = 1:8
     * 0100 = 1:16
     */
    T0CON1bits.T0CKPS = 0b0100;  // Prescaler as above
}

/**
 * Assumptions: At 1 microsecond, countdown of 1 unit is 1 microsecond.
 *              If prescale forces the count back to this time difference,
 *              no special calculations need be made.
 *              Since TMR0 counts up to 0xFFFF, I can subtract the number of
 *              microseconds from 0xFFFF and get the number of clock ticks.
 *              The 16-bit value must be broken into TMR0L and TMR0H.
 *              At rollover from 0xFFFF to 0, the interrupt flag is set.
 * @param us number of microseconds.  Count from FFFF-us to FFFF.
 * @param count number of loop iterations to count from FFFF-us to FFFF.
 */
void delay(unsigned int us) {
    // Take the input value and use it for an interrupt.
    // 16-bit TMR0 rolls over from FFFFh
    unsigned int adjustedUS = 0xFFFF - us; //(us*CYCLES_PER_US);
    T0CON0bits.T0EN = 0;   // Disable TMR0
    PIR0bits.TMR0IF = 0;    // clear the interrupt flag
    T0CON0bits.T0EN = 1;    // enable TMR0

    // Count-up start point established
    TMR0H = (unsigned char)(adjustedUS >> 8);   // load TMR0H
    TMR0L = (unsigned char)(adjustedUS & 0xFF); // load TMR0L after!

    PIR0bits.TMR0IF = 0;    // clear the interrupt flag

    while(PIR0bits.TMR0IF == 0) {
        // Now, we wait...
    }

    T0CON0bits.T0EN = 0;    // disable TMR0

}


// -----------------------INTERRUPT
void __interrupt() isr(void) {
    if (IOCAFbits.IOCAF2 == 1) {
        if (PORTAbits.RA2) {
            // Risen to 1
            wave_period = (uint16_t)(CCPR1H << 8) + (uint16_t)CCPR1L;
            // reset the timer
            TMR1H = 0;
            TMR1L = 0;
            if (wave_period != prev_wave_period) {
                prev_wave_period = wave_period;
            }
        } else {            
            duty_period = (uint16_t)(CCPR1H << 8) + (uint16_t)CCPR1L;
            if (duty_period != prev_duty_period) {
                prev_duty_period = duty_period;
            }
        }
        // Clear interrupt-related flags
        PIR4bits.CCP1IF = 0;
        IOCAFbits.IOCAF2 = 0;
    }
}


void main(void) {
    setup_uart_tx();
    setup_delay_clock();
    setup_timer_capture();

    while (1) {
        // Wait 1 second between.
        for (uint8_t i = 0; i < 200; i++) {
            delay(5000);  // 5ms
        }
        // Present double-buffered output
        prep_msg(prev_wave_period, prev_duty_period);
        transmit();
    }
    return;
}
