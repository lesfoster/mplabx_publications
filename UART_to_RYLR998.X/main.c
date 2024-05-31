/*
 * File:   main.c
 * Author: llf
 *
 * PIC16LF18324 communicating via a RYLR998 LoRa device
 * Created on May 16, 2024, 8:49 PM
 */

// Errors expected from receipt of data
#define RCV_FRAMERR 1
#define RCV_OVERRUN 2
#define RCV_BADCONTENT 3
#define RCV_LONGINPUT 4
#define RCV_DELUGE 5

// First digit in ASCII.
#define DIGIT_BASE '0'

// Input operand, operator offsets
#define L_OND_OFFS 10
#define OPER_OFFS 11
#define R_OND_OFFS 12

// Output answer offsets
#define L_ANS_OFFS 13
#define R_ANS_OFFS 14

#define _XTAL_FREQ 32000000 //define crystal frequency to 32MHz

// PIC16LF18324 Configuration Bit Settings

// 'C' source line config statements
// CONFIG1
#pragma config FEXTOSC = OFF    // No FEXTOSC External Oscillator mode Selection bits (LP (crystal oscillator) optimized for 32.768 kHz)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with 2x PLL (32MHz))
#pragma config CLKOUTEN = ON    // Clock Out Enable bit (CLKOUT function is enabled; FOSC/4 clock appears at OSC2)
#pragma config CSWEN = OFF      // Clock Switch Enable bit (The NOSC and NDIV bits cannot be changed by user software)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config MCLRE = OFF      // Master Clear Enable bit (MCLR/VPP pin function is digital input; MCLR internally disabled; Weak pull-up under control of port pin's WPU control bit.)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN = ON     // Low-power BOR enable bit (ULPBOR enabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled)
#pragma config BORV = HIGH      // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.7V)
#pragma config PPS1WAY = OFF    // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will not cause a Reset)
#pragma config DEBUG = ON       // Debugger enable bit (Background debugger enabled)

// CONFIG3
#pragma config WRT = ALL        // User NVM self-write protection bits (0000h to 0FFFh write protected, no addresses may be modified)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (High Voltage on MCLR/VPP must be used for programming.)

// CONFIG4
#pragma config CP = ON          // User NVM Program Memory Code Protection bit (User NVM code protection enabled)
#pragma config CPD = ON         // Data NVM Memory Code Protection bit (Data NVM code protection enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

// Receipt can fail for various reasons.
static volatile uint8_t rcv_status = 0;
static volatile uint8_t complete_msg = 0;

// sending 2-character message for the answer to address 15 on the RYLR's set network ID
static unsigned char TX_MSG[] = {
    "AT+SEND=15,2,XX\r\n"
};
const static int TX_MSG_SIZE = (sizeof (TX_MSG) / sizeof (TX_MSG[0])) - sizeof(TX_MSG[0]);

//Example:
//+RCV=15,3,1+1,-99,40\r\n
// Giving plenty of input room in case of oddities.
static unsigned char RCV_MSG[80];
static uint8_t rcv_pos = 0;

// -----------------------SETUP

/**
 * See Data Sheet:
 * https://ww1.microchip.com/downloads/en/DeviceDoc/PIC16-L-F18324-18344-Data-Sheet-40001800D.pdf
 * Chapter 31, Section 31.1 "USART Asynchronous Mode" for the full settings
 */
void setup_uart_tx(void) {
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

/*
 * NOTE: parts of this setup like UART enable and BAUD rate are shared
 * by the transmission setup. If transmission were removed (copy/paste)
 * the setup would be incomplete
 */
void setup_uart_recv(void) {
    ANSELCbits.ANSC5 = 0;       // Pin C5 is not an analog input
    RXPPS = 0b10101;            // Receive pin of USART is C5
    TRISCbits.TRISC5 = 1;       // Pin C5 is a (digital) input

    // Receipt will be interrupt driven
    PIE1bits.RCIE = 1;          // RCIE PIE1 interrupt enable
    INTCONbits.PEIE = 1;        // PEIE peripheral interrupt enable
    INTCONbits.GIE = 1;         // GIE general interrupt enable

    // Enable reception
    RC1STAbits.CREN = 1;
}

// -----------------------TRANSMISSIONS

/**
 * Work out what the message should be. 
 */
void prep_msg(void) {
    // Working out what the message should be.
    if (rcv_status != 0) {
        // Front with indication of error
        TX_MSG[L_ANS_OFFS] = 'E';
        TX_MSG[R_ANS_OFFS] = DIGIT_BASE + rcv_status;
        rcv_status = 0;
    } else {
        int val = 0;
        if (RCV_MSG[OPER_OFFS] == '+' || RCV_MSG[OPER_OFFS] == '*') {
            if (RCV_MSG[OPER_OFFS] == '*') {
                val = (RCV_MSG[L_OND_OFFS] - DIGIT_BASE) *
                      (RCV_MSG[R_OND_OFFS] - DIGIT_BASE);                
            } else {
                val = (RCV_MSG[L_OND_OFFS] - DIGIT_BASE) +
                      (RCV_MSG[R_OND_OFFS] - DIGIT_BASE);                
            }
            TX_MSG[L_ANS_OFFS] = DIGIT_BASE + (uint8_t) (val / 10);
            TX_MSG[R_ANS_OFFS] = DIGIT_BASE + (val % 10);
        } else if (RCV_MSG[OPER_OFFS] == '-') {
            val = RCV_MSG[L_OND_OFFS] - RCV_MSG[R_OND_OFFS];
            if (val < 0) {
                TX_MSG[L_ANS_OFFS] = '-';
                val = -val;
            } else {
                TX_MSG[L_ANS_OFFS] = DIGIT_BASE;
            }
            TX_MSG[R_ANS_OFFS] = (uint8_t) val + DIGIT_BASE;
        } else {
            TX_MSG[L_ANS_OFFS] = 'N';
            TX_MSG[14] = 'O';
        }
    }
}

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
    complete_msg = 0;
    rcv_pos = 0;
}

// -----------------------INTERRUPT

void __interrupt() isr(void) {
    if (PIE1bits.RCIE == 1 && PIR1bits.RCIF == 1) {
        // Handling the receive interrupt.
        if (RC1STAbits.FERR == 1) {
            rcv_status = RCV_FRAMERR;
        }
        RCV_MSG[rcv_pos] = RC1REG;
        if (RC1STAbits.OERR == 1) {
            // frame error discarded in this case
            rcv_status = RCV_OVERRUN;
            RC1STAbits.CREN = 0;
        }

        if (RCV_MSG[rcv_pos] == '\n') {
            complete_msg = 1;
        }

        rcv_pos++;
        if (rcv_pos >= 75) {
            rcv_status = RCV_LONGINPUT;
            rcv_pos = 75; // Keep pushing this back...
        }
    }
}


// -----------------------MAIN

void main(void) {
    setup_uart_tx();
    transmit(); // opening message
    setup_uart_recv();
    while (1) {
        if (complete_msg == 1) {
            if (rcv_pos >= 12) {
                // got whole problem.
                // Room for improvement: could add magic num to 'protocol'
                prep_msg();
                transmit();                
            } else {
                // Possibly just OK response.
                // When a 'send' is carried out, the
                // RYLR responds with +OK\r\n
                rcv_pos = 0;
                complete_msg = 0;
            }
        }
        __delay_ms(1000); // Delay of 1 second

    }
    return;
}
