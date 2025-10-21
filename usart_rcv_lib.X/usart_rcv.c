#include <xc.h>

#include <string.h> 
#include "usart_rcv.h"

/*
 * NOTE: parts of this setup like UART enable and BAUD rate are shared
 * by the transmission setup. If transmission were removed (copy/paste)
 * the setup would be incomplete
 */
void uart_setup_rx(void)
{
#ifdef USART_RCV_PIN_A0
    ANSELAbits.ANSA0 = 0;       // Pin A0 is not an analog input
    RXPPS = 0b00000;            // Receive pin of USART is A0
    TRISAbits.TRISA0 = 1;       // Pin A0 is a (digital) input
#else
    ANSELCbits.ANSC5 = 0;       // Pin C5 is not an analog input
    RXPPS = 0b10101;            // Receive pin of USART is C5
    TRISCbits.TRISC5 = 1;       // Pin C5 is a (digital) input
#endif
    // Receipt will be interrupt driven
    PIE1bits.RCIE = 1;          // RCIE PIE1 interrupt enable
    INTCONbits.PEIE = 1;        // PEIE peripheral interrupt enable
    INTCONbits.GIE = 1;         // GIE general interrupt enable

    // Enable reception
    RC1STAbits.CREN = 1;
}

/**
 * Non interrupt-driven version.
 */
void uart_setup_rx_polled(void)
{
#if USART_RCV_PIN == C5
    ANSELCbits.ANSC5 = 0;       // Pin C5 is not an analog input
    RXPPS = 0b10101;            // Receive pin of USART is C5
    TRISCbits.TRISC5 = 1;       // Pin C5 is a (digital) input
#else
    ANSELCbits.ANSC2 = 0;       // Pin C2 is not an analog input
    RXPPS = 0b10010;            // Receive pin of USART is C2
    TRISCbits.TRISC2 = 1;       // Pin C2 is a (digital) input
#endif
    // Enable reception
    RC1STAbits.CREN = 1;
}

/**
 * Provide buffer to receive data. Use USART polling to pull in characters
 * 
 * @param buf place to put data
 * @param buflen how much space is available
 * @return length of read message
 */
uint8_t uart_poll_rx(unsigned char *buf, uint8_t buflen)
{
    // Poll for data.
    uint8_t usart_rcv_pos = 0;
    while (buf[usart_rcv_pos] != '\n')
    {
        if (RC1STAbits.FERR == 1) {
            RC1STAbits.SPEN = 0;
            RC1STAbits.SPEN = 1;
            break;
        } else if (RC1STAbits.OERR == 1) {
            buf[usart_rcv_pos++] = RC1REG;
            RC1STAbits.SPEN = 0;
            RC1STAbits.SPEN = 1;
            break;
        }
        while (PIR1bits.RCIF == 0)
        {
            asm("NOP");
        }
        buf[usart_rcv_pos++] = RC1REG;
        if (usart_rcv_pos == buflen) {
            usart_rcv_pos --; // Push back
        }
    }

    return usart_rcv_pos;
}

/**
 * Call this prior to SLEEP
 */
void uart_disable_rx(void)
{
    RC1STAbits.SPEN = 0;        // Disabled.
    RC1STAbits.CREN = 0;        // Receive is disabled
}

/**
 * Call this after wake from SLEEP
 */
void uart_reenable_rx(void)
{
    // Enable reception
    RC1STAbits.CREN = 1;
    RC1STAbits.SPEN = 1;        // Enabled.
}
