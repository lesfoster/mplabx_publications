/*
 * File:   main_esp01.c
 * Author: lesfo
 *
 * Created on December 6, 2024, 8:48 PM
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
#pragma config WDTE = SWDTEN    // Watchdog Timer Enable bits (WDT controlled by the SWDTEN bit in the WDTCON register)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = LOW       // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
#pragma config PPS1WAY = OFF    // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection enabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#define ESP01_ECHO 0

#include <xc.h>
#include <string.h>
#include "../dht11_lib.X/dht11.h"
#include "../usart_xmit_lib.X/usart.h"
#include "../usart_rcv_lib.X/usart_rcv.h"
#include "../clock_timeout.X/clock_timeout.h"

// Connectivity has everything custom to your system, including
// target website address and WiFi credentials

#include "connectivity.h"

// Some sources say CWMODE_CUR
static unsigned const char SET_MODE[] = "AT+CWMODE=3\r\n";
static unsigned const char OK_RESP[] = "OK\r\n";
static unsigned const char CONNECT_RESP[] = "WIFI CONNECTED";
static unsigned const char ERR_RESP[] = "ERROR\r\n";
static const char *FAIL_RESP = "FAIL";
static const char *PING = "AT\r\n";
#if ESP01_ECHO == 1
static unsigned char ECHO_ON[] = "ATE1\r\n";
#else
static unsigned const char ECHO_OFF[] = "ATE0\r\n";
#endif

// The payload is 80 bytes long in example code. If your payload differs
// (for instance if port 80 or 18080 is used, or the web service resource
//  is not just the same) the value '80' below will have to be updated.
static unsigned const char GET_PRFX[] = "AT+CIPSEND=80\r\n";
static unsigned const char CLOSE[] = "AT+CIPCLOSE\r\n";
static unsigned const char HEXMAP[] = "0123456789ABCDEF";

// Giving plenty of input room in case of oddities.
static unsigned char USART_RCV_MSG[RCV_MSG_LEN];
static uint8_t usart_rcv_pos = 0;

// -----------------------FUNCTION DEF

static void setup_led_pin()
{
    WPUCbits.WPUC2 = 0;           // Active-high: no pull-up
    ANSELCbits.ANSC2 = 0;         // Unselect pin as analog.
    TRISCbits.TRISC2 = 0;         // Enable as output
    PORTCbits.RC2 = 0;            // Pre-clear

}

// Expect 10ms
static void short_wait(uint16_t waitcount)
{
    for (uint16_t i = 0; i < waitcount; i++)
    {
        for (uint16_t i = 0; i < 100; i++)
        {
            asm("NOP");
        }
    }
}

static void flash_led(uint16_t waitcount)
{
    // Can go on. For now, signal on LED
    PORTCbits.RC2 = 1;
    short_wait(waitcount);
    PORTCbits.RC2 = 0;
    short_wait(waitcount);

}

/**
 * Compare the expected message with buffer contents. Usually "OK\r\n" can go
 * there. This does a substring test
 * 
 * @param expected what is considered correct at current run phase
 * @return positive - expected > received; negative - received > expected; or 0
 */
static uint8_t uart_comp_msg(unsigned char *expected)
{
    if (NULL != strstr((const char *)USART_RCV_MSG, (const char *)expected))
    {
        return 1;
    }
    return 0;
}

static void uart_rcv_data_reset(void)
{
    for (uint8_t i = 0; i < RCV_MSG_LEN; i++)
    {
        USART_RCV_MSG[i] = 0;
    }
    usart_rcv_pos = 0;    // Can start over.        
}

/**
 * Sending the text and checking result. ESP01 may chatter back several
 * lines (even with echo off) before sending OK\r\n. So, multiple lines of
 * output must be expected.
 * 
 * @param msg
 * @return 1 = OK; 0 = not OK
 */
static uint8_t send_to_esp01(const char *msg, uint16_t waitct)
{
    uart_rcv_data_reset();    // Clear previous response
    usart_xmit_str((unsigned char *)msg);
    uint16_t retries = waitct;
    uint8_t rcv_ok = 0;
    while (! rcv_ok && ! uart_comp_msg((unsigned char *)ERR_RESP) && ! uart_comp_msg((unsigned char *)FAIL_RESP))
    {
        short_wait(100);
        //uint8_t msg_size = uart_poll_rx(USART_RCV_MSG, RCV_MSG_LEN);
        rcv_ok = uart_comp_msg((unsigned char *)OK_RESP);
        if (rcv_ok == 0)
        {
            rcv_ok = uart_comp_msg((unsigned char *)CONNECT_RESP);
        }
        if (-- retries == 0)
        {
            break; // Whatever status was in rcv_OK, is the status
        }
    }

    // Last chance - clear for new message.
    uart_rcv_data_reset();
    if (rcv_ok)
    {
        // Can go on. For now, signal on LED
        flash_led(10000);
        return 0;
    }
    else
    {
        flash_led(1000);
        flash_led(1000);
        flash_led(1000);
        return 1;
    }

}

static void setup_interrupts(void)
{
    PIR0 = 0;                     // Clear all interrupted flags
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;           // Enable general interrupts
}

static void hexpack(uint8_t val, unsigned char* rcvr, uint8_t offs)
{
    uint8_t nibble_l = val & 0x0f;
    uint8_t nibble_h = val >> 4;
    rcvr[offs] = HEXMAP[nibble_h];
    rcvr[offs+1] = HEXMAP[nibble_l];
}

static void transmit(struct dht11_reading reading)
{
    //012345678901234567890123
    //GET /reading/XXXXXXXXXX
    uint8_t offs = 13;
    hexpack(reading.humidityInt, GET_REQ, offs);
    hexpack(reading.humidityDec, GET_REQ, offs+2);
    hexpack(reading.tempInt, GET_REQ, offs+4);
    hexpack(reading.tempDec, GET_REQ, offs+6);
    hexpack(reading.cksumFlag, GET_REQ, offs+8);
    while (0 != send_to_esp01((const char *)CREATE_TCP, 10000))
    {
    }
    while (0 != send_to_esp01((const char *)GET_PRFX, 10000))
    {
    }
    while (0 != send_to_esp01((const char *)GET_REQ, 50000))
    {
    }
    while (0 != send_to_esp01((const char *)CLOSE, 20000))
    {
    }

}
// -----------------------INTERRUPT

void __interrupt() isr(void)
{
    // Handle interrupt for the USART receipt
    if (PIR1bits.RCIF == 1)
    {
        // Handling the receive interrupt.
        USART_RCV_MSG[usart_rcv_pos] = RC1REG;
        usart_rcv_pos ++;
        if (usart_rcv_pos > 75)
        {
            usart_rcv_pos = 75;
        }
    }
    if (IOCAFbits.IOCAF2 == 1)
    {
        if (PORTAbits.RA2)
        {
            // reset the timer
            TMR1H = 0;
            TMR1L = 0;
        }
        else
        {
            dht11_handle_interrupt();
        }
        // Clear interrupt-related flags
        PIR4bits.CCP1IF = 0;
        IOCAFbits.IOCAF2 = 0;
    }

    PIR0 = 0;
}

// -----------------------MAIN

void main(void)
{
    WDTCONbits.SWDTEN = 0;        // Turn off watchdog timer (WDT)  

    clktmo_setup_delay_clock();

    setup_led_pin(); // probably temporary.

    // Wait for ESP01 to warm up.
    short_wait(5000);

    setup_interrupts();

    uart_setup_tx();
    uart_setup_rx();

    send_to_esp01(PING, 5); // Simple AT/OK
    send_to_esp01(PING, 5); // Simple AT/OK
    while (0 != send_to_esp01((const char *) SET_MODE, 200))
    {
    }
    while (0 != send_to_esp01((const char *) CONNECT, 50000))
    {
    }

#if ESP01_ECHO==1
    send_to_esp01((const char *) ECHO_ON, 1000);
#else
    send_to_esp01((const char *) ECHO_OFF, 1000);
#endif

    while (1)
    {
        // multi second delay between:
        //  Multiply outer loop by inner loop and divide by 1,000,000
        //  to get seconds of wait time
        for (uint16_t i = 0; i < 3000; i++)
        {
            clktmo_delay(10000);  // 10ms total
        }
        // Using the edge count as an indicator of interrupt activity
        dht_init_read();

        // Present output
        transmit(dht_read());
    }

    while (1)
    {
        short_wait(12000);
    }
    return;
}
