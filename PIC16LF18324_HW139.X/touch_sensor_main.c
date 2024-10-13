/*
 * File:   touch_sensor_main.c
 * Author: lesfo
 *
 * Created on October 12, 2024, 2:03 PM
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
#include "../usart_xmit_lib.X/usart.h"

#define TS_LOW_POWER 1

static volatile uint8_t transmitting = 0;

static void setup_sensor_pin()
{
    WPUAbits.WPUA2 = 0;           // Active-high: no pull-up
    ANSELAbits.ANSA2 = 0;         // Unselect pin as analog. [required for input]
    TRISAbits.TRISA2 = 1;         // Disable as output
    PORTAbits.RA2 = 0;            // Pre-clear.

}

static void setup_led_pin()
{
    WPUAbits.WPUA1 = 0;           // Active-high: no pull-up
    ANSELAbits.ANSA1 = 0;         // Unselect pin as analog.
    TRISAbits.TRISA1 = 0;         // Enable as output
    PORTAbits.RA1 = 0;            // Pre-clear

}

static void enable_interrupts()
{
    IOCAFbits.IOCAF2 = 0;         // Clear the edge-trigger flag
    IOCAPbits.IOCAP2 = 1;         // Enable positive edge trigger/RA2
    PIE0bits.IOCIE = 1;           // Enable interrupt-on-change
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;           // Enable general interrupts
}

// Expect 10ms
static void short_wait(void)
{
    for (uint16_t i = 0; i < 50; i++)
    {
        for (uint16_t i = 0; i < 100; i++)
        {
            asm("NOP");
        }
    }
}

// Expect 1/10 of second or 100ms
static void mid_wait(void)
{
    for (uint16_t i = 0; i < 50; i++)
    {
        for (uint16_t i = 0; i < 1000; i++)
        {
            asm("NOP");
        }
    }
}

// Expect ~3s
static void long_wait(void)
{
    for (uint16_t i = 0; i < 50; i++)
    {
        for (uint16_t i = 0; i < 30000; i++)
        {
            asm("NOP");
        }
    }
}

// -----------------------INTERRUPT
void __interrupt() isr(void)
{
    if (IOCAFbits.IOCAF2 == 1)
    {
        if (transmitting == 0)
        {
            transmitting = 1;
        }
        IOCAFbits.IOCAF2 = 0;         // Clear the edge-trigger flag        
    }
    // Clear interrupt-related flags
    PIR0 = 0;                         // Clear all interrupted flags
}

void main(void)
{
    uart_setup_tx();
    setup_sensor_pin();
    enable_interrupts();

    // Delay and send message
    short_wait();
#if TS_LOW_POWER
    usart_xmit_str((unsigned char *)"AT+MODE=1\r\n");
    mid_wait();
    uart_disable_tx();
    setup_led_pin();
    SLEEP();
#else
    short_wait();
    usart_xmit_str((unsigned char *)"AT+MODE=0\r\n");
#endif

    while (1)
    {
        if (transmitting)
        {
#if TS_LOW_POWER
            uart_reenable_tx();
            // Wait between operations so that communications can complete
            mid_wait(); // long_wait works.
            usart_xmit_str((unsigned char *)"AT+MODE=0\r\n");
            // Flash as a sanity check: is the chip really awake?
            PORTAbits.RA1 = 1;
            mid_wait();
            PORTAbits.RA1 = 0;
#else
            // Flash as a sanity check: is the transmitting bit really set?
            PORTAbits.RA1 = 0;
            long_wait();
            PORTAbits.RA1 = 1;
#endif

            short_wait();
            usart_xmit_str((unsigned char *)"AT+SEND=15,10,Touched...\r\n");
            long_wait();
#if TS_LOW_POWER
            usart_xmit_str((unsigned char *)"AT+MODE=1\r\n");
            mid_wait();
            uart_disable_tx();
#endif
            transmitting = 0;
        }
#if TS_LOW_POWER
        SLEEP();
#else
        long_wait();
#endif
    }

}
