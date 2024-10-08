/*
 * File:   bmp180_to_usart.c
 * Author: lesfo
 *
 * Created on August 30, 2024, 12:01 PM
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
#include "bmp180.h"
#include "usart.h"
#include "i2c.h"

void main(void)
{
    uart_setup_tx();
    i2c_as_controller();

    // *** TEMP *** long wait to see _all_ usart output
    for (uint16_t w = 0; w < 65535; w++)
    {
        for (uint8_t iw = 0; iw < 200; iw++)
        {
            asm("NOP");        
        }
    }    

    // Collect the parameters from the sensor
    bmp180_init();

    while(1)
    {
        // Grabbing the reading register for T
        bmp180_set_sensor_req(TEMP_REQ);
        int16_t UT = bmp180_get_raw_temp();
        usart_prep_signed_32_msg(UT, "Raw T");
        usart_transmit();
        // Converting to true value
        usart_prep_signed_32_msg(bmp180_get_true_temp(UT), "T");
        usart_transmit();

        // Grabbing the reading register for P
        bmp180_set_sensor_req(PRESSURE_REQ);
        int24_t UP = bmp180_get_raw_pressure();
        usart_prep_signed_32_msg(UP, "Raw P");
        usart_transmit();

        // Converting to true value
        usart_prep_signed_32_msg(bmp180_get_true_pressure(UP), "P");
        usart_transmit();
        
        // Delay between attempts
        for (uint16_t j = 0; j < 10; j++)
        {
            for (uint16_t i = 0; i < 60000; i++)
            {
                asm("NOP");
            }
        }

    }

    return;
}
