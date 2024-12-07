#include <xc.h>

/**
 * @File      Clock Timeout Library
 * @Purpose   Use the PIC16LF18324's built in TIMER0 for accurate delays
 * @Author    Les Foster
 * @Date      Begun 2024/12/06
 * @Notes     Warning: this will use up your PIC's Timer0. It may not
 *            be used for other purposes; any other library or implied use
 *            could interfere with, or be interferred by this usage.
 *            It is assumed that the chip will run at 32MHz.
 */

/**
 * Call this prior to attempting to request delays.
 */
void clktmo_setup_delay_clock(void) {
    T0CON0bits.T016BIT = 1; // 16-bit mode.
    T0CON1bits.T0CS = 0b011;// select HFINTOSC and disable TMR0 sync
    T0CON1bits.T0ASYNC = 1;
    /*
     * 0000 = 1:1
     * 0001 = 1:2
     * 0010 = 1:4
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
void clktmo_delay(uint16_t us) {
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


