/*
 * File:   simple_pwm.c
 * Author: L Foster
 * 
 * https://www.mouser.com/datasheet/2/268/40001800C-1314258.pdf, page 193
 * The following steps should be taken when configuring
 * the module for using the PWMx outputs:
 * 1. Disable the PWMx pin output driver(s) by setting
 * the associated TRIS bit(s).
 * 2. Configure the PWM output polarity by
 * configuring the PWMxPOL bit of the PWMxCON
 * register.
 * 3. Load the PR2 register with the PWM period
 * value, as determined by Equation 19-1.
 * 4. Load the PWMxDCH register and bits <7:6> of
 * the PWMxDCL register with the PWM duty cycle
 * value, as determined by Equation 19-2.
 * 5. Configure and start Timer2:
 *    a. Clear the TMR2IF interrupt flag bit of the PIR1 register.
 *    b. Select the Timer2 prescale value by configuring the T2CKPS bit of the
 *       T2CON register.
 *    c. Enable Timer2 by setting the TMR2ON bit of the T2CON register.
 * 6. Wait until the TMR2IF is set.
 * 7. When the TMR2IF flag bit is set:
 *    a. Clear the associated TRIS bit(s) to enable the output driver.
 *    b. Route the signal to the desired pin by configuring the RxyPPS
 *       register.
 *    c. Enable the PWMx module by setting the PWMxEN bit of the PWMxCON
 *       register
 * 
 * ASSUMPTIONS: clock configuration RSTOSC bits are set at 000
 * for option "HFINT32" or HFINTOSC with 2x PLL (32MHz).  Oddly,
 * to leverage this I need to assume 16MHz for my FOSC value.
 *
 * Created on May 16, 2023, 11:57 PM
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

#pragma config WDTE = OFF

// Range of pulse ratio.
#define MIN_PULSE_RATIO 0.15
#define MAX_PULSE_RATIO 0.85

static const long _XTAL_FREQ = 32000000;

// NOTE:
// If this frequency is changed 'very much', it is necessary to try the
// PR2 calculation below, with the values in these variables.
// * the timer2 prescale
// * oscillator frequency
//
// Otherwise, the calculation for PR2 will yield a value that simply overflows
// its 8 bit capacity and does not give the PWM frequency expected.
// It is because of this limitation that the prescale value exists.
// Further, the prescale assumption in this constant must match the
// prescale register bits timer-2-clock-prescale 1 and 0 (T2CLKPS1, T2CLKPS0)
// See also https://www.mouser.com/datasheet/2/268/40001800C-1314258.pdf 19.1.1
//
static const long F_PWM = 20000;

static const long F_OSC = 32000000;
static const long TMR2_PRESCALE = 64;

// Direction for changing pulse ratio
static double ratio_increment = 0.01;
static double pulse_ratio = MIN_PULSE_RATIO;

static void set_duty_cycle(double pulse_ratio)
{
    // Duty cycle defined in terms of PR2 - the wavelength (or 1/frequency)
    unsigned short dutyCycle = (unsigned short)(pulse_ratio * 4 * (PR2+1));

    // 10 bits of duty cycle to play with
    // High 8 bits to PWM 5's Duty Cycle Hi
    PWM5DCH = (unsigned char)(dutyCycle >> 2);

    // Low 2 bits to PWM 5's Duty Cycle Low
    PWM5DCLbits.PWM5DCL0 = dutyCycle & 1;
    PWM5DCLbits.PWM5DCL1 = dutyCycle >> 1 & 1;
}

static void setup_pwm(void)
{
    TRISAbits.TRISA2 = 1; // Disable as output, until config complete
    // Using PWM Module 5 (but not 6)
    //Establishing polarity (active-high=0)
    PWM5CONbits.PWM5POL = 0;
    // PR2 gets the PWM period value.  Length of the whole wave.
    // PR2 = PWMPer/(4*Tosc*TMR2prescale) - 1
    // -or-
    // PR2 = Fosc/(4*Fpwm*TMR2prescale) - 1
    //
    //PR2 = ((unsigned char)(F_OSC / (double)(4 * F_PWM * TMR2_PRESCALE)) - 1);
    PR2 = 255;

    set_duty_cycle(MIN_PULSE_RATIO);

    // Clear interrupt.
    PIR1bits.TMR2IF = 0;
    
    // Prescaler as direct input (x 1)
    // b1=0,b0=0 ==> prescaler=1; b1=0,b0=1 ==> prescaler=4;
    // b1=1,b0=0 ==> prescaler=16; b1=1,b0=1 ==> prescaler=64
    T2CONbits.T2CKPS1 = 1;
    T2CONbits.T2CKPS0 = 1;
    // Turn on timer, and then wait for it to equilibrate
    T2CONbits.TMR2ON = 1;
    while(PIR1bits.TMR2IF == 0) {
        // Now, we wait...
    }
    ANSELAbits.ANSA2 = 0; // Unselect pin as analog.
    TRISAbits.TRISA2 = 0; // Enable as output
    WPUAbits.WPUA2 = 0;   // No pullup.  Active-high polarity
    // Need to route this PWM to the pin of choice: RA2
    // PPS = Peripheral Pin Select.
    // 00010 = Rxy source is PWM5 (pg 161)
    RA2PPS = 0b00010;

    // Finally - enable the PWM module    
    PWM5CONbits.PWM5EN = 1;

}

void main(void)
{
    setup_pwm();
    while(1)
    {
        //__delay_ms(10);
        pulse_ratio += ratio_increment;
        if (pulse_ratio >= MAX_PULSE_RATIO)
        {
            ratio_increment = -0.01;
        }
        else if (pulse_ratio <= MIN_PULSE_RATIO)
        {
            ratio_increment = 0.01;
        }
        set_duty_cycle(pulse_ratio);
    }
    return;
}
