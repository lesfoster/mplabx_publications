/*
 * File:   organ.c
 * Author: lesfo
 * Created on October 20, 2025, 10:13 PM
 * 
 * Setup PWM to send over to a passive buzzer.
 * Send information from UART to allow it to play music.
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
 * ? Clear the TMR2IF interrupt flag bit of the
 * PIR1 register.
 * ? Select the Timer2 prescale value by
 * configuring the T2CKPS bit of the T2CON
 * register.
 * ? Enable Timer2 by setting the TMR2ON bit of
 * the T2CON register.
 * 6. Wait until the TMR2IF is set.
 * 7. When the TMR2IF flag bit is set:
 * ? Clear the associated TRIS bit(s) to enable
 * the output driver.
 * ? Route the signal to the desired pin by
 * configuring the RxyPPS register.
 * ? Enable the PWMx module by setting the
 * PWMxEN bit of the PWMxCON register
 * 
 * ASSUMPTIONS: clock configuration RSTOSC bits are set at 000
 * for option "HFINT32" or HFINTOSC with 2x PLL (32MHz).  Oddly,
 * to leverage this I need to assume 16MHz for my FOSC value.
 *
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
// Override
#define USART_RCV_PIN_A0 1
#include "../usart_rcv_lib.X/usart_rcv.h"
#include "../usart_xmit_lib.X/usart.h"
#include "../clock_timeout.X/clock_timeout.h"

static int C_ = 523;
static int c_ = 554;
static int D_ = 587;
static int d_ = 622;
static int E_ = 659;
static int F_ = 698;
static int f_ = 740;
static int G_ = 734;
static int g_ = 831;
static int A_ = 880;
static int a_ = 932;
static int B_ = 988;

// Giving plenty of input room in case of oddities.
static unsigned char USART_RCV_MSG[RCV_MSG_LEN];
static uint8_t usart_rcv_pos = 0;

static uint8_t is_eobuff = 0;

static long F_OSC = 16000000;
static long TMR2_PRESCALE = 64;

// Desired pulse ratio is 1/2.
static double PULSE_RATIO = 0.5;

static void setup_pwm(void)
{
    TRISAbits.TRISA2 = 1; // Disable as output
    // Using PWM Module 5 (but not 6)
    //Establishing polarity (active-high=0)
    PWM5CONbits.PWM5POL = 0;

    // Clear interrupt.
    PIR1bits.TMR2IF = 0;
    
    // Prescaler as direct input (x 1)
    // b1=0,b0=0 ==> prescaler=1; b1=0,b0=1 ==> prescaler=4;
    // b1=1,b0=0 ==> prescaler=16; b1=1,b0=1 ==> prescaler=64
    T2CONbits.T2CKPS0 = 1;
    T2CONbits.T2CKPS1 = 1;
    // Turn on timer, and then wait for it to equilibrate
    T2CONbits.TMR2ON = 1;
    while(PIR1bits.TMR2IF == 0)
    {
        // Now, we wait...
    }
    ANSELAbits.ANSA2 = 0; // Unselect pin as analog.
    TRISAbits.TRISA2 = 0; // Enable as output
    WPUAbits.WPUA2 = 0;   // No pullup.  Active-high polarity
    // Need to route this PWM to the bit of choice: RA2
    // PPS = Peripheral Pin Select.
    // 00010 = Rxy source is PWM5 (pg 161)
    RA2PPS = 0b00010;

}

static void frequency_and_duty(uint16_t freq)
{
    // disable the PWM module
    PWM5CONbits.PWM5EN = 0;
    
    // PR2 gets the PWM period value.  Length of the whole wave.
    // PR2 = PWMPer/(4*Tosc*TMR2prescale) - 1
    // -or-
    // PR2 = Fosc/(4*Fpwm*TMR2prescale) - 1
    //
    PR2 = ((unsigned char)(F_OSC / (double)(4 * freq * TMR2_PRESCALE)) - 1);

    // 10 bits of duty cycle to play with
    unsigned short dutyCycle = (unsigned short)(PULSE_RATIO * 4 * (PR2+1));
    //unsigned short dutyCycle = (unsigned short)(PR2 * PULSE_RATIO);
    //debugPR2 = PR2;
    PWM5DCH = (unsigned char)(dutyCycle >> 2);
    //debugDutyShifted = (unsigned char)(dutyCycle >> 2);

    PWM5DCLbits.PWM5DCL0 = dutyCycle & 1;
    PWM5DCLbits.PWM5DCL1 = dutyCycle >> 1 & 1;

    // enable the PWM module
    PWM5CONbits.PWM5EN = 1;
}

static void short_wait(uint16_t waitcount)
{
    for (uint16_t i = 0; i < waitcount; i++)
    {
        // 200 µs
        clktmo_delay(200);
    }
}

static void setup_interrupts(void)
{
    PIR0 = 0;                     // Clear all interrupted flags
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;           // Enable general interrupts
}

static void clear_buffer(void)
{
    for (uint8_t i=0; i < RCV_MSG_LEN; i++)
    {
        USART_RCV_MSG[i] = 0;
    }
}

// -----------------------INTERRUPT
void __interrupt() isr(void)
{
    // Handle interrupt for the USART receipt
    if (PIR1bits.RCIF == 1)
    {
        if (usart_rcv_pos >= RCV_MSG_LEN)
        {
            // Force back to beginning if too many
            // characters.
            usart_rcv_pos = 0;
        }

        // Handling the receive interrupt.        
        USART_RCV_MSG[usart_rcv_pos] = RC1REG;
        if (USART_RCV_MSG[usart_rcv_pos] == '\n')
        {
            // Set only here in the interrupt handler
            is_eobuff = 1;
        }
        
        usart_rcv_pos ++;
    }

    PIR0 = 0;
}

static int is_note(uint8_t ch)
{
    return (ch >= 'a' && ch <= 'g') || (ch >= 'A' && ch <= 'G');
}

void main(void)
{
    uart_setup_tx();
    uart_setup_rx();
    setup_pwm();

    clear_buffer();
    setup_interrupts();
    clktmo_setup_delay_clock();

    unsigned char song_buffer[RCV_MSG_LEN];
    uint8_t song_len = 0;
    uint8_t note = 0;
    
    while(1)
    {
        if (is_eobuff == 1)
        {
            // Reload the "song buffer"
            for (uint8_t i=0; i < RCV_MSG_LEN && is_note(USART_RCV_MSG[i]); i++)
            {
                song_buffer[i] = USART_RCV_MSG[i];
                song_len = i+1;
            }

            // No longer at end
            // Clear only here in main loop
            is_eobuff = 0;

            // Clear buffer to avoid accidental end-line detection
            clear_buffer();
            
            if (song_len < 0)
            {
                song_len = 0;
            }
            // Reset buffer receipt and lag.
            usart_rcv_pos = 0;
            note = 0;

        }
        if (song_len == 0)
        {
            continue;
        }
        
        if (note == song_len)
        {
            note = 0;
        }
        // Note-to-frequency mapping.
        // Upper case is natural. Lower case is sharp
        switch (song_buffer[note])
        {
        case 'A':
            frequency_and_duty(A_);
            break;
        case 'a':
            frequency_and_duty(a_);
            break;
        case 'B':
            frequency_and_duty(B_);
            break;
        case 'b':
            // B# - since some signatures include a sharp at the
            // 'position' of B, this is being re-mapped to the next
            // half-step value of 'C'
            frequency_and_duty(C_);
            break;
        case 'C':
            frequency_and_duty(C_);
            break;
        case 'c':
            frequency_and_duty(c_);
            break;
        case 'D':
            frequency_and_duty(D_);
            break;
        case 'd':
            frequency_and_duty(d_);
            break;
        case 'E':
            frequency_and_duty(E_);
            break;
        case 'e':
            // E# - since some signatures include a sharp at the
            // 'position' of E, this is being re-mapped to the next
            // half-step value of 'F'
            frequency_and_duty(F_);
            break;
        case 'F':
            frequency_and_duty(F_);
            break;
        case 'f':
            frequency_and_duty(f_);
            break;
        case 'G':
            frequency_and_duty(G_);
            break;
        case 'g':
            frequency_and_duty(g_);
            break;
        default:
            // disable the PWM module
            PWM5CONbits.PWM5EN = 0;
            break;
        }
        // Wait a while...
        short_wait(2000);
        // Next note
        note++;

    }
    return;
}
