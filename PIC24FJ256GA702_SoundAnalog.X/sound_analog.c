/*
 * File:   sound_analog.c
 * Author: lesfo
 *
 * Created on June 10, 2025, 12:01 AM
 */
// FSEC
#pragma config BWRP = OFF               // Boot Segment Write-Protect bit (Boot Segment may be written)
#pragma config BSS = DISABLED           // Boot Segment Code-Protect Level bits (No Protection (other than BWRP))
#pragma config BSEN = OFF               // Boot Segment Control bit (No Boot Segment)
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = DISABLED           // General Segment Code-Protect Level bits (No Protection (other than GWRP))
#pragma config CWRP = OFF               // Configuration Segment Write-Protect bit (Configuration Segment may be written)
#pragma config CSS = DISABLED           // Configuration Segment Code-Protect Level bits (No Protection (other than CWRP))
#pragma config AIVTDIS = OFF            // Alternate Interrupt Vector Table bit (Disabled AIVT)

// FBSLIM
#pragma config BSLIM = 0x1FFF           // Boot Segment Flash Page Address Limit bits (Enter Hexadecimal value)

// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Source Selection (Fast RC Oscillator with divide-by-N with PLL module (FRCPLL) )
#pragma config PLLMODE = PLL96DIV2      // PLL Mode Selection (96 MHz PLL. Oscillator input is divided by 2 (8 MHz input))
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFCN = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config SOSCSEL = ON             // SOSC Power Selection Configuration bits (SOSC is used in crystal (SOSCI/SOSCO) mode)
#pragma config PLLSS = PLL_PRI          // PLL Secondary Selection Configuration bit (PLL is fed by the Primary oscillator)
#pragma config IOL1WAY = ON             // Peripheral pin select configuration bit (Allow only one reconfiguration)
#pragma config FCKSM = CSDCMD           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler bits (1:32,768)
#pragma config FWPSA = PR128            // Watchdog Timer Prescaler bit (1:128)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bits (WDT and SWDTEN disabled)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config WDTWIN = WIN25           // Watchdog Timer Window Select bits (WDT Window is 25% of WDT period)
#pragma config WDTCMX = WDTCLK          // WDT MUX Source Select bits (WDT clock source is determined by the WDTCLK Configuration bits)
#pragma config WDTCLK = LPRC            // WDT Clock Source Select bits (WDT uses LPRC)

// FPOR
#pragma config BOREN = ON               // Brown Out Enable bit (Brown Out Enable Bit)
#pragma config LPCFG = OFF              // Low power regulator control (No Retention Sleep)
#pragma config DNVPEN = ENABLE          // Downside Voltage Protection Enable bit (Downside protection enabled using ZPBOR when BOR is inactive)

// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = ON              // JTAG Enable bit (JTAG is enabled)

// FDEVOPT1
#pragma config ALTCMPI = DISABLE        // Alternate Comparator Input Enable bit (C1INC, C2INC, and C3INC are on their standard pin locations)
#pragma config TMPRPIN = OFF            // Tamper Pin Enable bit (TMPRN pin function is disabled)
#pragma config SOSCHP = ON              // SOSC High Power Enable bit (valid only when SOSCSEL = 1 (Enable SOSC high power mode (default))
#pragma config ALTI2C1 = ALTI2CEN       // Alternate I2C pin Location (SDA1 and SCL1 on RB9 and RB8)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define SYS_FREQ        32000000L
#define FCY             (SYS_FREQ/2)

#include <libpic30.h>
#include <xc.h>

/**
 * According to documentation, this should be enough to move the OC1 PWM
 * output from RB7 to RB6. However at time of writing (5/31/2025) it
 * does not change anything. RB7 still has PWM. RB6 does not.
 */
static void rb6_as_oc1(void)
{
    uint8_t old_gie = INTCON2bits.GIE;
    INTCON2bits.GIE = 0;
    // 1. Unlock PPS
    __builtin_write_OSCCONL(OSCCON & 0xBF);  // Clear IOLOCK
    // 2. Map OC1 (function 13) to RP6 (RB6)
    RPOR3bits.RP7R = 0;
    RPOR3bits.RP6R = 13;  // RP6 (RB6) assigned to OC1 output
    // 3. Lock PPS
    __builtin_write_OSCCONL(OSCCON | 0x40);  // Set IOLOCK
    INTCON2bits.GIE = old_gie;
}

static void setup_led_pin(void)
{
    // RB6 is for LED output
    CNPUBbits.CNPUB6 = 0;         // Active-high: no pull-up
    // Apparently, not bit 6. ANSELBbits.ANSB6 = 0; // Unselect pin as analog.
    TRISBbits.TRISB6 = 0;         // Enable as output
    PORTBbits.RB6 = 0;            // Low
}

/**
 * https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/ReferenceManuals/30003035b.pdf
 * Example 7-1
 */
static void setup_pwm(void)
{
    rb6_as_oc1();
    
    // Set MCCP operating mode
    CCP1CON1Lbits.CCSEL = 0; // Set MCCP operating mode (OC mode)
    CCP1CON1Lbits.MOD = 0b0101; // Set mode (Buffered Dual-Compare/PWM mode)
    //Configure MCCP Timebase
    CCP1CON1Lbits.T32 = 0; // Set timebase width (16-bit)
    CCP1CON1Lbits.TMRSYNC = 0; // Set timebase synchronization (Synchronized)
    CCP1CON1Lbits.CLKSEL = 0b000; // Set the clock source (Tcy)
    CCP1CON1Lbits.TMRPS = 0b00; // Set the clock pre-scaler (1:1)
    CCP1CON1Hbits.TRIGEN = 0; // Set Sync/Triggered mode (Synchronous)
    CCP1CON1Hbits.SYNC = 0b00000; // Select Sync/Trigger source (Self-sync)
    //Configure MCCP output for PWM signal
    CCP1CON2Hbits.OCAEN = 1; // Enable desired output signals (OC1A)
    CCP1CON3Hbits.OUTM = 0b000; // Set advanced output modes (Standard output)
    CCP1CON3Hbits.POLACE = 0; //Configure output polarity (Active High)
    CCP1TMRL = 0x0000; //Initialize timer prior to enable module.
    CCP1PRL = 0x0458; //Configure timebase period

    CCP1RA = 0x0230; // Set the rising edge compare value
    CCP1RB = 0x0458; // Set the falling edge compare value
    CCP1CON1Lbits.CCPON = 1; // Turn on MCCP module
}

static void proportional_duty(uint16_t a)
{
    if (a < 0xB000)
    {
        CCP1CON1Lbits.CCPON = 0; // Turn on MCCP module
        CCP1RB = 0x0458;
    }
    else
    {        
        CCP1CON1Lbits.CCPON = 1; // Turn on MCCP module
        CCP1RB = a / 60;
    }
}

static void setup_analog_pin(void)
{
    ANSAbits.ANSA0 = 1;  // Select for Analog
    TRISAbits.TRISA0 = 1;// For input
}

void delay(uint16_t ms)
{
    __delay_ms(ms);
}

void __attribute__ ((__interrupt__)) _ADC1Interrupt(void)
{
    IFS0bits.AD1IF = 0;
}

// From Example 51.1
// https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/ReferenceManuals/39739b.pdf
static void setup_analog_ops(void)
{
    AD1CON1 = 0x2200; // Configure sample clock source
    // and conversion trigger mode.
    // Unsigned Fraction format (FORM<1:0>=10),
    // Manual conversion trigger (SSRC<3:0>=0000),
    // Manual start of sampling (ASAM=0),
    // No operation in Idle mode (ADSIDL=1),
    // S/H in Sample (SAMP = 1)
    AD1CON2 = 0; // Configure A/D voltage reference
    // and buffer fill modes.
    // Vr+ and Vr- from AVdd and AVss(PVCFG<1:0>=00, NVCFG=0),
    // Inputs are not scanned,
    // Interrupt after every sample
    AD1CON3 = 2; // Configure sample time = 1Tad,
    // A/D conversion clock as Tcy
    AD1CHS = 0; // Configure input channels,
    // S/H+ input is AN0,
    // S/H- input is Vr- (AVss).
    AD1CSSL = 0; // No inputs are scanned.
    IFS0bits.AD1IF = 0; // Clear A/D conversion interrupt.
    // Configure A/D interrupt priority bits (AD1IP<2:0>) here, if
    // required. Default priority level is 4.
    IEC0bits.AD1IE = 1; // Enable A/D conversion interrupt
    AD1CON1bits.ADON = 1; // Turn on A/D
    //    AD1CON1bits.SAMP = 1; // Start sampling the input
    //    delay(50); // Ensure the correct sampling time has elapsed
    //    // before starting conversion.
    //    AD1CON1bits.SAMP = 0; // End A/D sampling and start conversion
}

void main(void)
{
    setup_led_pin();
    setup_pwm();
    setup_analog_pin();
    setup_analog_ops();
    while (1) // repeat continuously
    {
        // From PIC24 ADC Ref Example 51-2
        AD1CON1bits.SAMP = 1; // start sampling...
        delay(1); // Ensure the correct sampling time has elapsed
        // before starting conversion.
        AD1CON1bits.SAMP = 0; // start converting
        while (!AD1CON1bits.DONE) {
        }; // conversion done?

        uint16_t adc_value = ADC1BUF0; // yes then get ADC value
        proportional_duty(adc_value);
    }    
    return;
}
