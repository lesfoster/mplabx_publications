/*
 * File:   dht_to_uart.c
 * Author: lesfo
 *
 * Created on July 13, 2024, 11:05 AM
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

#define DHT_BIT_LEN 40
#define MIN_ONE_CYCLES 50 * 8
#define MAX_DHT_START 95 * 8
#define MIN_DHT_START 75 * 8

// Sending back a numeric value that overwrites the blanks
static unsigned char TX_MSG[] = {
    //        %         C
    // 012345678901234567 7 8
    "                  \r\n"
};
const static uint8_t TX_MSG_SIZE = (sizeof (TX_MSG) / sizeof (TX_MSG[0])) - sizeof (TX_MSG[0]);
const static uint8_t TX_BLANK_SIZE = TX_MSG_SIZE - 3;
const static uint8_t FLAG_POS = TX_MSG_SIZE - 4;
static uint16_t DHT_DIFF_ACCUM[DHT_BIT_LEN];

static volatile uint8_t dht_fall_edge_count = 0;
static volatile uint8_t in_dht_payload = 0;

struct dht11_reading
{
    uint8_t tempInt; // limited T range 0 - 50 C
    uint8_t tempDec;
    uint8_t humidityInt;
    uint8_t humidityDec;
    uint8_t cksumFlag;
};

// -----------------------Timing

static void setup_delay_clock()
{
    T0CON0bits.T016BIT = 1; // 16-bit mode.
    T0CON1bits.T0CS = 0b011; // select HFINTOSC and disable TMR0 sync
    T0CON1bits.T0ASYNC = 1;
    /*
     * 0000 = 1:1
     * 0001 = 1:2
     * 0010 = 1:4   <- saw some success against VCR combo
     * 0011 = 1:8
     * 0100 = 1:16
     */
    T0CON1bits.T0CKPS = 0b0100; // Prescaler as above
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
void delay(uint16_t us)
{
    // Take the input value and use it for an interrupt.
    // 16-bit TMR0 rolls over from FFFFh
    unsigned int adjustedUS = 0xFFFF - us; //(us*CYCLES_PER_US);
    T0CON0bits.T0EN = 0; // Disable TMR0
    PIR0bits.TMR0IF = 0; // clear the interrupt flag
    T0CON0bits.T0EN = 1; // enable TMR0

    // Count-up start point established
    TMR0H = (unsigned char) (adjustedUS >> 8); // load TMR0H
    TMR0L = (unsigned char) (adjustedUS & 0xFF); // load TMR0L after!

    PIR0bits.TMR0IF = 0; // clear the interrupt flag

    while (PIR0bits.TMR0IF == 0)
    {
        // Now, we wait...
    }

    T0CON0bits.T0EN = 0; // disable TMR0

}

// -----------------------SETUP

/**
 * See Data Sheet:
 * https://ww1.microchip.com/downloads/en/DeviceDoc/PIC16-L-F18324-18344-Data-Sheet-40001800D.pdf
 * Chapter 31, Section 31.1 "USART Asynchronous Mode" for the full settings
 */
void setup_uart_tx(void)
{
    // Using PIN RC4 for transmit (TX)
    // Setup the I/O pin
    ANSELCbits.ANSC4 = 0; // Pin C4 is not an analog input
    RC4PPS = 0b10100; // Pin C4 is the Transmit pin of USART

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
    RC1STAbits.SPEN = 1; // Serial port is enabled
    TX1STAbits.TXEN = 1; // Transmit is enabled
}

static void dht_timer_read_terminate(void)
{
    CCP1CONbits.CCP1EN = 0; // Disable Capture Hardware
    T1CONbits.TMR1ON = 0; // Disable timer
}

/**
 * DHT will be used from RA2 pin.
 */
static void setup_timer_capture()
{
    TRISAbits.TRISA2 = 0;  // Recommended prior to going open-drain
    ODCONAbits.ODCA2 = 1;  // Setup Open Drain
    TRISAbits.TRISA2 = 1;  // Input
    ANSELAbits.ANSA2 = 0;  // Not analog input
    WPUAbits.WPUA2 = 0;    // No pullup
    CCP1PPS = 00010;
    // Timer backing
    T1CONbits.TMR1CS = 00; // Fosc/4
    T1CONbits.T1CKPS = 00; // Prescale 1:1
    T1CONbits.TMR1ON = 1;  // Enable timer
    // Capture hardware
    CCP1CONbits.CCP1MODE = 0b0011; // Edge triggering (rise or fall)
    CCP1CONbits.CCP1EN = 1;// Enable
    // Interrupt-on-change: IOC
    IOCAPbits.IOCAP2 = 1;  // Interrupt-on-RA2 pin positive
    IOCANbits.IOCAN2 = 1;  // Interrupt-on-RA2 pin negative
    CCP1CAPbits.CCP1CTS = 0b0100; // Capture source IOC interrupt
    PIE0bits.IOCIE = 1;    // Enable IOC

    INTCONbits.PEIE = 1;   // Enable peripheral interrupts
    INTCONbits.GIE = 1;    // Enable general interrupts
}

/**
 * This will send a request to the DHT for a read, and will establish
 * the timer characteristics to capture it via interrupt handling
 */
static void init_dht_read()
{
    // In case full read was not received earlier
    in_dht_payload = 0;
    dht_timer_read_terminate();

    // At first, need to use the pin for output.
    ANSELAbits.ANSA2 = 0;  // Not analog input
    TRISAbits.TRISA2 = 0;  // Output
    PORTAbits.RA2 = 1;     // Set pin to match pulled-high state of line
    ODCONAbits.ODCA2 = 0;  // Not Open Drain

    // Sending the command by pulling the line low for a defined period.
    PORTAbits.RA2 = 0;     // Clear the pin
    delay(20000);          // Hold low for 20ms
    PORTAbits.RA2 = 1;     // Set the pin

    // Change the pin to timer capture input
    setup_timer_capture();
}

static struct dht11_reading read_dht()
{
    struct dht11_reading reading;
    // Now need to await completion of timer capture.
    for (uint8_t ms = 0; ms < 200; ms++)
    {
        delay(1000);
        if (dht_fall_edge_count >= DHT_BIT_LEN)
        {
            in_dht_payload = 0;

            uint8_t intRH = 0;
            uint8_t decRH = 0;
            uint8_t intT = 0;
            uint8_t decT = 0;
            uint8_t checksum = 0;
            for (int i = 0; i < 8; i++)
            {
                // Single loop to copy all bytes
                intRH = (uint8_t) ((intRH << 1) | (DHT_DIFF_ACCUM[i] > MIN_ONE_CYCLES ? 1 : 0));
                decRH = (uint8_t) ((decRH << 1) | (uint8_t) (DHT_DIFF_ACCUM[i + 8] > MIN_ONE_CYCLES ? 1 : 0));
                intT = (uint8_t) ((intT << 1) | (uint8_t) (DHT_DIFF_ACCUM[i + 16] > MIN_ONE_CYCLES ? 1 : 0));
                decT = (uint8_t) ((decT << 1) | (uint8_t) (DHT_DIFF_ACCUM[i + 24] > MIN_ONE_CYCLES ? 1 : 0));
                checksum = (uint8_t) (checksum << 1 | (uint8_t) (DHT_DIFF_ACCUM[i + 32] > MIN_ONE_CYCLES ? 1 : 0));
            }

            // Checking the sum
            uint16_t all_check = intRH + decRH + intT + decT;
            reading.cksumFlag = 0;
            if (checksum != (uint8_t) (all_check & 0xff))
            {
                // Not OK
                reading.cksumFlag = 1;
            }
            reading.humidityDec = decRH;
            reading.humidityInt = intRH;
            reading.tempDec = decT;
            reading.tempInt = intT;

            // Clear old content
            for (uint8_t i = 0; i < DHT_BIT_LEN; i++)
            {
                DHT_DIFF_ACCUM[i] = 0;
            }
            break;
        }
    }
    return reading;
}

/**
 * Temperatures can go negative!
 * 
 * @param intPart 
 * @param decPart
 * @param offs
 */
static void prep_num_content(uint8_t intPart, uint8_t decPart, uint8_t offs)
{
    // 100, 10, 1
    float pow10 = 100.0f;
    float decumulator = (float) intPart;
    if (intPart < 0)
    {
        TX_MSG[offs] = '-';
    }
    uint8_t out_pos = offs + 1;
    for (uint8_t digit = 0; digit < 3; digit++)
    {
        uint8_t next_digit = (uint8_t) (decumulator / pow10);
        TX_MSG[out_pos++] = '0' + next_digit;
        decumulator -= next_digit * pow10;
        pow10 /= 10.0f;

    }
    TX_MSG[out_pos++] = '.';

    decumulator = (float) decPart;
    pow10 = 10.0f;
    for (uint8_t digit = 0; digit < 2; digit++)
    {
        uint8_t next_digit = (uint8_t) (decumulator / pow10);
        TX_MSG[out_pos++] = '0' + next_digit;
        decumulator -= next_digit * pow10;
        pow10 /= 10.0f;
    }
}

static void prep_msg(struct dht11_reading reading)
{
    // Reset all blanks
    for (uint8_t i = 0; i < TX_BLANK_SIZE; i++)
    {
        TX_MSG[i] = ' ';
    }
    if (dht_fall_edge_count == 0)
    {
        TX_MSG[0] = 'X';
        return;
    }
    if (dht_fall_edge_count < DHT_BIT_LEN)
    {
        TX_MSG[0] = '?';
    }

    prep_num_content(reading.humidityInt, reading.humidityDec, 0);
    prep_num_content(reading.tempInt, reading.tempDec, 9);
    TX_MSG[7] = '%';
    TX_MSG[16] = 'C';
    if (reading.cksumFlag)
    {
        TX_MSG[FLAG_POS] = '!';
    }

}

// -----------------------UART Interactions

/*
 * Loop over the message, character-by-character.
 */
static void transmit()
{
    PIE0bits.IOCIE = 0; // Disable IOC

    INTCONbits.PEIE = 0; // Disable peripheral interrupts
    INTCONbits.GIE = 0; // Disable general interrupts

    // Write characters
    for (unsigned int i = 0; i < TX_MSG_SIZE; i++)
    {
        // Shovel the character into the fast shift buffer.
        TX1REG = TX_MSG[i];
        // Now we wait...
        // Polling immediately will return invalid results.
        asm("NOP");
        // This is only set when this USART1 is transmit enabled and no character
        // is waiting.
        while (PIR1bits.TXIF == 0)
        {
            // ...
        }
    }
}


// -----------------------INTERRUPT

void __interrupt() isr(void)
{
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
            uint16_t hi_v_time = (uint16_t) (CCPR1H << 8) + (uint16_t) CCPR1L;
            if (in_dht_payload)
            {
                DHT_DIFF_ACCUM[dht_fall_edge_count] = hi_v_time;
                dht_fall_edge_count++;
            }
            else if (hi_v_time <= MAX_DHT_START && hi_v_time >= MIN_DHT_START)
            {
                // 80 µs ready/about-to-send response
                dht_fall_edge_count = 0;
                in_dht_payload = 1;
            }
        }
        // Clear interrupt-related flags
        PIR4bits.CCP1IF = 0;
        IOCAFbits.IOCAF2 = 0;
    }
    if (dht_fall_edge_count >= DHT_BIT_LEN)
    {
        // Shutdown for "the season".  A full DHT read has been completed.
        dht_timer_read_terminate();
    }

}

void main(void)
{
    setup_uart_tx();
    setup_delay_clock();

    while (1)
    {
        // multi second delay between
        for (uint16_t i = 0; i < 2000; i++)
        {
            delay(5000); // 5ms
        }
        // Using the edge count as an indicator of interrupt activity
        dht_fall_edge_count = 0;

        init_dht_read();
        struct dht11_reading reading = read_dht();

        // Present output
        prep_msg(reading);
        transmit();
    }
    return;
}
