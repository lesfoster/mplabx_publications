#include <xc.h>
#include "dht11.h"
#include "dht11_overrides.h"
#include "../clock_timeout.X/clock_timeout.h"

/**
 * @File      dht11.c - DHT-11 Sensor library
 * @Purpose   Use the PIC16LF18324's pin RA2 (or override as RA4)
 *            to communicate with DHT-11 sensor
 * @Author    Les Foster
 * @Date      Begun 2024/12/06
 * @Notes     - Warning: no attempt has been made to operate pin-neutral
 *            - This library depends on the use (availability) of pin RA2
 *            - This library also depends on the clock_timeout library by
 *              same author
 *            - This library uses TIMER1. No other uses of TIMER1 should be
 *              done while using this library. Please be aware of all uses
 *              of TIMER1 on the PIC16LF18324 device, to avoid conflicts
 */

// Sending back a numeric value that overwrites the blanks
static unsigned char TX_MSG[] =
{
  //        %         C
  // 012345678901234567 7 8
    "                  \r\n"
};

const static uint8_t TX_MSG_SIZE = (sizeof (TX_MSG) / sizeof (TX_MSG[0])) - sizeof(TX_MSG[0]);
const static uint8_t TX_BLANK_SIZE = TX_MSG_SIZE - 3;
const static uint8_t FLAG_POS = TX_MSG_SIZE - 4;
static uint16_t DHT_DIFF_ACCUM[DHT_BIT_LEN];

static volatile uint8_t dht_fall_edge_count = 0;
static volatile uint8_t in_dht_payload = 0;


/**
 * DHT will be used from RA2 or RA4 pin.
 */
void dht_setup_timer_capture(void)
{
#ifdef DHT_PIN_RA4
    TRISAbits.TRISA4 = 0;        // Allow open-drain to change
    ODCONAbits.ODCA4 = 1;        // Setup Open Drain
    TRISAbits.TRISA4 = 1;        // Input
    ANSELAbits.ANSA4 = 0;        // Not analog input
    WPUAbits.WPUA4 = 0;          // No pullup
#else
    TRISAbits.TRISA2 = 0;        // Allow open-drain to change
    ODCONAbits.ODCA2 = 1;        // Setup Open Drain
    TRISAbits.TRISA2 = 1;        // Input
    ANSELAbits.ANSA2 = 0;        // Not analog input
    WPUAbits.WPUA2 = 0;          // No pullup
#endif
    CCP1PPS = 00010;
    // Timer backing
    T1CONbits.TMR1CS = 00;       // Fosc/4
    T1CONbits.T1CKPS = 00;       // Prescale 1:1
    T1CONbits.TMR1ON = 1;        // Enable timer
    // Capture hardware
    CCP1CONbits.CCP1MODE = 0b0011; // Edge triggering
    CCP1CONbits.CCP1EN = 1;      // Enable
    // Interrupt-on-change: IOC
#ifdef DHT_PIN_RA4
    IOCAPbits.IOCAP4 = 1;        // Interrupt-on-RA4 pin positive
    IOCANbits.IOCAN4 = 1;        // Interrupt-on-RA4 pin negative
#else
    IOCAPbits.IOCAP2 = 1;        // Interrupt-on-RA2 pin positive
    IOCANbits.IOCAN2 = 1;        // Interrupt-on-RA2 pin negative
#endif
    CCP1CAPbits.CCP1CTS = 0b0100;// Capture source IOC interrupt
    PIE0bits.IOCIE = 1;          // Enable IOC

    INTCONbits.PEIE = 1;         // Enable peripheral interrupts
    INTCONbits.GIE = 1;          // Enable general interrupts
}

void dht_timer_read_terminate(void)
{
    CCP1CONbits.CCP1EN = 0;      // Disable Capture Hardware
    T1CONbits.TMR1ON = 0;        // Disable timer
}

/**
 * This will send a request to the DHT for a read, and will establish
 * the timer characteristics to capture it via interrupt handling
 */
void dht_init_read(void)
{
    dht_fall_edge_count = 0;

    // In case full read was not received earlier
    in_dht_payload = 0;
    dht_timer_read_terminate();

    // At first, need to use the pin for output.

#ifdef DHT_PIN_RA4    
    ANSELAbits.ANSA4 = 0;       // Not analog input
    TRISAbits.TRISA4 = 0;       // Output
    PORTAbits.RA4 = 1;          // Set pin to match pulled-high state of line
    ODCONAbits.ODCA4 = 0;       // Not Open Drain
#else
    ANSELAbits.ANSA2 = 0;       // Not analog input
    TRISAbits.TRISA2 = 0;       // Output
    PORTAbits.RA2 = 1;          // Set pin to match pulled-high state of line
    ODCONAbits.ODCA2 = 0;       // Not Open Drain
#endif

    // Sending the command by pulling the line low for a defined period.
#ifdef DHT_PIN_RA4
    PORTAbits.RA4 = 0;          // Clear the pin
    clktmo_delay(20000);               // Hold low for 20ms
    PORTAbits.RA4 = 1;          // Set the pin
#else
    PORTAbits.RA2 = 0;          // Clear the pin
    clktmo_delay(20000);               // Hold low for 20ms
    PORTAbits.RA2 = 1;          // Set the pin
#endif

    // Change the pin to timer capture input
    dht_setup_timer_capture();
}


/**
 * Call this from interrupt handler to service the interrupts.
 */
void dht11_handle_interrupt(void)
{
    uint16_t hi_v_time = (uint16_t)(CCPR1H << 8) + (uint16_t)CCPR1L;
    if (in_dht_payload)
    {
        DHT_DIFF_ACCUM[dht_fall_edge_count] = hi_v_time;
        dht_fall_edge_count ++;           
    }
    else if (hi_v_time <= MAX_DHT_START && hi_v_time >= MIN_DHT_START)
    {
        // 80 µs ready/about-to-send response
        dht_fall_edge_count = 0;
        in_dht_payload = 1;
    }
    if (dht_fall_edge_count >= DHT_BIT_LEN)
    {
        // Shutdown for "the season".  A full DHT read has been completed.
        dht_timer_read_terminate();
    }
}


struct dht11_reading dht_read(void)
{
    struct dht11_reading reading;
    // Now need to await completion of timer capture.
    for (uint8_t ms = 0; ms < 200; ms++)
    {
        clktmo_delay(1000);
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
                intRH = (uint8_t)((intRH << 1) | (DHT_DIFF_ACCUM[i] > MIN_ONE_CYCLES ? 1: 0));
                decRH = (uint8_t)((decRH << 1) | (uint8_t)(DHT_DIFF_ACCUM[i+8] > MIN_ONE_CYCLES ? 1: 0));
                intT = (uint8_t)((intT << 1) | (uint8_t)(DHT_DIFF_ACCUM[i+16] > MIN_ONE_CYCLES ? 1: 0));
                decT = (uint8_t)((decT << 1) | (uint8_t)(DHT_DIFF_ACCUM[i+24] > MIN_ONE_CYCLES ? 1: 0));
                checksum = (uint8_t)(checksum << 1 | (uint8_t)(DHT_DIFF_ACCUM[i+32] > MIN_ONE_CYCLES ? 1: 0));
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
    float decumulator = (float)intPart;
    if (intPart < 0)
    {
        TX_MSG[offs] = '-';
    }
    uint8_t out_pos = offs + 1;
    for (uint8_t digit = 0; digit < 3; digit++)
    {
        uint8_t next_digit = (uint8_t)(decumulator / pow10);
        TX_MSG[out_pos++] = '0' + next_digit;
        decumulator -= next_digit * pow10;
        pow10 /= 10.0f;
        
    }
    TX_MSG[out_pos++] = '.';
    
    decumulator = (float)decPart;
    pow10 = 10.0f;
    for (uint8_t digit = 0; digit < 2; digit++)
    {
        uint8_t next_digit = (uint8_t)(decumulator / pow10);
        TX_MSG[out_pos++] = '0' + next_digit;
        decumulator -= next_digit * pow10;
        pow10 /= 10.0f;
    }
}

