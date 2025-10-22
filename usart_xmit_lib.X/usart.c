#include <xc.h>
#include "usart.h"

// Sending back a numeric value that overwrites the blanks
static unsigned char TX_MSG[] = {
  //                  
  // 01234567890123456 7 8
    "                 \r\n"
};
const static uint8_t TX_MSG_SIZE = (sizeof (TX_MSG) / sizeof (TX_MSG[0])) - sizeof (TX_MSG[0]);
const static uint8_t TX_BLANK_SIZE = TX_MSG_SIZE - 2;

//-----------------------------STATIC PROTOTYPES
static void prep_int32_content(int32_t val, uint8_t offs);
static void reset_blanks(void);
static uint8_t set_label(const char* label);

//-----------------------------UART Interactions
/**
 * See Data Sheet:
 * https://ww1.microchip.com/downloads/en/DeviceDoc/PIC16-L-F18324-18344-Data-Sheet-40001800D.pdf
 * Chapter 31, Section 31.1 "USART Asynchronous Mode" for the full settings
 */
void uart_setup_tx(void)
{
    // Using PIN RC4 for transmit (TX)
    // Setup the I/O pin
    ANSELCbits.ANSC4 = 0;      // Pin C4 is not an analog input
    RC4PPS = 0b10100;          // Pin C4 is the Transmit pin of USART

#ifndef USART_GNL_SETUP
    // When SYNC = 0, BRGH = 1, BRG16 = 1 or SYNC = 1, BRG16 = 1,
    // these settings affect the baud rate.
    TX1STAbits.BRGH = 1;
    BAUD1CONbits.BRG16 = 1;
    TX1STAbits.SYNC = 0;

    // Set the BAUD rate
    // See table 31-4 of Data Sheet.
    SPBRGH = 0x0D;
    SPBRGL = 0x04;

    // Enable the USART functions
    RC1STAbits.SPEN = 1;        // Serial port is enabled
    TX1STAbits.TXEN = 1;        // Transmit is enabled

#define USART_GNL_SETUP 1
#endif
}

/**
 * Call this prior to SLEEP
 */
void uart_disable_tx(void)
{
    TX1STAbits.TXEN = 0;        // Transmit is disabled
    RC1STAbits.SPEN = 0;        // Disabled.
    TRISCbits.TRISC4 = 1;       // Tristated
    PORTCbits.RC4 = 0;          // Low voltage
}

/**
 * Call this after wake from SLEEP
 */
void uart_reenable_tx(void)
{
    TRISCbits.TRISC4 = 0;       // Not tristated
    RC1STAbits.SPEN = 1;        // Enabled.
    TX1STAbits.TXEN = 1;        // Transmit is enabled
}

/**
 * Calling this will pack the output buffer with the desired message
 * 
 * @param val to be presented as digits
 * @param label prefix label for output
 */
void usart_prep_signed_32_msg(int32_t val, const char *label)
{
    reset_blanks();
    prep_int32_content(val, set_label(label));
}

/*
 * Transmit the message, up to either first 0 character or 512 chars.
 * More efficient to use usart_xmit_msg, which does not require size calc.
 * 
 * @param msg expected to be terminated with \r\n, and under 512 chars
 */
void usart_xmit_str(unsigned char msg[])
{
    uint16_t msg_size = 0;
    while (msg_size < 512 && msg[msg_size] != 0)
    {
        msg_size++;
    }

    usart_xmit_msg(msg, msg_size);
}

/*
 * Transmit the message, up to its size.
 * 
 * @param msg expected to be terminated with \r\n
 * @param msg_size includes termination characters \r\n
 */
void usart_xmit_msg(unsigned char msg[], uint16_t msg_size)
{
    // Write characters
    for (unsigned int i = 0; i < msg_size; i++)
    {
        // Shovel the character into the fast shift buffer.
        TX1REG = msg[i];
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

/*
 * Loop over the previously prepared message, character-by-character.
 */
void usart_xmit_signed_32_msg()
{
    PIE0bits.IOCIE = 0; // Disable IOC

    INTCONbits.PEIE = 0; // Disable peripheral interrupts
    INTCONbits.GIE = 0; // Disable general interrupts

    usart_xmit_msg(TX_MSG, TX_MSG_SIZE);
}

/**
 * Error messages can be pushed to output.
 * Ex: 'U', 5 : the fifth thing tested, while doing 'U', went wrong
 * 
 * @param statpt simple one-letter marker
 * @param stat integer value.
 */
void usart_stat(unsigned char statpt, uint8_t stat)
{
    // send status back to USART.
    TX_MSG[0] = '0' + stat;
    TX_MSG[1] = ' ';
    TX_MSG[2] = statpt;
    TX_MSG[3] = ' ';
    usart_xmit_msg(TX_MSG, TX_MSG_SIZE);

}


/**
 * Rolls digits out of a number as text characters
 */
static void prep_int32_content(int32_t val, uint8_t offs)
{
    uint32_t pow10 = 100000;
    uint8_t out_pos = offs;
    out_pos++;
    uint32_t decumulator;
    if (val < 0)
    {
        decumulator = (uint32_t)(-val);
        TX_MSG[offs] = '-'; // reserved for sign
    }
    else
    {
        decumulator = (uint32_t)val;
    }

    uint8_t reject_0s = 1;
    for (uint8_t digit = 0; digit < 6; digit++)
    {
        uint8_t next_digit = (uint8_t) (decumulator / pow10);
        TX_MSG[out_pos] = '0' + next_digit;
        if (next_digit != 0)
        {
            reject_0s = 0;
        }
        if (reject_0s == 0)
        {
            // Allow next non-zero digit to overwrite this one,
            // thus discarding "leading zeros"
            out_pos++;
        }

        decumulator -= next_digit * pow10;
        pow10 /= 10;
    }
}

static void reset_blanks()
{
    // Reset all blanks
    for (uint8_t i = 0; i < TX_BLANK_SIZE; i++)
    {
        TX_MSG[i] = ' ';
    }

}

static uint8_t set_label(const char* label)
{
    const char *ch = label;
    uint8_t i = 0;
    while (*ch != 0)
    {
        TX_MSG[i++] = *ch++;
    }

    return i;
}
