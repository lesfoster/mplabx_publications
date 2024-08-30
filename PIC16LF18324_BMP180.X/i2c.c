#include <xc.h>

#include "i2c.h"
#include "utils.h"
#include "usart.h"

static uint8_t wait_for_donebit(void);
static uint8_t wait_for_buf(uint8_t exval);

//-----------------------------Sensor Interactions
/**
 * Data Sheet:
 * "Typical Transmit Sequence
 * 1. The user generates a Start condition by setting
 * the SEN bit of the SSP1CON2 register.
 * 2. SSP1IF is set by hardware on completion of the
 * Start.
 * 3. SSP1IF is cleared by software.
 * 4. The MSSP1 module will wait the required start
 * time before any other operation takes place.
 * 5. The user loads the SSP1BUF with the slave
 * address to transmit.
 * 6. Address is shifted out the SDA pin until all eight
 * bits are transmitted. Transmission begins as
 * soon as SSP1BUF is written to.
 * 7. The MSSP1 module shifts in the ACK bit from
 * the slave device and writes its value into the
 * ACKSTAT bit of the SSP1CON2 register.
 * 8. The MSSP1 module generates an interrupt at
 * the end of the ninth clock cycle by setting the
 * SSP1IF bit.
 * 9. The user loads the SSP1BUF with eight bits of
 * data.
 * 10. Data is shifted out the SDA pin until all eight bits
 * are transmitted.
 * 11. The MSSP1 module shifts in the ACK bit from
 * the slave device and writes its value into the
 * ACKSTAT bit of the SSP1CON2 register.
 * 12. Steps 8-11 are repeated for all transmitted data
 * bytes.
 * 13. The user generates a Stop or Restart condition
 * by setting the PEN or RSEN bits of the
 * SSP1CON2 register. Interrupt is generated
 * once the Stop/Restart condition is complete."
 */

/**
 * Data Sheet:
 * "Typical Receive Sequence:
 * 1. The user generates a Start condition by setting
 * the SEN bit of the SSP1CON2 register.
 * 2. SSP1IF is set by hardware on completion of the
 * Start.
 * 3. SSP1IF is cleared by software.
 * 4. User writes SSP1BUF with the slave address to
 * transmit and the R/W bit set.
 * 5. Address is shifted out the SDA pin until all eight
 * bits are transmitted. Transmission begins as
 * soon as SSP1BUF is written to.
 * 6. The MSSP1 module shifts in the ACK bit from
 * the slave device and writes its value into the
 * ACKSTAT bit of the SSP1CON2 register.
 * 7. The MSSP1 module generates an interrupt at
 * the end of the ninth clock cycle by setting the
 * SSP1IF bit.
 * 8. User sets the RCEN bit of the SSP1CON2 regis-
 * ter and the master clocks in a byte from the slave.
 * 9. After the eighth falling edge of SCL, SSP1IF and
 * BF are set.
 * 10. Master clears SSP1IF and reads the received
 * byte from SSP1BUF, clears BF.
 * 11. Master sets ACK value sent to slave in ACKDT
 * bit of the SSP1CON2 register and initiates the
 * ACK by setting the ACKEN bit.
 * 12. Master?s ACK is clocked out to the slave and
 * SSP1IF is set.
 * 13. Software clears SSP1IF.
 * 14. Steps 8-13 are repeated for each received byte
 * from the slave.
 * 15. Master sends a not ACK or Stop to end
 * communication."
 */

/**
 * Data sheet:
 * "Master mode is enabled by setting and clearing the
 * appropriate SSPM<3:0> bits in the SSP1CON1 regis-
 * ter and by setting the SSPEN bit. In Master mode, the
 * SDA and SCK pins must be configured as inputs. The
 * MSSP peripheral hardware will override the output
 * driver TRIS controls when necessary to drive the pins
 * low"
 * 
 * Master mode settings vary to set BAUD as follows:
 * 0000 - FOSC/4
 * 0001 - FOSC/16
 * 0010 - FOSC/64
 * 0011 - T2_match/2
 */
void i2c_as_controller()
{
    ANSELAbits.ANSA2 = 0;      // Pin A2 is not an analog input
    ANSELCbits.ANSC0 = 0;      // Pin C0 is not an analog input
    // Tri-stating is overridden by h/w as needed
    TRISAbits.TRISA2 = 1;      // Tri-stating for 'input' use.
    TRISCbits.TRISC0 = 1;      // Tri-stating for 'input' use.

    // Because of the dual nature of I2C pins, both input and output
    // settings must be in place.
    SSP1CLKPPS = 0b00010;      // RA2 'Input' pin for SCL
    SSP1DATPPS = 0b10000;      // RC0 'Input' pin for SDA

    RA2PPS = 0b11000;          // 11000 = Rxy source is SCK1/SCL1
    RC0PPS = 0b11001;          // 11001 = Rxy source is SDO1/SDA1

    SSP1CON1bits.SSPM = 0b1000;// Mode is Master mode.
    SSP1CON1bits.SSPEN = 1;
    SSP1ADD = 19;              // 79 --> 100khz. 31,250 Hz lowest possible Baud
}

/**
 * Data sheet:
 * "Master sends a not ACK or Stop to end communication."
 */
uint8_t i2c_stop()
{
    SSP1CON2bits.PEN = 1;
    return (uint8_t)(! wait_for_donebit());
}

/**
 * The address transmitted here will be either for read (leading 1)
 * or write (leading 0).
 * 
 * @param periph_addr address of peripheral device (or register)
 * @return status from attempting to send address;
 *      0=OK,2=start time out,3=ack receipt time out,1=ackstat false
 */
uint8_t i2c_transmit_addr(uint8_t periph_addr)
{
    SSP1CON1bits.WCOL = 0;          // pre-clear.
    PIR1bits.SSP1IF = 0;            // pre-clear.

    //
    // Generate the start condition in I2C. Start condition completion
    // is signaled by the "done bit", and it uses the buffer.
    //
    SSP1CON2bits.SEN = 1;

    if (! wait_for_donebit())
    {
        return 1;
    }
    if (! wait_for_buf(0))
    {
        return 4;
    }

    //
    // Send the address of the peripheral we want to send data to
    //
    SSP1BUF = periph_addr;

    if (SSP1CON1bits.WCOL == 1)
    {
        // We didn't wait long enough for completion
        SSP1CON1bits.WCOL = 0;
        return 3;
    }

    if (! wait_for_donebit())
    {
        // Transmission of address did not complete in time
        return 2;
    }
    if (! wait_for_buf(0))
    {
        return 5;
    }

    // The interrupt flag is set upon ack received.
    // But the ack value may be 0 (acknowledged) or 1
    // negative-ack.
    if (SSPCON2bits.ACKSTAT)
    {
        return 6;
    }
    return 0;
}

/**
 * Send should happen after sending a 7-bit address byte whose high-order bit
 * is set to 0.
 * 
 * Data sheet:
 * 9. The user loads the SSP1BUF with eight bits of
 * data.
 * 10. Data is shifted out the SDA pin until all eight bits
 * are transmitted.
 * 11. The MSSP1 module shifts in the ACK bit from
 * the slave device and writes its value into the
 * ACKSTAT bit of the SSP1CON2 register.
 * 12. Steps 8-11 are repeated for all transmitted data
 * bytes.
 * 13. The user generates a Stop or Restart condition
 * by setting the PEN or RSEN bits of the
 * SSP1CON2 register. Interrupt is generated
 * once the Stop/Restart condition is complete."
 * 
 * @param val what to send
 * @return 0 = successful send; 1 = send timeout
 */
uint8_t i2c_send(uint8_t val)
{
    SSP1BUF = val;
    return (uint8_t)(! wait_for_donebit() && ! wait_for_buf(0));
}

/**
 * Receipt should happen after sending a 7-bit address byte whose
 * high-order bit is set to 1.
 * 
 * Data sheet:
 *"8. User sets the RCEN bit of the SSP1CON2 register and
 *    the master clocks in a byte from the slave.
 * 9. After the eighth falling edge of SCL, SSP1IF and
 *    BF are set.
 * 10. Master clears SSP1IF and reads the received
 *    byte from SSP1BUF, clears BF.
 * 11. Master sets ACK value sent to slave in ACKDT
 *    bit of the SSP1CON2 register and initiates the
 *    ACK by setting the ACKEN bit.
 * 12. Master?s ACK is clocked out to the slave and
 *    SSP1IF is set.
 * 13. Software clears SSP1IF."
 * 
 * @return data byte, or 0 if failure
 */
uint8_t i2c_rcv_ubyte(uint8_t do_ack)
{
    SSP1CON1bits.WCOL = 0;          // pre-clear.
    PIR1bits.SSP1IF = 0;            // pre-clear.

    // Suggested by ChatGPT
    volatile uint8_t dummy = SSP1BUF;         // Discard first byte
    SSP1CON2bits.RCEN = 1;

    if (! wait_for_donebit())
    {
        usart_stat('U', 2);
    }
    if (! wait_for_buf(1))
    {
        usart_stat('U', 3);
    }

    // This could be undefined. Need to pull the value to clear BF
    uint8_t rtn_val = SSP1BUF;

    // Ack regardless of returned value
    if (do_ack)
    {
        SSP1CON2bits.ACKDT = 0;
        SSP1CON2bits.ACKEN = 1;
        if (!wait_for_donebit())
        {
            usart_stat('U', 4);
            rtn_val = 0;
        }

    }

    return rtn_val;
}

/**
 * Receipt should happen after sending a 7-bit address byte whose
 * high-order bit is set to 1.
 * 
 * Data Sheet:
 * "14. Steps 8-13 are repeated for each received byte
 * from the slave.
 * 15. Master sends a not ACK or Stop to end
 * communication.
 * 
 * @return data word, or 0 if failure
 */
uint16_t i2c_rcv_ushort()
{
    uint8_t b1 = i2c_rcv_ubyte(1);
    uint8_t b2 = i2c_rcv_ubyte(1);
    uint8_t b3 = i2c_rcv_ubyte(0);
    return (uint16_t)b1 << 8 | (uint16_t)b2;
}

/**
 * Receipt should happen after sending a 7-bit address byte whose
 * high-order bit is set to 1.
 * 
 * Data Sheet:
 * "14. Steps 8-13 are repeated for each received byte
 * from the slave.
 * 15. Master sends a not ACK or Stop to end
 * communication.
 * 
 * @return data word, or 0 if failure
 */
uint24_t i2c_rcv_u24()
{
    uint8_t b1 = i2c_rcv_ubyte(1);
    uint8_t b2 = i2c_rcv_ubyte(1);
    uint8_t b3 = i2c_rcv_ubyte(1);
    uint8_t b4 = i2c_rcv_ubyte(0);

    return (uint24_t)b1 << 16 | (uint24_t)b2 << 8 | (uint24_t)b3;
}

/**
 * Hold up processing until the SSP1IF bit has been set. Clear it afterward.
 * 
 * @return 1=SSP1IF was set; 0=SSP1IF was not set during wait time
 */
static uint8_t wait_for_donebit()
{
    uint16_t loopct = 0;
    uint8_t time_remains = 1;
    // SSP1IF - interrupt flag for completion in MSSP subsystem.
    //   Set by h/w; read by s/w; cleared by s/w
    while (time_remains && (PIR1bits.SSP1IF == 0))
    {
        // Check if time remains
        time_remains = waiting(++loopct);
    }
    if (time_remains)
    {
        PIR1bits.SSP1IF = 0;
    }
    return time_remains;
}

/**
 * Wait for the I2C buffer to contain data
 * 
 * @return 1=BF has expected value; 0=BF does not have expected value
 */
static uint8_t wait_for_buf(uint8_t exval)
{
    uint16_t loopct = 0;
    // BF : buffer in use. Set by h/w; cleared by h/w; read by s/w
    uint8_t time_remains = 1;
    while (time_remains && (SSP1STATbits.BF != exval))
    {
        // Check if time remains
        time_remains = waiting(++loopct);
    }
    return (uint8_t)(SSP1STATbits.BF == exval);
}

