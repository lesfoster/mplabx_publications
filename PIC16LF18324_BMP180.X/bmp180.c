/*
 * File:   bmp180_to_usart.c
 * Author: lesfo
 *
 * Created on August 2, 2024, 6:23 PM
 */
#include <xc.h>
#include "bmp180.h"
#include "usart.h"
#include "i2c.h"

static int16_t AC1 = 0;
static int16_t AC2 = 0;
static int16_t AC3 = 0;
static uint16_t AC4 = 0;
static uint16_t AC5 = 0;
static uint16_t AC6 = 0;
static int16_t B1 = 0;
static int16_t B2 = 0;
static int16_t MB = 0;
static int16_t MC = 0;
static int16_t MD = 0;

static int32_t s_B5 = 0;


static uint24_t bmp180_get_24bit_val(uint8_t regaddr);
static uint16_t bmp180_get_16bit_regval(uint8_t regaddr);

void bmp180_init()
{
    // Collect the parameters from the sensor
    while (AC1 == 0)
    {
        AC1 = (int16_t)bmp180_get_16bit_regval(AC1_REG_ADDR);
        AC2 = (int16_t)bmp180_get_16bit_regval(AC2_REG_ADDR);
        AC3 = (int16_t)bmp180_get_16bit_regval(AC3_REG_ADDR);

        // Unsigned values begin
        AC4 = bmp180_get_16bit_regval(AC4_REG_ADDR);
        AC5 = bmp180_get_16bit_regval(AC5_REG_ADDR);
        AC6 = bmp180_get_16bit_regval(AC6_REG_ADDR);
        // Unsigned values end

        B1 = (int16_t)bmp180_get_16bit_regval(B1_REG_ADDR);
        B2 = (int16_t)bmp180_get_16bit_regval(B2_REG_ADDR);
        MB = (int16_t)bmp180_get_16bit_regval(MB_REG_ADDR);
        MC = (int16_t)bmp180_get_16bit_regval(MC_REG_ADDR);
        MD = (int16_t)bmp180_get_16bit_regval(MD_REG_ADDR);
        if (AC1 == 0)
        {
            for (uint16_t w = 0; w < 65535; w++)
            {
                asm("NOP");        
            }                
        }
    }
}

void bmp180_set_sensor_req(uint8_t req)
{
    uint8_t stat = 0;
    if (0 != (stat = i2c_transmit_addr((uint8_t)BMP_WRITE_ADDR)))
    {
        i2c_stop();
        usart_stat('A', stat);
    }
    else
    {
        // Send the register address w/in device
        // This completes the request for what is to be received next.
        stat = i2c_send(0xF4);
        if (stat != 0)
        {
            usart_stat('T', stat);
        }
        else
        {
            stat = i2c_send(req);
            if (stat != 0)
            {
                usart_stat('t', stat);
            }
        }
        i2c_stop();
    }

    // Delay 7.5 ms
    for (uint16_t w = 0; w < 60000; w++)
    {
        asm("NOP");        
    }

}

int24_t bmp180_get_raw_pressure(void)
{
    // Same register for T and P
    return (int24_t)((uint24_t)bmp180_get_24bit_val(0xF6) >> (8 - OSS));
}

int16_t bmp180_get_raw_temp(void)
{
    return (int16_t)bmp180_get_16bit_regval(0xF6);
}

int32_t bmp180_get_true_temp(int16_t UT)
{
    int32_t X1 = (((int32_t)UT - (int32_t)AC6) * (int32_t)AC5) >> 15; // Divide by 2^15
    // Derivative calculations used in adjusting temperature
    int32_t X2 = ((int32_t)MC << 11) / ((int32_t)X1 + (int32_t)MD);

    s_B5 = X1 + X2;
    return (s_B5 + 8) >> 4; // Divide by 2^4
}

int32_t bmp180_get_true_pressure(int32_t UP)
{
    int32_t B6 = s_B5 - 4000;
    int32_t X1 = (B2 * ((B6*B6) >> 12)) >> 11;
    int32_t X2 = ((int32_t)AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;
    int32_t p_B3 = ((((int32_t)AC1*4 + (int32_t)X3) << OSS) + 2) / 4;
    X1 = ((int32_t)AC3 * (int32_t)B6) >> 13;
    X2 = ((int32_t)B1 * (((int32_t)B6*(int32_t)B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) / 4;

    uint32_t p_B4 = (uint32_t)((AC4 * ((uint32_t)X3 + 32768)) >> 15);

    uint32_t B7 = ((uint32_t)UP - (uint32_t)p_B3) * (50000 >> OSS);
    int32_t temp_p = 0;
    // Keeping numbers small
    if (B7 < 0x80000000) {
    	temp_p = (int32_t)((B7 * 2) / p_B4);
    } else {
     	temp_p = (int32_t)((B7 / p_B4) * 2);
    }
    X1 = (temp_p >> 8) * (temp_p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * temp_p) >> 16;
    return temp_p + ((X1 + X2 + 3791) >> 4);
}

static uint24_t bmp180_get_24bit_val(uint8_t regaddr)
{
    uint24_t val = 0;

    // Initial step: can I read just one register off the BMP180?
    // Send the address with write=0 high bit
    uint8_t stat = 0;
    if (0 != (stat = i2c_transmit_addr((uint8_t)BMP_WRITE_ADDR)))
    {
        i2c_stop();
        usart_stat('A', stat);
    }
    else
    {
        // Send the register address w/in device
        // This completes the request for what is to be received next.
        stat = i2c_send(regaddr);
        i2c_stop();
        if (stat != 0)
        {
            usart_stat('R', stat);
        }
        else
        {
            // Receipt includes sending a "read" address
            if (0 != (stat = i2c_transmit_addr((uint8_t)BMP_READ_ADDR)))
            {
                i2c_stop();
                usart_stat('B', stat); // B = read addr
            }
            else
            {
                // Successfully sent reg addr. Expect to read 24-bit int
                val = i2c_rcv_u24();
                i2c_stop();
                if (val == 0)
                {
                    usart_stat('U', 1);

                }
            }
        }
    }
    return val;

}

static uint16_t bmp180_get_16bit_regval(uint8_t regaddr)
{
    uint16_t reg_val = 0;

    // Initial step: can I read just one register off the BMP180?
    // Send the address with write=0 high bit
    uint8_t stat = 0;
    if (0 != (stat = i2c_transmit_addr((uint8_t)BMP_WRITE_ADDR)))
    {
        i2c_stop();
        usart_stat('A', stat);
    }
    else
    {
        // Send the register address w/in device
        // This completes the request for what is to be received next.
        stat = i2c_send(regaddr);
        i2c_stop();
        if (stat != 0)
        {
            usart_stat('R', stat);
        }
        else
        {
            // Receipt includes sending a "read" address
            if (0 != (stat = i2c_transmit_addr((uint8_t)BMP_READ_ADDR)))
            {
                i2c_stop();
                usart_stat('B', stat); // B = read addr
            }
            else
            {
                // Successfully sent reg addr. Expect to read short
                reg_val = i2c_rcv_ushort();
                i2c_stop();
                if (reg_val == 0)
                {
                    usart_stat('U', 1);

                }
            }
        }
    }
    return reg_val;

}

