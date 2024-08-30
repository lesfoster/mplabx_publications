/*
 * File:   bmp180_to_usart.c
 * Author: lesfo
 *
 * Created on August 2, 2024, 6:23 PM
 */
#ifndef BMP_180_H_
#define BMP_180_H_
#define BMP_WRITE_ADDR 0xEE
#define BMP_READ_ADDR 0xEF

#define AC1_REG_ADDR 0xAA
#define AC2_REG_ADDR 0xAC
#define AC3_REG_ADDR 0xAE
#define AC4_REG_ADDR 0xB0
#define AC5_REG_ADDR 0xB2
#define AC6_REG_ADDR 0xB4
#define B1_REG_ADDR 0xB6
#define B2_REG_ADDR 0xB8
#define MB_REG_ADDR 0xBA
#define MC_REG_ADDR 0xBC
#define MD_REG_ADDR 0xBE

#define OSS 1

#include <xc.h>

const static uint8_t PRESSURE_REQ = (uint8_t) (0x34 + (OSS << 6));
const static uint8_t TEMP_REQ = 0x2E;


void bmp180_init(void);

int32_t bmp180_get_true_pressure(int32_t UP);
int32_t bmp180_get_true_temp(int16_t UT);

int24_t bmp180_get_raw_pressure(void);
int16_t bmp180_get_raw_temp(void);

void bmp180_set_sensor_req(uint8_t req);

//uint16_t bmp180_get_16bit_regval(uint8_t regaddr);
//void bmp180_set_sensor_req(uint8_t req);
//uint24_t bmp180_get_24bit_val(uint8_t regaddr);

#endif