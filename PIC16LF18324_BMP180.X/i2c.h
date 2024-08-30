/* 
 * File:   i2c.h
 * Author: lesfo
 *
 * Created on August 29, 2024, 4:46 PM
 */

#ifndef I2C_H
#define	I2C_H
void i2c_as_controller(void);
uint8_t i2c_stop(void);
uint8_t i2c_transmit_addr(uint8_t periph_addr);
uint8_t i2c_send(uint8_t val);
uint8_t i2c_rcv_ubyte(uint8_t do_ack);
uint16_t i2c_rcv_ushort(void);
uint24_t i2c_rcv_u24(void);

#endif	/* I2C_H */

