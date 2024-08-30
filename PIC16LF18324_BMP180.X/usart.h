/* 
 * File:   usart.h
 * Author: lesfo
 *
 * Created on August 29, 2024, 4:23 PM
 */

#ifndef USART_H
#define	USART_H

void uart_setup_tx(void);
void usart_prep_signed_32_msg(int32_t val, const char *label);
void usart_transmit(void);
void usart_stat(unsigned char statpt, uint8_t stat);

#endif	/* USART_H */

