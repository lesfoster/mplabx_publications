/* 
 * File:   usart.h
 * Author: lesfo
 *
 * Created on August 29, 2024, 4:23 PM
 */

#include <xc.h>
#ifndef USART_H
#define	USART_H

void uart_setup_tx(void);
void uart_disable_tx(void);
void uart_reenable_tx(void);
void usart_prep_signed_32_msg(int32_t val, const char *label);
void usart_xmit_signed_32_msg(void);
void usart_xmit_str(unsigned char *msg);
void usart_xmit_msg(unsigned char *msg, uint16_t msg_size);
void usart_stat(unsigned char statpt, uint8_t stat);

#endif	/* USART_H */

