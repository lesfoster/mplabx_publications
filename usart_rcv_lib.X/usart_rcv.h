/* 
 * File:   usart_rcv.h
 * Author: Les Foster
 *
 * Created on December 6, 2024, 8:44 PM
 */

#ifndef USART_RCV_H
#define	USART_RCV_H

// Errors expected from receipt of data
#define RCV_FRAMERR 1
#define RCV_OVERRUN 2
#define RCV_BADCONTENT 3
#define RCV_LONGINPUT 4
#define RCV_DELUGE 5

#define RCV_MSG_LEN 80
#define RCV_OFLOW_MAX 75

#ifdef	__cplusplus
extern "C" {
#endif

void uart_setup_rx(void);
void uart_disable_rx(void);
void uart_reenable_rx(void);
void uart_setup_rx_polled(void);
uint8_t uart_poll_rx(unsigned char* buf, uint8_t bufsize);

#ifdef	__cplusplus
}
#endif

#endif	/* USART_RCV_H */

