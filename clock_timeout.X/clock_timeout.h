/* 
 * File:   clock_timeout.h
 * Author: Les Foster
 *
 * Created on December 6, 2024, 8:42 PM
 */

#ifndef CLOCK_TIMEOUT_H
#define	CLOCK_TIMEOUT_H

#ifdef	__cplusplus
extern "C" {
#endif

void clktmo_setup_delay_clock(void);
void clktmo_delay(uint16_t us);

#ifdef	__cplusplus
}
#endif

#endif	/* CLOCK_TIMEOUT_H */

