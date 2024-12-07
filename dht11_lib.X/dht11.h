/* 
 * File:   dht11.h
 * Author: Les Foster
 *
 * Created on December 6, 2024, 8:40 PM
 */

#ifndef DHT11_H
#define	DHT11_H

#define _XTAL_FREQ 32000000     //define crystal frequency to 32MHz
#define INSTR_FREQ  8000000     // Fosc / 4 is instruction clock frequency

#define DHT_BIT_LEN 40
#define MIN_ONE_CYCLES 50 * 8
#define MAX_DHT_START 95 * 8
#define MIN_DHT_START 75 * 8

#ifdef	__cplusplus
extern "C" {
#endif

struct dht11_reading {
  uint8_t tempInt;      // limited T range 0 - 50 C
  uint8_t tempDec;
  uint8_t humidityInt;
  uint8_t humidityDec;
  uint8_t cksumFlag;
};

void dht_setup_timer_capture(void);
void dht_timer_read_terminate(void);
void dht_init_read(void);
struct dht11_reading dht_read(void);
void dht11_handle_interrupt(void);

#ifdef	__cplusplus
}
#endif

#endif	/* DHT11_H */

