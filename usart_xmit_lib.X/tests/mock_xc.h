#ifndef MOCK_XC_H
#define MOCK_XC_H

#define uint8_t unsigned int

typedef struct {
    uint8_t ANSC4;
} ANSELCbits_t;
extern ANSELCbits_t ANSELCbits;

typedef struct {
    uint8_t TXIF;
} PIR1bits_t;
extern PIR1bits_t PIR1bits;

typedef struct {
    uint8_t SPEN;
    uint8_t TXEN;
    uint8_t SYNC;
    uint8_t BRGH;
} TX1STAbits_t;
extern TX1STAbits_t TX1STAbits;

typedef struct {
    uint8_t BRG16;
} BAUD1CONbits_t;
extern BAUD1CONbits_t BAUD1CONbits;

extern uint8_t SPBRGH;
extern uint8_t SPBRGL;
extern uint8_t TX1REG;
extern uint8_t RC4PPS;

#endif // MOCK_XC_H

