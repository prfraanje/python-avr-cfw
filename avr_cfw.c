#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <stdlib.h>
#include <stdbool.h>
#include "myutils.h"
#include "z85.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// status bits:
#define RECORD 0
#define CONTROL 1

// command bits:
#define DATA_REQUEST 0
#define PACKET_TWO 1
/* #define DATASIZE 1024 */
#define DATASIZE 2000 


uint16_t readADC0(void);

struct block {
    //port not necessary anymore (?)
    volatile uint8_t *port; // port
    uint8_t  pin;        // pin number in port
    uint16_t period;     // in number of samples
    uint16_t duration;   // number of samples signal is high
    uint8_t low;        // low value
    uint8_t high;       // high value
    uint16_t counter;    // current sample value
    uint8_t value;       // current output value
    }; 


void block_wave(struct block *p);



volatile uint16_t sensor;
volatile uint16_t control;
volatile uint16_t data_meas[DATASIZE];
volatile uint16_t data_counter = 0x0000;
volatile uint16_t counter = 0x0000;
volatile uint8_t status = 0x00; // status bits

struct block block0 = {&PORTB, PB1, 2000, 1000, 85, 170, 0}; 


ISR(TIMER1_COMPA_vect) {
    PORTB ^= (1 << PB0); /* toggle led on pin B0 */

    if bit_is_set(status,CONTROL) {
        /* do control */

        }

    sensor = readADC0();

    block_wave(&block0);

    if bit_is_set(status,RECORD) {
        data_meas[data_counter] = sensor;
        data_counter++;
        if (data_counter>=DATASIZE) data_counter = 0;
        }

    counter++;
}


void initTimers(void) {
    TCCR0A |= (1 << WGM01) | (1 << WGM00);   /* fast PWM mode */
    TCCR0A |= (1 << COM0A1) ; /* clear output on compare */  
    TCCR0B |= (1 << CS00);   /* no prescaling, run as fast as possible */
    //TCCR0B |= (1 << CS01);   /* factor 8 prescaling */
    }

void initInterrupt1(void) {
    TCCR1B |= (1 << WGM12);   /* CTC mode (clear timer after match) */
    TCCR1B |= (1 << CS12) | (1 << CS10); /* scale clock to 16MHz/1024 = 16kHz */
//    TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10); /* scale clock to 16MHz/64 = = 250kHz */
    /* OCR1AH = 0x40;           /1* set count value most significant byte *1/ */ 
    OCR1AH = 0x00;           /* set count value most significant byte */ 
    OCR1AL = 0x0f;           /* set count value most least byte */ 

    TIMSK1 |= (1 << OCIE1A); /* enable interrupt on COMPA of timer 1 */ 

    sei();                   /* set (global) interrupt enable bit */
}

void initADC0(void) {
    ADMUX |= (1 << REFS0); /* reference voltage on AVCC */
    ADMUX &= ~(1<<ADLAR); /* right adjust */
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); /* ADC clock prescaler: 16Mhz/128 = 125kHz */
    ADCSRA |= (1 << ADEN); /* enable ADC */
    }

uint16_t readADC0(void) {
    ADCSRA |= (1 << ADSC);  /* start ADC conversion */
    loop_until_bit_is_clear(ADCSRA,ADSC);  /* wait until done */
    return ADC;  /* read ADC in */
    }

void initData(void) {
    counter = 0;
    status  |= (1 << RECORD) ;
}

void initUSART(void) {
    UBRR0H = UBRRH_VALUE; /* requires BAUD defined in e.g. makefile */
    UBRR0L = UBRRL_VALUE; /* setbaud.h needed */
#if USE_2X
    UCSR0A |= (1 << U2X0);
#else
    UCSR0A &= ~(1 << U2X0);
#endif

    UCSR0B = (1 << TXEN0) | (1 << RXEN0);   /* enable USART trans/rec. */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); /* 8 data bits, 1 stop bit */

    }


void block_wave(struct block *p) {
    if (p->counter<p->duration) {
//        *(p->port) |= (1 << p->pin);
        OCR0A = p->high;          
        p->value = p->high;
        }
    else {
//        *(p->port) &= ~(1 << p->pin);
        OCR0A = p->low;
        p->value = p->low;
        }
    if (p->counter>=p->period-1) {
        p->counter = 0;
        }
    else {
        p->counter++;
        }
    }

/* Protocol being used
Because Z85 encoding is used for transferring data on serial bus
the data packets being send and received must be multiples of 4 bytes.

Except for (large) arrays of data most signals, commands, variable sets and gets
can be send just within 4 bytes. 

Command register
B0       B1       B2       B3
+------- +--------+--------+--------+
|........|........|........|........|
+--------+--------+--------+--------+

Response register
B0       B1       B2       B3
+------- +--------+--------+--------+
|........|........|........|........|
+--------+--------+--------+--------+
 */

int main(void) {
    uint8_t command[4] = {0x00, 0x00, 0x00, 0x00};
    uint8_t response[4] = {0x00, 0x00, 0x00, 0x00};
 
    DDRB  |= (1 << PB0); /* enable PB0 as output */ 
    DDRB  |= (1 << PB1); /* enable PB1 as output */ 
    DDRB  |= (1 << PB7); /* enable PB7 as output */ 


    initInterrupt1();
    initTimers();
    initADC0();
    initData();
    initUSART();

    PORTB &= ~(1 << PB0); /* led off on pin B0 */
    PORTB &= ~(1 << PB1); /* led off on pin B1 */
    
    while (1) {
        receive_and_decode_Z85(command,sizeof(command));
        /* parse command */
        if(bit_is_set(command[0],DATA_REQUEST)) { // data request
            switch(command[1]) {
                case 0x00: // get datasize value
                    ((uint16_t*)response)[1] = (uint16_t)DATASIZE;
                    encode_Z85_and_transmit(response,sizeof(response));
                    break;
                case 0x01: // get counter value
                    ((uint16_t*)response)[1] = counter;
                    encode_Z85_and_transmit(response,sizeof(response));
                    break;
                case 0x02: // get data_counter value
                    ((uint16_t*)response)[1] = data_counter;
                    encode_Z85_and_transmit(response,sizeof(response));
                    break;
                case 0x03: // get data_meas
                    status &= ~(1<<RECORD); // don't allow to write new values during transmission
                    PORTB |= (1 << PB1); /* led on on pin B0 */
                    encode_Z85_and_transmit((uint8_t *)data_meas,sizeof(data_meas)); 
                    status |= (1<<RECORD);
                    ((uint16_t*)response)[1] = data_counter;
                    encode_Z85_and_transmit(response,sizeof(response));
                    PORTB &= ~(1 << PB1); /* led off on pin B0 */
                    break;
                case 0x04: // get last measured value
                    ((uint16_t*)response)[1] = sensor;
                    encode_Z85_and_transmit(response,sizeof(response));
                    break;
                case 0x05: // get period of block wave 
                    ((uint16_t*)response)[1] = block0.period;
                    encode_Z85_and_transmit(response,sizeof(response));
                    break;
                case 0x06: // get duration of block wave 
                    ((uint16_t*)response)[1] = block0.duration;
                    encode_Z85_and_transmit(response,sizeof(response));
                    break;
                case 0x07: // get low value of block wave 
                    response[3] = block0.low;
                    encode_Z85_and_transmit(response,sizeof(response));
                    break;
                case 0x08: // get duration of block wave 
                    response[3] = block0.high;
                    encode_Z85_and_transmit(response,sizeof(response));
                    break;
                default: // set response to zero, but don't receive anything 
                    response[0] = 0x00;
                    response[1] = 0x00;
                    response[2] = 0x00;
                    response[3] = 0x00;
                }
            }
        else { // data send
            switch(command[1]) {
                case 0x01: // set counter value
                    counter = (command[2] << 8) + command[3];
                    break;
                case 0x02: // get data_counter value
                    data_counter = (command[2] << 8) + command[3];
                    break;
                /* case 0x03: // set data_meas */
                /*     if bit_is_set(command[3],0){ */
                /*            PORTB |= (1 << PB1); /1* led on pin B1 *1/ */
                /*         } */
                /*     else { */
                /*            PORTB &= ~(1 << PB1); /1* led off pin B1 *1/ */
                /*         } */
                /*     break; */
                case 0x05: // get block perdiod value
                    block0.period = (command[2] << 8) + command[3];
                    break;
                case 0x06: // get block duration value
                    block0.duration = (command[2] << 8) + command[3];
                    break;
                case 0x07: // get block low value
                    block0.low =  command[3];
                    break;
                case 0x08: // get block high value
                    block0.high = command[3];
                    break;
                default: // set response to zero, but don't receive anything 
                    response[0] = 0x00;
                    response[1] = 0x00;
                    response[2] = 0x00;
                    response[3] = 0x00;
                }
            
            }
        }
    /* free(p); */
    return(0);
}
