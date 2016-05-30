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

// for atmega2560 2000 is still possible
/* #define DATASIZE 1024 */
// #define DATASIZE 2000 
// for atmega328 512 is good choice
#define DATASIZE 512 

// ENCODER status bits
#define ENC_DIR 0
#define ENC_OFLW_INCR 1
#define ENC_UFLW_INCR 2
#define ENC_OFLW_TIME 3

// timer0 (8-bit) for interrupt generation at about 1kHz
// timer1 (16-bit) for encoder reading
// timer2 (8-bit) for PWM control signal

uint16_t readADC0(void);

struct block {
    uint16_t period;     // in number of samples
    uint16_t duration;   // number of samples signal is high
    uint8_t low;        // low value
    uint8_t high;       // high value
    uint16_t counter;    // current sample value
    uint8_t value;       // current output value
}; 

struct PID_controller {
    int16_t error;     // error signal 
    int16_t error_sum; // running sum of the error signal, needed for integral action 
    int16_t error_prev; // previous error, needed for differential action

    /* int32_t control_p;   // control signal */
    /* int32_t control_i;   // control signal */
    /* int32_t control_d;   // control signal */
    uint8_t control;   // control signal
    uint8_t offset;   // offset for control signal

    int16_t Kp;      // proportional gain
    int16_t Ki;      // integral gain
    int16_t Kd;      // differential gain

}; 

struct encoder {
    int16_t increment;      // in number of increments/decrements 
    uint16_t time;      // time (in ticks/interrupts) between to ticks/interrupts
    uint8_t status;     // status byte with information on direction, over-/underflow of angle
    // status:
    //  bit0 (ENC_DIR):       0: clockwise,              1: counterclockwise
    //  bit1 (ENC_OFLW_INCR): 0: no overflow increment,  1: overflow increment 
    //  bit2 (ENC_OFLW_UNCR): 0: no underflow increment, 1: underflow increment 
    //  bit3 (ENC_OFLW_TIME): 0: no overflow time,       1: overflow time 
};

void block_wave(void);

// global variables
volatile uint16_t reference;
volatile uint16_t sensor;
volatile uint16_t control;
/* volatile int16_t data_meas[DATASIZE]; */
volatile uint16_t data_meas[DATASIZE];
volatile uint16_t data_counter = 0x0000;
volatile uint16_t counter = 0x0000;
volatile uint8_t status = 0x00; // status bits

volatile struct block block0 = {2000, 1000, 85, 170, 0}; 
volatile struct encoder encoder0 = {0, 0, 0}; 
/* volatile struct PID_controller PID0 = {0, 0, 0, 0,0,0,0, 0, 0, 0, 0}; */ 
volatile struct PID_controller PID0 = {0, 0, 0, 0, 0, 0, 0, 0}; 

volatile uint8_t motor_control = 0x00;


// timer0 (8-bit)
void initSampleInterrupt(void) {
    TCCR0A |= (1 << WGM01);   /* CTC mode (clear timer after match) */
    TCCR0B |= (1 << CS02) ; /* scale clock to 16MHz/256 = 62.5kHz */
    OCR0A = 0x3f;           /* set count value most significant byte 0x3f = 63, makes about 1kHz*/ 

    TIMSK0 |= (1 << OCIE0A); /* enable interrupt on COMPA of timer 1 */ 

}

// timer1 (16-bit) for measuring time between encoder interrupts
static inline void initTimer1(void) {
    TCCR1B |= (1 << CS12); // normal mode, just counting
    // clock speed 16MHz / 256, each tick is 16 micro seconds, 
    // max time between ticks is: 2**16 * 16 micro seconds = 1.05 seconds

    // set overflow interrupt enable
    TIMSK1 |= (1 << TOIE1);

    TCNT1 = 0; 
}

// encoder external interrupts on INT0 and INT1:
void initEncoderInterrupt(void) {
    // any logical change on INT0 or INT1 will cause an interrupt 
    EICRA |= (1 << ISC10) | (1 << ISC00); 
    /* // falling edge on INT0 or INT1 will cause an interrupt */ 
    /* EICRA |= (1 << ISC11) | (1 << ISC01); */ 
    // enable both INT0 and INT0 to give an interrupt 
    EIMSK |= (1 << INT1) | (1 << INT0);

    // set pin INT0 = PD0 and INT1 = PD1 to input
    DDRD &= ~((1 << PD0) | (1 << PD1));

    // set pin INT0 = PD0 and INT1 = PD1 to pullup 
    PORTD |= (1 << PD0) | (1 << PD1);

}

// timer2 (8-bit)
void initPWM(void) {
    TCCR2A |= (1 << WGM21) | (1 << WGM20);   /* fast PWM mode */
    TCCR2A |= (1 << COM2A1) ; /* clear output on compare */  
    //TCCR2B |= (1 << CS20);   /* no prescaling, run as fast as possible */
    //TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);   /* factor 1024 prescaling */
    // following turns out to be good on a small size DC motor with L298N H-bridge
    //TCCR2B |= (1 << CS22) | (1 << CS20);   /* factor 128 prescaling */
    TCCR2B |= (1 << CS22) | (1 << CS21);   /* factor 256 prescaling */
    //TCCR2B |= (1 << CS22);   /* factor 64 prescaling */
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


void block_wave(void) {
    if (block0.counter<block0.duration) {
        OCR2A = block0.high;          
        block0.value = block0.high;
        }
    else {
        OCR2A = block0.low;
        block0.value = block0.low;
        }
    if (block0.counter>=block0.period-1) {
        block0.counter = 0;
        }
    else {
        block0.counter++;
        }
    }

void PID_calc(void) {
   PID0.error_prev = PID0.error;     // store previous error signal
   PID0.error = reference - sensor;  // calc current error
   PID0.error_sum += PID0.error;     // calc running sum of error
   if (PID0.error_sum > 8000) PID0.error_sum = 8000;
   if (PID0.error_sum < -8000) PID0.error_sum = -8000;

   PID0.control = (uint8_t) ( ( (int32_t)PID0.Kp*PID0.error  + (int32_t)PID0.Ki * PID0.error_sum +  (int32_t)PID0.Kd * (PID0.error - PID0.error_prev)) / 256 ); 
   PID0.control += PID0.offset;

   /* PID0.control_p = ( (int32_t)PID0.Kp*PID0.error / 256 ); */
   /* PID0.control_i = ( (int32_t)PID0.Ki * PID0.error_sum / 256); */
   /* PID0.control_d = ( (int32_t)PID0.Kd * (PID0.error - PID0.error_prev) / 256); */

   /* PID0.control=0; */
   /* if (PID0.control_p>0) { */
   /*      if  (PID0.control_p<255) { */
   /*              PID0.control += (uint8_t)PID0.control_p; */
   /*     } */
   /*      else { */
   /*         PID0.control = 0xff; */ 
   /*          } */
   /*     } */
   /* if (PID0.control+PID0.control_i>0) { */
   /*    if (PID0.control+PID0.control_i<255) { */
   /*              PID0.control += (uint8_t)PID0.control_i; */ 
   /*        } */ 
   /*     } */
   /* if (PID0.control+PID0.control_d>0) { */
   /*    if (PID0.control+PID0.control_d<255) { */
   /*              PID0.control += (uint8_t)PID0.control_d; */ 
   /*        } */ 
   /*     } */

    
    }


ISR(TIMER0_COMPA_vect) {
    /* PORTB ^= (1 << PB0); /1* toggle led on pin B0 *1/ */

    // read sensor
    sensor = readADC0();

    // calculate error signal (input to controller)
    
    // do control
    if bit_is_set(status,CONTROL) {
        /* do control */
        PID_calc();
        OCR2A = PID0.control; 
        }

    // other stuff
    /* block_wave(); */

    // record signal(s)
    if bit_is_set(status,RECORD) {
        /* data_meas[data_counter] = encoder0.increment; */
        /* data_meas[data_counter] = encoder0.time; */
        data_meas[data_counter] = sensor;
        data_counter++;
        if (data_counter>=DATASIZE) data_counter = 0;
        }

    counter++;
}

ISR(INT0_vect) {
    encoder0.time = TCNT1;
    encoder0.status &= ~(1 << ENC_OFLW_TIME); // clear time overflow in status
    initTimer1();
    if bit_is_set(PIND,PD0) { // rising edge on INT0
       if bit_is_set(PIND,PD1) { // INT1 is high; CCW (counter clockwise)
          encoder0.increment--; 
          encoder0.status |= (1 << ENC_DIR); 
       } 
       else { // INT1 is low; CW (clockwise) 
          encoder0.increment++; 
          encoder0.status &= ~(1 << ENC_DIR); 
       }
    }
    else { // falling edge on INT0
       if bit_is_set(PIND,PD1) { // INT1 is high; CW (clockwise)
          encoder0.increment++; 
          encoder0.status &= ~(1 << ENC_DIR); 
       } 
       else { // INT1 is low; CCW (counter clockwise) 
          encoder0.increment--; 
          encoder0.status |= (1 << ENC_DIR); 
       }
    }
}

ISR(INT1_vect) {
    encoder0.time = TCNT1;
    encoder0.status &= ~(1 << ENC_OFLW_TIME); // clear time overflow in status
    initTimer1();
    if bit_is_set(PIND,PD1) { // rising edge on INT1
       if bit_is_set(PIND,PD0) { // INT0 is high; CW (clockwise)
          encoder0.increment++; 
          encoder0.status &= ~(1 << ENC_DIR); 
       } 
       else { // INT0 is low; CCW (counter clockwise) 
          encoder0.increment--; 
          encoder0.status |= (1 << ENC_DIR); 
       }
    }
    else { // falling edge on INT1
       if bit_is_set(PIND,PD0) { // INT0 is high; CCW (counter clockwise)
          encoder0.increment--; 
          encoder0.status |= (1 << ENC_DIR); 
       } 
       else { // INT0 is low; CW (clockwise) 
          encoder0.increment++; 
          encoder0.status &= ~(1 << ENC_DIR); 
       }
    }
}

// overflow of timer1
ISR(TIMER1_OVF_vect) {
     encoder0.status |= (1 << ENC_OFLW_TIME); // set time overflow in status
     encoder0.time = 0xffff; 
    // stop counting:
    TCCR1B = 0x00;
}

/* ISR(INT0_vect) { // INT0 falling edge */
/*     if bit_is_set(PIND,PD1) { // INT1 is high; CW (clockwise) */
/*         encoder0.increment++; */ 
/*         } */ 
/*     else { // INT1 is low; CCW (counter clockwise) */ 
/*         encoder0.increment--; */ 
/*         } */
/*     } */

/* ISR(INT1_vect) { // INT1 falling edge */
/*     if bit_is_set(PIND,PD0) { // INT0 is high; CCW (counter clockwise) */
/*         encoder0.increment--; */ 
/*         } */ 
/*     else { // INT0 is low; CW (clockwise) */ 
/*         encoder0.increment++; */ 
/*         } */
/*     } */

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
 
    /* DDRB  |= (1 << PB0); /1* enable PB0 as output *1/ */ 
    /* DDRB  |= (1 << PB1); /1* enable PB1 as output *1/ */ 
    /* DDRB  |= (1 << PB7); /1* enable PB7 as output *1/ */ 
    DDRB |= (1 << PB4); // pwm output
    DDRB |= (1 << PB0); // input 1 for motor
    DDRB |= (1 << PB1); // input 2 for motor

    initSampleInterrupt();
    initTimer1();
    initEncoderInterrupt();
    initPWM();
    initADC0();
    initData();
    initUSART();

    sei();                   /* set (global) interrupt enable bit */

    /* PORTB &= ~(1 << PB0); /1* led off on pin B0 *1/ */
    /* PORTB &= ~(1 << PB1); /1* led off on pin B1 *1/ */
    
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
                    /* PORTB |= (1 << PB1); /1* led on on pin B0 *1/ */
                    encode_Z85_and_transmit((uint8_t *)data_meas,sizeof(data_meas)); 
                    status |= (1<<RECORD);
                    ((uint16_t*)response)[1] = data_counter;
                    encode_Z85_and_transmit(response,sizeof(response));
                    /* PORTB &= ~(1 << PB1); /1* led off on pin B0 *1/ */
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
                case 0x09: // get encoder increment 
                    ((int16_t*)response)[1] = encoder0.increment;
                    encode_Z85_and_transmit(response,sizeof(response));
                    break;
                case 0x0a: // get encoder time 
                    ((uint16_t*)response)[1] = encoder0.time;
                    encode_Z85_and_transmit(response,sizeof(response));
                    break;
                case 0x0b: // get control value 
                    response[3] = PID0.control;
                    encode_Z85_and_transmit(response,sizeof(response));
                    break;
                case 0x0c: // get motor control value 
                    response[3] = motor_control;
                    encode_Z85_and_transmit(response,sizeof(response));
                    break;
                case 0x0d: // get reference 
                    ((uint16_t*)response)[1] = reference;
                    encode_Z85_and_transmit(response,sizeof(response));
                    break;
                case 0x0e: // get sum error  
                    ((int16_t*)response)[1] = PID0.error_sum;
                    encode_Z85_and_transmit(response,sizeof(response));
                    break;
                case 0x0f: // get status value 
                    response[3] = status;
                    encode_Z85_and_transmit(response,sizeof(response));
                    break;
                case 0x10: // get Kp  
                    ((int16_t*)response)[1] = PID0.Kp;
                    encode_Z85_and_transmit(response,sizeof(response));
                    break;
                case 0x11: // get Ki  
                    ((int16_t*)response)[1] = PID0.Ki;
                    encode_Z85_and_transmit(response,sizeof(response));
                    break;
                case 0x12: // get Kd  
                    ((int16_t*)response)[1] = PID0.Kd;
                    encode_Z85_and_transmit(response,sizeof(response));
                    break;
                case 0x13: // get error increment 
                    ((int16_t*)response)[1] = PID0.error;
                    encode_Z85_and_transmit(response,sizeof(response));
                    break;
                case 0x14: // get offset value 
                    response[3] = PID0.offset;
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
                case 0x05: // get block period value
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
                case 0x09: // get encoder value
                    encoder0.increment = *((int16_t*)(command+2));
                    break;
                /* case 0x0b: // get OCR2A value */
                /*     OCR2A = command[3]; */
                /*     break; */
                case 0x0c: // get motor_control value
                    motor_control = command[3];
                    command[2] = OCR2A; // save current value of OCR2A in memory
                    OCR2A = 0; // set to 0, to make pwm 0
                    if bit_is_set(motor_control,0) {
                        PORTB |= (1 << PB0);
                        } 
                    else {
                        PORTB &= ~(1 << PB0); 
                        }
                    if bit_is_set(motor_control,1) {
                        PORTB |= (1 << PB1);
                        } 
                    else {
                        PORTB &= ~(1 << PB1); 
                        }
                    OCR2A = command[2]; // recall OCR2A
                    break;
                case 0x0d: // set reference value
                    reference = (command[2] << 8) + command[3];
                    break;
                case 0x0e: // set sum error  
                    PID0.error_sum = (command[2] << 8) + command[3];
                    break;
                case 0x0f: // get status value
                    status = command[3];
                    break;
                case 0x10: // set Kp   
                    /* PID0.Kp = (command[2]<<8) + command[3]; */
                    PID0.Kp = *((int16_t*)(command+2));
                    break;
                case 0x11: // set Ki   
                    /* PID0.Ki = (command[2]<<8) + command[3]; */
                    PID0.Ki = *((int16_t*)(command+2));
                    break;
                case 0x12: // set Kd   
                    /* PID0.Kd = (command[2]<<8) + command[3]; */
                    PID0.Kd = *((int16_t*)(command+2));
                    break;
                case 0x13: // get error value
                    PID0.error = *((int16_t*)(command+2));
                    break;
                case 0x14: // get offset value
                    PID0.offset = command[3];
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
