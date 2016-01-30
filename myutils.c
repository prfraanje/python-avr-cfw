#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/setbaud.h>

void transmitByte(uint8_t data) {
    loop_until_bit_is_set(UCSR0A,UDRE0); /* wait for empty transmit buffer */
    UDR0 = data;
}

uint8_t receiveByte(void) {
    loop_until_bit_is_set(UCSR0A,RXC0); /* wait for incoming data */
    return UDR0; /* return register value */
}

void transmitNumber(uint8_t byte) {
    transmitByte('0' + (byte / 100));
    transmitByte('0' + (byte / 10) % 10);
    transmitByte('0' + (byte % 10));
}

void transmitString(const char myString[]) {
    uint8_t i = 0;
    while (myString[i]) {
        transmitByte(myString[i]);
        i++;
    }
}

