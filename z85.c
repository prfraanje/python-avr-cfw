//  --------------------------------------------------------------------------
//  Reference implementation for rfc.zeromq.org/spec:32/Z85
//
//  This implementation provides a Z85 codec as an easy-to-reuse C class 
//  designed to be easy to port into other languages.

//  --------------------------------------------------------------------------
//  Copyright (c) 2010-2013 iMatix Corporation and Contributors
//  
//  Permission is hereby granted, free of charge, to any person obtaining a 
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation 
//  the rights to use, copy, modify, merge, publish, distribute, sublicense, 
//  and/or sell copies of the Software, and to permit persons to whom the 
//  Software is furnished to do so, subject to the following conditions:
//  
//  The above copyright notice and this permission notice shall be included in 
//  all copies or substantial portions of the Software.
//  
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
//  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
//  DEALINGS IN THE SOFTWARE.
//  --------------------------------------------------------------------------

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include <avr/io.h>

#include "myutils.h"

//  Basic language taken from CZMQ's prelude
/* typedef uint8_t byte; */
/* typedef uint32_t uint; */
#define streq(s1,s2) (!strcmp ((s1), (s2)))

//  Maps base 256 to base 85
static char encoder [85 + 1] = {
    "0123456789" 
    "abcdefghij" 
    "klmnopqrst" 
    "uvwxyzABCD"
    "EFGHIJKLMN" 
    "OPQRSTUVWX" 
    "YZ.-:+=^!/" 
    "*?&<>()[]{" 
    "}@%$#"
};


//  Maps base 85 to base 256
//  We chop off lower 32 and higher 128 ranges
static uint8_t decoder [96] = {
    0x00, 0x44, 0x00, 0x54, 0x53, 0x52, 0x48, 0x00, 
    0x4B, 0x4C, 0x46, 0x41, 0x00, 0x3F, 0x3E, 0x45, 
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 
    0x08, 0x09, 0x40, 0x00, 0x49, 0x42, 0x4A, 0x47, 
    0x51, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 
    0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 
    0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 
    0x3B, 0x3C, 0x3D, 0x4D, 0x00, 0x4E, 0x43, 0x00, 
    0x00, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 
    0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 
    0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 
    0x21, 0x22, 0x23, 0x4F, 0x00, 0x50, 0x00, 0x00
};

//  --------------------------------------------------------------------------
//  Encode a byte array as a string

/* Encodes the byte array data to z85 and transmits on serial interface */
uint8_t encode_Z85_and_transmit(uint8_t *data, size_t size)
{
    //  Accepts only byte arrays bounded to 4 bytes
    if (size % 4)
        return 1;
    
    /* size_t encoded_size = size * 5 / 4; */
    /* char *encoded = malloc (encoded_size + 1); */
    /* uint char_nbr = 0; */
    size_t byte_nbr = 0;
    uint32_t value = 0;
    while (byte_nbr < size) {
        //  Accumulate value in base 256 (binary)
        value = value * 256 + data [byte_nbr++];
        if (byte_nbr % 4 == 0) {
            //  Output value in base 85
            uint32_t divisor = 52200625; // = 85 * 85 * 85 * 85;
            while (divisor) {
                /* encoded [char_nbr++] = encoder [value / divisor % 85]; */
                transmitByte(encoder [value / divisor % 85]); // write to USART
                divisor /= 85;
            }
            value = 0;
        }
    }
    /* assert (char_nbr == encoded_size); */
    /* encoded [char_nbr] = 0x00; */
    /* return encoded; */
    /* transmitByte(0x00); */
    return 0;
}

    
//  --------------------------------------------------------------------------
//  Decode an encoded string into a byte array; size of array will be
//  strlen (string) * 4 / 5.

/* Reads a z85 encoded string from serial interface and decodes the string into byte array of size size */ 
uint8_t receive_and_decode_Z85(uint8_t *data, size_t size)
{
    //  Accepts only strings bounded to 5 bytes
    /* if (strlen (string) % 5) */
    /*     return NULL; */
    
    /* size_t decoded_size = strlen (string) * 4 / 5; */
    /* byte *decoded = malloc (decoded_size); */
    if (size % 4) // only multiples of 4 bytes are possible
        return 1;
    size_t size_encoded = size*5/4;
    
    size_t byte_nbr = 0;
    size_t char_nbr = 0;
    uint32_t value = 0;
    while (char_nbr < size_encoded) {
        //  Accumulate value in base 85
        /* value = value * 85 + decoder [(byte) string [char_nbr++] - 32]; */
        value = value * 85 + decoder [receiveByte() - 32];
        char_nbr++;
        if (char_nbr % 5 == 0) {
            //  Output value in base 256
            uint32_t divisor = 16777216; // = 256 * 256 * 256;
            while (divisor) {
                data [byte_nbr++] = value / divisor % 256;
                divisor /= 256;
            }
            value = 0;
        }
    }

    if (byte_nbr != size) return 1;

    return 0;
}

