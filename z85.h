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



/* Encodes the byte array data to z85 and transmits on serial interface */
uint8_t encode_Z85_and_transmit(uint8_t *data, size_t size);
    
//  --------------------------------------------------------------------------
//  Decode an encoded string into a byte array; size of array will be
//  strlen (string) * 4 / 5.

/* Reads a z85 encoded string from serial interface and decodes the string into byte array of size size */ 
uint8_t receive_and_decode_Z85(uint8_t *data, size_t size);
