# utility to encode byte-arrays using Z85 encoding and
# transmit the resulting string over serial interface
# and vise verse

import z85
from time import sleep

def encode_Z85_and_transmit(ser,data):
    size = len(data)
    if (size % 4):
        raise ValueError("data should be multiple of 4 bytes in length.")

    z85bytes = z85.encode(data)

    ser.write(z85bytes)
    return 0
    

def receive_and_decode_Z85(ser,size):

    if (size % 4):
        raise ValueError("size should be multiple of 4.")

    size_encoded = size*5/4

    z85bytes = ser.read(size_encoded)
    # ser.reset_input_buffer()
    data = z85.decode(z85bytes)

    return data




