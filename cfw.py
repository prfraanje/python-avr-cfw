""" 
Control FrameWork (CFW)

"""

from __future__ import absolute_import, division, print_function

import serial
from serial_z85 import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from time import sleep
import struct
#from seaborn import *

PORT     = "/dev/ttyACM0"
BAUDRATE = 115200
TIMEOUT  = None

class Cfw(object):
    def __init__(self,port=PORT,baudrate=BAUDRATE,timeout=TIMEOUT):
        self.__serial_lock = 0 # 1 to lock serial interface
        self.__pause_monitor = 0 # 1 to pause monitor
        self.__serial = serial.Serial(port,baudrate,timeout=timeout)
        sleep(1) # wait a bit to start serial interface and give avr 
                 # some time to arrive at event loop
        self.__datasize = self.datasize
        self.__counter = self.counter
        self.__data_counter = self.data_counter
        self.__data_meas = self.data_meas
        self.__sensor = self.sensor
        self.__pinb1 = 0
        self.__monitor = np.zeros((300,1))

        # First set up the figure, the axis, and the plot element we want to animate
        self.__fig = plt.figure('monitor')
        self.__ax = plt.axes(xlim=(0, len(self.__monitor)-1), ylim=(0, 2**10))
        self.__ax.grid(True)
        self.__line, = self.__ax.plot(self.__monitor, lw=2)

        # call the animator.  blit=True means only re-draw the parts that have changed.
        self.__anim = animation.FuncAnimation(self.__fig, self.__animate,
                init_func=self.__init_graph, interval=20,
                repeat=True,blit=False)
        plt.show()

    @property
    def serial_lock(self):
        return self.__serial_lock

    @property
    def datasize(self):
        """load datasize"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x00\x00\x00')
        self.__datasize = struct.unpack("<2H",receive_and_decode_Z85(self.__serial,4))[1]
        self.__serial_lock = 0
        return self.__datasize

    @property
    def counter(self):
        """load counter"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x01\x00\x00')
        self.__counter = struct.unpack("<2H",receive_and_decode_Z85(self.__serial,4))[1]
        self.__serial_lock = 0
        return self.__counter

    @counter.setter
    def counter(self,val):
        command = b'\x00\x01'+struct.pack(">1H",val)
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,command)
        self.__serial_lock = 0

    @property
    def data_counter(self):
        """load data_counter"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x02\x00\x00')
        self.__data_counter = struct.unpack("<2H",receive_and_decode_Z85(self.__serial,4))[1]
        self.__serial_lock = 0
        return self.__data_counter

    @data_counter.setter
    def data_counter(self,val):
        command = b'\x00\x02'+struct.pack(">1H",val)
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,command)
        self.__serial_lock = 0

    @property
    def data_meas(self):
        """load measured data"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x03\x00\x00')
        self.__data_meas = np.fromstring(receive_and_decode_Z85(self.__serial,2*self.__datasize),dtype=np.uint16)
        self.__data_counter = struct.unpack("<2H",receive_and_decode_Z85(self.__serial,4))[1]-1
        self.__serial_lock = 0
        if self.__data_counter < 0:
            self.__data_counter += self.__datasize
        self.__data_meas = np.roll(self.__data_meas,-1-self.__data_counter)
        return self.__data_meas

    @property
    def sensor(self):
        """load sensor"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x04\x00\x00')
        self.__sensor = struct.unpack("<2H",receive_and_decode_Z85(self.__serial,4))[1]
        self.__serial_lock = 0
        return self.__sensor


    @property
    def pinb1(self):
        return self.__pinb1

    @pinb1.setter
    def pinb1(self,val):
        self.__pinb1=val
        command = b'\x00\x03'+struct.pack(">1H",val)
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,command)
        self.__serial_lock = 0

    @property
    def pause_monitor(self):
        return self.__pause_monitor
    @pause_monitor.setter
    def pause_monitor(self,val):
        if (val == 1):
            self.__pause_monitor=1
        else:
            self.__pause_monitor=0

    @property
    def serial(self):
        return self.__serial

    

    def __init_graph(self):
        # self.__line.set_ydata(self.__monitor)
        return self.__line,

    def __animate(self,i):
        if (self.pause_monitor == 0):
            self.__monitor = np.vstack((self.__monitor[1:],self.sensor))
            self.__line.set_ydata(self.__monitor)
        return self.__line,


    def close(self):
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        self.__serial.close()
        plt.close(self.__fig)

