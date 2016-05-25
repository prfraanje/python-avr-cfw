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
from pylab import ion
#from seaborn import *

PORT     = "/dev/ttyACM0"
BAUDRATE = 115200
TIMEOUT  = None

class Cfw(object):
    def __init__(self,port=PORT,baudrate=BAUDRATE,timeout=TIMEOUT):
        ion() # make plots (monitor) interactive, running on background
        self.__serial_lock = 0 # 1 to lock serial interface
        self.__pause_monitor = 0 # 1 to pause monitor
        self.__serial = serial.Serial(port,baudrate,timeout=timeout)
        sleep(2) # wait a bit to start serial interface and give avr 
                 # some time to arrive at event loop
        self.__datasize = self.datasize
        self.__counter = self.counter
        self.__data_counter = self.data_counter
        self.__data_meas = self.data_meas
        self.__sensor = self.sensor
        self.__block_period = self.block_period
        self.__block_duration = self.block_duration
        self.__block_low = self.block_low
        self.__block_high = self.block_high
        self.__encoder = self.encoder
        self.__enc_time = self.enc_time
        self.__pwm = self.pwm;
        self.__motor_control = self.motor_control
        self.__reference = self.reference
        self.__error_sum = self.error_sum
        self.__status = self.status
        self.__Kp = self.Kp
        self.__Ki = self.Ki
        self.__Kd = self.Kd
        self.__offset = self.offset
        self.__error = self.error
        self.__pinb1 = 0
        self.__monitor = np.zeros((300,1))

        # First set up the figure, the axis, and the plot element we want to animate
        self.__fig = plt.figure('monitor')
        #self.__ax = plt.axes(xlim=(0, len(self.__monitor)-1), ylim=(0,2**16))
        self.__ax = plt.axes(xlim=(0, len(self.__monitor)-1), ylim=(0,2**10))
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
    def block_period(self):
        """load block_period"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x05\x00\x00')
        self.__block_period = struct.unpack("<2H",receive_and_decode_Z85(self.__serial,4))[1]
        self.__serial_lock = 0
        return self.__block_period

    @block_period.setter
    def block_period(self,val):
        command = b'\x00\x05'+struct.pack(">1H",val)
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,command)
        self.__serial_lock = 0

    @property
    def block_duration(self):
        """load block_duration"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x06\x00\x00')
        self.__block_duration = struct.unpack("<2H",receive_and_decode_Z85(self.__serial,4))[1]
        self.__serial_lock = 0
        return self.__block_duration

    @block_duration.setter
    def block_duration(self,val):
        command = b'\x00\x06'+struct.pack(">1H",val)
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,command)
        self.__serial_lock = 0

    @property
    def block_low(self):
        """load block_low"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x07\x00\x00')
        self.__block_low = struct.unpack("<4B",receive_and_decode_Z85(self.__serial,4))[3]
        self.__serial_lock = 0
        return self.__block_low

    @block_low.setter
    def block_low(self,val):
        command = b'\x00\x07\x00'+struct.pack("1B",val)
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,command)
        self.__serial_lock = 0

    @property
    def block_high(self):
        """load block_high"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x08\x00\x00')
        self.__block_high = struct.unpack("<4B",receive_and_decode_Z85(self.__serial,4))[3]
        self.__serial_lock = 0
        return self.__block_high

    @block_high.setter
    def block_high(self,val):
        command = b'\x00\x08\x00'+struct.pack("1B",val)
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,command)
        self.__serial_lock = 0

    @property
    def encoder(self):
        """load encoder increment"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x09\x00\x00')
        self.__encoder = struct.unpack("<2h",receive_and_decode_Z85(self.__serial,4))[1]
        self.__serial_lock = 0
        return self.__encoder

    @encoder.setter
    def encoder(self,val):
        command = b'\x00\x09'+struct.pack("<1h",val)
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,command)
        self.__serial_lock = 0

    @property
    def enc_time(self):
        """load encoder time"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x0a\x00\x00')
        self.__enc_time = struct.unpack("<2H",receive_and_decode_Z85(self.__serial,4))[1]
        self.__serial_lock = 0
        return self.__enc_time

    @property
    def pwm(self):
        """load pwm"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x0b\x00\x00')
        self.__pwm = struct.unpack("<4B",receive_and_decode_Z85(self.__serial,4))[3]
        self.__serial_lock = 0
        return self.__pwm

    # @pwm.setter
    # def pwm(self,val):
    #     command = b'\x00\x0b\x00'+struct.pack("1B",val)
    #     while(self.__serial_lock): sleep(0.001)
    #     self.__serial_lock = 1
    #     encode_Z85_and_transmit(self.__serial,command)
    #     self.__serial_lock = 0

    @property
    def motor_control(self):
        """load motor_control"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x0c\x00\x00')
        self.__motor_control = struct.unpack("<4B",receive_and_decode_Z85(self.__serial,4))[3]
        self.__serial_lock = 0
        return self.__motor_control

    @motor_control.setter
    def motor_control(self,val):
        command = b'\x00\x0c\x00'+struct.pack("1B",val)
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,command)
        self.__serial_lock = 0

    @property
    def reference(self):
        """load reference"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x0d\x00\x00')
        self.__reference = struct.unpack("<2H",receive_and_decode_Z85(self.__serial,4))[1]
        self.__serial_lock = 0
        return self.__reference

    @reference.setter
    def reference(self,val):
        # command = b'\x00\x0d'+struct.pack(">1H",val)
        command = b'\x00\x0d'+struct.pack(">1H",val)
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,command)
        self.__serial_lock = 0

    @property
    def error_sum(self):
        """load error_sum"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x0e\x00\x00')
        self.__error_sum = struct.unpack("<2h",receive_and_decode_Z85(self.__serial,4))[1]
        self.__serial_lock = 0
        return self.__error_sum

    @error_sum.setter
    def error_sum(self,val):
        command = b'\x00\x0e'+struct.pack(">1h",val)
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,command)
        self.__serial_lock = 0

    @property
    def status(self):
        """load status"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x0f\x00\x00')
        self.__status = struct.unpack("<4B",receive_and_decode_Z85(self.__serial,4))[3]
        self.__serial_lock = 0
        return self.__status

    @status.setter
    def status(self,val):
        command = b'\x00\x0f\x00'+struct.pack("1B",val)
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,command)
        self.__serial_lock = 0

    @property
    def Kp(self):
        """load Kp"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x10\x00\x00')
        self.__Kp = struct.unpack("<2h",receive_and_decode_Z85(self.__serial,4))[1]
        self.__serial_lock = 0
        return self.__Kp

    @Kp.setter
    def Kp(self,val):
        command = b'\x00\x10'+struct.pack("<1h",val)
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,command)
        self.__serial_lock = 0

    @property
    def Ki(self):
        """load Ki"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x11\x00\x00')
        self.__Ki = struct.unpack("<2h",receive_and_decode_Z85(self.__serial,4))[1]
        self.__serial_lock = 0
        return self.__Ki

    @Ki.setter
    def Ki(self,val):
        command = b'\x00\x11'+struct.pack("<1h",val)
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,command)
        self.__serial_lock = 0

    @property
    def Kd(self):
        """load Kd"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x12\x00\x00')
        self.__Kd = struct.unpack("<2h",receive_and_decode_Z85(self.__serial,4))[1]
        self.__serial_lock = 0
        return self.__Kd

    @Kd.setter
    def Kd(self,val):
        command = b'\x00\x12'+struct.pack("<1h",val)
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,command)
        self.__serial_lock = 0

    @property
    def error(self):
        """load error increment"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x13\x00\x00')
        self.__error = struct.unpack("<2h",receive_and_decode_Z85(self.__serial,4))[1]
        self.__serial_lock = 0
        return self.__error

    @property
    def offset(self):
        """load offset"""
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,b'\x01\x14\x00\x00')
        self.__offset = struct.unpack("<4B",receive_and_decode_Z85(self.__serial,4))[3]
        self.__serial_lock = 0
        return self.__offset

    @offset.setter
    def offset(self,val):
        command = b'\x00\x14\x00'+struct.pack("1B",val)
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,command)
        self.__serial_lock = 0

    @error.setter
    def error(self,val):
        command = b'\x00\x13'+struct.pack("<1h",val)
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        encode_Z85_and_transmit(self.__serial,command)
        self.__serial_lock = 0

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
            # self.__monitor = np.vstack((self.__monitor[1:],self.encoder))
            # self.__monitor = np.vstack((self.__monitor[1:],self.enc_time))
            self.__monitor = np.vstack((self.__monitor[1:],self.sensor))
            self.__line.set_ydata(self.__monitor)
        return self.__line,


    def close(self):
        while(self.__serial_lock): sleep(0.001)
        self.__serial_lock = 1
        self.__serial.close()
        plt.close(self.__fig)

