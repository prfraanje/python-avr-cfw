# Python - AVR Control FrameWork (python-avr-cfw)
=================================================

Framework for easy signal monitoring and getting / setting variables 
in an AVR program from Python through a serial interface using 
Z85 encoding.

## Goal
Provide a flexible, fast and easy to use data-acquisition and
control interface between controller boards like AVR microcontrollers and
Python.

## Usage
* Flash the file avr_cfw.c in the AVR microcontroller (hook up sensors /
  actuators and adjust the code if necessary)
```
make flash
```
* Start python (e.g. ipython), loa the module cfw and start an instance of Cfw:
```
ipython
from cfw import *
cfw = Cfw()
```
* You should see a Matplotlib figure showing you a running monitor of a sensor
  signal.
* Now you should be able to get and set for example the counter:
```
print(cfw.counter)
cfw.counter = 0
print(cfw.counter)
```
* You should also be able to get the latest sample of the sample and a numpy-array
  of the last DATASIZE samples:
```
print(cfw.sensor)
print(cfw.data_meas)
```

## To be done
* Add PID control
* Simplify the creation of 'shared' variables / datastructures on both the AVR
  and the Python side.
* Work out a nice example and improve documentation 
* Promote the code such that it may be of use to others, may get feedback, and
  may be of inspiration to others to augment, extend to other platforms,
  improve, etc.

## Acknowledgements
* [AVR-Programming](https://github.com/hexagon5un/AVR-Programming) by Elliot
Williams
* [PyZMQ (for Z85.py)](https://github.com/zeromq/pyzmq)

