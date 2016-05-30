# example commands (give these commands in your python shell):
from cfw import *

cfw = Cfw()

# set initial guess of controller gains
cfw.Kp = 50
cfw.Ki = 5
cfw.Kd = 0

# reset integrator:
cfw.error_sum = 0

# turn on motor:
cfw.motor_control = 1

# set CONTROL AND RECORD bits in status:
cfw.status = 3

# set reference
cfw.reference = 200

# aks for monitor with
# monitor_data = cfw.monitor

# ask for real-time sampled data with
# data_meas = cfw.data_meas

# close down with
# cfw.close()

