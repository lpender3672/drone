
import math
from odrive.enums import *
import odrive
import fibre.libfibre
import time
import serial

import numpy as np

class lift_rig():

    def __init__(self, com_port):
        self.ser = serial.Serial(com_port, 115200, timeout=0.1)

    
    def get_reading(self):
        self.ser.write(b'read\n')
        # block until we get a response
        while True:
            line = self.ser.readline().decode('utf-8')
            if line:
                break
        
        number = line.split(' ')[1]
        return float(number)
    
    def calibrate(self):

        self.ser.write(b'calb\n')
        # block until we get a response
        while True:
            line = self.ser.readline().decode('utf-8')
            if line:
                return

    def exit(self):
        pass
        #self.ser.write(b'exit\n')

def reset_config():
    print("Erasing previous configuration")

    odrv0 = odrive.find_any()

    try:
        odrv0.erase_configuration()
    except fibre.libfibre.ObjectLostError:
        pass

    # wait for the odrive to reboot
    time.sleep(5)

    odrv0 = odrive.find_any()

    odrv = odrv0
    odrv.config.dc_bus_overvoltage_trip_level = 15
    odrv.config.dc_bus_undervoltage_trip_level = 10.5
    odrv.config.dc_max_positive_current = 10
    odrv.config.dc_max_negative_current = -math.inf
    odrv.config.brake_resistor0.enable = False
    odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
    odrv.axis0.config.motor.pole_pairs = 7
    odrv.axis0.config.motor.torque_constant = 0.0035956521739130432
    odrv.axis0.config.motor.current_soft_max = 10
    odrv.axis0.config.motor.current_hard_max = 15
    odrv.axis0.config.motor.calibration_current = 10
    odrv.axis0.config.motor.resistance_calib_max_voltage = 2
    odrv.axis0.config.calibration_lockin.current = 10
    odrv.axis0.motor.motor_thermistor.config.enabled = False

    odrv.axis0.config.torque_soft_min = -math.inf
    odrv.axis0.config.torque_soft_max = math.inf
    odrv.axis0.trap_traj.config.accel_limit = 5
    odrv.can.config.protocol = Protocol.NONE
    odrv.axis0.config.enable_watchdog = False
    odrv.config.enable_uart_a = False

    ## sensorless control

    odrv.axis0.controller.config.vel_limit = 358 # max vel at 12V for S1+this motor
    odrv.axis0.config.sensorless_ramp.vel = 200
    odrv.axis0.config.sensorless_ramp.accel = 20
    odrv.axis0.config.sensorless_ramp.current = 10
    odrv.axis0.controller.config.vel_gain = 0.001
    odrv.axis0.controller.config.vel_integrator_gain = 0.005
    odrv.axis0.config.load_encoder = EncoderId.SENSORLESS_ESTIMATOR
    odrv.axis0.config.commutation_encoder = EncoderId.SENSORLESS_ESTIMATOR

    odrv.axis0.config.sensorless_ramp.current = odrv.axis0.config.motor.current_soft_max

    # set  ramped velocity input mode
    #odrv0.axis0.controller.config.vel_ramp_rate = 0.5
    odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
    odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP

    print("Applying new configuration")

    try:
        odrv.save_configuration()
        odrv.reboot()
    except fibre.libfibre.ObjectLostError:
        pass

    # now wait for the odrive to reboot
    time.sleep(2)

    return 


#reset_config()

odrv0 = odrive.find_any()

# calibrate the motor
print("Calibrating motor and load cell")

device = lift_rig('COM3')

odrv0.axis0.requested_state = AxisState.MOTOR_CALIBRATION
time.sleep(0.1) # wait to start calibration
while odrv0.axis0.current_state != AxisState.IDLE: # wait for calibration to finish
    time.sleep(0.1)

device.calibrate()

time.sleep(2)

reading = device.get_reading()
print(f"Load cell 0 reading: {reading}")
assert np.isclose(reading, 0, atol=0.2), "Load cell not zeroed"

print("Performing ramp")
# set the velocity setpoint
odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL

target_speeds = np.linspace(10, 250, 10)

powers = np.zeros_like(target_speeds)
thrusts = np.zeros_like(target_speeds)

i = 0

for speed in target_speeds:
    print("Setting speed to ", speed)
    odrv0.axis0.controller.input_vel = speed

    time.sleep(10) # wait to reach speed

    # power readings
    power = 0
    for j in range(5):
        power += odrv0.ibus * odrv0.vbus_voltage
        time.sleep(0.1)
    power /= 5

    # thrust readings
    reading = device.get_reading()

    print(f"Power: {power}, Reading: {reading}")
    
    powers[i] = power # W
    thrusts[i] = reading # N
    
    time.sleep(2)
    i += 1

odrv0.axis0.requested_state = AxisState.IDLE

odrive.utils.dump_errors(odrv0, True)

# disp graph
import matplotlib.pyplot as plt
import pandas as pd

df = pd.DataFrame({'Power (W)': powers, 'Thrust (N)': thrusts, 'Speed (RPM)': target_speeds * 60})
df.to_csv('loading/bi-blade.csv', index=False)

plt.plot(powers, thrusts, '-o', label="Power")
plt.show()