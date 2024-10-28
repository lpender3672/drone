

import math
from odrive.enums import *


import time
import serial

import numpy as np

class lift_rig():

    def __init__(self, com_port):
        self.ser = serial.Serial(com_port, 115200, timeout=0.1)

    
    def get_reading(self):
        self.ser.flush()
        self.ser.write(b'read\n')
        # block until we get a response
        while True:
            line = self.ser.readline().decode('utf-8')
            if line:
                break
        
        number = line.split(' ')[1]
        return float(number)
    
    def calibrate(self):
        self.ser.flush()
        self.ser.write(b'calb\n')
        # block until we get a response
        while True:
            line = self.ser.readline().decode('utf-8')
            if line:
                return
            
    def set_speed(self, speed):
        self.ser.flush()
        self.ser.write(f'speed {speed:.3f}\n'.encode('utf-8'))
        # block until we get a response
        while True:
            line = self.ser.readline().decode('utf-8')
            if line and "Setting speed" in line:
                return
            
    def arm_esc(self):
        self.ser.write(b'arm\n')
        # block until we get a response
        while True:
            line = self.ser.readline().decode('utf-8')
            if line:
                return

    def exit(self):
        pass
        #self.ser.write(b'exit\n')


print("Calibrating motor and load cell")

device = lift_rig('COM3')

device.calibrate()

time.sleep(2)

reading = device.get_reading()
print(f"Load cell 0 reading: {reading}")
assert np.isclose(reading, 0, atol=0.2), "Load cell not zeroed"

print("Performing ramp")

target_speeds = np.linspace(0, 0.5, 8)

target_speeds = np.concatenate([target_speeds, target_speeds[::-1]])
measured_speeds = np.zeros_like(target_speeds)

powers = np.zeros_like(target_speeds)
thrusts = np.zeros_like(target_speeds)
i = 0

for speed in target_speeds:
    print("Setting speed to ", speed)
    device.set_speed(speed)

    time.sleep(3) # wait to reach speed

    # power readings cannot be done with escs
    # thrust readings
    reading = device.get_reading()
    
    thrusts[i] = reading # N
    
    time.sleep(1)
    i += 1


# disp graph
import matplotlib.pyplot as plt
import pandas as pd

df = pd.DataFrame({'Power (W)': powers, 'Thrust (N)': thrusts, 'Throttle (%)': target_speeds})
df.to_csv('loading/data/tri-blade_esc.csv', index=False)

plt.plot(powers, thrusts, '-o', label="Power")
plt.show()