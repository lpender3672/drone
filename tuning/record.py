import serial
import time
import numpy as np
import scipy.io as sio

class SerialReader:
    def __init__(self, port, baud_rate, max_time=10, save_folder=""):
        self.serial_port = serial.Serial(port, baud_rate)
        self.serial_port.flushInput()

        self.data = np.zeros((0, 7))
        self.max_time = max_time
        self.save_folder = save_folder

    def run(self):
        ## write to .mat file
        # readline
        while True:
            if self.serial_port.in_waiting:
                data = self.serial_port.readline().decode().strip()
                try:
                    data = np.array(data.split(" "), dtype=float)
                    self.data = np.append(self.data, [data], axis=0)
                except (ValueError, IndexError):
                    print("Error in reading data point")
                    continue
                else :
                    if (self.data[-1,0] - self.data[0,0]) > self.max_time:
                        sio.savemat(self.save_folder + '6ax_raw.mat', {'raw': self.data})
                        break


if __name__ == "__main__":
    reader = SerialReader("COM5", 
                          115200, 
                          max_time=600, 
                          save_folder="tuning/AV Matlab_SW/")
    reader.run()
                
