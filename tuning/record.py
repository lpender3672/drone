import serial
import numpy as np
import scipy.io as sio
import os, threading
import time, datetime

class SerialReader:
    def __init__(self, port, baud_rate, max_time=10, save_folder=""):
        self.serial_port = serial.Serial(port, baud_rate)
        self.serial_port.flushInput()

        self.data = np.zeros((0, 7)) # time, ax, ay, az, gx, gy, gz

        self.max_time = max_time
        self.save_folder = save_folder

        self.save_thread = save_thread(self.data)

    def run(self):
        ## write to .mat file
        # readline
        self.start_time = time.time()
        while True:
            if self.serial_port.in_waiting:
                data = self.serial_port.readline().decode().strip()
                try:
                    data = np.array(data.split(" "), dtype=float)
                    self.data = np.append(self.data, [data], axis=0)
                except (ValueError, IndexError):
                    print(datetime.datetime.now(), " Error in reading data; ", data)
                    continue
                else:
                    now = time.time()
                    if now - self.start_time > self.max_time:
                        self.save_data()
                        break
        
            elif (self.data.shape[0] > 0 and self.data.shape[0] % 60000 == 0):
                # save data every 10 minutes
                self.save_data()

    
    def save_data(self, name="6ax_raw.mat"):
        # try to load the data if it exists
        file = os.path.join(self.save_folder, name)
        # probably gets really slow if the file is too big so done in a separate thread
        self.save_thread = save_thread(self.data, file)
        self.save_thread.start()
        self.data = np.zeros((0, 7))


class save_thread(threading.Thread):
    def __init__(self, data_batch, file_path = ""):
        threading.Thread.__init__(self)
        self.data = data_batch
        self.file_path = file_path

    def run(self):

        try:
            old_data = sio.loadmat(self.file_path)["raw"]
            data = np.append(old_data, self.data, axis=0)
            del old_data
        except (FileNotFoundError, KeyError):
            data = self.data

        sio.savemat(self.file_path, {"raw": data})
        print(datetime.datetime.now(), "Data saved to ", self.file_path)
        self.data = np.zeros((0, 7))

        del data


if __name__ == "__main__":
    reader = SerialReader("COM5", 
                          115200, 
                          max_time=10 * 3600, 
                          save_folder="tuning\AV Matlab_SW")
    
    try:
        reader.run()
    except KeyboardInterrupt:
        reader.save_data()
        reader.save_thread.join()
        print("Data saved before exiting")
    finally:
        reader.serial_port.close()
