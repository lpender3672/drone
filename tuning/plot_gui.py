import sys
import serial
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt6.QtCore import QThread, pyqtSignal
import pyqtgraph as pg
import time


class SerialReader(QThread):
    data_received = pyqtSignal(float)

    def __init__(self, port, baud_rate):
        super().__init__()
        self.serial_port = serial.Serial(port, baud_rate)
        self.serial_port.flushInput()

    def run(self):
        while True:
            if self.serial_port.in_waiting:
                data = self.serial_port.readline().decode().strip().split(" ")
                type = data[0].strip(":")
                if not ("states" in type):
                    continue
            
                try:
                    print(data)
                    value = float(data[6])
                    self.data_received.emit(value)
                except (ValueError, IndexError):
                    pass
            
            self.msleep(1)
        

class SerialPlotter(QMainWindow):
    def __init__(self, port, baud_rate):
        super().__init__()
        self.setWindowTitle("Serial Plotter")
        self.setGeometry(100, 100, 800, 600)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w')
        self.plot_widget.setLabel('left', 'Value')
        self.plot_widget.setLabel('bottom', 'Time')
        self.plot_widget.showGrid(x=True, y=True)

        self.plot_data = self.plot_widget.plot(pen='b')

        layout = QVBoxLayout()
        layout.addWidget(self.plot_widget)

        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        self.x_data = []
        self.y_data = []
        self.start_time = time.time()

        self.serial_reader = SerialReader(port, baud_rate)
        self.serial_reader.data_received.connect(self.update_plot)
        self.serial_reader.start()

    def update_plot(self, value):
        x_time = time.time() - self.start_time
        self.x_data.append(x_time)
        self.y_data.append(value)
        
        if len(self.x_data) > 5000:
            self.plot_data.setData(self.x_data[-5000:], self.y_data[-5000:])
        else:
            self.plot_data.setData(self.x_data, self.y_data)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    plotter = SerialPlotter("COM5", 115200)  # Replace with your serial port and baud rate
    plotter.show()
    sys.exit(app.exec())