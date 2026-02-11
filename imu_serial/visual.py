import tkinter as tk
from tkinter import ttk
import serial
import json
import threading
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# Global variables
accX_data, accY_data, accZ_data = [], [], []
time_data = []

class IMUGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("MPU9250 Real-Time Viewer")
        self.running = True

        # Serial connection
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

        frame = ttk.Frame(root)
        frame.pack(pady=10)

        self.roll_var = tk.StringVar()
        self.pitch_var = tk.StringVar()
        self.yaw_var = tk.StringVar()

        ttk.Label(frame, text="Roll:").grid(row=0, column=0, sticky="w")
        ttk.Label(frame, textvariable=self.roll_var).grid(row=0, column=1, sticky="w")

        ttk.Label(frame, text="Pitch:").grid(row=1, column=0, sticky="w")
        ttk.Label(frame, textvariable=self.pitch_var).grid(row=1, column=1, sticky="w")

        ttk.Label(frame, text="Yaw:").grid(row=2, column=0, sticky="w")
        ttk.Label(frame, textvariable=self.yaw_var).grid(row=2, column=1, sticky="w")

        # Setup matplotlib figure
        self.fig, self.ax = plt.subplots(figsize=(5, 3))
        self.ax.set_title("Akselerasi Global")
        self.ax.set_ylim(-20, 20)
        self.ax.set_xlabel("Waktu")
        self.ax.set_ylabel("m/s²")
        self.line_x, = self.ax.plot([], [], label="X")
        self.line_y, = self.ax.plot([], [], label="Y")
        self.line_z, = self.ax.plot([], [], label="Z")
        self.ax.legend(loc="upper right")

        canvas = FigureCanvasTkAgg(self.fig, master=root)
        canvas.get_tk_widget().pack()
        self.canvas = canvas

        # Start serial thread
        self.thread = threading.Thread(target=self.read_serial)
        self.thread.daemon = True
        self.thread.start()

        # Update loop
        self.update_plot()

    def read_serial(self):
        while self.running:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if not line:
                    continue
                data = json.loads(line)
                self.roll_var.set(f"{data['roll']:.2f}°")
                self.pitch_var.set(f"{data['pitch']:.2f}°")
                self.yaw_var.set(f"{data['yaw']:.2f}°")

                accX = data["accX"]
                accY = data["accY"]
                accZ = data["accZ"]

                timestamp = time.time()

                accX_data.append(accX)
                accY_data.append(accY)
                accZ_data.append(accZ)
                time_data.append(timestamp - time_data[0] if time_data else 0)

               #  if len(time_data) > 200:
               #      accX_data.pop(0)
               #      accY_data.pop(0)
               #      accZ_data.pop(0)
               #      time_data.pop(0)

            except (json.JSONDecodeError, UnicodeDecodeError):
                continue

    def update_plot(self):
        self.line_x.set_data(time_data, accX_data)
        self.line_y.set_data(time_data, accY_data)
        self.line_z.set_data(time_data, accZ_data)

        if time_data:
            self.ax.set_xlim(max(0, time_data[-1] - 10), time_data[-1])

        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw()
        self.root.after(100, self.update_plot)

    def on_close(self):
        self.running = False
        self.ser.close()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = IMUGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
