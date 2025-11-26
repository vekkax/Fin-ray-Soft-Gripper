import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel

import serial
import sys
import re
from collections import deque
import time
import random
import csv
import atexit
import signal

# === CONFIGURATION ===
PORT = 'COM7'  # ← change this to match your COM port
BAUD = 115200
MAX_POINTS = 500
UPDATE_INTERVAL_MS = 50  # Update plot every 50ms (~20Hz)

# === REGEX PARSER ===
pattern = re.compile(
    r"fuerza sensada:\s*([-+]?[0-9.]+), set point:\s*([-+]?[0-9.]+), error:\s*([-+]?[0-9.]+), accion:\s*([0-9]+)"
)

# === DATA BUFFERS ===
peso = deque([0.0]*MAX_POINTS, maxlen=MAX_POINTS)
setpoint = deque([0.0]*MAX_POINTS, maxlen=MAX_POINTS)
error = deque([0.0]*MAX_POINTS, maxlen=MAX_POINTS)
accion = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

# === SERIAL SETUP ===
try:
    ser = serial.Serial(PORT, BAUD, timeout=0.01)
except Exception as e:
    print(f"[WARN] Could not open serial {PORT}: {e}")
    ser = None

def send_serial(cmd: bytes):
    """Send a single-byte command to the serial port (safe wrapper)."""
    try:
        if ser is not None and getattr(ser, "is_open", False):
            ser.write(cmd)
            print(f"[USB OUT] {cmd!r}")
        else:
            print("[WARN] Serial port not open")
    except Exception as e:
        print(f"[ERROR] Failed to write to serial: {e}")

# === PYQTGRAPH SETUP ===
app = QApplication([])
win = pg.GraphicsLayoutWidget(title="Real-Time Gripper Visualization")
win.setMinimumSize(800, 600)

# Plot 1: Peso & Set Point
p1 = win.addPlot(title="Force vs Set Point")
curve_peso = p1.plot(pen='b', name="Force")
curve_setp = p1.plot(pen='y', name="Set Point")
p1.enableAutoRange('y', True)

# Plot 2: Error
win.nextRow()
p2 = win.addPlot(title="Error")
curve_error = p2.plot(pen='r')
p2.enableAutoRange('y', True)

# Plot 3: Acción
win.nextRow()
p3 = win.addPlot(title="Control Action")
curve_accion = p3.plot(pen='g')
p3.enableAutoRange('y', True)

# --- Control panel (buttons that send 's', 'i', 'c') ---
control_panel = QWidget()
cp_layout = QVBoxLayout()
control_panel.setLayout(cp_layout)

lbl = QLabel(f"Serial: {PORT} @ {BAUD}")
cp_layout.addWidget(lbl)

btn_start = QPushButton("Start")
btn_idle = QPushButton("Idle")
btn_control = QPushButton("Control")

# connect buttons to send_serial with the requested characters
btn_start.clicked.connect(lambda: send_serial(b's'))
btn_idle.clicked.connect(lambda: send_serial(b'i'))
btn_control.clicked.connect(lambda: send_serial(b'c'))

cp_layout.addWidget(btn_start)
cp_layout.addWidget(btn_idle)
cp_layout.addWidget(btn_control)
cp_layout.addStretch()

# Put the plot widget and the control panel side by side in a main widget
main_widget = QWidget()
main_layout = QHBoxLayout()
main_widget.setLayout(main_layout)
main_layout.addWidget(win, stretch=3)
main_layout.addWidget(control_panel, stretch=1)

main_widget.resize(1000, 600)
main_widget.show()

# === UPDATE FUNCTION ===
def update():
    global peso, setpoint, error, accion

    while ser.in_waiting:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                print(f"[USB] {line}")

            match = pattern.search(line)
            if match:
                peso.append(float(match.group(1)))
                setpoint.append(float(match.group(2)))
                error.append(float(match.group(3)))
                accion.append(int(match.group(4)))
        except Exception as e:
            print(f"[ERROR] {e}")
            pass

    # Redraw plots every frame, even if values haven't changed
    x = list(range(MAX_POINTS))

    curve_peso.setData(x, list(peso), copy=True)
    curve_peso.setPos(0, random.uniform(0, 1e-10))

    curve_setp.setData(x, list(setpoint), copy=True)
    curve_setp.setPos(0, random.uniform(0, 1e-10))

    curve_error.setData(x, list(error), copy=True)
    curve_error.setPos(0, random.uniform(0, 1e-10))

    curve_accion.setData(x, list(accion), copy=True)
    curve_accion.setPos(0, random.uniform(0, 1e-10))

    # Force event processing
    pg.QtGui.QApplication.processEvents()

def save_plot_data():
    try:
        with open('plot_data.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            # Write headers
            writer.writerow(['Index', 'Force', 'Setpoint', 'Error', 'Action'])
            # Write data
            for i in range(len(peso)):
                writer.writerow([
                    i,
                    peso[i],
                    setpoint[i],
                    error[i],
                    accion[i]
                ])
        print("[INFO] Data saved to plot_data.csv")
    except Exception as e:
        print(f"[ERROR] Failed to save data: {e}")
    finally:
        # Attempt to close serial port cleanly
        try:
            if ser is not None and getattr(ser, "is_open", False):
                ser.close()
                print("[INFO] Serial port closed")
        except Exception as e:
            print(f"[WARN] Failed to close serial: {e}")

# Register the cleanup function
atexit.register(save_plot_data)

# Make sure Qt quit triggers the same save routine
try:
    app.aboutToQuit.connect(save_plot_data)
except Exception:
    pass

# Allow Ctrl-C in the terminal to quit the app
def _handle_sigint(sig, frame):
    print("[INFO] SIGINT received, exiting...")
    # This will cause the Qt event loop to quit and trigger aboutToQuit/atexit
    QApplication.quit()

signal.signal(signal.SIGINT, _handle_sigint)

# === START TIMER ===
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(UPDATE_INTERVAL_MS)

# === RUN APP ===
if __name__ == '__main__':
    print(f"[INFO] Plotting live from {PORT}...")
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QApplication.instance().exec_()
