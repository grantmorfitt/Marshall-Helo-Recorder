import tkinter as tk
from tkinter import messagebox, scrolledtext
from datetime import datetime
import serial
import serial.tools.list_ports
import threading
import time
import csv

# Globals
ser = serial.Serial()
acquiring = False
stop_event = threading.Event()
slist = [0x0A00, 0x0B01, 0x1702]  # Customize as needed
range_table = []
analog_ranges = [.5, 0.25, 0.1, .05, .025, .01, 0, 0, 50, 25, 10, 5, 2.5, 1, 0, 0]

# --- GUI SETUP ---
root = tk.Tk()
root.title("FAA Flight Test Sensor Data Recorder")

# Filename entry
tk.Label(root, text="Filename:").grid(row=0, column=0, padx=(10, 2), pady=5, sticky='e')
filename_entry = tk.Entry(root, width=40)
filename_entry.grid(row=0, column=1, columnspan=3, padx=(2, 10), pady=5, sticky='w')
filename_entry.insert(0, f"{datetime.now().strftime('%Y-%m-%d_%H%M:%S')}.csv")

log_box = scrolledtext.ScrolledText(root, width=75, height=20, state='disabled')
log_box.grid(row=2, column=0, columnspan=4, padx=10, pady=10)

def log(msg):
    timestamp = datetime.now().strftime('%H:%M:%S')
    log_box.config(state='normal')
    log_box.insert(tk.END, f"[{timestamp}] {msg}\n")
    log_box.see(tk.END)
    log_box.config(state='disabled')

def send_cmd(cmd):
    global acquiring
    ser.write((cmd + '\r').encode())
    time.sleep(0.1)
    if not acquiring:
        while ser.in_waiting:
            try:
                line = ser.readline().decode().strip()
                if line:
                    log(f"Device: {line}")
                    break
            except:
                continue

def discover_device():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if "VID:PID=0683" in port.hwid:
            ser.port = port.device
            ser.baudrate = 115200
            ser.timeout = 0
            try:
                ser.open()
                log(f"Connected to {port.device}")
                return True
            except Exception as e:
                log(f"Failed to open port: {e}")
                return False
    log("No DI-2008 device found.")
    return False

def config_scan_list():
    range_table.clear()
    for i, item in enumerate(slist):
        send_cmd(f"slist {i} {item}")
        if (item & 0xf < 8) and not (item & 0x1000):
            range_table.append(analog_ranges[item >> 8])
        else:
            range_table.append(0)

def initialize():
    if discover_device():
        send_cmd("stop")
        send_cmd("ps 0")
        send_cmd("dec 20")
        send_cmd("srate 1")
        config_scan_list()
        log("Device initialized and ready.")
    else:
        messagebox.showerror("Error", "DI-2008 not found.")
        root.quit()

def get_filename():
    name = filename_entry.get().strip()
    if not name:
        return "data_log.csv"
    if not name.lower().endswith(".csv"):
        name += ".csv"
    return name

def start_acquisition():
    global acquiring
    if acquiring:
        log("Already acquiring.")
        return
    acquiring = True
    stop_event.clear()
    threading.Thread(target=acquire_data, daemon=True).start()
    log("Start pressed. Acquisition started.")

def stop_acquisition():
    global acquiring
    if not acquiring:
        log("Already stopped.")
        return
    acquiring = False
    stop_event.set()
    send_cmd("stop")
    log("Stop pressed. Acquisition stopped.")

def reset_counter():
    send_cmd("reset 1")
    log("Counter reset sent.")

def acquire_data():
    global acquiring
    try:
        send_cmd("start")
        filename = get_filename()
        with open(filename, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Time"] + [f"Ch{i}" for i in range(len(slist))])
            while acquiring and not stop_event.is_set():
                if ser.in_waiting >= 2 * len(slist):
                    row = [datetime.now().strftime('%H:%M:%S')]
                    for _ in slist:
                        raw = ser.read(2)
                        val = int.from_bytes(raw, 'little', signed=True)
                        row.append(val)
                    writer.writerow(row)
                    log(f"Data: {', '.join(map(str, row))}")
                time.sleep(0.1)
        log(f"Data written to {filename}")
    except Exception as e:
        log(f"Error: {e}")
    finally:
        acquiring = False
        send_cmd("stop")

# --- Buttons ---
tk.Button(root, text="Start", command=start_acquisition, width=15).grid(row=1, column=0, padx=10, pady=10)
tk.Button(root, text="Stop", command=stop_acquisition, width=15).grid(row=1, column=1, padx=10, pady=10)
tk.Button(root, text="Reset Counter", command=reset_counter, width=15).grid(row=1, column=2, padx=10, pady=10)
tk.Button(root, text="Quit", command=root.quit, width=15).grid(row=1, column=3, padx=10, pady=10)

# Start discovery on launch
threading.Thread(target=initialize, daemon=True).start()

root.mainloop()
