import tkinter as tk
from tkinter import messagebox, scrolledtext
from datetime import datetime
import serial
import serial.tools.list_ports
import threading
import time
import csv
import sys
import math

import an_devices.spatial_device as spatial_device
from anpp_packets.an_packet_protocol import ANPacket
from anpp_packets.an_packets import PacketID




class AppGUI:
    def __init__(self, root, data_recorder1, data_recorder2):
        self.root = root
        self.data_recorder1 = data_recorder1
        self.data_recorder2 = data_recorder2
        self.root.title("FAA Flight Test Sensor Data Recorder")

        # Filename entry
        tk.Label(root, text="Filename:").grid(row=0, column=0, padx=(10, 2), pady=5, sticky='e')
        self.filename_entry = tk.Entry(root, width=40)
        self.filename_entry.grid(row=0, column=1, columnspan=3, padx=(2, 10), pady=5, sticky='w')
        self.filename_entry.insert(0, f"Write filename here")

        # Log box (scrolled text)
        self.log_box = scrolledtext.ScrolledText(root, width=75, height=20, state='disabled')
        self.log_box.grid(row=2, column=0, columnspan=4, padx=10, pady=10)

        # Buttons
        tk.Button(root, text="Start", command=self.start_acquisition, width=15).grid(row=1, column=0, padx=10, pady=10)
        tk.Button(root, text="Stop", command=self.stop_acquisition, width=15).grid(row=1, column=1, padx=10, pady=10)
        tk.Button(root, text="Quit", command= lambda: root.destroy(), width=15).grid(row=1, column=3, padx=10, pady=10)
        
        self.led = tk.Canvas(root, width=25, height=25, highlightthickness=0)
        self.led.grid(row=0, column=3, padx=10, pady=10)
        self.led_circle = self.led.create_oval(2, 2, 22, 22, fill="gray")
        
    def log(self, msg):
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.log_box.config(state='normal')
        self.log_box.insert(tk.END, f"[{timestamp}] {msg}\n")
        self.log_box.see(tk.END)
        self.log_box.config(state='disabled')

    def get_filename(self):
        name = self.filename_entry.get().strip()
        now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        
        if not name:
            return f"{now}_data"
        else: return f"{now}_{name}"
        
    def start_acquisition(self):
        self.led.itemconfigure(self.led_circle, fill="green")
        DIfilename = "ControlPos_" + self.get_filename()
        if not DIfilename.lower().endswith(".csv"):
            DIfilename += ".csv"
        threading.Thread(target=self.data_recorder1.start_acquisition, args = (DIfilename,), daemon=True).start()
        
        SpatialFileName = "SpatialFOG_" + self.get_filename()
        threading.Thread(target=self.data_recorder2.start_acquisition, args = (SpatialFileName,), daemon=True).start()

        
        #this will need to pass filenames over once I get it set up properly
        print("lorem")
    def stop_acquisition(self):
        self.led.itemconfigure(self.led_circle, fill="gray")
        self.data_recorder1.stop_acquisition()
        self.data_recorder2.stop_acquisition()
    

class DIDataRecorder:
    def __init__(self, log_callback):
        self. heartbeat_interval = 5  # seconds
        self.ser = serial.Serial()
        self.acquiring = False
        self.stop_event = threading.Event()
        self.range_table = []
        self.log = log_callback

        self.slist = [0x0900, 0x0901, 0x0902, 0X0903]  # Customize as needed
        #self.slist = [0xB00]
        #self.slist = [2304,2305, 2306, 2307]
        self.colNames = ["Pitch", "Roll", "Collective", "Pedal"]
        #Change slist tuple to vary analog channel configuration.
        
        self.analog_ranges = [.5, 0.25, 0.1, .05, .025, .01, 0, 0, 50, 25, 10, 5, 2.5, 1, 0, 0]

    def send_cmd(self, cmd):
        try:
            self.ser.write((cmd + '\r').encode())
            time.sleep(0.1)
            if not self.acquiring:
                while self.ser.in_waiting:
                    try:
                        line = self.ser.readline().decode().strip()
                    except:
                        continue
        except Exception as e:
            self.log(f"Send command error: {e}")

    def discover_device(self):
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if "VID:PID=0683" in port.hwid:
                self.ser.port = port.device
                self.ser.baudrate = 115200
                self.ser.timeout = 0
                try:
                    if self.ser.is_open:
                        self.ser.close()
                    self.ser.open()
                    self.log(f"Connected to {port.device}")
                    return True
                except Exception as e:
                    self.log(f"Failed to open DI-2008 port: {e}")
                    return False
        self.log("No DI-2008 device found.")
        return False

    def config_scan_list(self):
        self.range_table.clear()
        for i, item in enumerate(self.slist):
            self.send_cmd(f"slist {i} {item}")
            if (item & 0xf < 8) and not (item & 0x1000):
                self.range_table.append(self.analog_ranges[item >> 8])
            else:
                self.range_table.append(0)

    def initialize(self):
        if self.discover_device():
            self.send_cmd("stop")
            self.send_cmd("ps 1")
            self.send_cmd("dec 1")
            self.send_cmd("srate 10")

            self.config_scan_list()
            self.log("DI-2008 Device initialized")
        else:
            messagebox.showerror("Error", "DI-2008 not found.")

    def start_acquisition(self, filename):
        if self.acquiring:
            self.log("Already acquiring.")
            return
        if not self.ser.is_open:
            self.log("Spatial FOG not connected.")
            return
        self.acquiring = True
        self.stop_event.clear()
        threading.Thread(target=self.acquire_data, args=(filename,), daemon=True).start()
        self.log("Start pressed. Acquisition started.")

    def stop_acquisition(self):
        if not self.acquiring:
            self.log("Already stopped.")
            return
        self.acquiring = False
        self.send_cmd("stop")
        self.stop_event.set()
        self.log("Stop pressed. Acquisition stopped.")
        self.ser.close()
    

    def acquire_data(self, filename):
        try:
            self.send_cmd("start")
            with open(filename, "w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["Time"] + [f"{v}" for i,v in enumerate(self.colNames)])
                
                last_heartbeat = time.time()

           
                while self.acquiring and not self.stop_event.is_set():
                    if self.ser.in_waiting >= 2 * len(self.slist):
                        row = [datetime.now().strftime('%H:%M:%S')]
                        for _ in self.slist:
                            raw = self.ser.read(2)
                            val = int.from_bytes(raw, 'little', signed=True)
                            row.append(val)
                        writer.writerow(row)
                        #self.log(f"Control Data: {', '.join(map(str, row))}")
                    
                    if time.time() - last_heartbeat >= self.heartbeat_interval:
                       self.log(f"{row[0]} DI-2008: Heartbeat")
                       last_heartbeat = time.time()
                       
            self.log(f"Data written to {filename}")
        except Exception as e:
            self.log(f"Control Record Error: {e}")
            


class SpatialFogDataRecorder:
    def __init__(self, log_callback):
        self.heartbeat_interval = 5
        self.ser = serial.Serial()
        self.acquiring = False
        self.stop_event = threading.Event()
        self.range_table = []
        self.log = log_callback
        self.spatial = None
        self.comport = 'COM5'
        self.baudrate = 1250000
        
        
    
    def initialize(self):
        self.log("Checking Spatial FOG device...")
   

        try:
            test_spatial = spatial_device.Spatial(self.comport, self.baudrate)
            test_spatial.start()
    
            if not test_spatial.is_open:
                self.log("Serial port not open.")
                messagebox.showerror("Device Error", "Spatial FOG device not found.")
                return False
    
            self.log("Serial port is open. Waiting for incoming ANPP packet...")
    
            # Flush any old data
            test_spatial.flush()
            time.sleep(0.5)
    
            # Read a few bytes from the buffer
            timeout = time.time() + 2  # 2 second timeout
            found_bytes = False
    
            while time.time() < timeout:
                if test_spatial.in_waiting() > 0:
                    found_bytes = True
                    break
                time.sleep(0.1)
    
            if found_bytes:
                self.log("Spatial FOG is powered on and sending data.")
                test_spatial.close()
                return True
            else:
                self.log("No response from Spatial FOG (in_waiting = 0).")
                messagebox.showerror("Device Error", "Spatial FOG is connected but not responding. Check if it's powered on.")
                return False
    
        except Exception as e:
            self.log(f"Spatial device check error: {e}")
            messagebox.showerror("Device Error", f"Error initializing Spatial FOG:\n{e}. \rCheck if device is connected!")
            return False
            
        

    def start_acquisition(self, filename):
        
        spatial = spatial_device.Spatial(self.comport, self.baudrate)
        spatial.start()
        
        if spatial.is_open:
          print(f"Connected to port:{spatial.port} with baud:{spatial.baud}")
          spatial.flush()
          
          # Creates log file for received binary data from device
          now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
          logFile = open(f"{filename}.anpp", 'xb')
    
          #an_packet = ANPacket()
    
          # Sets sensor ranges
          spatial.set_sensor_ranges(True,
                                    spatial_device.AccelerometerRange.accelerometer_range_4g,
                                    spatial_device.GyroscopeRange.gyroscope_range_500dps,
                                    spatial_device.MagnetometerRange.magnetometer_range_8g)
    
          spatial.get_device_and_configuration_information()
          
          last_heartbeat = time.time()
          
          self.acquiring = True
          try:
              while spatial.is_open:
                  if spatial.in_waiting() > 0:
                      if not self.stop_event.is_set():
                          # Get bytes in serial buffer
                          bytes_in_buffer = spatial.in_waiting()
                          data_bytes = spatial.read(bytes_in_buffer)
            
                          # Record in log file the raw binary of ANPP packets
                          logFile.write(data_bytes)
                          if time.time() - last_heartbeat >= self.heartbeat_interval:
                             self.log(f"{datetime.now().strftime('%H:%M:%S')} Spatial Fog: Heartbeat")
                             last_heartbeat = time.time()
                             
                             
                          #self.log(datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + " Spatial Record Bytes")
                      else:
                          logFile.close()
                          self.acquiring = False
                          self.log(f"Spatial Data written to {filename}")
                          break;
          except Exception as e:
              self.log(f"Spatial FOG Error: {e}")       
              
          
              
        elif not spatial.is_open():
            self.log("WARNING: Spatial Fog not detected!") 
            
        
    
    def stop_acquisition(self):
        if not self.acquiring:
            self.log("Already stopped.")
            return
        self.acquiring = False
        self.stop_event.set()

        self.log("Stop pressed. Spatial Acquisition stopped.")
        self.ser.close()

def MasterCloseSerial(instanceOne, instanceTwo):
    """
    Ensure all serial connections are closed in the event that the user clicks the close button instead of quit,
    or program has an unexpected error

    Returns
    -------
    None.

    """
    instanceOne.ser.close()
    instanceTwo.ser.close()
    root.destroy()
    sys.exit()

if __name__ == "__main__":
    
    root = tk.Tk()

    
    DIrecorder = DIDataRecorder(log_callback=lambda msg: gui.log(msg))  # placeholder, will reassign after gui created
    SpatialRecorder = SpatialFogDataRecorder(log_callback=lambda msg: gui.log(msg))
    gui = AppGUI(root, DIrecorder, SpatialRecorder)
    

    

    DIrecorder.log = gui.log  # assign log callback after GUI init.
    SpatialRecorder.log = gui.log
        
    # Start discovery on launch in a thread
    #threading.Thread(target=DIrecorder.initialize, args=(root.quit,), daemon=True).start()
    SpatialRecorder.initialize()
    DIrecorder.initialize()

    root.protocol("WM_DELETE_WINDOW", lambda: MasterCloseSerial(SpatialRecorder, DIrecorder))
    root.mainloop()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
