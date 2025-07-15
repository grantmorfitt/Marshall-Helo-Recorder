import tkinter as tk
from tkinter import messagebox, scrolledtext
from tkinter import ttk, scrolledtext
from datetime import datetime
import serial
import serial.tools.list_ports
import threading
import time
import csv
import sys
import math
import queue
import os
import tomli
import an_devices.spatial_device as spatial_device
from anpp_packets.an_packet_protocol import ANPacket
from anpp_packets.an_packets import PacketID




class AppGUI:
    def __init__(self, root, IOHelper, data_recorder1, data_recorder2, data_recorder3):
        self.root = root
        self.data_recorder1 = data_recorder1
        self.data_recorder2 = data_recorder2
        self.data_recorder3 = data_recorder3
        
        self.root.title("FAA Flight Test Sensor Data Recorder")
     
        # Initialize files
        self.IOHelper = IOHelper
     
        # === FILENAME & LED ===
        tk.Label(root, text="Filename:").grid(row=0, column=0, padx=(10, 2), pady=10, sticky='e')
        self.filename_entry = tk.Entry(root, width=40)
        self.filename_entry.grid(row=0, column=1, columnspan=2, padx=(2, 10), pady=10, sticky='w')
     
        self.led = tk.Canvas(root, width=25, height=25, highlightthickness=0)
        self.led.grid(row=0, column=3, padx=10, pady=10)
        self.led_circle = self.led.create_oval(2, 2, 22, 22, fill="gray")
     
        # === BUTTONS ===
        tk.Button(root, text="Start", command=self.start_acquisition, width=15).grid(row=1, column=0, padx=10, pady=10)
        tk.Button(root, text="Stop", command=self.stop_acquisition, width=15).grid(row=1, column=1, padx=10, pady=10)
        tk.Button(root, text="Quit", command=lambda: root.destroy(), width=15).grid(row=1, column=3, padx=10, pady=10)
     
        # === LOG BOX ===
        self.log_box = scrolledtext.ScrolledText(root, width=75, height=10, state='disabled')
        self.log_box.grid(row=2, column=0, columnspan=4, padx=10, pady=(10, 10))
     
        # === COMMENT SECTION ===
        self.comment_entry = tk.Text(root, width=60, height=4)
        self.comment_entry.grid(row=3, column=0, columnspan=2, padx=(10, 5), pady=(10, 20), sticky='w')
     
        self.comment_time = tk.StringVar()
        self.comment_time.set("Time")
        tk.Button(root, textvariable=self.comment_time, command=self.capture_time, width=10).grid(row=3, column=2, pady=(10, 20))
        tk.Button(root, text="Submit Comment", command=self.submit_comment, width=15).grid(row=3, column=3, padx=(5, 10), pady=(10, 20))
     
        # === MANEUVER SECTION ===
        self.maneuver_frame = tk.Frame(root)
        self.maneuver_frame.grid(row=4, column=0, columnspan=4, padx=10, pady=(0, 30), sticky='w')
     
        tk.Label(self.maneuver_frame, text="Maneuver:").grid(row=0, column=0, padx=(0, 5), pady=5, sticky='e')
        self.maneuver_var = tk.StringVar()
        self.maneuver_combobox = ttk.Combobox(self.maneuver_frame, textvariable=self.maneuver_var, values=["Hover", "Climb", "Descent", "Turn", "Autorotation"], width=35)
        self.maneuver_combobox.grid(row=0, column=1, padx=(0, 10), pady=5, sticky='w')
     
        self.maneuver_start_button = tk.Button(self.maneuver_frame, text="Start Maneuver", width=15, command=self.start_maneuver)
        self.maneuver_start_button.grid(row=0, column=2, padx=5, pady=5)
        self.maneuver_stop_button = tk.Button(self.maneuver_frame, text="Stop Maneuver", width=15, command=self.stop_maneuver)
        self.maneuver_stop_button.grid(row=0, column=3, padx=5, pady=5)
     
        # === RIGHT SIDE VISUALIZATION ===
        self.visuals_frame = tk.Frame(root)
        self.visuals_frame.grid(row=0, column=4, rowspan=6, padx=(30, 30), pady=20, sticky='ns')
     
        # Pitch/Roll
        tk.Label(self.visuals_frame, text="Pitch / Roll").grid(row=0, column=0, pady=(0, 10))
        self.pitchroll_canvas = tk.Canvas(self.visuals_frame, width=120, height=120, bg='white', highlightthickness=0)
        self.pitchroll_canvas.grid(row=1, column=0, pady=(0, 30))
        self.pitchroll_canvas.create_rectangle(0, 0, 120, 120, width=2)
        self.pitchroll_canvas.create_line(60, 0, 60, 120, width=1)
        self.pitchroll_canvas.create_line(0, 60, 120, 60, width=1)
        self.pitchroll_dot = self.pitchroll_canvas.create_oval(55, 55, 65, 65, fill='green')
     
        # Collective
        tk.Label(self.visuals_frame, text="Collective").grid(row=2, column=0, pady=(0, 10))
        self.collective_canvas = tk.Canvas(self.visuals_frame, width=200, height=40, bg='white', highlightthickness=0)
        self.collective_canvas.grid(row=3, column=0, pady=(0, 30))
        self.collective_canvas.create_rectangle(0, 0, 200, 40, width=2)
        self.collective_canvas.create_line(100, 0, 100, 40, width=1, dash=(4, 2))
        self.collective_dot = self.collective_canvas.create_oval(95, 10, 105, 30, fill='green')
     
        # Pedals
        tk.Label(self.visuals_frame, text="Pedals").grid(row=4, column=0, pady=(0, 10))
        self.pedal_canvas = tk.Canvas(self.visuals_frame, width=200, height=40, bg='white', highlightthickness=0)
        self.pedal_canvas.grid(row=5, column=0, pady=(0, 20))
        self.pedal_canvas.create_rectangle(0, 0, 200, 40, width=2)
        self.pedal_canvas.create_line(100, 0, 100, 40, width=1, dash=(4, 2))
        self.pedal_dot = self.pedal_canvas.create_oval(95, 10, 105, 30, fill='green')
     
        self.root.update()
        self.root.minsize(self.root.winfo_width(), self.root.winfo_height())
        
        status = IOHelper.InitializeParameters()
        if status == 1:
            self.UpdateComboBox()
        elif status == 0:
            self.log("Error loading files")

    def capture_time(self):
        current_time = datetime.now().strftime("%H:%M:%S")
        self.comment_time.set(current_time)

    def submit_comment(self):
        timestamp = self.comment_time.get("1.0", tk.END)
        comment = self.comment_entry.get("1.0", tk.END)
        if comment:
            self.log(f"[COMMENT] {timestamp} - {comment}")
            IOHelper.comment_que.put(comment)
            self.comment_entry.delete("1.0", tk.END)
            self.comment_time.set("Time")

    def start_maneuver(self):
        maneuver = self.maneuver_var.get()
        current_time = datetime.now().strftime("%H:%M:%S")
        
        if (maneuver != ""):
            self.log(f"Maneuver started: {maneuver}")
            que_message = f"{current_time}_MANEUVER_START_{maneuver}"
            print(que_message)
            IOHelper.comment_que.put(que_message)
        else:
            self.log("No maneuver selected!")
            
    def stop_maneuver(self):
        maneuver = self.maneuver_var.get()
        current_time = datetime.now().strftime("%H:%M:%S")
        
        if (maneuver != "") :
            self.log(f"Maneuver stopped: {maneuver}")
            que_message = f"{current_time}_MANEUVER_STOP_{maneuver}"
            IOHelper.comment_que.put(que_message)
        else:
            self.log("No maneuver selected!")
            
            
    def UpdateComboBox(self):
        self.maneuver_combobox["values"] = self.IOHelper.maneuver_Lookup
        

    def UpdateSensorVisualization(self, cylcic_pitch, cyclic_roll, collective, pedals):
        """
        

        Parameters
        ----------
        cylic_pitch : TYPE
            DESCRIPTION.
        cyclic_roll : TYPE
            DESCRIPTION.
        collective : TYPE
            DESCRIPTION.
        pedals : TYPE
            DESCRIPTION.

        Returns
        -------
        None.

        """
        
        #0 - 15v on far right side
        cyclic_pitch_clamped =  max(0, min(15, cylcic_pitch))
        cyclic_roll_clamped =  max(0, min(15, cyclic_roll))
        collective_clamped =  max(0, min(15, collective))
        pedals_clamped =  max(0, min(15, pedals))
        
        cyclic_x = (cyclic_roll_clamped / 15) * 120
        cyclic_y = 120 - (cyclic_pitch_clamped / 15) * 120
        collective_x = (collective_clamped / 15) * 200
        pedals_x = (pedals_clamped / 15) * 200
        
        self.pitchroll_canvas.coords(self.pitchroll_dot, cyclic_x - 5, cyclic_y - 5, cyclic_x + 5, cyclic_y + 5)
        
        self.collective_canvas.coords(self.collective_dot, collective_x - 5, 10, collective_x + 5, 30)
        self.pedal_canvas.coords(self.pedal_dot, pedals_x - 5, 10, pedals_x + 5, 30)

        
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
        
        dirname = os.path.abspath(os.getcwd())
        filepath = dirname + '\\data\\'
          
        DIfilename = filepath + "ControlPos_" + self.get_filename() + ".csv"
        self.data_recorder1.start_acquisition(DIfilename)
        
        SpatialFileName = filepath + "SpatialFOG_" + self.get_filename() + ".anpp"
        self.data_recorder2.start_acquisition(SpatialFileName)

        ManeuverFileName = filepath + "ManeuverLog_" + self.get_filename() + ".csv"
        self.data_recorder3.start_acquisition(ManeuverFileName)

        
        #this will need to pass filenames over once I get it set up properly
    def stop_acquisition(self):
        self.led.itemconfigure(self.led_circle, fill="gray")
        self.data_recorder1.stop_acquisition()
        self.data_recorder2.stop_acquisition()
        self.data_recorder3.stop_acquisition()
    

class DIDataRecorder:
    def __init__(self, log_callback, gui_ref=None):
        self. heartbeat_interval = 5  # seconds
        self.gui = gui_ref #placeholder for the root reference
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
            self.log("DI-2008 not connected.")
            return
        
        self._process_thread(filename)

    def _process_thread(self, filename):
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
        pitch_val = 7
        roll_val = 7
        pedal_val = 7
        collective_val = 7
        
        try:
            self.send_cmd("start")
            with open(filename, "w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["Time"] + [f"{v}" for i,v in enumerate(self.colNames)])
                
                last_heartbeat = time.time()

           
                while not self.stop_event.is_set():
                    if self.ser.in_waiting >= 2 * len(self.slist):
                        row = [datetime.now().strftime('%H:%M:%S')]
                        for i in self.slist:
                            #LFB = i&0xf
                            raw = self.ser.read(2)
                            val =  ((int.from_bytes(raw, 'little', signed=True) * 25)/32768)
                            
                            match i:
                                case 2304: #pitch
                                    pitch_val = val
                                case 2305: #pedal
                                    pedal_val = val
                                case 2306: #collective
                                    collective_val = val
                                case 2307: #roll
                                    roll_val = val
                                case _:
                                    print(f"no match: {i}")
                            
                            
                            row.append(val)
                        
                        writer.writerow(row)
                        self.gui.UpdateSensorVisualization(pitch_val, roll_val, collective_val, pedal_val)
                    
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
            
    def start_acquisition(self, SpatialFileName):
        self.acquiring = True
        self.stop_event.clear()
        threading.Thread(target=self._process_thread, args = (SpatialFileName,), daemon=True).start()
        
    def _process_thread(self, filename):
        
        spatial = spatial_device.Spatial(self.comport, self.baudrate)
        spatial.start()
        
        if spatial.is_open:
          print(f"Spatial FOG connected to port:{spatial.port} with baud:{spatial.baud}")
          spatial.flush()
          
          # Creates log file for received binary data from device          
          logFile = open(f"{filename}", 'xb')

          # Sets sensor ranges
          spatial.set_sensor_ranges(True,
                                    spatial_device.AccelerometerRange.accelerometer_range_4g,
                                    spatial_device.GyroscopeRange.gyroscope_range_500dps,
                                    spatial_device.MagnetometerRange.magnetometer_range_8g)
    
          spatial.get_device_and_configuration_information()
          
          last_heartbeat = time.time()
          
          self.acquiring = True
          try:
              while spatial.is_open and not self.stop_event.is_set():
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
                      
              logFile.close()
              self.acquiring = False
              self.log(f"Spatial Data written to {filename}")

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
        
class ManueverRecorder:
    def __init__(self, IOHelper, log_callback):

        self.message_que = IOHelper.comment_que
        self.acquiring = False
        self.stop_event = threading.Event()
        self.log = log_callback
        
    def _process_thread(self, filename):
        print("STARTING MANEUVER LOGGING")
        
        logFile = open(f"{filename}", 'w')
        writer = csv.writer(logFile)
        writer.writerow(["Time"] + ["Maneuver/Comments"])
        
        while not self.stop_event.is_set():

           try:
               # Default is blocking with no timeout
               message = self.message_que.get(block=True, timeout=None)
               if (message == "END"):
                   break
               
               print(message)
               message_time = message.split("_")[0]
               message = "_".join(message.split("_")[1:])
                
               row = [message_time, message]
               writer.writerow(row)
               print(f"Writing message: {row}")
               
           except Exception as e:
               self.log(f"Process maneuver error: {e}")
               print(f"ERROR: {e}")
    
        logFile.close()
        
        print("log file closed")
        
    def start_acquisition(self, filename):        
        self.acquiring = True
        self.stop_event.clear()
        threading.Thread(target=self._process_thread, args = (filename,), daemon=True).start()
    
    def stop_acquisition(self):
        self.acquiring = False
        self.stop_event.set()
        self.message_que.put("END")
        
class IOHelper:
    """
        Helper function for file creation and management
    """
    
    def __init__ (self, root): #intance will pass in root variable

        self.maneuver_Lookup = []
        self.initialized = False
        self.simPaused = False
        self.writer = None
        self.call_count = 0 #Will be used to print a heartbeat message to the console every so many calls
        self.heartbeat_message_interval = 50 #Every 50 lines it will print a heartbeat message to the console                            #so this queue will be checked when writing rows
        self.root = root
        self.comment_que = queue.Queue()
        root.protocol("WM_DELETE_WINDOW", lambda: self.OnClose())

        
        
    def GetPilots(self):
        return self.pilot_Lookup;
    
    def GetBlocks(self):
        return self.block_Lookup
    
    def GetLessons(self):
        return self.lesson_Lookup
    
    def InitializeParameters(self):
        """
        Imports required config files 
        
        Returns
        -------
         Returns two status items. One for each config toml
        """
        lesson_status = self._ImportLessonToml()
        
        if lesson_status == 1:   
            self.initialized = True
            #file = self._CreateFile(self.blankOutputFileHeader)
            return lesson_status
        
        return lesson_status
        
    def _ImportLessonToml(self):
        """
        Private function that imports the lesson toml config for parameters from loft sim

        Returns
        -------
        1 if successful, 0 if not successful

        """
        try: 
            dirname = os.path.abspath(os.getcwd())
            filename = dirname + '\\config\\maneuverconfig.toml'
            toml = open(filename, "rb")
            
            toml_dict = tomli.load(toml)['maneuverconfig']
            
            for value in toml_dict:

                if value == "maneuvers":
                    for i in toml_dict[value]:
                        self.maneuver_Lookup.append(i) #add each pilot to the lookup table
                     
            return 1
        except: 
            return 0


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
    IOHelper = IOHelper(root)
    ManueverRecorder = ManueverRecorder(IOHelper, log_callback=lambda msg: gui.log(msg))
    
    gui = AppGUI(root, IOHelper, DIrecorder, SpatialRecorder, ManueverRecorder)
    

    
    DIrecorder.gui = gui
    DIrecorder.log = gui.log  # assign log callback after GUI init.
    SpatialRecorder.log = gui.log
    ManueverRecorder.log = gui.log
    # Start discovery on launch in a thread
    #threading.Thread(target=DIrecorder.initialize, args=(root.quit,), daemon=True).start()
    SpatialRecorder.initialize()
    DIrecorder.initialize()

    root.protocol("WM_DELETE_WINDOW", lambda: MasterCloseSerial(SpatialRecorder, DIrecorder))
    root.mainloop()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
