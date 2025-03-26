import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time
from threading import Thread
from IPython import get_ipython
import numpy as np
from src.utils import *

# Main application class for IMU Recording Studio
class IMURecordingStudio(tk.Tk):
    def __init__(self):
        super().__init__()

        # Global variables
        self.imu_status_lamps = [] # List of IMU lamps
        self.imu_status_labels = [] # List of IMU labels
        self.imu_battery_status_labels = [] # List of labels with status labels
        self.imu_tags = [] # List With the tags from each IMU
        self.MAXCAMERAS = DEFAULT_SETTINGS.max_cameras() # Maximum number of cameras
        self.camera_lamps = [] # List of camera lamps
        self.camera_labels = [] # List of camera labels
        self.InitiatedCameras = [] # List of initiated cameras
        self.idxInitiatedCameras = [] # List of indexes of initiated cameras
        self.imu = None # Stores the class for IMU sensors
        self.imu_comboboxes = [] # List that stores the comboboxes for imus, one for each leg component.
        self.camera_list_1_previous_Value = None # List of previous values of Combobox 1
        self.camera_list_2_previous_Value = None # List of previous values of Combobox 2
        
        self.imu_lock_status = False #Flag for storing if IMU configuration is locked (checked)
        self.imu_streaming = False # Flag for storing if IMUs are streaming in the IMU vector view
        self.imu_configuration_list = [] # List for storing the configuration list with IMUs
        self.imu_ordered_configuration = [] # List that stores the correct order of the IMUs based on the configuration in the settings

        # Thread explanation: #Thread[0] and Thread[1] are used by webcams to stream upon pressing the Start/Stop button
        # Thread[2] will be used when Run/Stop Streaming button is pressed to stream IMU data in the IMU vector view.
        # When Start Recording Button is pressed -> Thread[0] will be used to Collect data from all sensors, Thread[1] will be used to stream webcam 1, 
        # Thread[2] will be used to stream webcam 2, Thread[3] will be used to plot IMU data
        self.thread = [None, None, None, None] # Stores threads
        self.thread_flag = [False, False, False, False] # Flag for stopping threads

       
       
        self.title("IMU Recording Studio")
        self.geometry("850x800")
        
        # Create Tab control
        self.tabControl = ttk.Notebook(self)
        
        # Create Main tab
        self.main_tab = ttk.Frame(self.tabControl)
        self.tabControl.add(self.main_tab, text='Main')
        
        # Create Settings tab
        self.settings_tab = ttk.Frame(self.tabControl)
        self.tabControl.add(self.settings_tab, text='Settings')
        
        self.tabControl.pack(expand=1, fill="both")
        
        # Setup the main tab (IMU Vector and Camera Views)
        self.create_main_tab()
        
        # Setup the settings tab (IMU Sensor Status)
        self.create_settings_tab()

    def create_main_tab(self):
        # Camera Views Frame
        camera_frame = ttk.LabelFrame(self.main_tab, width=500, height=250, text="Camera Views")
        camera_frame.grid(row=0, column=0, padx=10, pady=10, columnspan=1)
        
        # Camera Control Frame
        camera_control_frame = ttk.LabelFrame(self.main_tab, text="")
        camera_control_frame.grid(row=1, column=0, padx=10, pady=10)
        camera_control_frame.place(x=10, y=340, width=420, height=50) 

        # IMU Vector View Frame
        imu_frame = ttk.LabelFrame(self.main_tab, width=250, height=250, text="IMU Vector View")
        imu_frame.grid(row=4, column=0, padx=10, pady=10)
        imu_frame.place(x=10, y=400)

        # IMU Vector Control Frame
        IMU_control_frame = ttk.LabelFrame(self.main_tab, text="")
        IMU_control_frame.grid(row=1, column=0, padx=10, pady=10)
        IMU_control_frame.place(x=10, y= 720, width=420, height=50) 

        # Functionality Buttons Frame
        functionality_frame = ttk.LabelFrame(self.main_tab, text="Functionality Buttons")
        functionality_frame.grid(row=4, column=1, padx=10, pady=10)
        functionality_frame.place(x=420, y=400, width=390, height=320)  

        # Create matplotlib figures for the camera views
        self.create_camera_views(camera_frame)
        
        # Create matplotlib figure for IMU Vector view
        self.create_imu_vector_view(imu_frame)

        # Create camera views control buttons
        self.create_camera_control_buttons(camera_control_frame)
        
        self.create_imu_control_button(IMU_control_frame)

        # Create buttons in functionality_frame
        ttk.Button(functionality_frame, text="Start Recording").pack(pady=5, side=tk.LEFT, anchor='nw')
        ttk.Button(functionality_frame, text="Stop Recording").pack(padx=20, pady=5, side=tk.LEFT, anchor='nw')
        
    def create_camera_views(self, frame):
        # Camera 1 View (using Matplotlib as placeholder)
        self.fig1, self.ax1 = plt.subplots(figsize=(4, 3))
        self.ax1.set_title("Front View")
        self.ax1.axis('off')
        self.canvas1 = FigureCanvasTkAgg(self.fig1, master=frame)
        self.canvas1.draw()
        self.canvas1.get_tk_widget().grid(row=0, column=0)
        
        # Camera 2 View (using Matplotlib as placeholder)
        self.fig2, self.ax2 = plt.subplots(figsize=(4, 3))
        self.ax2.set_title("Side View")
        self.ax2.axis('off')
        self.canvas2 = FigureCanvasTkAgg(self.fig2, master=frame)
        self.canvas2.draw()
        self.canvas2.get_tk_widget().grid(row=0, column=2)

    def create_camera_control_buttons(self, frame):
       
        #Dropdown menu to select camera on the front view
        self.camera_list_1= ttk.Combobox(frame, state="disabled",values=[])
        self.camera_list_1.grid(row=1, column=0, columnspan=3)
        # self.camera_list_1.bind("<<ComboboxSelected>>", self.stream_webcam)

        #Dropdown menu to select camera on the side view
        self.camera_list_2 = ttk.Combobox(frame, state="disabled", values=[])
        self.camera_list_2.grid(row=1, column=6, columnspan=3)
        # self.camera_list_2.bind("<<ComboboxSelected>>", self.stream_webcam)

        #Button to stop streaming in Camera View 1
        self.start_stop_button= ttk.Button(frame, text="Start/Stop streaming", command=self.start_stop_button_press)
        self.start_stop_button.grid(row=1, column=3, columnspan=3)

    
    def create_imu_vector_view(self, frame):
        # IMU Vector View (using Matplotlib as placeholder)
        # 
        # self.fig, self.ax = plt.subplots(figsize=(4, 3))
        self.fig = plt.figure(figsize=(4, 3))
        self.ax = self.fig.add_subplot(111, projection='3d')   
        self.ax.set_title("IMU Vector View")
        self.ax.axis('off')
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack()

    def create_imu_control_button(self, frame):
        start_stop_imu_button = ttk.Button(frame, text="Start/Stop streaming", command=self.start_stop_button_streaming_imu)
        start_stop_imu_button.grid(row=0, column=1, columnspan=1)
        start_stop_imu_button.place(x=150)

    def create_settings_tab(self):
        # IMU Sensor Status Frame
        self.imu_control_frame = ttk.LabelFrame(self.settings_tab, text="IMU Sensor Control")
        self.imu_control_frame.grid(row=1, column=0, padx=10, pady=10)

        # Connect Sensors Button
        self.frame_rate_list= ttk.Combobox(self.imu_control_frame, state="normal", values=["60", "30", "20", "15", "12", "10", "4", "1"])
        self.frame_rate_list.grid(row=1, column=0, columnspan=1)
        self.frame_rate_list.current(0)

        self.connect_button = ttk.Button(self.imu_control_frame, text="Connect Sensors", command=self.conenct_IMU_sensors)
        self.connect_button.grid(row=2, column=0, columnspan=2, pady=10)
        
        self.discoconnect_button = ttk.Button(self.imu_control_frame, text="Disconnect Sensors", command=self.disconnect_IMU_sensors)
        self.discoconnect_button.grid(row=3, column=0, columnspan=2, pady=10)
        
        self.sync_button = ttk.Button(self.imu_control_frame, text="Sync Sensors", command=self.sync_sensors)
        self.sync_button.grid(row=4, column=0, columnspan=2, pady=10) 

        self.imu_configuration_frame = ttk.LabelFrame(self.settings_tab, text="IMU Sensors Configuration")
        self.imu_configuration_frame.grid(row=2, column=0, padx=10, pady=10)
        
        # Assume that the maximum number of IMU sensors to be connected are 6
        temp_list = ["Left Thigh:", "Left Calf:", "Left Foot:","Right Thigh:", "Right Calf:", "Right Foot:"]
        for imu in range(6):
            imu_label = ttk.Label(self.imu_configuration_frame, text=temp_list[imu], justify='left')
            imu_label.grid(row=imu+3, column=0, padx=10, pady=10)

            imu_combo= ttk.Combobox(self.imu_configuration_frame, state="disabled", values=["None"])
            imu_combo.grid(row=imu+3, column=1, columnspan=1)
            imu_combo.current(0)
            self.imu_comboboxes.append(imu_combo)

        imu_lock_button = ttk.Button(self.imu_configuration_frame, text="Lock configuration", state="enabled", command=self.lock_imu_configuration)    
        imu_lock_button.grid(row=imu+4, column=1, columnspan=1)
       
        imu_lock_button = ttk.Button(self.imu_configuration_frame, text="Unlock configuration", state="enabled", command=self.unlock_imu_configuration)    
        imu_lock_button.grid(row=imu+4, column=2, columnspan=1)

        self.imu_status_frame = ttk.LabelFrame(self.settings_tab, text="IMU Sensors Status")
        self.imu_status_frame.grid(row=2, column=2, padx=10, pady=10)

        self.camera_status_frame = ttk.LabelFrame(self.settings_tab, text="Camera Sensor Status")
        self.camera_status_frame.grid(row=1, column=2, padx=10, pady=10)              

        # Connect Cameras Button
        self.connect_camera_button = ttk.Button(self.camera_status_frame, text="Read Cameras", command=self.connect_webcams)
        self.connect_camera_button.grid(row=0, column=0, columnspan=2, pady=10)
       
        self.disconnect_camera_button = ttk.Button(self.camera_status_frame, text="Diconnect Cameras", command=self.disconnect_webcams)
        self.disconnect_camera_button.grid(row=1, column=0, columnspan=2, pady=10)

        # # Log Label
        # log_label = ttk.Label(self.camera_status_frame, text="Log:")
        # log_label.grid(row=4, column=0, sticky='w')

        # # Log text placeholder
        # log_placeholder = ttk.Label(self.camera_status_frame, text="", relief="sunken")
        # log_placeholder.grid(row=5, column=0, columnspan=2, padx=10, pady=10, sticky='ew')     

    def create_imu_status_lamp(self, frame, text, row, column, batteryLevel):
        label = ttk.Label(frame, text=text)
        label.grid(row=row, column=column, sticky='e', padx=10)
        self.imu_status_labels.append(label)
        
        # Simulate a lamp with a label (can be replaced with a more advanced widget)
        lamp = ttk.Label(frame, text="Connected", background="Orange", width=10)
        lamp.grid(row=row, column=column+1, padx=10)
        self.imu_status_lamps.append(lamp)

        # Create labels that show the battery of each connected dot
        label = ttk.Label(frame, text=f"{batteryLevel}%", width=10)
        label.grid(row=row, column=column+2, padx=10)
        self.imu_battery_status_labels.append(label)

    def create_camera_status_lamp(self, frame, text, row, column):
        label = ttk.Label(frame, text=text, justify='left')
        label.grid(row=row, column=column, sticky='e', padx=10)
        self.camera_labels.append(label)

        # Simulate a lamp with a label (can be replaced with a more advanced widget)
        lamp = ttk.Label(frame, text="Connected", background="Green", width=10)
        lamp.grid(row=row, column=column+1, padx=10)

        #store each lamp in a dictionary for future modification
        self.camera_lamps.append(lamp)

    def connect_webcams(self):
        #set camera_lamps to OFF - in case of reconnection
        for lamp in self.camera_lamps:
            lamp.destroy()

        for labels in self.camera_labels:
            labels.destroy()

        self.openAvailableCameras()

        # create the Labels and lamps for each camera
        for i in range(len(self.idxInitiatedCameras)):
            self.create_camera_status_lamp(self.camera_status_frame, f"Camera {self.idxInitiatedCameras[i]}", i, 0)
        self.connect_camera_button.grid(row=len(self.idxInitiatedCameras)+1, column=0, columnspan=2, pady=10)
        self.disconnect_camera_button.grid(row=len(self.idxInitiatedCameras)+2, column=0, columnspan=2, pady=10)
        self.update_camera_view_list()

    def disconnect_webcams(self):
        for cameras in self.InitiatedCameras:
            cameras.stop_stream()

        for lamp in self.camera_lamps:
            lamp.destroy()

        for labels in self.camera_labels:
            labels.destroy()
            

        self.camera_list_1['values'] = self.idxInitiatedCameras
        self.camera_list_1['state'] = 'disabled'
        self.camera_list_2['values'] = self.idxInitiatedCameras
        self.camera_list_2['state'] = 'disabled'

    def update_camera_view_list(self):
        #Update the list with the number of available cameras
        print(f"Number of detected cameras: {len(self.idxInitiatedCameras)}")
        self.camera_list_1['values'] = self.idxInitiatedCameras
        self.camera_list_1['state'] = 'readonly'
        self.camera_list_2['values'] = self.idxInitiatedCameras
        self.camera_list_2['state'] = 'readonly'
                      
    def openAvailableCameras(self):
        self.InitiatedCameras.clear()
        self.idxInitiatedCameras.clear()

        for i in range(self.MAXCAMERAS):
            cam = CameraManager(i)
            cam.open_camera()
            
            if cam.camera.read()[0]:
                self.InitiatedCameras.append(cam)
                self.idxInitiatedCameras.append(i)
                print(f"Camera {i} is working")
            else:
                print(f"Camera {i} not available")

    def stream_webcam(self): # No very elegant, maybe can be converted to a for loop
        try:
            if self.camera_list_1.get() is not self.camera_list_2.get():
                print(f"Start streaming camera {self.camera_list_1.get()} to front view")
                self.thread[0] = Thread(target=self.update_view, args=(0, self.camera_list_1.get(), self.ax1, self.canvas1), daemon=False)
                self.InitiatedCameras[int(self.camera_list_1.get())].streaming = True
                print(f"Start streaming camera {self.camera_list_2.get()} to side view")
                self.thread[1]=Thread(target=self.update_view, args=(1, self.camera_list_2.get(), self.ax2, self.canvas2), daemon=False)
                self.InitiatedCameras[int(self.camera_list_2.get())].streaming = True
                self.thread_flag[0] = True
                self.thread_flag[1] = True
                self.camera_list_1['state'] = ['disabled']
                self.camera_list_2['state'] = ['disabled']
                self.thread[0].start()
                self.thread[1].start()
            else:
                print("Viewers have the same camera, please change")

        except Exception as e:
            print(f"Error streaming camera {self.camera_list_1.get()} to front view, {e}")
            self.InitiatedCameras[int(self.camera_list_1.get())].stop_stream()

    def update_view(self, threadNum, cameranumber, cameraview_axis, cameraview_canvas):
        #get frame from camera
        while self.thread_flag[threadNum] and self.InitiatedCameras[int(cameranumber)].streaming:# update axis
            camera_frame = self.InitiatedCameras[int(cameranumber)].get_frame()
            if camera_frame is not None:
                cameraview_axis.clear()
                cameraview_axis.imshow(camera_frame)
                cameraview_axis.axis('off')
                cameraview_canvas.draw()                
            # time.sleep(0.001)
            # print("Running")
            if not self.thread_flag[threadNum]:
                self.InitiatedCameras[int(cameranumber)].streaming = False
        # print('Run')
        print(f"Stoped reading from camera {cameranumber}")
        
    def stop_thread_streaming(self):
        self.InitiatedCameras[int(self.camera_list_1.get())].streaming = False
        print(f"Camera {int(self.camera_list_1.get())} set to NO streaming ")

        self.InitiatedCameras[int(self.camera_list_2.get())].streaming = False
        print(f"Camera {int(self.camera_list_2.get())} set to NO streaming ")

        for i in range(2):
            self.thread_flag[i] = False
            print(f"Flag for thread {i} set to False")
            print(f"Thread is alive: {self.thread[i].is_alive()}")
            self.thread[i].join(0)
            print(f"Thread {i} joined")
        
        self.camera_list_1['state'] = 'normal'
        self.camera_list_2['state'] = 'normal'
        
    def start_stop_button_press(self):
        if (self.InitiatedCameras[int(self.camera_list_1.get())].streaming == False or self.InitiatedCameras[int(self.camera_list_2.get())].streaming == False):
            self.stream_webcam()
            
        else:
            self.stop_thread_streaming()
        
    def conenct_IMU_sensors(self):
        if self.imu == None:
            self.imu = IMUManager(int(self.frame_rate_list.get()))

        if self.imu.devices._XdpcHandler__connectedDots:
            self.imu_update_list()
            self.connect_button.state(["disabled"])
            for combo in self.imu_comboboxes:
                combo['values'] = ["None"] + self.imu_tags
                combo['state'] = 'normal'
        else:
            self.imu = None
            
    def disconnect_IMU_sensors(self):
        if not self.imu == None:
            self.imu.disconnect_IMU_sensors()
            self.imu = None
            self.imu_tags.clear()
            self.imu_tags = []
            self.connect_button.state(["!disabled"])
            for labels in self.imu_status_labels:
                labels.destroy()
            self.imu_status_labels = []  

            for lamps in self.imu_status_lamps:
                lamps.destroy()
            self.imu_status_lamps= []

            for labels in self.imu_battery_status_labels:
                labels.destroy()    
            self.imu_battery_status_labels= []

            for combo in self.imu_comboboxes:
                combo["state"] = "disable"
                combo["values"] = "None"

            self.imu_battery_update_button.destroy()
        else:
            print("No IMU sensors are connected")

    def imu_update_list(self):
        for device in range(len(self.imu.devices._XdpcHandler__connectedDots)):
            self.imu_tags.append(self.imu.devices._XdpcHandler__connectedDots[device].deviceTagName())
            batteryLevel = self.imu.devices._XdpcHandler__connectedDots[device].batteryLevel()
            self.create_imu_status_lamp(self.imu_status_frame, self.imu_tags[device], device, 0, batteryLevel)
        self.imu_battery_update_button = ttk.Button(self.imu_status_frame, text="Check Battery", command=self.imu_update_battery_levels)
        self.imu_battery_update_button.grid(row=len(self.imu_tags), column=0, padx=10, pady=10 )
    
    def imu_update_battery_levels(self):
        for labels in range(len(self.imu_battery_status_labels)):
            self.imu_battery_status_labels[labels]["text"] = f"{self.imu.devices._XdpcHandler__connectedDots[labels].batteryLevel()}%"   
        print(1)

    def lock_imu_configuration(self):
        # TODO: Should add a check if IMU is used somewhere else.
        for combo in self.imu_comboboxes:
            combo['state'] = 'disable'
            self.imu_lock_status = True

    
    def unlock_imu_configuration(self):
        for combo in self.imu_comboboxes:
            combo['state'] = 'readonly'
            self.imu_lock_status = False

    def sync_sensors(self):
      self.imu.sync_IMU_sensors()
        
    def start_stop_button_streaming_imu(self):
        if self.imu_streaming:
            self.stop_imu_streaming()
            self.imu_streaming = False
            
        else:
            self.start_imu_streaming()
            self.imu_streaming = True
    
    def stop_imu_streaming(self):
        if self.imu_streaming:
            self.thread_flag[2] = False
            self.imu.stop_measuring_mode()
            self.thread[2].join(0)

            print("IMU THREAD JOINED")
            self.imu_configuration_list.clear()
            self.imu_configuration_list = []
            self.imu_streaming = False
        else:
            print("No IMUs are streaming at the moment")

    def start_imu_streaming(self):
        if self.imu_lock_status:
            self.imu_ordered_configuration = []
            flag = False # Dummy for checking if at least one movella has been selected 
            for combobox in self.imu_comboboxes:
                # We must loop over the combo, read the value of each dropdown menu, 
                # loop over imu.devices and see what idx this device has, and then store this idx in a new list
                value = combobox.get()

                if value == "None":
                    self.imu_ordered_configuration.append(-1) # -1 Means that combobox is set to "None" i.e. no Sensor was set to that part
                else:
                    for idx in range(len(self.imu.devices.connectedDots())):
                        if self.imu.devices.connectedDots()[idx].deviceTagName() == value:
                            self.imu_ordered_configuration.append(idx)
                            flag = True
                            break                    
            
            # If everthing is ok distribute the work to another Thread
            if flag:
                self.imu_streaming = True
                self.initiate_plot()
                self.imu.start_measuring_mode()
                self.thread_flag[2] = True
                self.thread[2] = Thread(target=self.update_imu_plot)
                self.thread[2].start()
              

        else: 
            print("IMU configuration is not locked. Please proceed checking the configuration and lock it brefore start streaming")
    
    def initiate_plot(self):
        self.ax.clear()
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_xlim3d([-2, 2])
        self.ax.set_ylim3d([-2, 2])
        self.ax.set_zlim3d([-2, 2])
        self.ax.set_autoscale_on(False)

        #change accordingy the drawing of a human pose
        self.unit_vector = np.array([1, 0, 0])
        self.quiver1 = self.ax.quiver(0, 0, 0, self.unit_vector[0], self.unit_vector[1], self.unit_vector[2], color='blue', label='Vector direction')
        self.quiver2 = self.ax.quiver(self.unit_vector[0], self.unit_vector[1], self.unit_vector[2], self.unit_vector[0], self.unit_vector[1], self.unit_vector[2], color='red', label='Vector direction')
        self.canvas.draw() 

    # TODO: More work is needed here
    def update_imu_plot(self):
        while self.thread_flag[2]:
            self.imu.get_measurments()
            quaternions = self.imu.quat_data

            #expand
            q1 = quaternions[0,:]
            rotmatrix = self.get_rotation_matrix_quaternions(q1)
            rotatedVector = np.dot(rotmatrix, self.unit_vector)

            self.quiver1.remove()
            self.quiver1 = self.ax.quiver(0, 0, 0, rotatedVector[0], rotatedVector[1], rotatedVector[2], color='blue', label='Vector direction')
            self.canvas.draw() 

    #Rotation matrix from quaternions as input
    def get_rotation_matrix_quaternions(self, qVector): # returns the rotation matrix from qunernions as input
        q0 = qVector[0]
        q1 = qVector[1]
        q2 = qVector[2]
        q3 = qVector[3]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        return np.array([[r00, r01, r02],
                        [r10, r11, r12],
                        [r20, r21, r22]])
    




# Run the application
if __name__ == "__main__":
    app = IMURecordingStudio()
    
    app.mainloop()