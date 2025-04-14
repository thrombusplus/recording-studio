import tkinter as tk
from tkinter import ttk, messagebox
from tkinter import filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time
from threading import Thread
import threading
from IPython import get_ipython
import numpy as np
import copy
import os
from queue import Queue
from src.utils.cameramanager import  CameraManager
from src.utils import *


# Main application class for IMU Recording Studio
class IMURecordingStudio(tk.Tk):
    def __init__(self):
        super().__init__()

        # Global variables
        # Constants
        self.MAXCAMERAS = DEFAULT_SETTINGS.max_cameras() # Maximum number of cameras
        self.EXERCISES = DEFAULT_SETTINGS.exercises_list() # List with exercises

        # Variables
        self.imu_status_lamps = [] # List of IMU lamps
        self.imu_status_labels = [] # List of IMU labels
        self.imu_battery_status_labels = [] # List of labels with status labels
        self.imu_tags = [] # List With the tags from each IMU
        self.camera_lamps = [] # List of camera lamps
        self.camera_labels = [] # List of camera labels
        self.InitiatedCameras = [] # List of initiated cameras
        self.idxInitiatedCameras = [] # List of indexes of initiated cameras
        self.imu = None # Stores the class for IMU sensors
        self.imu_comboboxes = [] # List that stores the comboboxes for imus, one for each leg component.
        self.imu_combobox_labels = [] # List that stores the labels of the imu_comboboxes
        self.camera_list_1_previous_Value = None # List of previous values of Combobox 1
        self.camera_list_2_previous_Value = None # List of previous values of Combobox 2
        
        self.imu_lock_status = False # Flag for storing if IMU configuration is locked (checked)
        self.imu_streaming = False # Flag for storing if IMUs are streaming in the IMU vector view
        self.imu_configuration_list = [] # List for storing the configuration list with IMUs
        self.imu_ordered_configuration = [] # List that stores the correct order of the IMUs based on the configuration in the settings
        self.reset_heading_flag = False # Flag to perform reset heading
        self.recording = False # Flag to check if recording is active
        

        self.data_queue = Queue() # Create a queue
        self.use_queue = False

        
        # Thread explanation: #Thread[0] and Thread[1] are used by webcams to stream upon pressing the Start/Stop button
        # Thread[2] will be used when Run/Stop Streaming button is pressed to stream IMU data in the IMU vector view.
        # When Start Recording Button is pressed -> Thread[0] will be used to Collect data from all sensors, Thread[1] will be used to stream webcam 1, 
        # Thread[2] will be used to stream webcam 2, Thread[3] will be used to plot IMU data

        self.thread = [None, None, None, None, None] # Stores threads
        self.thread_flag = [False, False, False, False, False] # Flag for stopping thread


        
       
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
        
        self.tabControl.pack(expand=1, fill='both')
        
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
        # Crearte the imu control buttons
        self.create_imu_control_button(IMU_control_frame)
        # Create the main frunctionality buttons
        self.create_functionality_frame_buttons(functionality_frame)

    def create_functionality_frame_buttons(self, frame):
        # Save directory label
        save_directory_label=ttk.Label(frame, text='Saving directory:')
        save_directory_label.grid(row=0, column=0, columnspan=1, padx=10, pady=10, sticky='w')
        # Save directory field
        self.save_directory_field=ttk.Label(frame, text='', wraplength=150)
        self.save_directory_field.grid(row=0, column=2, columnspan=1, padx=10, pady=10, sticky='w')
        # Save direcoty selection button
        save_directory_button=ttk.Button(frame, text='Save to...', command=self.update_saving_directory)
        save_directory_button.grid(row=1, column=2, columnspan=1, padx=10, pady=10, sticky='w')

        # Patient's ID label
        patients_id_label=ttk.Label(frame, text='Patient ID:')
        patients_id_label.grid(row=2, column=0, columnspan=1, padx=10, pady=10, sticky='w')
        # Patient's ID Field
        self.patients_id_field=ttk.Entry(frame, validate='focusout', validatecommand=self.check_patient_id)
        self.patients_id_field.grid(row=2, column=2, columnspan=1, padx=10, pady=10, sticky='w')
        # Patient's ID report label
        self.patient_id_check_label=ttk.Label(frame, text='')
        self.patient_id_check_label.grid(row=2, column=3, columnspan=1, padx=10, pady=10, sticky='w')

        # Exercise List Label
        exercise_list_label=ttk.Label(frame, text='Exercise:')
        exercise_list_label.grid(row=3, column=0, columnspan=1, padx=10, pady=10, sticky='w')
        # List for selectring the exercises - for metadata.
        self.exercises_list=ttk.Combobox(frame, text ='', state='readonly' , values=self.EXERCISES,)
        self.exercises_list.grid(row=3, column=2, rowspan=1, padx=10, pady=10, sticky='w')

        # Create Start/ Stop buttons in functionality_frame
        start_recording=ttk.Button(frame, text="Start Recording", command = self.start_recording )
        start_recording.grid(row=4, column=0, columnspan=1, padx=10, pady=10, sticky='w')
        stop_recording=ttk.Button(frame, text="Stop Recording", command= self.stop_recording)
        stop_recording.grid(row=4, column=2, columnspan=1, padx=10, pady=10, sticky='w')

    
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
        # start_stop_imu_button.place(x=150)

        imu_reset_heading_button =ttk.Button(frame, text="Reset Heading", command=self.reset_heading)
        imu_reset_heading_button.grid(row=0,column=0, columnspan=1 )

        self.imu_pose_selection=ttk.Combobox(frame, state="readonly", values = ["Sitting","Laying"])
        self.imu_pose_selection.set("Sitting")
        self.imu_pose_selection.grid(row=0,column=3, columnspan=1 )
    

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
            self.imu_combobox_labels.append(imu_label)

            imu_combo= ttk.Combobox(self.imu_configuration_frame, state="disabled", values=["None"])
            imu_combo.grid(row=imu+3, column=1, columnspan=1)
            imu_combo.current(0)
            self.imu_comboboxes.append(imu_combo)
        
        # imu_reset_heading_button =ttk.Button(self.imu_configuration_frame, text="Reset Heading", command=self.reset_heading)
        # imu_reset_heading_button.grid(row=imu+4,column=0, columnspan=1 )

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

        # store each lamp in a dictionary for future modification
        self.camera_lamps.append(lamp)

    def update_saving_directory(self):
        self.saving_directory = filedialog.askdirectory()
        self.save_directory_field['text'] = self.saving_directory

    def check_patient_id(self):
        if self.patients_id_field['text'] !='':
            if self.save_directory_field['text'] == '':
                self.patient_id_check_label['text'] = 'Select a saving dir!'
                self.patient_id_check_label['background'] = 'red'
                self.patients_id_field.delete(0,'end')
            else:
                path = os.path.join(self.save_directory_field['text'], self.patients_id_field.get())
                try:
                    os.makedirs(path, exist_ok=False)
                    self.patient_id_check_label['background'] = 'green' 
                    self.patient_id_check_label['text'] = 'Folder created!'
                except Exception as e:
                    self.patient_id_check_label['text'] = 'ID already in use!'
                    self.patient_id_check_label['background'] = 'orange'
                    print(f'This patient ID already exists. Error: {e}')
        return True


    def connect_webcams(self):
        # set camera_lamps to OFF - in case of reconnection
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

    # Threads for handling camera 1 and camera 2
    def camera1_thread(self):
        camera_manager_1 = CameraManager(camera_num=1)

        flag_index =1 if self.recording else 0

        while self.thread_flag[flag_index]:
            try:
                frame_1 = camera_manager_1.get_frame()
                self.latest_frame_cam1 = frame_1
                
                #Check if camera is already on recording mode
                if self.recording:
                    pass

            except Exception as e :
                print (f"[Camera Thread] Error: {e}")
                self.latest_frame_cam1 = None

            time.sleep(0.03)

    def camera2_thread (self):
        camera_manager_2 = CameraManager(camera_num=2)

        flag_index =2 if self.recording else 1


        while self.thread_flag[flag_index]:
            try:
                frame_2 = camera_manager_2.get_frame()
                self.latest_frame_cam2 = frame_2  

                if self.recording:
                    pass

            except Exception as e:
                print(f"[Camera2 Thread] Error: {e}") 
                self.latest_frame_cam2 = None  

            time.sleep(0.03)   


    def stream_webcam(self): # No very elegant, maybe can be converted to a for loop
        if self.recording:
            print("Cannot start webcam streaming while recording.")
            return
        if self.InitiatedCameras==[]:
            print("No cameras connected")
            return
        
        cam1_index = self.camera_list_1.get()
        cam2_index = self.camera_list_2.get()

        if cam1_index == ''or cam2_index=='':
            print("Please se;ect both camera views.")
            return
        if cam1_index == cam2_index:
           print("Viewers have the same camera. Please select different cameras.")
           return

        self.InitiatedCameras[int(cam1_index)].streaming = True
        self.InitiatedCameras[int(cam2_index)].streaming = True

    # Thread 0: Camera 1
        if self.thread[0] is None or not self.thread[0].is_alive():
           self.thread_flag[0] = True
           self.thread[0] = Thread(target=self.camera1_thread)
           self.thread[0].start()
           print(f"Thread 0 (Camera {cam1_index}) started.")

    # Thread 1: Camera 2
        if self.thread[1] is None or not self.thread[1].is_alive():
           self.thread_flag[1] = True
           self.thread[1] = Thread(target=self.camera2_thread)
           self.thread[1].start()
           print(f"Thread 1 (Camera {cam2_index}) started.")

    #Start view update threads as detached
        Thread(target=self.update_view, args=(0, cam1_index, self.ax1, self.canvas1)).start()
        Thread(target=self.update_view, args=(1, cam2_index, self.ax2, self.canvas2)).start()

    # Lock comboboxes
        self.camera_list_1['state'] = 'disabled'
        self.camera_list_2['state'] = 'disabled'

        print("Camera streaming started.")
        

    def update_view(self, threadNum, cameranumber, cameraview_axis, cameraview_canvas):
        #get frame from camera
        while self.thread_flag[threadNum] and self.InitiatedCameras[int(cameranumber)].streaming:# update axis
           try:
            if threadNum == 0 : # camera 1
               camera_frame = self.latest_frame_cam1
            if threadNum == 1: # camera2
                 camera_frame = self.latest_frame_cam2

           # camera_frame = self.InitiatedCameras[int(cameranumber)].get_frame()
            if camera_frame is not None:
                cameraview_axis.clear()
                cameraview_axis.imshow(camera_frame)
                cameraview_axis.axis('off')
                cameraview_canvas.draw()                
            # time.sleep(0.001)
            # print("Running")

            #if not self.thread_flag[threadNum]:
            #self.InitiatedCameras[int(cameranumber)].streaming = False
        # print('Run')
           except Exception as e:
               print(f"[Thread{threadNum}] Error while updating view: {e}")
               
        print(f"Stoped reading from camera {cameranumber}")
        
        
    def start_stop_button_press(self):

        if self.InitiatedCameras == []:
           print("No cameras connected.")
           return
        if self.InitiatedCameras[int(self.camera_list_1.get())].streaming:
           self.stop_streaming_mode()
           self.InitiatedCameras[int(self.camera_list_1.get())].streaming = False
           self.InitiatedCameras[int(self.camera_list_2.get())].streaming = False
           self.camera_list_1['state'] = 'readonly'
           self.camera_list_2['state'] = 'readonly'
           print("Streaming stopped.")
        else:
           self.stream_webcam()
           self.InitiatedCameras[int(self.camera_list_1.get())].streaming = True
           self.InitiatedCameras[int(self.camera_list_2.get())].streaming = True
           self.camera_list_1['state'] = 'disabled'
           self.camera_list_2['state'] = 'disabled'
           print("Streaming started.")
        
        
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
        selected_values = []
        for combobox in self.imu_comboboxes:
            value = combobox.get()
            if value != "None":
                if value in selected_values:
                    messagebox.showwarning(
                    "Warning",
                    f"IMU {value} is already in use. Please select another IMU sensor."
                    )
                    return  # Stop execution if an IMU is already in use
                selected_values.append(value)

    # If no conflicts, lock configuration
        for combobox in self.imu_comboboxes:
            combobox['state'] = 'disable'
            self.imu_lock_status = True
            print("IMU configuration locked successfully.")

    def unlock_imu_configuration(self):
        for combo in self.imu_comboboxes:
            combo['state'] = 'readonly'
            self.imu_lock_status = False

    def sync_sensors(self):
      self.imu.sync_IMU_sensors
     
    def start_stop_button_streaming_imu(self):
        if self.imu_streaming:
            self.stop_imu_streaming()
            # self.imu_streaming = False
            
        else:
            self.start_imu_streaming()
            # self.imu_streaming = True

    
    # Stop streaming for camera1 and camera 2
    def stop_streaming_mode(self):
        print("Stopping streaming mode...")
        for i in [0,1]:
            self.thread_flag[i] = False
            if self.thread[i] is not None:
               self.thread[i].join()
               self.thread[i] = None
               print(f"Thread {i} stopped.")

        self.camera_list_1['state']= 'readonly'
        self.camera_list_2['state']='readonly'

        for cam in self.InitiatedCameras:
            cam.streaming=False

    def stop_threads(self):
        for i in range(4):
            self.thread_flag[i] = False
            if self.thread[i] is not None:
               self.thread[i].join()
               self.thread[i] = None
               print(f"Thread {i} stopped.")

    
    def start_recording(self):
        if self.recording:
           print("Recording already in progress.")
           return
        
        # Add more validations
        #....
        #....
        #....

        if not self.imu_lock_status:
           print("Configuration must be locked before starting recording.")
           return

        print("Preparing to start recording...")
        self.stop_threads()
         #Clear queue
        while not self.data_queue.empty():
            try:
               self.data_queue.get_nowait()
            except Exception as e:
                print(f"Error while clearing queue: {e}")
                break
            
        #Reset IMU heading    
        try:
            self.imu.reset_heading()
            print("IMU heading reset successfully.")
        except Exception as e:
            print(f"Error while resetting IMU heading: {e}")

            
        self.initiate_plot() #reset plot to initial position
        print("Waiting for IMUs to stabilize after heading reset...")
        time.sleep(4)

        for _ in range(5):
            try :
                self.imu.get_measurments()
                time.sleep(0.05)
            except Exception as e:
                print(f"Error after heading reset: {e}")

        for i in range(5,-1,-1):
            print(f"Recording starts in: {i}...")
            time.sleep(1)
        
        self.start_imu_streaming()

        #prepare flags and threads
        self.use_queue = True
        self.recording = True
        
        self.thread_flag[0] = True
        self.thread[0] =  Thread(target=self.data_collection_thread)

        self.thread_flag[1] = True
        self.thread[1]= Thread(target=self.camera1_thread)

        self.thread_flag[2] = True
        self.thread[2]=Thread(target=self.camera2_thread)

        self.thread_flag[3]= True
        self.thread[3] = Thread(target=self.update_imu_plot)

        for i in range(4):
            self.thread[i].start()
            print(f"Thread{i} started.")
        

        print("Recording Started")

        
    def stop_recording(self):
        if not self.recording:
            print("Recording stopped")
            return
        
        self.recording = False
        self.stop_threads()
        
        print("Recording stopped")


    def data_collection_thread(self):

        while self.thread_flag[0]:
            data_entry ={}
            timestamp = time.time()

            #Data from sensors
            try:
                if not self.imu_streaming:
                    print(("IMU not streaming"))
                    data_entry ['imu']= None
                else:
                   self.imu.get_measurments()
                   data_entry['imu']= self.imu.quat_data.copy()
            except Exception as e:
                print(f"[IMU] Error while collecting data: {e}")
                data_entry['imu'] = None

            #Data from camera 1
            try: 
                if self.latest_frame_cam1 is not None:
                    data_entry['frame_cam1'] = self.latest_frame_cam1.copy()
                else:
                    data_entry['frame_cam1']= None
            except Exception as e:
                print("f[Camera 1] Error during data save: {e}")
                data_entry['frame_cam1'] = None
           
           
            # Data from camera 2
            try :
                if self.latest_frame_cam2 is not None:
                    data_entry['frame_cam2'] =  self.latest_frame_cam2.copy()
                else:
                    data_entry['frame_cam2'] = None
            except Exception as e:
                print(f"[Camera 2] Error during data save: {e}")
                data_entry['frame_cam2'] = None
                
            data_entry['timestamp'] = timestamp
            
            if self.recording:  
                 
            #save data
              if self.use_queue:
                self.data_queue.put(data_entry)
                print("Data added to queue")

            time.sleep(0.050)

    
    """def update_all_figures(self):
        while self.thread_flag[3]:
           
           try:
              
            # Update figure camera 1
              if self.latest_frame_cam1 is not None:
                  self.update_canvas (self.ax1, self.canvas1, self.latest_frame_cam1)

            # update figure camera 2
              if self.latest_frame_cam2 is not None:
                 self.update_canvas (self.ax2, self.canvas2, self.latest_frame_cam2)  
            
              time.sleep(0.05)         
           except Exception as e:       
                 print(f"[Thread 3] Error during figure update: {e}")
                 break"""

    def stop_imu_streaming(self):
        if self.imu_streaming:
           self.thread_flag[2] = False
           self.imu.stop_measuring_mode()

           if self.thread[2] is not None:
               self.thread[2].join()
               self.thread[2]= None

               print("IMU sreaming stopped")
               self.imu_streaming = False
        else:
            print("No IMUs are streaming at the moment")

        
    def start_imu_streaming(self):
        if self.imu_streaming:
            print("IMU is already streaming.")

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

                flag_index = 3 if self.recording else 2
                self.thread_flag[flag_index] =True
                self.thread[flag_index]= Thread(target=self.update_imu_plot)
                self.thread[flag_index].start()

                print("IMU streaming  started")
                     
            else: 
                print("IMU configuration is not locked. Please proceed checking the configuration and lock it brefore start streaming")
    
    def initiate_plot(self):
        self.ax.clear()
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_xlim3d([-1, 6])
        self.ax.set_ylim3d([-3.5, 3.5])
        self.ax.set_zlim3d([-3.5, 3.5])
        self.ax.set_autoscale_on(False)
        self.get_pose()
        

    # TODO: More work is needed here
    def update_imu_plot(self):
      self.new_joints = copy.deepcopy(self.joints)

      while True:
        # Βρες το index του thread που τρέχει αυτή τη στιγμή
        current_thread_index = 3 if self.recording else 2

        if not self.thread_flag[current_thread_index]:
            print("[IMU Thread] Exit signal received.")
            break

        try:
            self.imu.get_measurments()
            quaternions = self.imu.quat_data

            for legSegment in range(len(self.imu_comboboxes)):
                deviceIdx = self.imu_ordered_configuration[legSegment]
                if deviceIdx != -1:
                    q = np.round(quaternions[deviceIdx, :], 4)
                    print(q)
                    rotationMatrix = self.get_rotation_matrix_quaternions(q)
                    self.rotate_leg_segment(self.imu_combobox_labels[legSegment]['text'][:-1], rotationMatrix)

            self.ax.clear()
            self.ax.set_xlabel("X")
            self.ax.set_ylabel("Y")
            self.ax.set_zlabel("Z")
            self.ax.set_xlim3d([-1, 6])
            self.ax.set_ylim3d([-3.5, 3.5])
            self.ax.set_zlim3d([-3.5, 3.5])
            self.ax.set_autoscale_on(False)
            DEFAULT_SETTINGS.plot_body_parts(self.ax, self.new_joints)
            # self.quiver = self.ax.quiver(self.joints['Left Hip'][0], self.joints['Left Hip'][1], self.joints['Left Hip'][2], self.new_joints['Left Knee'][0], self.new_joints['Left Knee'][
            # 1], self.new_joints['Left Knee'][2], color='red', label='Vector direction')
            self.canvas.draw()

            if self.reset_heading_flag:
                self.imu.reset_heading()
                self.reset_heading_flag = False

            time.sleep(0.05)

        except Exception as e:
            print(f"[IMU plot] Error: {e}")
            break

    #Rotation matrix from quaternions as input
    def get_rotation_matrix_quaternions(self, qVector): # returns the rotation matrix from qunternions as input
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
    
    def get_pose(self):
        if self.imu_pose_selection.get() == 'Sitting':
            self.joints = DEFAULT_SETTINGS.skeleton_pose_sitting_joints()
        else:
            self.joints = DEFAULT_SETTINGS.skeleton_pose_laying_joints()
        DEFAULT_SETTINGS.plot_body_parts(self.ax, self.joints)
        self.canvas.draw() 
        
    def rotate_leg_segment(self, legSegment, rotationMatrix):
      #Function to move the entire chain of joints by a displacement 
      def move_chain(joint_names, displacement):
        #   for joint in joint_names:
        self.new_joints[joint_names] += displacement   #update the position of each joint 

      # Left leg segments
      # Left Thight rotation
      if legSegment == 'Left Thigh':
         temp_knee = copy.deepcopy(self.new_joints['Left Knee'])  # Store the current position 
         start = self.joints['Left Hip']  #start point is the left hip
         stop = self.joints['Left Knee'] - start  #direction from hip to knee
         norm = stop / np.linalg.norm(stop)  # Normalize the direction vector
         rotated = np.dot(rotationMatrix, norm)  # Apply the rotation matrix
         
         if not np.isnan(rotated).any(): 
           self.new_joints['Left Knee'] = rotated * np.linalg.norm(stop) + start  # scale the rotated vetcor to the original length
           displacement = self.new_joints['Left Knee'] - temp_knee  #update new knee position
           if self.imu_comboboxes[1].get() == "None":
             move_chain('Left Ankle', displacement)
           if self.imu_comboboxes[2].get() == "None":
             move_chain('Left Toes', displacement)  

      # Rotate the left calf segment 
      elif legSegment == 'Left Calf':
        temp_ankle = copy.deepcopy(self.new_joints['Left Ankle'])
        start = self.joints['Left Knee'] 
        stop = self.joints['Left Ankle'] - start # direction from knee to ankle
        norm = stop / np.linalg.norm(stop)
        rotated = np.dot(rotationMatrix, norm)

        if not np.isnan(rotated).any():
            self.new_joints['Left Ankle'] = rotated * np.linalg.norm(stop) + self.new_joints['Left Knee']
            displacement = self.new_joints['Left Ankle'] - temp_ankle 
            if self.imu_comboboxes[2].get() == "None":
             move_chain('Left Toes', displacement) 

      # Rotate the left foot segment
      elif legSegment == 'Left Foot':
        start = self.joints['Left Ankle']
        stop = self.joints['Left Toes'] - start
        norm = stop / np.linalg.norm(stop)
        rotated = np.dot(rotationMatrix, norm)
        if not np.isnan(rotated).any():
            self.new_joints['Left Toes'] = rotated * np.linalg.norm(stop) + self.new_joints['Left Ankle']            
      
      # Right leg segments rotation
      # Right Thight rotation
      if legSegment == 'Right Thigh':
         temp_knee = copy.deepcopy(self.new_joints['Right Knee'])  # Store the current position 
         start = self.joints['Right Hip']  #start point is the left hip
         stop = self.joints['Right Knee'] - start  #direction from hip to knee
         norm = stop / np.linalg.norm(stop)  # Normalize the direction vector
         rotated = np.dot(rotationMatrix, norm)  # Apply the rotation matrix
         
         if not np.isnan(rotated).any(): 
           self.new_joints['Right Knee'] = rotated * np.linalg.norm(stop) + start  # scale the rotated vetcor to the original length
           displacement = self.new_joints['Right Knee'] - temp_knee  #update new knee position
           if self.imu_comboboxes[4].get() == "None":
             move_chain('Right Ankle', displacement)
           if self.imu_comboboxes[5].get() == "None":
             move_chain('Right Toes', displacement) 

      # Rotate the left calf segment 
      elif legSegment == 'Right Calf':
        temp_ankle = copy.deepcopy(self.new_joints['Right Ankle'])
        start = self.joints['Right Knee'] 
        stop = self.joints['Right Ankle'] - start # direction from knee to ankle
        norm = stop / np.linalg.norm(stop)
        rotated = np.dot(rotationMatrix, norm)
        if not np.isnan(rotated).any():
            self.new_joints['Right Ankle'] = rotated * np.linalg.norm(stop) + self.new_joints['Right Knee']
            displacement = self.new_joints['Right Ankle'] - temp_ankle 
            if self.imu_comboboxes[5].get() == "None":
             move_chain('Right Toes', displacement) 
        
      # Rotate the left foot segment
      elif legSegment == 'Right Foot':
        start = self.joints['Right Ankle']
        stop = self.joints['Right Toes'] - start
        norm = stop / np.linalg.norm(stop)
        rotated = np.dot(rotationMatrix, norm)
        if not np.isnan(rotated).any():
            self.new_joints['Right Toes'] = rotated * np.linalg.norm(stop) + self.new_joints['Right Ankle']

    def reset_heading(self):
        # self.imu.reset_heading()  
        self.reset_heading_flag = True

# Run the application
if __name__ == "__main__":
    app = IMURecordingStudio()
    
    app.mainloop()
    