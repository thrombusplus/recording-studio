import tkinter as tk
from tkinter import ttk, messagebox
from tkinter import filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg,NavigationToolbar2Tk
from matplotlib.ticker import MaxNLocator
import time
from threading import Thread
import threading
from PIL import Image, ImageTk
import ctypes
import numpy as np
import pandas as pd
import copy
import os
import cv2
import logging, sys
from queue import Queue
from src.utils.cameramanager import  CameraManager
from src.utils import *
from src.utils.logger import get_logger
from src.utils.gui_terminal import LogTab
from src.utils.websocket_server import WebSocketServer
import json



logger = get_logger(__name__)


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
        
        self.imu_lock_status = False # Flag for storing if IMU configuration is locked (checked)
        self.imu_streaming = False # Flag for storing if IMUs are streaming in the IMU vector view
        self.imu_configuration_list = [] # List for storing the configuration list with IMUs
        self.imu_ordered_configuration = [] # List that stores the correct order of the IMUs based on the configuration in the settings
        self.reset_heading_flag = False # Flag to perform reset heading
        self.recording = False # Flag to check if recording is active
        
        self.latest_frame_cam1 =None
        self.latest_frame_cam2 = None
        self.t_cam1 = None
        self.t_cam2 = None

        self.data_queue = Queue() # Create a queue
        self.use_queue = False

        self.frame_scroll_active = False
        # Annotation state
        self.annotations = []           # list of annotation dicts
        self.viz_annotations = []       # list of annotation dicts (Visualization tab preview only)
        self.segment_pending_start = None  # dict with start info while segment is open

        self.save_path = ""         # Directory for saving
        self.load_path = ""         # Directory for loading (visualization)

        # WebSocket server for live IMU data streaming
        self.websocket_server = None
        self.websocket_enabled = False
        self.websocket_port = 8185

        self.is_playing = False
        self.play_speed_ms = 20
        self.play_step = 1

        # Fast playback (blit-based IMU cursor + PIL camera labels)
        self._imu_vlines = []   # list of (ax, vline) tuples for cursor blit
        self._imu_bg = None     # saved figure background for blit
        self.cam3_photo = None  # ImageTk reference for Front View label
        self.cam4_photo = None  # ImageTk reference for Side View label

        # Thread explanation:
        # Thread[0] and Thread[1]: webcam capture loops (streaming and recording modes)
        # Thread[2] (streaming mode): acquires IMU packets, renders the 3D pose
        #   preview, and relays data to WebSocket clients (if any)
        # When the Start Recording button is pressed:
        #  - Thread[0] and Thread[1]: event-triggered frame grabs, paced by Thread[5]
        #  - Thread[3]: 3D pose preview + WebSocket relay (display only, off the data path)
        #  - Thread[4]: camera figure updates in the GUI (display only)
        #  - Thread[5]: paces the acquisition cycle -- polls IMU packets (up to six
        #    sensors), timestamps, triggers camera grabs, queues snapshots
        #  -  [Thread_6] (_consumer_thread): drains the work queue, assembles data entries into
        #    data_queue; disk writing happens in FileManager.save_recording() on stop


        self.thread = [None, None, None, None, None, None] # Stores threads
        self.thread_flag = [False, False, False, False, False, False] # Flag for stopping thread

        # Barrier to synchronize grab() calls across both camera threads.
        # Forces both hardware captures to fire within microseconds of each other.
        self.capture_event = threading.Event()
        self._grab_barrier = threading.Barrier(3)


        self.title("IMU Recording Studio")
        self.geometry("850x900")
        
        # Initate a Tab control
        self.tabControl = ttk.Notebook(self)
        
        def closeWindow():
            self.disconnect_IMU_sensors()
            self.disconnect_webcams()
            if self.websocket_server and self.websocket_server.is_running():
                self.websocket_server.stop_server()
            self.destroy()
            logger.info("EXIT")

        self.protocol('WM_DELETE_WINDOW', closeWindow)

        # Initiate Main tab
        self.main_tab = ttk.Frame(self.tabControl)
        self.tabControl.add(self.main_tab, text='Main')
        
        self.main_tab.rowconfigure(0, weight=2)
        self.main_tab.rowconfigure(1, weight=1)
        self.main_tab.rowconfigure(2, weight=2)
        self.main_tab.columnconfigure(0, weight=1)
        self.main_tab.columnconfigure(1, weight=1)
        
        # Initiate Settings tab
        self.settings_tab = ttk.Frame(self.tabControl)
        self.tabControl.add(self.settings_tab, text='Settings')
        self.tabControl.pack(expand=1, fill="both")
        
        # Initiate the Visialization tab
        self.visualization_tab = ttk.Frame(self.tabControl)
        self.tabControl.add(self.visualization_tab, text='Visualization')

        # Initiate the Annotations tab
        self.annotation_tab = ttk.Frame(self.tabControl)
        self.tabControl.add(self.annotation_tab, text='Annotations')

        self.visualization_tab.grid_rowconfigure(0, weight=2)
        self.visualization_tab.grid_rowconfigure(1, weight=2)  
        self.visualization_tab.grid_rowconfigure(2, weight=2)  
        self.visualization_tab.grid_columnconfigure(0, weight=2)  
        self.visualization_tab.grid_columnconfigure(1, weight=2)
        self.visualization_tab.grid_columnconfigure(2, weight=2)
        self.visualization_tab.grid_columnconfigure(3, weight=2)
          
        # Setup the main tab (IMU Vector and Camera Views)
        self.create_main_tab()
        
        # Setup the settings tab (IMU Sensor Status)
        self.create_settings_tab()

        # Setup the Record Visualization tab
        self.create_visualization_tab()

        # Setup the Annotations tab
        self.create_annotation_tab()

    def create_main_tab(self):
        # Camera Views Frame
        camera_frame = ttk.LabelFrame(self.main_tab,  text="Camera Views")
        camera_frame.grid(row=0, column=0, columnspan=2, padx=10, pady=10, sticky="nsew")
        
        # Camera Control Frame
        camera_control_frame = ttk.LabelFrame(self.main_tab, text="")
        camera_control_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")
         

         # IMU View and Controls Container 
        imu_container = ttk.Frame(self.main_tab)
        imu_container.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")

        # IMU Vector View Frame
        imu_frame = ttk.LabelFrame(imu_container, text="IMU Vector View")
        imu_frame.grid(row=0, column=0, sticky="nsew")

        # IMU Vector Control Frame
        IMU_control_frame = ttk.LabelFrame(imu_container)
        IMU_control_frame.grid(row=1, column=0, pady=(5, 0), sticky="ew")

        imu_container.rowconfigure(0, weight=1)
        imu_container.columnconfigure(0, weight=1)

        # Functionality Buttons Frame
        functionality_frame = ttk.LabelFrame(self.main_tab, text="Functionality Buttons")
        functionality_frame.grid(row=2, column=1, padx=10, pady=10, sticky="nsew")
        

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
        self.save_directory_var =tk.StringVar()
        self.save_directory_field=ttk.Label(frame, textvariable =self.save_directory_var, wraplength=150)
        self.save_directory_field.grid(row=0, column=2, columnspan=1, padx=10, pady=10, sticky='w')
        # Save direcoty selection button
        save_directory_button=ttk.Button(frame, text='Save to...', command=lambda: Thread(target=self.update_saving_directory, daemon=True).start())
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

        # Combobox for exercises
        self.exercises_list = ttk.Combobox(frame, state='readonly')
        self.exercises_list.grid(row=3, column=2, padx=10, pady=10, sticky='w')
        self.exercises_list.bind("<<ComboboxSelected>>", self.update_subcategories_list)

        # Label ID
        subcategory_label = ttk.Label(frame, text='Label ID:')
        subcategory_label.grid(row=4, column=0, padx=10, pady=10, sticky='w')

        # Combobox for Label ID
        self.sub_exercises_list = ttk.Combobox(frame, state='readonly')
        self.sub_exercises_list.grid(row=4, column=2, padx=10, pady=10, sticky='w')

        # Create Start/ Stop buttons in functionality_frame
        start_recording=ttk.Button(frame, text="Start Recording", command = self.start_recording )
        start_recording.grid(row=5, column=0, columnspan=1, padx=10, pady=10, sticky='w')
        stop_recording=ttk.Button(frame, text="Stop Recording", command= self.stop_recording)
        stop_recording.grid(row=5, column=2, columnspan=1, padx=10, pady=10, sticky='w')

        #Create a label sto shop if in recording/ready mode
        self.exercise_list_label=ttk.Label(frame, font= 10, text='Ready for recording', background= 'yellow' )
        self.exercise_list_label.grid(row=6, column=0, columnspan=1, padx=10, pady=10, sticky='w')

    
    def create_camera_views(self, frame):
        # Camera 1 View (using Matplotlib as placeholder)
        self.fig1, self.ax1 = plt.subplots(figsize=(4, 3))
        self.ax1.set_title("Front View")
        self.ax1.axis('off')
        self.canvas1 = FigureCanvasTkAgg(self.fig1, master=frame)
        self.canvas1.draw()
        self.canvas1.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        
        # Camera 2 View (using Matplotlib as placeholder)
        self.fig2, self.ax2 = plt.subplots(figsize=(4, 3))
        self.ax2.set_title("Side View")
        self.ax2.axis('off')
        self.canvas2 = FigureCanvasTkAgg(self.fig2, master=frame)
        self.canvas2.draw()
        self.canvas2.get_tk_widget().grid(row=0, column=2, sticky="nsew")

        # Enable dynamic resizing
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(2, weight=1)
        frame.rowconfigure(0, weight=1)

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
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

    def create_imu_control_button(self, frame):
        start_stop_imu_button = ttk.Button(frame, text="Start/Stop streaming", command=self.start_stop_button_streaming_imu)
        start_stop_imu_button.grid(row=0, column=1, columnspan=1)

        imu_reset_heading_button =ttk.Button(frame, text="Reset Heading", command=self.reset_heading)
        imu_reset_heading_button.grid(row=0,column=0, columnspan=1)

        self.imu_pose_selection=ttk.Combobox(frame, state="readonly", values = ["Sitting","Lying","Standing"])
        self.imu_pose_selection.set("Sitting")
        self.imu_pose_selection.grid(row=0,column=3, columnspan=1 )
        self.imu_pose_selection.bind("<<ComboboxSelected>>", self.update_exercises_list)

        # self.imu_pose_selection=ttk.Combobox(frame, state="readonly", values = ["Sitting","Laying"])
        # self.imu_pose_selection.set("Sitting")
        # self.imu_pose_selection.grid(row=0,column=3, columnspan=1 )

        imu_calibrate_button =ttk.Button(frame, text="Calibrate Data", command=self.get_calibration_data)
        imu_calibrate_button.grid(row=0,column=4, columnspan=1)

        self.show_calibrated_checkbox_var = tk.BooleanVar()
        self.show_calibrated_checkbox_var.set(False)

        self.show_calibrated_data_tickbox = ttk.Checkbutton(frame, text="Show Calibrated", onvalue=True, offvalue=False, variable=self.show_calibrated_checkbox_var)
        self.show_calibrated_data_tickbox.grid(row=0, column=5, columnspan=1)
    

    def create_settings_tab(self):

        # IMU Sensor Status Frame
        self.imu_control_frame = ttk.LabelFrame(self.settings_tab, text="IMU Sensor Control")
        self.imu_control_frame.grid(row=0, column=0, padx=10, pady=10, sticky='nsew')

        # Connect Sensors Button
        self.frame_rate_list= ttk.Combobox(self.imu_control_frame, state="normal", values=["60", "30", "20", "15", "12", "10", "4", "1"])
        self.frame_rate_list.grid(row=0, column=0, columnspan=1)
        self.frame_rate_list.current(0)

        self.connect_button = ttk.Button(self.imu_control_frame, text="Connect Sensors", command=lambda: Thread(target=self.conenct_IMU_sensors, daemon=True).start())
        self.connect_button.grid(row=1, column=0, columnspan=2, pady=10, sticky='nsew')
        
        self.discoconnect_button = ttk.Button(self.imu_control_frame, text="Disconnect Sensors", command=self.disconnect_IMU_sensors)
        self.discoconnect_button.grid(row=2, column=0, columnspan=2, pady=10, sticky='nsew')
        
        self.sync_button = ttk.Button(self.imu_control_frame, text="Sync Sensors", command= lambda: Thread(target=self.sync_sensors,daemon =True).start())
        self.sync_button.grid(row=3, column=0, columnspan=2, pady=10, sticky='nsew') 

        #self.stop_sync_button =ttk.Button(self.imu_control_frame, text="Stop Sync Sensors", command=self.stop_sync_sensors)
        #self.stop_sync_button.grid( row=4, column =0,columnspan=2, pady=10, sticky='nsew')

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

        imu_lock_button = ttk.Button(self.imu_configuration_frame, text="Lock configuration", state="enabled", command=self.lock_imu_configuration)    
        imu_lock_button.grid(row=imu+4, column=0, columnspan=1)
       
        imu_lock_button = ttk.Button(self.imu_configuration_frame, text="Unlock configuration", state="enabled", command=self.unlock_imu_configuration)    
        imu_lock_button.grid(row=imu+4, column=1, columnspan=1)

        self.imu_status_frame = ttk.LabelFrame(self.settings_tab, text="IMU Sensors Status")
        self.imu_status_frame.grid(row=2, column=1, padx=5, pady=5, sticky='nsew')

        self.camera_status_frame = ttk.LabelFrame(self.settings_tab, text="Camera Sensor Status")
        self.camera_status_frame.grid(row=0, column=1, padx=5, pady=5, sticky='nsew')              

        # Connect Cameras Button
        self.connect_camera_button = ttk.Button(self.camera_status_frame, text="Read Cameras", command=self.connect_webcams)
        self.connect_camera_button.grid(row=0, column=0, columnspan=1, pady=10)
       
        self.disconnect_camera_button = ttk.Button(self.camera_status_frame, text="Disconnect Cameras", command=self.disconnect_webcams)
        self.disconnect_camera_button.grid(row=0, column=1, columnspan=1, pady=10)   

        # WebSocket Server Control Frame
        self.websocket_control_frame = ttk.LabelFrame(self.settings_tab, text="WebSocket Live Streaming")
        self.websocket_control_frame.grid(row=1, column=1, padx=5, pady=5, sticky='nsew')

        # WebSocket Enable/Disable Checkbox
        self.websocket_enabled_var = tk.BooleanVar()
        self.websocket_enabled_var.set(False)
        self.websocket_enable_checkbox = ttk.Checkbutton(
            self.websocket_control_frame, 
            text="Enable WebSocket server", 
            variable=self.websocket_enabled_var,
            command=self.toggle_websocket_server
        )
        self.websocket_enable_checkbox.grid(row=0, column=0, columnspan=2, padx=10, pady=5, sticky='w')

        # WebSocket Status Display
        ttk.Label(self.websocket_control_frame, text="Status:").grid(row=1, column=0, padx=10, pady=5, sticky='w')
        self.websocket_status_label = ttk.Label(self.websocket_control_frame, text="Stopped", foreground="red")
        self.websocket_status_label.grid(row=1, column=1, padx=10, pady=5, sticky='w')

        # WebSocket Client Count Display
        ttk.Label(self.websocket_control_frame, text="Clients:").grid(row=2, column=0, padx=10, pady=5, sticky='w')
        self.websocket_clients_label = ttk.Label(self.websocket_control_frame, text="0")
        self.websocket_clients_label.grid(row=2, column=1, padx=10, pady=5, sticky='w')

        # WebSocket URL Display
        ttk.Label(self.websocket_control_frame, text="URL:").grid(row=3, column=0, padx=10, pady=5, sticky='w')
        self.websocket_url_label = ttk.Label(self.websocket_control_frame, text=f"ws://localhost:{self.websocket_port}", foreground="blue")
        self.websocket_url_label.grid(row=3, column=1, padx=10, pady=5, sticky='w')

        # Terminal Output Frame
        self.terminal_frame = ttk.LabelFrame(self.settings_tab, text="Terminal")
        self.terminal_frame.grid(row=0, column=2, rowspan=5, columnspan=5, padx=10, pady=10, sticky='nsew')

        self.log_view = LogTab(self.terminal_frame)
        self.log_view.pack(fill=tk.BOTH, expand=True)
        self.settings_tab.columnconfigure(2, weight=1)

        gui_handler = self.log_view.get_handler()
        gui_handler.setLevel(logging.NOTSET) 

        formatter = None
        if logger.handlers:
            formatter = logger.handlers[0].formatter
        if formatter is not None:
            gui_handler.setFormatter(formatter)

        root_logger = logging.getLogger()
        root_logger.addHandler(gui_handler)

        if root_logger.level > logging.DEBUG:
            root_logger.setLevel(logging.DEBUG)

        self._gui_log_handler = gui_handler

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
        directory = filedialog.askdirectory()
        logger.debug(f"Folder Selected: {directory}")
        if directory:
            self.save_path = directory
            self.save_directory_var.set(directory)
            self.selected_data_dir = FileManager(directory)

    def update_exercises_list(self, event=None):
        pose = self.imu_pose_selection.get()
        exercises = list(self.EXERCISES.get(pose, {}).keys())
        self.exercises_list['values'] = exercises
        self.exercises_list.set(exercises[0])
        self.update_subcategories_list()
        

    def update_subcategories_list(self, event=None):
        pose = self.imu_pose_selection.get()
        exercise = self.exercises_list.get()
        label_ID = self.EXERCISES.get(pose, {}).get(exercise, [])
        self.sub_exercises_list['values'] = label_ID
        self.sub_exercises_list.set(label_ID[0])
        

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
                    logger.warning(f'This patient ID already exists. Error: {e}')
        return True

    def create_visualization_tab(self):
        # Give the IMU plot (rows 0-5) more weight than the camera/pose views (rows 6-9)
        # to make the camera views slightly smaller vertically.
        for i in range(6):
            self.visualization_tab.grid_rowconfigure(i, weight=3)
        for i in range(6, 10):
            self.visualization_tab.grid_rowconfigure(i, weight=2)
            
        # Left-column panels: fixed content, less vertical stretch
        for j in range(3):
            self.visualization_tab.grid_columnconfigure(j, weight=1)
        # Right area: expands more aggressively
        for j in range(3, 11):
            self.visualization_tab.grid_columnconfigure(j, weight=3)

        # IMU Data Visualization (right, rows 0-5)
        visualize_imus_frame = ttk.LabelFrame(self.visualization_tab, text="IMU Data Visualization")
        visualize_imus_frame.grid(row=0, column=3, rowspan=6, columnspan=8, padx=5, pady=5, sticky='nsew')
        visualize_imus_frame.grid_columnconfigure(0, weight=1)
        visualize_imus_frame.grid_rowconfigure(0, weight=1)

        self.fig_imu = plt.Figure(figsize=(8, 6))
        self.canvas_imu = FigureCanvasTkAgg(self.fig_imu, master=visualize_imus_frame)
        self.canvas_imu.draw()
        self.canvas_imu.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

        self.toolbar = NavigationToolbar2Tk(self.canvas_imu, visualize_imus_frame)
        self.toolbar.update()
        self.toolbar.pack(side=tk.BOTTOM, fill=tk.X)
        self.toolbar.pan()

        # Front View (right-bottom left half, rows 6-9) 
        visualize_camera_frame_front = ttk.LabelFrame(self.visualization_tab, text="Front View")
        visualize_camera_frame_front.grid(row=6, column=3, columnspan=4, rowspan=4, padx=5, pady=5, sticky='nsew')
        visualize_camera_frame_front.grid_rowconfigure(0, weight=1)
        visualize_camera_frame_front.grid_columnconfigure(0, weight=1)

        # Use a plain tk.Label for fast PIL-based rendering (no matplotlib overhead)
        self.cam3_label = tk.Label(visualize_camera_frame_front, bg='white')
        self.cam3_label.grid(row=0, column=0, sticky='nsew')

        # Side View Frame
        visualize_camera_frame_side = ttk.LabelFrame(self.visualization_tab, text="Side View")
        visualize_camera_frame_side.grid(row=6, column=7, columnspan=4, rowspan=4, padx=5, pady=5, sticky='nsew')
        visualize_camera_frame_side.grid_rowconfigure(0, weight=1)
        visualize_camera_frame_side.grid_columnconfigure(0, weight=1)

        # Use a plain tk.Label for fast PIL-based rendering (no matplotlib overhead)
        self.cam4_label = tk.Label(visualize_camera_frame_side, bg='white')
        self.cam4_label.grid(row=0, column=0, sticky='nsew')

        # Data Control 
        select_data_frame = ttk.LabelFrame(self.visualization_tab, text="Data Control")
        select_data_frame.grid(row=0, column=0, rowspan=4, columnspan=3, padx=5, pady=5, sticky='nsew')
        for c in range(3):
            select_data_frame.grid_columnconfigure(c, weight=1)

        # Frame slider
        self.frame_nr_var = tk.IntVar(value=0)
        self.select_frame_scrollbar = tk.Scale(select_data_frame, from_=0, to=1,
                                               orient='horizontal', label="Frame Slider",
                                               variable=self.frame_nr_var)
        self.select_frame_scrollbar.grid(row=0, column=0, columnspan=3, padx=5, pady=(5,2), sticky='ew')
        self.select_frame_scrollbar.config(command=self.on_frame_slider_change)

        decrease_frame_button = ttk.Button(select_data_frame, text="◀", width=2)
        decrease_frame_button.grid(row=1, column=0, padx=5, pady=2, sticky='w')
        decrease_frame_button.bind("<ButtonPress>",   lambda e: self.decrease_frame_nr_button())
        decrease_frame_button.bind("<ButtonRelease>", lambda e: self.stop_frame_scroll())
        increase_frame_button = ttk.Button(select_data_frame, text="▶", width=2)
        increase_frame_button.grid(row=1, column=2, padx=5, pady=2, sticky='e')
        increase_frame_button.bind("<ButtonPress>",   lambda e: self.increase_frame_nr_button())
        increase_frame_button.bind("<ButtonRelease>", lambda e: self.stop_frame_scroll())

        # Show Calibrated
        self.show_calibrated_visual_checkbox_var = tk.BooleanVar(value=False)
        self.select_show_calibrated_tickbox = ttk.Checkbutton(
            select_data_frame, text="Show Calibrated",
            variable=self.show_calibrated_visual_checkbox_var,
            onvalue=True, offvalue=False, command=self.trigger_update)
        self.select_show_calibrated_tickbox.grid(row=2, column=0, columnspan=3, padx=5, pady=2, sticky='w')

        # Signal type checkboxes
        self.acc_checkbox_var = tk.BooleanVar(value=True)
        self.ang_checkbox_var = tk.BooleanVar(value=True)
        self.mag_checkbox_var = tk.BooleanVar(value=False)

        ttk.Checkbutton(select_data_frame, text="Acceleration",
                        onvalue=True, offvalue=False, variable=self.acc_checkbox_var,
                        command=self.trigger_update).grid(row=3, column=0, padx=5, pady=2, sticky='w')
        self.select_acceleration_tickbox = select_data_frame.winfo_children()[-1]
        ttk.Checkbutton(select_data_frame, text="Angular Velocity",
                        onvalue=True, offvalue=False, variable=self.ang_checkbox_var,
                        command=self.trigger_update).grid(row=3, column=1, padx=5, pady=2, sticky='w')
        self.select_angular_velocity_tickbox = select_data_frame.winfo_children()[-1]
        ttk.Checkbutton(select_data_frame, text="Magnetic Field",
                        onvalue=True, offvalue=False, variable=self.mag_checkbox_var,
                        command=self.trigger_update).grid(row=3, column=2, padx=5, pady=2, sticky='w')
        self.select_magnetic_filed_tickbox = select_data_frame.winfo_children()[-1]

        self.show_annotations_checkbox_var= tk.BooleanVar(value=False)
        self.show_annotations_tickbox = ttk.Checkbutton(select_data_frame, text="Show Annotations", variable=self.show_annotations_checkbox_var, onvalue=True, offvalue=False, command=self.toggle_annotations_visibility)
        self.show_annotations_tickbox.grid (row=2, column=1,columnspan=2,padx=5,pady=2, sticky="w")

        # Leg segment checkboxes
        ttk.Label(select_data_frame, text="Plot Leg Segment:").grid(
            row=4, column=0, columnspan=3, padx=5, pady=(6,2), sticky='w')

        self.left_thigh_checkbox_var  = tk.BooleanVar(value=True)
        self.left_calf_checkbox_var   = tk.BooleanVar(value=True)
        self.left_foot_checkbox_var   = tk.BooleanVar(value=True)
        self.right_thigh_checkbox_var = tk.BooleanVar(value=True)
        self.right_calf_checkbox_var  = tk.BooleanVar(value=True)
        self.right_foot_checkbox_var  = tk.BooleanVar(value=True)

        self.select_left_thigh_tickbox = ttk.Checkbutton(
            select_data_frame, text="Left Thigh",
            onvalue=True, offvalue=False, variable=self.left_thigh_checkbox_var, command=self.trigger_update)
        self.select_left_thigh_tickbox.grid(row=5, column=0, padx=5, pady=1, sticky='w')
        self.select_left_calf_tickbox = ttk.Checkbutton(
            select_data_frame, text="Left Calf",
            onvalue=True, offvalue=False, variable=self.left_calf_checkbox_var, command=self.trigger_update)
        self.select_left_calf_tickbox.grid(row=5, column=1, padx=5, pady=1, sticky='w')
        self.select_left_foot_tickbox = ttk.Checkbutton(
            select_data_frame, text="Left Foot",
            onvalue=True, offvalue=False, variable=self.left_foot_checkbox_var, command=self.trigger_update)
        self.select_left_foot_tickbox.grid(row=5, column=2, padx=5, pady=1, sticky='w')
        self.select_right_thigh_tickbox = ttk.Checkbutton(
            select_data_frame, text="Right Thigh",
            onvalue=True, offvalue=False, variable=self.right_thigh_checkbox_var, command=self.trigger_update)
        self.select_right_thigh_tickbox.grid(row=6, column=0, padx=5, pady=1, sticky='w')
        self.select_right_calf_tickbox = ttk.Checkbutton(
            select_data_frame, text="Right Calf",
            onvalue=True, offvalue=False, variable=self.right_calf_checkbox_var, command=self.trigger_update)
        self.select_right_calf_tickbox.grid(row=6, column=1, padx=5, pady=1, sticky='w')
        self.select_right_foot_tickbox = ttk.Checkbutton(
            select_data_frame, text="Right Foot",
            onvalue=True, offvalue=False, variable=self.right_foot_checkbox_var, command=self.trigger_update)
        self.select_right_foot_tickbox.grid(row=6, column=2, padx=5, pady=1, sticky='w')

        # Recording selector + Load button
        self.available_recording_combobox = ttk.Combobox(select_data_frame, state="disabled", values='None')
        self.available_recording_combobox.grid(row=7, column=0, columnspan=2, padx=5, pady=5, sticky='ew')
        self.available_recording_combobox.config(width=15)
        self.available_recording_combobox.bind('<<ComboboxSelected>>', self.on_select_recording)

        ttk.Button(select_data_frame, text="Load Data",
                   command=lambda: Thread(target=self.load_recordings, daemon=True).start()
                   ).grid(row=7, column=2, padx=5, pady=5, sticky='ew')

        # Camera file comboboxes (hidden until camera loaded)
        self.camera1_mp4_combobox = ttk.Combobox(select_data_frame, state="disabled", values=[])
        self.camera1_mp4_combobox.grid(row=8, column=0, columnspan=3, sticky='ew', padx=5)
        self.camera2_mp4_combobox = ttk.Combobox(select_data_frame, state="disabled", values=[])
        self.camera2_mp4_combobox.grid(row=9, column=0, columnspan=3, sticky='ew', padx=5)

        decrease_frame_button = ttk.Button(select_data_frame, text="◀", width=2)
        decrease_frame_button.grid(row=1, column=0, padx=5, pady=2, sticky='w')
        decrease_frame_button.bind("<ButtonPress>",   lambda e: self.decrease_frame_nr_button())
        decrease_frame_button.bind("<ButtonRelease>", lambda e: self.stop_frame_scroll())
        
        self.play_button = ttk.Button(select_data_frame, text="▶ Play", command=self.toggle_play)
        self.play_button.grid(row=10, column=0, columnspan=2, padx=5, pady=(6, 2), sticky='ew')
        self.speed_combo = ttk.Combobox(select_data_frame, state="readonly", values=["0.5x", "1.0x", "1.5x", "2.0x", "3.0x"])
        self.speed_combo.set("1.0x")
        self.speed_combo.grid(row=10, column=2, padx=5, pady=(6, 2), sticky='ew')
        self.speed_combo.bind("<<ComboboxSelected>>", self.change_playback_speed)
        
        increase_frame_button = ttk.Button(select_data_frame, text="▶", width=2)
        increase_frame_button.grid(row=1, column=2, padx=5, pady=2, sticky='e')
        increase_frame_button.bind("<ButtonPress>",   lambda e: self.increase_frame_nr_button())
        increase_frame_button.bind("<ButtonRelease>", lambda e: self.stop_frame_scroll())

        # Pose Viewer (left, rows 7-9) 
        recorded_data_imu_vector_viewer_frame = ttk.LabelFrame(self.visualization_tab, text="Pose Viewer")
        recorded_data_imu_vector_viewer_frame.grid(row=7, column=0, rowspan=3, columnspan=3,
                                                   padx=5, pady=5, sticky='nsew')
        recorded_data_imu_vector_viewer_frame.grid_rowconfigure(0, weight=1)
        recorded_data_imu_vector_viewer_frame.grid_columnconfigure(0, weight=1)

        self.fig6 = plt.figure(figsize=(4, 4))
        self.ax6 = self.fig6.add_subplot(111, projection='3d')
        self.ax6.set_title("Pose Viewer")
        self.ax6.axis('off')
        self.canvas6 = FigureCanvasTkAgg(self.fig6, master=recorded_data_imu_vector_viewer_frame)
        self.canvas6.draw()
        self.canvas6.get_tk_widget().grid(row=0, column=0, sticky='nsew')
        
    def create_annotation_tab(self):
        """Annotations tab: big cameras + pose viewer, small IMU signals, annotation panel."""

        ann = self.annotation_tab  # shorthand

        # Outer grid
        # Rows:  0 = cameras (big)  |  1 = pose + imu (medium)  |  2 = slider bar (slim)
        # Cols:  0-3 = content area |  4 = annotation tools panel (narrow)
        ann.rowconfigure(0, weight=3)   # camera row — tallest
        ann.rowconfigure(1, weight=2)   # pose/imu row
        ann.rowconfigure(2, weight=0)   # slider bar — fixed height
        for c in range(4):
            ann.columnconfigure(c, weight=2)
        ann.columnconfigure(4, weight=1)  # annotation panel — narrower

        # Front Camera (row 0, cols 0-1) 
        cam1_lf = ttk.LabelFrame(ann, text="Front View")
        cam1_lf.grid(row=0, column=0, columnspan=2, padx=4, pady=4, sticky='nsew')
        cam1_lf.rowconfigure(0, weight=1)
        cam1_lf.columnconfigure(0, weight=1)

        self.ann_fig3, self.ann_ax3 = plt.subplots(figsize=(5, 4))
        self.ann_fig3.subplots_adjust(left=0, right=1, top=1, bottom=0)
        self.ann_ax3.set_title("Front View")
        self.ann_ax3.axis('off')
        self.ann_canvas3 = FigureCanvasTkAgg(self.ann_fig3, master=cam1_lf)
        self.ann_canvas3.draw()
        self.ann_canvas3.get_tk_widget().grid(row=0, column=0, sticky='nsew')

        #Side Camera 
        cam2_lf = ttk.LabelFrame(ann, text="Side View")
        cam2_lf.grid(row=0, column=2, columnspan=2, padx=4, pady=4, sticky='nsew')
        cam2_lf.rowconfigure(0, weight=1)
        cam2_lf.columnconfigure(0, weight=1)

        self.ann_fig4, self.ann_ax4 = plt.subplots(figsize=(5, 4))
        self.ann_fig4.subplots_adjust(left=0, right=1, top=1, bottom=0)
        self.ann_ax4.set_title("Side View")
        self.ann_ax4.axis('off')
        self.ann_canvas4 = FigureCanvasTkAgg(self.ann_fig4, master=cam2_lf)
        self.ann_canvas4.draw()
        self.ann_canvas4.get_tk_widget().grid(row=0, column=0, sticky='nsew')

        # Pose Viewer (row 1, cols 0-1) 
        pose_lf = ttk.LabelFrame(ann, text="Pose Viewer")
        pose_lf.grid(row=1, column=0, columnspan=2, padx=4, pady=4, sticky='nsew')
        pose_lf.rowconfigure(0, weight=1)
        pose_lf.columnconfigure(0, weight=1)

        self.ann_fig6 = plt.figure(figsize=(6, 5))
        self.ann_fig6.subplots_adjust(left=0, right=1, top=1, bottom=0)
        self.ann_ax6 = self.ann_fig6.add_subplot(111, projection='3d')
        #self.ann_ax6.set_title("Pose Viewer")
        self.ann_ax6.axis('off')
        self.ann_canvas6 = FigureCanvasTkAgg(self.ann_fig6, master=pose_lf)
        self.ann_canvas6.draw()
        self.ann_canvas6.get_tk_widget().grid(row=0, column=0, sticky='nsew')

        # IMU Signals (
        imu_lf = ttk.LabelFrame(ann, text="IMU Signals")
        imu_lf.grid(row=1, column=2, columnspan=2, padx=4, pady=4, sticky='nsew')
        imu_lf.rowconfigure(0, weight=1)
        imu_lf.columnconfigure(0, weight=1)

        self.ann_fig_imu = plt.Figure(figsize=(5, 3))
        self.ann_canvas_imu = FigureCanvasTkAgg(self.ann_fig_imu, master=imu_lf)
        self.ann_canvas_imu.draw()
        self.ann_canvas_imu.get_tk_widget().pack(fill=tk.BOTH, expand=True)
# Scroll-to-zoom and R-to-reset on the annotation IMU graph only
        def _ann_imu_scroll(event):
            if not hasattr(self, 'ann_ax_map') or not self.ann_ax_map:
                return
            ax = next(iter(self.ann_ax_map.values()))
            xmin, xmax = ax.get_xlim()
            span = xmax - xmin
            try:
                w = self.ann_canvas_imu.get_tk_widget().winfo_width()
                x_frac = event.x / w if w > 0 else 0.5
            except Exception:
                x_frac = 0.5
            center = xmin + x_frac * span
            factor = 0.8 if event.delta > 0 else 1.0 / 0.8
            new_span = span * factor
            new_xmin = center - x_frac * new_span
            new_xmax = new_xmin + new_span
            for a in self.ann_ax_map.values():
                a.set_xlim(new_xmin, new_xmax)
            self.ann_canvas_imu.draw_idle()

        def _ann_imu_key(event):
            if not hasattr(self, 'ann_ax_map') or not self.ann_ax_map:
                return
            key = event.keysym.lower()
            if key == 'r':
                for a in self.ann_ax_map.values():
                    a.autoscale(axis='both')
                self.ann_canvas_imu.draw_idle()
            elif key in ('left', 'right'):
                ax = next(iter(self.ann_ax_map.values()))
                xmin, xmax = ax.get_xlim()
                step = (xmax - xmin) * 0.1
                shift = step if key == 'right' else -step
                for a in self.ann_ax_map.values():
                    a.set_xlim(xmin + shift, xmax + shift)
                self.ann_canvas_imu.draw_idle()
            elif key in ('up', 'down'):
                ax = next(iter(self.ann_ax_map.values()))
                ymin, ymax = ax.get_ylim()
                step = (ymax - ymin) * 0.1
                shift = step if key == 'up' else -step
                for a in self.ann_ax_map.values():
                    a.set_ylim(ymin + shift, ymax + shift)
                self.ann_canvas_imu.draw_idle()

        self.ann_canvas_imu.get_tk_widget().bind('<MouseWheel>', _ann_imu_scroll)
        self.ann_canvas_imu.get_tk_widget().bind('<KeyPress>', _ann_imu_key)
        self.ann_canvas_imu.get_tk_widget().bind('<Button-1>',
            lambda e: self.ann_canvas_imu.get_tk_widget().focus_set())

        # Annotation Tools Panel (rows 0-1, col 4) 
        tools_lf = ttk.LabelFrame(ann, text="Annotation Tools")
        tools_lf.grid(row=0, column=4, rowspan=4, padx=4, pady=4, sticky='nsew')
        tools_lf.rowconfigure(0, weight=1)
        tools_lf.columnconfigure(0, weight=1)

        # Scrollable canvas inside the panel — no scrollbar, mousewheel only
        sc = tk.Canvas(tools_lf, highlightthickness=0)
        sc.grid(row=0, column=0, sticky='nsew')

        inner = ttk.Frame(sc)
        inner_win = sc.create_window((0, 0), window=inner, anchor='nw')

        def _on_inner_cfg(event):
            sc.configure(scrollregion=sc.bbox('all'))
        inner.bind('<Configure>', _on_inner_cfg)

        def _on_sc_cfg(event):
            sc.itemconfig(inner_win, width=event.width)
        sc.bind('<Configure>', _on_sc_cfg)

        def _on_mw(event):
            sc.yview_scroll(int(-1 * (event.delta / 120)), 'units')
        sc.bind('<MouseWheel>', _on_mw)
        inner.bind_all('<MouseWheel>', lambda e: sc.yview_scroll(int(-1*(e.delta/120)), 'units'))

        for c in range(3):
            inner.grid_columnconfigure(c, weight=1)

        #  Label + Type 
        ttk.Label(inner, text="Label:").grid(
            row=0, column=0, sticky='w', padx=5, pady=(8, 2))
        self.annotation_label_var = tk.StringVar(value="Event")
        ttk.Entry(inner, textvariable=self.annotation_label_var,
                  width=10).grid(row=0, column=1, sticky='ew', padx=3, pady=(8, 2))
        self.annotation_type_var = tk.StringVar(value="Event")
        ttk.Combobox(inner, textvariable=self.annotation_type_var,
                     state='readonly', width=9,
                     values=["Event", "Phase Start", "Phase End",
                             "Artifact", "Custom"]
                     ).grid(row=0, column=2, sticky='ew', padx=3, pady=(8, 2))

        # Leg segment
        ttk.Label(inner, text="Leg:").grid(
            row=1, column=0, sticky='w', padx=5, pady=2)
        self.annotation_leg_var = tk.StringVar(value="")
        ttk.Combobox(inner, textvariable=self.annotation_leg_var, state='readonly', width=9,
                    values=["", "Left", "Right","Both"]
                    ).grid(row=1, column=1, columnspan=2, sticky='ew', padx=3, pady=2)

        # Confidence 
        ttk.Label(inner, text="Confidence:").grid(
            row=2, column=0, sticky='w', padx=5, pady=2)
        self.annotation_confidence_var = tk.IntVar(value=5)
        conf_frame = ttk.Frame(inner)
        conf_frame.grid(row=2, column=1, columnspan=2, sticky='ew', padx=3, pady=2)
        conf_frame.grid_columnconfigure(0, weight=1)
        tk.Scale(conf_frame, from_=1, to=5, orient='horizontal',variable=self.annotation_confidence_var,showvalue=True, length=100,tickinterval=1).grid(row=0, column=0, sticky='ew')
        ttk.Label(conf_frame, text="(1=unsure  5=certain)",font=('TkDefaultFont', 7)).grid(row=1, column=0, sticky='w')

        # Point Annotation 
        ttk.Separator(inner, orient='horizontal').grid(row=3, column=0, columnspan=3, sticky='ew', padx=5, pady=(8, 2))
        ttk.Label(inner, text="Point Annotation",font=('TkDefaultFont', 8, 'bold')).grid(row=4, column=0, columnspan=3, sticky='w', padx=5, pady=(0, 3))

        ttk.Button(inner, text="+ Add at Frame",command=self.add_annotation_at_frame).grid(row=5, column=0, columnspan=2, sticky='ew', padx=3, pady=2)
        #self.click_mode_btn = ttk.Button(inner, text="Click Graph",command=self.toggle_annotation_click_mode)
        #self.click_mode_btn.grid(row=5, column=2, sticky='ew', padx=3, pady=2)

        # Segment Annotation 
        ttk.Separator(inner, orient='horizontal').grid(row=6, column=0, columnspan=3, sticky='ew', padx=5, pady=(8, 2))
        ttk.Label(inner, text="Segment Annotation",font=('TkDefaultFont', 8, 'bold')).grid(row=7, column=0, columnspan=3, sticky='w', padx=5, pady=(0, 3))

        self.seg_start_btn = ttk.Button(inner, text="▶  Start Segment",command=self.start_segment)
        self.seg_start_btn.grid(row=8, column=0, columnspan=2,sticky='ew', padx=3, pady=2)
        self.seg_end_btn = ttk.Button(inner, text="■  End Segment",command=self.end_segment, state='disabled')
        self.seg_end_btn.grid(row=8, column=2, sticky='ew', padx=3, pady=2)

        self.seg_status_var = tk.StringVar(value="No segment open")
        ttk.Label(inner, textvariable=self.seg_status_var,font=('TkDefaultFont', 7), foreground='gray').grid(row=9, column=0, columnspan=3, sticky='w', padx=5, pady=(0, 4))

        # Annotations Listbox 
        ttk.Separator(inner, orient='horizontal').grid(row=10, column=0, columnspan=5, sticky='ew', padx=5, pady=(6, 2))
        ann_header_frame = ttk.Frame(inner)
        ann_header_frame.grid(row=11, column=0, columnspan=3, sticky='ew', padx=5, pady=(2, 2))
        ann_header_frame.columnconfigure(0, weight=1)
        ttk.Label(ann_header_frame, text="Annotations:",
                  font=('TkDefaultFont', 8, 'bold')).grid(row=0, column=0, sticky='w')
        self.ann_count_label_var = tk.StringVar(value="0 annotations")
        ttk.Label(ann_header_frame, textvariable=self.ann_count_label_var,
                  font=('TkDefaultFont', 7), foreground='gray').grid(row=0, column=1, sticky='e')

        annot_frame = ttk.Frame(inner)
        annot_frame.grid(row=12, column=0, columnspan=3, sticky='nsew', padx=5, pady=2)
        annot_frame.grid_columnconfigure(0, weight=1)
        self.annotation_listbox = tk.Listbox(annot_frame, height=10, selectmode='single',
                                             font=('Courier', 8),
                                             activestyle='none',
                                             selectbackground='#c8dff7',
                                             selectforeground='#1a1a1a')
        annot_sb_v = ttk.Scrollbar(annot_frame, orient='vertical',
                                   command=self.annotation_listbox.yview)
        annot_sb_h = ttk.Scrollbar(annot_frame, orient='horizontal',
                                   command=self.annotation_listbox.xview)
        self.annotation_listbox.configure(yscrollcommand=annot_sb_v.set,
                                          xscrollcommand=annot_sb_h.set)
        self.annotation_listbox.grid(row=0, column=0, sticky='nsew')
        annot_sb_v.grid(row=0, column=1, sticky='ns')
        annot_sb_h.grid(row=1, column=0, sticky='ew')
        annot_frame.rowconfigure(1, weight=0)

        # Jump / Delete / Clear 
        act = ttk.Frame(inner)
        act.grid(row=13, column=0, columnspan=3, sticky='ew', padx=5, pady=3)
        for c in range(3):
            act.grid_columnconfigure(c, weight=1)
        ttk.Button(act, text="Jump To",command=self.jump_to_selected_annotation).grid(row=0, column=0, sticky='ew', padx=2, pady=2)
        ttk.Button(act, text="Delete",command=self.delete_selected_annotation).grid(row=0, column=1, sticky='ew', padx=2, pady=2)
        ttk.Button(act, text="Clear All",command=self.clear_all_annotations).grid(row=0, column=2, sticky='ew', padx=2, pady=2)
        ttk.Button(act, text="Edit",command=self.edit_selected_annotation).grid(row=0, column=3, sticky='ew', padx=2, pady=2)
        ttk.Button(act, text="Duplicate",command=self.duplicate_selected_annotation).grid(row=0, column=4, sticky='ew', padx=2, pady=2)
        # Save / Load 
        ttk.Separator(inner, orient='horizontal').grid(row=14, column=0, columnspan=3, sticky='ew', padx=5, pady=(6, 3))
        io = ttk.Frame(inner)
        io.grid(row=15, column=0, columnspan=3, sticky='ew', padx=5, pady=(0, 4))
        for c in range(2):
            io.grid_columnconfigure(c, weight=1)
        ttk.Button(io, text="💾 Save",
                   command=self.save_annotations_to_json).grid(row=0, column=0, sticky='ew', padx=3, pady=2)
        ttk.Button(io, text="📂 Load",
                   command=self.load_annotations_from_json).grid(row=0, column=1, sticky='ew', padx=3, pady=2)

        # ── Display Controls (inside tools panel, below Save/Load) ──────────
        ttk.Separator(inner, orient='horizontal').grid(
            row=16, column=0, columnspan=3, sticky='ew', padx=5, pady=(6, 2))
        ttk.Label(inner, text="Signals:",
                  font=('TkDefaultFont', 8, 'bold')).grid(
            row=17, column=0, columnspan=3, sticky='w', padx=5, pady=(2, 1))

        self.ann_acc_checkbox_var     = tk.BooleanVar(value=True)
        self.ann_ang_checkbox_var     = tk.BooleanVar(value=True)
        self.ann_mag_checkbox_var     = tk.BooleanVar(value=False)
        self.ann_show_calibrated_var  = tk.BooleanVar(value=False)

        _cb_cmd = lambda: self.update_annotation_tab_views(self.ann_frame_nr_var.get())
        ttk.Checkbutton(inner, text="Acceleration",
                        variable=self.ann_acc_checkbox_var,
                        onvalue=True, offvalue=False,
                        command=_cb_cmd).grid(row=18, column=0, columnspan=2, sticky='w', padx=5, pady=1)
        ttk.Checkbutton(inner, text="Angular Velocity",
                        variable=self.ann_ang_checkbox_var,
                        onvalue=True, offvalue=False,
                        command=_cb_cmd).grid(row=19, column=0, columnspan=2, sticky='w', padx=5, pady=1)
        ttk.Checkbutton(inner, text="Magnetic Field",
                        variable=self.ann_mag_checkbox_var,
                        onvalue=True, offvalue=False,
                        command=_cb_cmd).grid(row=20, column=0, columnspan=2, sticky='w', padx=5, pady=1)
        ttk.Checkbutton(inner, text="Show Calibrated",
                        variable=self.ann_show_calibrated_var,
                        onvalue=True, offvalue=False,
                        command=_cb_cmd).grid(row=18, column=1, columnspan=2, sticky='w', padx=5, pady=1)

        ttk.Separator(inner, orient='horizontal').grid(
            row=22, column=0, columnspan=3, sticky='ew', padx=5, pady=(6, 2))
        ttk.Label(inner, text="Leg Segments:",
                  font=('TkDefaultFont', 8, 'bold')).grid(
            row=23, column=0, columnspan=3, sticky='w', padx=5, pady=(2, 2))

        self.ann_left_thigh_var  = tk.BooleanVar(value=True)
        self.ann_left_calf_var   = tk.BooleanVar(value=True)
        self.ann_left_foot_var   = tk.BooleanVar(value=True)
        self.ann_right_thigh_var = tk.BooleanVar(value=True)
        self.ann_right_calf_var  = tk.BooleanVar(value=True)
        self.ann_right_foot_var  = tk.BooleanVar(value=True)

        ttk.Checkbutton(inner, text="Left Thigh",  variable=self.ann_left_thigh_var,
                        onvalue=True, offvalue=False, command=_cb_cmd
                        ).grid(row=24, column=0, sticky='w', padx=5, pady=1)
        ttk.Checkbutton(inner, text="Left Calf",   variable=self.ann_left_calf_var,
                        onvalue=True, offvalue=False, command=_cb_cmd
                        ).grid(row=25, column=0, sticky='w', padx=5, pady=1)
        ttk.Checkbutton(inner, text="Left Foot",   variable=self.ann_left_foot_var,
                        onvalue=True, offvalue=False, command=_cb_cmd
                        ).grid(row=26, column=0, sticky='w', padx=5, pady=1)
        ttk.Checkbutton(inner, text="Right Thigh", variable=self.ann_right_thigh_var,
                        onvalue=True, offvalue=False, command=_cb_cmd
                        ).grid(row=24, column=1, sticky='w', padx=5, pady=1)
        ttk.Checkbutton(inner, text="Right Calf",  variable=self.ann_right_calf_var,
                        onvalue=True, offvalue=False, command=_cb_cmd
                        ).grid(row=25, column=1, sticky='w', padx=5, pady=1)
        ttk.Checkbutton(inner, text="Right Foot",  variable=self.ann_right_foot_var,
                        onvalue=True, offvalue=False, command=_cb_cmd
                        ).grid(row=26, column=1, sticky='w', padx=5, pady=1)

        # ── Frame Navigation Bar (row 2, cols 0-3) — enlarged ───────────────
        nav_lf = ttk.LabelFrame(ann, text="Frame Navigation")
        nav_lf.grid(row=2, column=0, columnspan=5, padx=4, pady=(2, 4), sticky='ew')
        nav_lf.grid_columnconfigure(1, weight=1)

        # Annotation timeline track (above the slider)
        self.ann_timeline_canvas = tk.Canvas(nav_lf, height=18, bg='#e8e8e8',
                                             highlightthickness=0)
        self.ann_timeline_canvas.grid(row=0, column=1, padx=4, pady=(6, 0), sticky='ew')
        self.ann_timeline_canvas.bind('<Configure>', lambda e: self.redraw_ann_timeline())

        # ◀ step-back button
        ttk.Button(nav_lf, text="◀", width=4,
                   command=self.ann_step_back).grid(row=1, column=0, padx=(8, 2), pady=6, sticky='w')

        # Frame slider 
        self.ann_frame_nr_var = tk.IntVar(value=0)
        self.ann_frame_slider = tk.Scale(
            nav_lf, from_=0, to=1, orient='horizontal',
            variable=self.ann_frame_nr_var, showvalue=False,
            sliderlength=20, width=14,
            command=self.on_ann_frame_slider_change)
        self.ann_frame_slider.grid(row=1, column=1, padx=4, pady=6, sticky='ew')

        # ▶ step-forward button
        ttk.Button(nav_lf, text="▶", width=4,
                   command=self.ann_step_forward).grid(row=1, column=2, padx=(2, 8), pady=6, sticky='w')

        # Frame label — larger font
        self.ann_frame_label_var = tk.StringVar(value="Frame: 0 / 0")
        ttk.Label(nav_lf, textvariable=self.ann_frame_label_var,
                  font=('TkFixedFont', 11)).grid(row=1, column=3, padx=12, pady=6, sticky='w')

    def on_ann_frame_slider_change(self, value):
        """Called when the annotation-tab frame slider moves."""
        frame_idx = int(float(value))
        self.ann_frame_nr_var.set(frame_idx)
        self.update_ann_frame_label(frame_idx)
        self.redraw_ann_timeline()
        self.update_annotation_tab_views(frame_idx)

    def ann_step_forward(self):
        max_frame = int(self.ann_frame_slider['to'])
        current = self.ann_frame_nr_var.get()
        if current < max_frame:
            next_f = current + 1
            self.ann_frame_nr_var.set(next_f)
            self.update_ann_frame_label(next_f)
            self.redraw_ann_timeline()
            self.update_annotation_tab_views(next_f)

    def ann_step_back(self):
        current = self.ann_frame_nr_var.get()
        if current > 0:
            prev_f = current - 1
            self.ann_frame_nr_var.set(prev_f)
            self.update_ann_frame_label(prev_f)
            self.redraw_ann_timeline()
            self.update_annotation_tab_views(prev_f)

    def update_ann_frame_label(self, frame_idx):
        """Refresh the 'Frame: N / Total' counter in the annotation tab."""
        try:
            total = int(self.ann_frame_slider['to'])
            self.ann_frame_label_var.set(f"Frame: {frame_idx} / {total}")
        except Exception:
            pass

    def redraw_ann_timeline(self):
        """Draw annotation markers on the timeline track above the scrubber."""
        if not hasattr(self, 'ann_timeline_canvas'):
            return
        c = self.ann_timeline_canvas
        c.delete('all')
        w = c.winfo_width()
        h = c.winfo_height()
        if w <= 1:
            return

        total = int(self.ann_frame_slider['to'])
        if total <= 0:
            return

        _type_colors = {
            "Event":       "#7F77DD",
            "Phase Start": "#1D9E75",
            "Phase End":   "#D85A30",
            "Artifact":    "#BA7517",
            "Custom":      "#378ADD",
        }

        for annot in self.annotations:
            color = _type_colors.get(annot.get('type', ''), '#7F77DD')
            kind  = annot.get('kind', 'point')
            f_start = annot.get('frame_idx', 0)
            x_start = int(f_start / total * w)

            if kind == 'segment':
                f_end = annot.get('frame_end', f_start)
                x_end = int(f_end / total * w)
                c.create_rectangle(x_start, 1, max(x_end, x_start + 2), h - 1,
                                   fill=color, outline='', stipple='gray50')
                c.create_line(x_start, 0, x_start, h, fill=color, width=2)
                c.create_line(x_end,   0, x_end,   h, fill=color, width=2)
            else:
                c.create_line(x_start, 0, x_start, h, fill=color, width=2)
                c.create_oval(x_start - 3, h // 2 - 3,
                              x_start + 3, h // 2 + 3,
                              fill=color, outline='white', width=1)

        # Current frame position line
        cur = self.ann_frame_nr_var.get()
        x_cur = int(cur / total * w)
        c.create_line(x_cur, 0, x_cur, h, fill='#1565C0', width=2)

    def update_annotation_tab_views(self, frame_idx):
        """
        Refresh all annotation-tab displays for the given frame:
          - Front & Side camera views
          - 3D Pose Viewer
          - IMU Signals panel
        """
        try:
            # Cameras 
            if (self.loaded_camera1_frames is not None
                    and frame_idx < len(self.loaded_camera1_frames)):
                frame = self.loaded_camera1_frames[frame_idx]
                self.ann_ax3.clear()
                self.ann_ax3.imshow(frame)
                self.ann_ax3.axis('off')
                self.ann_canvas3.draw()

            if (self.loaded_camera2_frames is not None
                    and frame_idx < len(self.loaded_camera2_frames)):
                frame = self.loaded_camera2_frames[frame_idx]
                self.ann_ax4.clear()
                self.ann_ax4.imshow(frame)
                self.ann_ax4.axis('off')
                self.ann_canvas4.draw()

            #  Pose Viewer 
            self.ann_ax6.clear()
            self.ann_ax6.set_xlabel("X")
            self.ann_ax6.set_ylabel("Y")
            self.ann_ax6.set_zlabel("Z")
            self.ann_ax6.set_xlim3d([-1, 7])
            self.ann_ax6.set_ylim3d([-3.5, 3.5])
            self.ann_ax6.set_zlim3d([-3.5, 3.5])
            self.ann_ax6.set_autoscale_on(False)

            if hasattr(self, 'loaded_imu_data') and self.loaded_imu_data is not None:
                self.get_pose()
                self.new_joints = copy.deepcopy(self.joints)
                segment_labels = [
                    "Left Thigh", "Left Calf", "Left Foot",
                    "Right Thigh", "Right Calf", "Right Foot"
                ]
                checkbox_vars = [
                    self.ann_left_thigh_var, self.ann_left_calf_var,
                    self.ann_left_foot_var, self.ann_right_thigh_var,
                    self.ann_right_calf_var, self.ann_right_foot_var
                ]
                if not hasattr(self, 'visual_calibration_reference'):
                    self.visual_calibration_reference = {}
                for i in range(6):
                    if not checkbox_vars[i].get():
                        continue
                    col_q = [f"quat.w({i})", f"quat.x({i})",
                             f"quat.y({i})", f"quat.z({i})"]
                    if not all(col in self.loaded_imu_data.columns for col in col_q):
                        continue
                    q = self.loaded_imu_data.loc[frame_idx, col_q].values.astype(float)
                    if np.any(np.isnan(q)):
                        continue
                    if self.ann_show_calibrated_var.get():
                        if i in self.visual_calibration_reference:
                            q = self.multiply_quaternions(
                                self.visual_calibration_reference[i], q)
                        else:
                            continue
                    R = self.get_rotation_matrix_quaternions(q)
                    self.rotate_leg_segment(segment_labels[i], R)

                DEFAULT_SETTINGS.plot_body_parts(self.ann_ax6, self.new_joints)
                title = ("Pose Viewer (Calibrated)"
                         if self.ann_show_calibrated_var.get()
                         else "Pose Viewer")
                self.ann_ax6.set_title(title)

            self.ann_canvas6.draw()
            self.sync_ann_imu(frame_idx)
            self.draw_annotations()

        except Exception as e:
            logger.error(f"[update_annotation_tab_views] {e}")

    def connect_webcams(self):
        # set camera_lamps to OFF - in case of reconnection
        for lamp in self.camera_lamps:
            lamp.destroy()

        for labels in self.camera_labels:
            labels.destroy()

        self.openAvailableCameras()

        # create the Labels and lamps for each camera
        for i in range(len(self.idxInitiatedCameras)):
            self.create_camera_status_lamp(self.camera_status_frame, f"Camera {self.idxInitiatedCameras[i]}", i+1, 0)
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
        logger.info(f"Number of detected cameras: {len(self.idxInitiatedCameras)}")
        self.camera_list_1['values'] = self.idxInitiatedCameras
        self.camera_list_1['state'] = 'readonly'
        self.camera_list_2['values'] = self.idxInitiatedCameras
        self.camera_list_2['state'] = 'readonly'

        if self.idxInitiatedCameras:
            default_idx =str(self.idxInitiatedCameras[0])
            self.camera_list_1.set(default_idx)
            self.camera_list_2.set(default_idx)
                      
    def openAvailableCameras(self):
        self.InitiatedCameras.clear()
        self.idxInitiatedCameras.clear()

        for i in range(self.MAXCAMERAS):
            cam = CameraManager(i)
            cam.open_camera()

            cap = cam.camera
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            if cap.read()[0]:
                self.InitiatedCameras.append(cam)
                self.idxInitiatedCameras.append(i)
                actual_fps = cap.get(cv2.CAP_PROP_FPS)
                actual_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                logger.info(f"Camera {i}: {actual_w}x{actual_h} @ {actual_fps:.1f} fps, buffer=1")
            else:
                logger.warning(f"Camera {i} not available")

    # Threads for handling camera 1 and camera 2
    def camera1_thread(self):
        try:
            cam_idx = int(self.camera_list_1.get())
            camera_manager = self.InitiatedCameras[cam_idx]
            camera_manager.streaming = True
            cap = camera_manager.camera
        except Exception as e:
            logger.error(f"[Camera 1 Thread] Could not initialize camera:{e}")
            return

        flag_index = 1 if self.recording else 0

        while self.thread_flag[flag_index]:
            try:
                if self.recording:
                    # Recording mode: wait for Thread 5 tick signal
                    if not self.capture_event.wait(timeout=0.5):
                        continue  # timeout — check thread_flag and loop

                    if cap.grab():
                        t_grab = time.monotonic_ns()
                        ret, frame_1 = cap.retrieve()
                        if ret:
                            self.latest_frame_cam1 = cv2.cvtColor(frame_1, cv2.COLOR_BGR2RGB)
                            self.t_cam1 = t_grab
                            self.cam1_timestamps_log.append({
                                'Frame_ID': self.frames_cam1,
                                'Timestamp': t_grab
                            })
                            self.frames_cam1 += 1

                    # Rendezvous with Thread 5 and camera2_thread
                    try:
                        self._grab_barrier.wait(timeout=0.5)
                    except threading.BrokenBarrierError:
                        break

                else:
                    # Preview mode: free-run, no synchronisation needed
                    if cap.grab():
                        t_grab = time.monotonic_ns()
                        ret, frame_1 = cap.retrieve()
                        if ret:
                            self.latest_frame_cam1 = cv2.cvtColor(frame_1, cv2.COLOR_BGR2RGB)
                            self.t_cam1 = t_grab
                    #time.sleep(0.033)

            except Exception as e:
                logger.error(f"[Camera Thread] Error: {e}")
                self.latest_frame_cam1 = None

        camera_manager.streaming = False

    def camera2_thread(self):
        try:
            cam_idx = int(self.camera_list_2.get())
            camera_manager = self.InitiatedCameras[cam_idx]
            camera_manager.streaming = True
            cap = camera_manager.camera
        except Exception as e:
            logger.error(f"[Camera 2 Thread] Could not initialize camera: {e}")
            return

        flag_index = 3 if self.recording else 1

        while self.thread_flag[flag_index]:
            try:
                if self.recording:
                    # Recording mode: wait for Thread 5 tick signal
                    if not self.capture_event.wait(timeout=0.5):
                        continue  # timeout — check thread_flag and loop

                    if cap.grab():
                        t_grab = time.monotonic_ns()
                        ret, frame_2 = cap.retrieve()
                        if ret:
                            self.latest_frame_cam2 = cv2.cvtColor(frame_2, cv2.COLOR_BGR2RGB)
                            self.t_cam2 = t_grab
                            self.cam2_timestamps_log.append({
                                'Frame_ID': self.frames_cam2,
                                'Timestamp': t_grab
                            })
                            self.frames_cam2 += 1

                    # Rendezvous with Thread 5 and camera1_thread
                    try:
                        self._grab_barrier.wait(timeout=0.5)
                    except threading.BrokenBarrierError:
                        break

                else:
                    # Preview mode: free-run, no synchronisation needed
                    if cap.grab():
                        t_grab = time.monotonic_ns()
                        ret, frame_2 = cap.retrieve()
                        if ret:
                            self.latest_frame_cam2 = cv2.cvtColor(frame_2, cv2.COLOR_BGR2RGB)
                            self.t_cam2 = t_grab
                    #time.sleep(0.033)

            except Exception as e:
                logger.error(f"[Camera2 Thread] Error: {e}")
                self.latest_frame_cam2 = None

        camera_manager.streaming = False


    def stream_webcam(self): # No very elegant, maybe can be converted to a for loop
        if self.recording:
            logger.warning("Cannot start webcam streaming while recording.")
            return
        if self.InitiatedCameras==[]:
            logger.warning("No cameras connected")
            return
        
        cam1_index = self.camera_list_1.get()
        cam2_index = self.camera_list_2.get()

        if cam1_index == ''or cam2_index=='':
            logger.warning("Please select both camera views.")
            return
        
        if cam1_index == cam2_index:
           logger.warning("Viewers have the same camera. Please select different cameras.")
           return

        self.InitiatedCameras[int(cam1_index)].streaming = True
        self.InitiatedCameras[int(cam2_index)].streaming = True

    # Thread 0: Camera 1
        if self.thread[0] is None or not self.thread[0].is_alive():
           self.thread_flag[0] = True
           self.thread[0] = Thread(target=self.camera1_thread)
           self.thread[0].start()
           logger.info(f"Thread 0 (Camera {cam1_index}) started.")

    # Thread 1: Camera 2
        if self.thread[1] is None or not self.thread[1].is_alive():
           self.thread_flag[1] = True
           self.thread[1] = Thread(target=self.camera2_thread)
           self.thread[1].start()
           logger.info(f"Thread 1 (Camera {cam2_index}) started.")

    #Start view update threads as detached
        Thread(target=self.update_view, args=(0, cam1_index, self.ax1, self.canvas1)).start()
        Thread(target=self.update_view, args=(1, cam2_index, self.ax2, self.canvas2)).start()

    # Lock comboboxes
        self.camera_list_1['state'] = 'disabled'
        self.camera_list_2['state'] = 'disabled'

        logger.info("Camera streaming started.")
        

    def update_view(self, threadNum, cameranumber, cameraview_axis, cameraview_canvas):
        cam_idx = int(cameranumber)
        logger.debug(f"[View] Starting update_view for camera index {cam_idx}")
        
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
            time.sleep(0.01)
            # print("Running")

            #if not self.thread_flag[threadNum]:
            #self.InitiatedCameras[int(cameranumber)].streaming = False
        # print('Run')
           except Exception as e:
               logger.error(f"[Thread{threadNum}] Error while updating view: {e}")
               
        logger.info(f"Stoped reading from camera {cameranumber}")
        
        
    def start_stop_button_press(self):
        if self.recording:
            logger.warning("Cannot start/stop streaming during recording.")

        if self.InitiatedCameras == []:
           logger.warning("No cameras connected.")
           return
        
        cam1=int(self.camera_list_1.get())
        cam2=int(self.camera_list_2.get())

        
        if self.InitiatedCameras[cam1].streaming:
           self.stop_streaming_mode()
           self.InitiatedCameras[cam1].streaming = False
           self.InitiatedCameras[cam2].streaming = False
           self.camera_list_1['state'] = 'readonly'
           self.camera_list_2['state'] = 'readonly'
           logger.info("Streaming stopped.")
        else:
           self.InitiatedCameras[cam1].streaming = True
           self.InitiatedCameras[cam2].streaming = True
           self.camera_list_1['state'] = 'disabled'
           self.camera_list_2['state'] = 'disabled'
           self.stream_webcam()
           logger.info("Streaming started.")
        
        
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
            logger.warning("No IMU sensors are connected")

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
        logger.debug(1)

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
            if combobox in self.imu_comboboxes:
               combobox['state'] = 'disable'
               self.imu_lock_status = True
        logger.info("IMU configuration locked successfully.")

    def unlock_imu_configuration(self):
        for combo in self.imu_comboboxes:
            combo['state'] = 'readonly'
            self.imu_lock_status = False


    def sync_sensors(self):
      self.imu.sync_IMU_sensors()
     
    def start_stop_button_streaming_imu(self):
       
        if self.recording:
            logger.warning("IMU already in recording mode.")
            return

        if self.imu_streaming:
            self.stop_imu_streaming()
            # self.imu_streaming = False
            
        else:
            self.start_imu_streaming()
            # self.imu_streaming = True

    
    # Stop streaming for camera1 and camera 2
    def stop_streaming_mode(self):
        logger.info("Stopping streaming mode...")

        # In preview mode cameras are free-running (no barrier/event),
        # so just clear the flags and join the threads.
        for i in [0, 1]:
            self.thread_flag[i] = False
            if self.thread[i] is not None:
                self.thread[i].join(timeout=1)
                self.thread[i] = None
                logger.debug(f"Thread {i} stopped.")

        self.camera_list_1['state'] = 'readonly'
        self.camera_list_2['state'] = 'readonly'

        for cam in self.InitiatedCameras:
            cam.streaming = False

    def stop_threads(self):
        for i in range(6):
        # Skip stopping IMU streaming thread if it’s running separately
            if i == 2 and self.imu_streaming and not self.recording:
                logger.debug("Skipping stop of IMU streaming thread[2]")
                continue

            self.thread_flag[i] = False
            if self.thread[i] is not None:
                self.thread[i].join(0)
                self.thread[i] = None
                logger.debug(f" Stopped and cleared thread[{i}]")


    def start_recording(self):
      
      if self.recording:
        logger.warning("Recording already in progress.")
        return

      if not self.imu_lock_status:
        logger.warning("Configuration must be locked before starting recording.")
        return

      logger.info("Preparing to start recording...")

      while not self.data_queue.empty():
        try:
            self.data_queue.get_nowait()
        except Exception as e:
            logger.error(f"Error while clearing queue: {e}")
            break

      if not self.imu_streaming:
        logger.info("Starting IMU measuring mode")
        self.start_imu_streaming()

    
      self.reset_heading_and_countdown(3)


    def reset_heading_and_countdown(self, seconds_left):
      self.exercise_list_label['text'] = f"Recording starts in: {seconds_left} seconds"
      self.exercise_list_label['background'] = 'orange'
      """if seconds_left == 2:  
        try: 
            # self.imu.reset_heading()
            # self.initiate_plot()
            # self.imu.get_measurments()
            self.get_calibration_data()
            print("Calibration data acquired!")
        except Exception as e:
            print(f"Error resetting IMU heading: {e}")"""

      if seconds_left > 0:
        logger.info(f"Recording starts in: {seconds_left}...")
        self.after(2000, lambda: self.reset_heading_and_countdown(seconds_left - 1))
      else:
        self.start_recording_proper()

    def start_recording_proper(self):
        
        logger.info("Starting actual recording...")

        # Add timestamp logging for cameras and IMU
        self.cam1_timestamps_log = []
        self.cam2_timestamps_log = []
        self.imu_timestamps_log = []
        self.frames_cam1 = 0
        self.frames_cam2 = 0
                
        self.stop_threads()

        self.latest_frame_cam1 = None
        self.latest_frame_cam2 = None

        # Determine which cameras participate in this recording.
        # A camera is active only if cameras were detected AND its
        # combobox has a selection.
        self.cam1_active = bool(self.InitiatedCameras) and self.camera_list_1.get() != ""
        self.cam2_active = bool(self.InitiatedCameras) and self.camera_list_2.get() != ""
        n_cams = int(self.cam1_active) + int(self.cam2_active)

        if n_cams == 0:
            logger.info("No cameras available/selected — recording in IMU-only mode.")

        # Reset sync primitives cleanly before starting recording threads
        self.capture_event.clear()
        self._grab_barrier = threading.Barrier(1 + n_cams)  # Thread5 + active cameras
       

        self.use_queue = True
        self.recording = True

        # Thread 0: Camera 1 (event-triggered grab in recording mode)
        if self.cam1_active:
            self.thread_flag[0] = True
            self.thread[0] = Thread(target=self.camera1_thread)
            self.thread[0].start()
            logger.debug("Thread 0 started.")

        # Thread 1: Camera 2 (event-triggered grab in recording mode)
        if self.cam2_active:
            self.thread_flag[1] = True
            self.thread[1] = Thread(target=self.camera2_thread)
            self.thread[1].start()
            logger.debug("Thread 1 started.")

        self.thread_flag[3] = True
        self.thread[3] = Thread(target=self.update_imu_plot)
        self.thread[3].start()
        logger.debug("Thread 3 started.")

        if n_cams > 0:
            self.thread_flag[4] = True
            self.thread[4] = Thread(target=self.update_camera_figures)
            self.thread[4].start()
            logger.debug("Thread 4 started.")

        self.thread_flag[5] = True
        self.thread[5] = Thread(target=self.get_imu_measurments)
        self.thread[5].start()
        logger.debug("Thread 5 started.")

        logger.info("Recording Started.")
        self.exercise_list_label['text'] = f"RECORDING!"
        self.exercise_list_label['background'] = 'green'

    def stop_recording(self):
        if not self.recording:
            logger.info("Recording stopped.")
            return

        logger.info("Stopping recording...")

        self.recording = False
        self.use_queue = False

        # Unblock camera threads BEFORE joining — they may be waiting on
        # capture_event or _grab_barrier, so release them first.
        self.capture_event.set()
        try:
            self._grab_barrier.abort()
        except Exception:
            pass

        for i in [0, 1, 3, 4, 5]:
            self.thread_flag[i] = False
            if self.thread[i] is not None:
                self.thread[i].join(timeout=2)
                self.thread[i] = None
                logger.debug(f"Thread {i} stopped.")

        # Reset sync primitives for next recording session
        self.capture_event.clear()
        self._grab_barrier = threading.Barrier(3)

        # wait for consumer thread to finish processing any remaining data
        if hasattr(self, '_consumer_thread') and self._consumer_thread.is_alive():
            logger.debug("Waiting for consumer to finish...")
            self._consumer_thread.join(timeout=5)

        for camera in self.InitiatedCameras:
            camera.streaming = False

        if self.imu_streaming:
            self.stop_imu_streaming()

        self.imu_streaming = False
        self.thread_flag[2] = False
        self.thread[2] = None

        logger.info("Recording fully stopped.")

        if not hasattr(self, 'selected_data_dir'):
            self.selected_data_dir = FileManager(self.save_directory_var.get())

        self.selected_data_dir.save_recording(
            self.data_queue,
            self.imu_ordered_configuration,
            self.patients_id_field.get(),
            self.exercises_list.get(),
            self.imu_pose_selection.get(),
            self.sub_exercises_list.get()
        )

        logger.info("Recording saved.")
        self.camera_list_1['state'] = 'readonly'
        self.camera_list_2['state'] = 'readonly'
        self.exercise_list_label['text'] = 'Ready for recording'
        self.exercise_list_label['background'] = 'yellow'

    def data_consumer_thread(self):
        """Consumer thread that processes data entries from the queue and prepares them for saving."""
        while True:
            snapshot = self._imu_work_queue.get()

            if snapshot is None:  
                logger.debug("[Consumer] Received stop signal, shutting down.")
                break

            try:
                t_imu = snapshot['t_imu']

                data_entry = {}

                # Timestamps
                data_entry['t_imu']    = t_imu
                data_entry['t_cam1']   = snapshot['t_cam1']
                data_entry['t_cam2']   = snapshot['t_cam2']
                data_entry['timestamp'] = t_imu

                # IMU data
                if self.frames == 0:
                    self.get_calibration_data()

                data_entry['imu']     = snapshot['quat']
                data_entry['imu_acc'] = snapshot['acc']
                data_entry['imu_ang'] = snapshot['ang']
                data_entry['imu_mag'] = snapshot['mag']
                data_entry['imu_ts']  = snapshot['ts']

                # Camera frames
                data_entry['frame_cam1'] = snapshot['frame_cam1']
                data_entry['frame_cam2'] = snapshot['frame_cam2']

                if self.recording and self.use_queue:
                    self.data_queue.put(data_entry)

                self.frames += 1
                logger.debug(f"Frame {self.frames} processed.")

            except Exception as e:
                logger.error(f"Error processing frame: {e}")
                
    def get_imu_measurments(self):
        
        self.frames = 0
        self._imu_work_queue = Queue()

        self._consumer_thread = Thread(target=self.data_consumer_thread, daemon=True)
        self._consumer_thread.start()
        logger.debug("[IMU] Consumer thread started.")


        while self.thread_flag[5]:
            try:
                self.imu.get_measurments()
                
                if  not self.imu.newPackets:            
                   time.sleep(0.001)
                   continue
                self.imu.newPackets = False
                
                if True:
                    t_imu = time.monotonic_ns()

                    if self.recording and not hasattr(self, 'imu_timestamps_log'):
                        self.imu_timestamps_log.append(t_imu)

                # Signal both camera threads to grab simultaneously
                    self.capture_event.set()

                # Wait for both camera threads to finish grabbing
                    try:
                        self._grab_barrier.wait(timeout=0.5)
                    except threading.BrokenBarrierError:
                        break

                # Clear event so cameras don't re-trigger until next tick
                self.capture_event.clear()

                if self.recording and hasattr(self, 'imu_timestamps_log'):
                    self.imu_timestamps_log.append(t_imu)

                # Build snapshot — cameras have already updated latest_frame and t_cam
                snapshot = {
                    'quat':       self.imu.quat_data.copy(),
                    'acc':        self.imu.acc_data.copy(),
                    'ang':        self.imu.gyr_data.copy(),
                    'mag':        self.imu.mag_data.copy(),
                    'ts':         self.imu.sensor_timestamp.copy(),
                    't_imu':      t_imu,
                    't_cam1':     self.t_cam1,
                    't_cam2':     self.t_cam2,
                    'frame_cam1': self.latest_frame_cam1.copy() if self.latest_frame_cam1 is not None else None,
                    'frame_cam2': self.latest_frame_cam2.copy() if self.latest_frame_cam2 is not None else None,
                }

                self.latest_quat_data = snapshot['quat']
                self._imu_work_queue.put(snapshot)

            except Exception as e:
                logger.error(f"[Get IMU Measurements] Error: {e}")

        # Signal consumer to stop and wait for it to drain
        self._imu_work_queue.put(None)
        self._consumer_thread.join(timeout=2)
        logger.debug("[IMU] Consumer thread stopped.")


    def update_camera_figures(self): # Method used in recording mode.
        while self.thread_flag[4]:
            try:
                if self.latest_frame_cam1 is not None:
                    self.ax1.clear()
                    self.ax1.imshow(self.latest_frame_cam1)
                    self.ax1.axis('off')
                    self.canvas1.draw()
                    ### print("Camera 1 updated")
                if self.latest_frame_cam2 is not None:
                    self.ax2.clear()
                    self.ax2.imshow(self.latest_frame_cam2)
                    self.ax2.axis('off')
                    self.canvas2.draw()
                    ### print("Camera 2 updated")
                
                time.sleep(0.02)
            
            except Exception as e:
                logger.error(f"[Thread 4] Error during figure update: {e}")
                

    def stop_imu_streaming(self):
        if self.imu_streaming:
           self.thread_flag[2] = False
           self.imu.stop_measuring_mode()

           if self.thread[2] is not None:
               self.thread[2].join(0)

               self.thread[2]= None

               logger.info("IMU streaming stopped")
               self.imu_streaming = False
        else:
            logger.warning("No IMUs are streaming at the moment")

        
    def start_imu_streaming(self):
        if self.imu_streaming:
            logger.warning("IMU is already streaming.")


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

                flag_index = 2
                self.thread_flag[flag_index] =True
                self.thread[flag_index]= Thread(target=self.update_imu_plot)
                self.thread[flag_index].start()

                logger.info("IMU streaming  started")
                     
            else: 
                logger.warning("IMU configuration is not locked. Please proceed checking the configuration and lock it brefore start streaming")
    
    def initiate_plot(self):
        self.reset_pose_plot()
        self.get_pose(self.ax , self.canvas)
        

    def update_imu_plot(self):
      self.new_joints = copy.deepcopy(self.joints)

      current_thread_index = 3 if self.recording else 2

      while self.thread_flag[current_thread_index]:
        try:
            if self.recording:
                #t_grab = time.monotonic_ns()
                self.latest_quat_data = self.imu.quat_data.copy()
                if not hasattr(self, 'latest_quat_data'):
                    logger.warning("[IMU Plot] No quaternion data available yet")
                    #time.sleep(0.05)

                quaternions = self.latest_quat_data

                self.send_websocket_data(
                    time.time(),
                    self.imu.quat_data,
                    self.imu.acc_data,
                    self.imu.gyr_data,
                    self.imu.mag_data
                )
                
            else:
                self.imu.get_measurments()
                quaternions = self.imu.quat_data
                # Send data via WebSocket if enabled
                self.send_websocket_data(
                    time.monotonic_ns(),
                    self.imu.quat_data,
                    self.imu.acc_data,
                    self.imu.gyr_data,
                    self.imu.mag_data
                )
            

            for legSegment in range(len(self.imu_comboboxes)):
                deviceIdx = self.imu_ordered_configuration[legSegment]
                if deviceIdx != -1:
                    q = quaternions[deviceIdx, :]
                    ### print(f"Raw Sensor Quaternions: {q}")
                    if self.show_calibrated_checkbox_var.get() and self.imu.calibration_status[deviceIdx]:
                        q = self.multiply_quaternions(self.imu.calibration_inverse[deviceIdx], q)
                        # q = q / np.linalg.norm(q)  # Normalize the quaternion
                        ### print(f"Calibrated q: {q}")
                        ### print(f"Norm of Quaternions: {np.linalg.norm(q)}")
                        
                    q = np.round(q, 4)
                    rotationMatrix = self.get_rotation_matrix_quaternions(q)
                    self.rotate_leg_segment(self.imu_combobox_labels[legSegment]['text'][:-1], rotationMatrix)

            self.reset_pose_plot()
            DEFAULT_SETTINGS.plot_body_parts(self.ax, self.new_joints)
            # self.quiver = self.ax.quiver(self.joints['Left Hip'][0], self.joints['Left Hip'][1], self.joints['Left Hip'][2], self.new_joints['Left Knee'][0], self.new_joints['Left Knee'][
            # 1], self.new_joints['Left Knee'][2], color='red', label='Vector direction')
            self.canvas.draw()

            if self.reset_heading_flag:
                self.imu.reset_heading()
                self.reset_heading_flag = False

            time.sleep(0.01)  

        except Exception as e:
            logger.error(f"[IMU plot] Error: {e}")
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
    
    def get_pose(self, ax=None, canvas =None):
        if self.imu_pose_selection.get() == 'Sitting':
            self.joints = DEFAULT_SETTINGS.skeleton_pose_sitting_joints()
        elif self.imu_pose_selection.get()== 'Lying':
            self.joints = DEFAULT_SETTINGS.skeleton_pose_lying_joints()
        else:
            self.joints = DEFAULT_SETTINGS.skeleton_pose_standing_joints()
        
        if ax is not None and canvas is not None:
            DEFAULT_SETTINGS.plot_body_parts(self.ax, self.joints)
            canvas.draw() 
     
    def rotate_leg_segment(self, legSegment, rotationMatrix):
          
        #Function to move the entire chain of joints by a displacement 
        def move_chain(joint, displacement):
            #   for joint in joint_names:
            self.new_joints[joint] += displacement   #update the position of each joint 
        
    # Left leg segments
    # Left Thight rotation
        if legSegment == 'Left Thigh':
            temp_knee = copy.deepcopy(self.new_joints['Left Knee'])  # Store the current position 
            start = self.joints['Left Hip']  #start point is the left hip
            stop = self.joints['Left Knee'] - start  #direction from hip to knee
            norm = stop / np.linalg.norm(stop)  # Normalize the direction vector
            rotated = np.dot(rotationMatrix, norm)  # Apply the rotation matrix
            temp_ankle = copy.deepcopy(self.new_joints['Left Ankle'])  # Store the current position
            if not np.isnan(rotated).any(): 
                self.new_joints['Left Knee'] = rotated * np.linalg.norm(stop) + start  # scale the rotated vetcor to the original length
                displacement = self.new_joints['Left Knee'] - temp_knee  #update new knee position
                if self.imu_comboboxes[1].get() == "None":
                    move_chain('Left Ankle', displacement)
                if self.imu_comboboxes[2].get() == "None":
                    displacement = self.new_joints['Left Ankle'] - temp_ankle  #update new ankle position
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
            temp_ankle = copy.deepcopy(self.new_joints['Right Ankle'])  # Store the current position
            if not np.isnan(rotated).any(): 
                self.new_joints['Right Knee'] = rotated * np.linalg.norm(stop) + start  # scale the rotated vetcor to the original length
                displacement = self.new_joints['Right Knee'] - temp_knee  #update new knee position
                if self.imu_comboboxes[4].get() == "None":
                    move_chain('Right Ankle', displacement)
                if self.imu_comboboxes[5].get() == "None":
                    displacement = self.new_joints['Right Ankle'] - temp_ankle  #update new ankle position
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

    def reset_pose_plot(self):
        self.ax.clear()
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_xlim3d([-1, 7])
        self.ax.set_ylim3d([-3.5, 3.5])
        self.ax.set_zlim3d([-3.5, 3.5])
        self.ax.set_autoscale_on(False)            
        
    def reset_heading(self):
        # self.imu.reset_heading()  
        self.reset_heading_flag = True
        
    # Methods for the Visualization tab
    def on_frame_slider_change(self, value):
        frame_idx = int(float(value))
        self.frame_nr_var.set(frame_idx)
        self.data_changed = False
        self._scrub_target = frame_idx
        if getattr (self,'_scrub_pending', None) is None:
            self._scrub_pending =self.after(5, self._do_scrub)

    def _do_scrub (self):
        self._scrub_pending = None
        self.viz_scrub(self._scrub_target)

    def increase_frame_nr_button(self):
        self.frame_scroll_active = True 
        self.auto_increase_frame()

    def stop_frame_scroll(self):
        self.frame_scroll_active = False

    def auto_increase_frame(self):
        if not self.frame_scroll_active:
            return
        current = self.frame_nr_var.get()
        max_frame = int(self.select_frame_scrollbar['to'])
        if current < max_frame:
            next_f = current + 1
            self.frame_nr_var.set(next_f)
            self.data_changed = False
            self.viz_scrub(next_f)
            self.after(10, self.auto_increase_frame) 

    def decrease_frame_nr_button(self):
        self.frame_scroll_active = True
        self.auto_decrease_frame()

    def auto_decrease_frame(self):
        if not self.frame_scroll_active:
            return
        current = self.frame_nr_var.get()
        if current > 0:
            prev_f = current - 1
            self.frame_nr_var.set(prev_f)
            self.data_changed = False
            self.viz_scrub(prev_f)
        self.after(10, self.auto_decrease_frame)  

    def viz_scrub(self, frame_idx):
        if not hasattr (self, 'loaded_acc_data') or not self.loaded_acc_data:
            return
        if self.data_changed:
            self.update_visualization_plots(frame_idx)
        else:
            self.fast_play_update(frame_idx)
            self._schedule_full_redraw(frame_idx)

    pending_redraw_id=None


    def _schedule_full_redraw(self, frame_idx):
        #cancel any pending full redraw and reschedule 300ms after the last scrub tick
    
        if self.pending_redraw_id is not None:
            self.after_cancel(self.pending_redraw_id)
        self.pending_redraw_id = self.after(30, lambda: self.full_redraw(frame_idx))

    def full_redraw(self, frame_idx):
        self.pending_redraw_id = None
        current = self.frame_nr_var.get()
        self.update_visualization_plots(current)
            



    def load_selected_camera_mp4(self, camera_id):
     try:
        if camera_id == 1:
            filename = self.camera1_mp4_combobox.get()
        else:
            filename = self.camera2_mp4_combobox.get()

        path = self.camera_mp4_files.get(filename)
        if not path:
            logger.warning(f"No path found for {filename}")
            return

        data = self.read_video_frames(path)  

        if camera_id == 1:
            self.loaded_camera1_frames = data
        else:
            self.loaded_camera2_frames = data

        logger.info(f"Loaded Camera {camera_id} from {path}")
        if hasattr(self, 'timestamps'):
            self.select_frame_scrollbar['to'] = len(data) - 1
            if hasattr(self, 'ann_frame_slider'):
                self.ann_frame_slider['to'] = len(data) - 1

        self.update_camera_views(self.frame_nr_var.get())

     except Exception as e:
        logger.error(f"[Load Camera {camera_id}] Error: {e}")


    def read_video_frames(self, video_path):
        frames = []
        cam = cv2.VideoCapture(video_path)
        while cam.isOpened():
            ret, frame = cam.read()
            if not ret:
                break
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frames.append(frame_rgb)
        cam.release()
        return frames

    def load_recordings(self):
        if hasattr(self, 'is_playing') and self.is_playing:
            self.toggle_play()
        try:
            folder_path = filedialog.askdirectory(title="Select Recordings Folder")
            if not folder_path:
                logger.warning("No folder selected.")
                return

            self.load_path = folder_path

        # Find all CSV files (full filenames)
            csv_files = [f for f in os.listdir(folder_path) if f.endswith('.csv')]
            if not csv_files:
                logger.warning("No CSV files found in the selected folder.")
                self.available_recording_combobox['values'] = []
                self.available_recording_combobox['state'] = 'disabled'
                return

            self.available_recording_combobox['values'] = csv_files
            self.available_recording_combobox['state'] = 'readonly'
            self.available_recording_combobox.set(csv_files[0])  # set first

        except Exception as e:
            logger.error(f"[Load Recordings] Error: {e}")

        selected_file = os.path.join(folder_path, self.available_recording_combobox.get())
        df = pd.read_csv(selected_file)

        num_frames = len(df)
        num_imus = 6

        self.loaded_acc_data = []
        self.loaded_ang_data = []
        self.loaded_mag_data = []

        # Start after timestamp
        col_idx = 1

        for imu_idx in range(num_imus):
            quat_cols = df.iloc[:, col_idx : col_idx + 4]
            acc_cols = df.iloc[:, col_idx + 4 : col_idx + 7]
            ang_cols = df.iloc[:, col_idx + 7 : col_idx + 10]
            mag_cols = df.iloc[:, col_idx + 10 : col_idx + 13]

            self.loaded_acc_data.append(acc_cols.values)
            self.loaded_ang_data.append(ang_cols.values)
            self.loaded_mag_data.append(mag_cols.values)

            col_idx += 13  # move to next IMU

        self.select_frame_scrollbar['to'] = num_frames - 1
        if hasattr(self, 'ann_frame_slider'):
            self.ann_frame_slider['to'] = num_frames - 1
            self.update_ann_frame_label(0)
        self.data_changed = True

        logger.debug(f"Loaded {num_frames} frames, {num_imus} IMUs, for visualization.")

    
    def on_select_recording(self, event):
     try:
        selected_file = self.available_recording_combobox.get()
        if not selected_file:
            logger.warning("No file selected in combobox.")
            return

        folder_path = self.load_path
        csv_path = os.path.join(folder_path, selected_file)
        base_path = os.path.splitext(csv_path)[0]
        cam1_path = base_path + "_camera1.mp4"
        cam2_path = base_path + "_camera2.mp4"

        # Load IMU data
        if os.path.exists(csv_path):
            df_full = pd.read_csv(csv_path)
            self.loaded_imu_data = df_full.iloc[:-1].copy()
            # Detect and set pose from last line of CSV
            try:
                with open(csv_path, 'r') as f:
                    lines = f.readlines()
                    if lines:
                        last_line = lines[-1].strip()
                        if last_line.startswith("#POSE="):
                            pose_code = last_line.split("=")[-1]
                            if pose_code == "L":
                                self.imu_pose_selection.set("Lying")
                            elif pose_code=="S":
                                self.imu_pose_selection.set("Sitting")
                            else:
                                self.imu_pose_selection.set("Standing")
            except Exception as e:
                logger.error(f"Could not read pose info from file: {e}")

            self.timestamps = self.loaded_imu_data["timestamp"].to_numpy()
    
            self.loaded_acc_data = []
            self.loaded_ang_data = []
            self.loaded_mag_data = []

            for imu_index in range(6):
                acc_cols = [f"acc.x({imu_index})", f"acc.y({imu_index})", f"acc.z({imu_index})"]
                ang_cols = [f"ang.x({imu_index})", f"ang.y({imu_index})", f"ang.z({imu_index})"]
                mag_cols = [f"mag.x({imu_index})", f"ang.y({imu_index})", f"mag.z({imu_index})"]

                acc_values = self.loaded_imu_data[acc_cols].to_numpy()
                ang_values = self.loaded_imu_data[ang_cols].to_numpy()
                mag_values = self.loaded_imu_data[mag_cols].to_numpy()

                self.loaded_acc_data.append(acc_values)
                self.loaded_ang_data.append(ang_values)
                self.loaded_mag_data.append(mag_values)
     
            logger.info(f"Loaded IMU data from {csv_path}")
        else:
            logger.error(f"CSV file not found: {csv_path}")
            return
        
        imu_tickbox_map = {
            0: self.left_thigh_checkbox_var,
            1: self.left_calf_checkbox_var,
            2: self.left_foot_checkbox_var,
            3: self.right_thigh_checkbox_var,
            4: self.right_calf_checkbox_var,
            5: self.right_foot_checkbox_var
        }

        
        for imu_index in range(6):
            acc_cols = [f"acc.x({imu_index})", f"acc.y({imu_index})", f"acc.z({imu_index})"]

            if all(col in self.loaded_imu_data.columns for col in acc_cols):
                acc_data = self.loaded_imu_data[acc_cols].to_numpy()
                if not np.isnan(acc_data).all():
                    imu_tickbox_map[imu_index].set(True)   # tickbox activation
                else:
                    imu_tickbox_map[imu_index].set(False)  # tickbox deactivation
            else:
                imu_tickbox_map[imu_index].set(False)

        # Load camera frames
        camera1_exists = os.path.exists(cam1_path)
        camera2_exists = os.path.exists(cam2_path)

        cam1_file = os.path.basename(cam1_path)
        cam2_file = os.path.basename(cam2_path)

        if camera1_exists:
            self.loaded_camera1_frames = self.read_video_frames(cam1_path)
            logger.info(f"Loaded camera1 frames from {cam1_path}")
            self.camera1_mp4_combobox['values'] = [cam1_file]
            self.camera1_mp4_combobox['state'] = 'readonly'
            self.camera1_mp4_combobox.set(cam1_file)
            self.camera1_mp4_combobox.bind("<<ComboboxSelected>>", lambda e: self.load_selected_camera_mp4(1))
            if not hasattr(self, 'camera_mp4_files'):
                self.camera_mp4_files = {}
            self.camera_mp4_files[cam1_file] = cam1_path
        else:
            self.loaded_camera1_frames = None
            self.cam3_label.config(image='', text="Front View (no video)", fg='gray')
            self.cam3_photo = None
            self.camera1_mp4_combobox['values'] = []
            self.camera1_mp4_combobox.set('')
            self.camera1_mp4_combobox['state'] = 'disabled'
            logger.error(f"Camera1 file not found: {cam1_path}")

        if camera2_exists:
            self.loaded_camera2_frames = self.read_video_frames(cam2_path)
            logger.info(f"Loaded camera2 frames from {cam2_path}")
            self.camera2_mp4_combobox['values'] = [cam2_file]
            self.camera2_mp4_combobox['state'] = 'readonly'
            self.camera2_mp4_combobox.set(cam2_file)
            self.camera2_mp4_combobox.bind("<<ComboboxSelected>>", lambda e: self.load_selected_camera_mp4(2))
            if not hasattr(self, 'camera_mp4_files'):
                self.camera_mp4_files = {}
            self.camera_mp4_files[cam2_file] = cam2_path
        else:
            self.loaded_camera2_frames = None
            self.cam4_label.config(image='', text="Side View (no video)", fg='gray')
            self.cam4_photo = None
            self.camera2_mp4_combobox['values'] = []
            self.camera2_mp4_combobox.set('')
            self.camera2_mp4_combobox['state'] = 'disabled'
            logger.error(f"Camera2 file not found: {cam2_path}")

        # Update slider for IMU frames
        if len(self.loaded_imu_data) > 0:
            num_frames = len(self.loaded_imu_data)
            self.select_frame_scrollbar.config(to=num_frames - 1)
            if hasattr(self, 'ann_frame_slider'):
                self.ann_frame_slider.config(to=num_frames - 1)
                self.update_ann_frame_label(self.frame_nr_var.get())
            logger.debug(f"Updated slider to {num_frames} IMU frames.")
        
        self.data_changed = True
        
        # Calibration reference = frame 0
        self.visual_calibration_reference = {}
        for i in range(6):
            col_q = [f"quat.w({i})", f"quat.x({i})", f"quat.y({i})", f"quat.z({i})"]
            if all(col in self.loaded_imu_data.columns for col in col_q):
                q_ref = self.loaded_imu_data.loc[0, col_q].values.astype(float)
                norm_sq = np.sum(q_ref ** 2)
                if not np.any(np.isnan(q_ref)):
                    q_inv = np.array([q_ref[0], -q_ref[1], -q_ref[2], -q_ref[3]]) / norm_sq
                    self.visual_calibration_reference[i] = q_inv
                    logger.debug(f" IMU {i} reference saved from frame 0.")
                else:
                    logger.warning(f" Invalid quaternion for IMU {i} in frame 0.")

        json_path = base_path +"_annotations.json"
        self.show_annotations_checkbox_var.set(False)
        self.viz_annotations = []
        if not os.path.exists(json_path):
            self.annotations.clear()

        self.update_visualization_plots()
        self.on_ann_frame_slider_change(0)

        logger.info("Finished loading and visualizing data.")

     except Exception as e:
        logger.error(f"Error: {e}")

    def toggle_annotations_visibility(self):
        if self.show_annotations_checkbox_var.get():
            try:
                sel=self.available_recording_combobox.get()
                if not sel:
                    self.show_annotations_checkbox_var.set(False)
                    return
                base_path = os.path.splitext(os.path.join(self.load_path, sel))[0]
                json_path = base_path +"_annotations.json"

                if os.path.exists(json_path):
                    self.execute_load_annotations_silently(json_path)
                else:
                    logger.info("No matching annotations file found to show.")
            except Exception as e:
               logger.error(f"Error autoshowing annotations:{e}")
        
        self.data_changed = True
        self.update_visualization_plots(self.frame_nr_var.get())
    
    def execute_load_annotations_silently (self,json_path):
        try:
            with open(json_path,"r") as f:
                payload = json.load(f)

            self.viz_annotations =[]
            for a in payload.get("annotations",[]):
                self.viz_annotations.append({
                    "kind":       a.get("kind", "point"),
                    "timestamp":  float(a["timestamp"]),
                    "frame_idx":  int(a["frame_idx"]),
                    "t_end":      float(a["t_end"]) if a.get("t_end") is not None else None,
                    "frame_end":  int(a["frame_end"]) if a.get("frame_end") is not None else None,
                    "label":      a.get("label", ""),
                    "type":       a.get("type", "Event"),
                    "confidence": int(a.get("confidence", 5)),
                })
            self.update_annotation_listbox()
            self.data_changed =True
        except Exception as e:
            logger.error(f"Failed silent load payload:{e}")

    def trigger_update(self):
        self.data_changed = True
        self.update_visualization_plots(self.frame_nr_var.get())

    def update_visualization_plots(self, frame_idx=None):
     try:
        num_frames = len(self.loaded_acc_data[0]) if self.loaded_acc_data else 0
        if num_frames == 0:
            logger.warning("[Visualization Update] No data loaded.")
            return

        idx = self.frame_nr_var.get() if frame_idx is None else frame_idx
        if idx < 0 or idx >= num_frames:
            logger.warning("[Visualization Update] Index out of bounds.")
            return

        time_axis = self.timestamps

        # Active tickboxes
        active_flags = {
            "acc": self.acc_checkbox_var.get(),
            "ang": self.ang_checkbox_var.get(),
            "mag": self.mag_checkbox_var.get()
        }

        active_keys = [k for k, v in active_flags.items() if v]
        n = len(active_keys)

        if n == 0:
            logger.warning("No visualization types selected.")
            return

        if self.data_changed:
            self.fig_imu.clf()
            axes = self.fig_imu.subplots(n, 1, sharex=True) if n > 1 else [self.fig_imu.add_subplot(111)]
            self.ax_map = dict(zip(active_keys, axes))

        ax_map = self.ax_map  

        # Clear scatter points from each axis
        for key in active_keys:
            ax = ax_map[key]
            for coll in list(ax.collections):
                coll.remove()

        imu_tickbox_map = {
            0: self.left_thigh_checkbox_var,
            1: self.left_calf_checkbox_var,
            2: self.left_foot_checkbox_var,
            3: self.right_thigh_checkbox_var,
            4: self.right_calf_checkbox_var,
            5: self.right_foot_checkbox_var
        }

        imu_labels = ['L Thigh', 'L Calf', 'L Foot', 'R Thigh', 'R Calf', 'R Foot']

        for i in range(6):
            if not imu_tickbox_map[i].get():
                continue

            label = imu_labels[i]

            if "acc" in ax_map and not np.isnan(self.loaded_acc_data[i]).all():
                acc = self.loaded_acc_data[i]
                ax = ax_map["acc"]
                if self.data_changed:
                    ax.plot(time_axis, acc[:, 0], label=f'{label} Acc X')
                    ax.plot(time_axis, acc[:, 1], label=f'{label} Acc Y')
                    ax.plot(time_axis, acc[:, 2], label=f'{label} Acc Z')
                ax.scatter(time_axis[idx], acc[idx, 0], color='blue', s=40)
                ax.scatter(time_axis[idx], acc[idx, 1], color='cyan', s=40)
                ax.scatter(time_axis[idx], acc[idx, 2], color='navy', s=40)

            if "ang" in ax_map and not np.isnan(self.loaded_ang_data[i]).all():
                ang = self.loaded_ang_data[i]
                ax = ax_map["ang"]
                if self.data_changed:
                    ax.plot(time_axis, ang[:, 0], label=f'{label} Ang X')
                    ax.plot(time_axis, ang[:, 1], label=f'{label} Ang Y')
                    ax.plot(time_axis, ang[:, 2], label=f'{label} Ang Z')
                ax.scatter(time_axis[idx], ang[idx, 0], color='orange', s=40)
                ax.scatter(time_axis[idx], ang[idx, 1], color='gold', s=40)
                ax.scatter(time_axis[idx], ang[idx, 2], color='red', s=40)

            if "mag" in ax_map and not np.isnan(self.loaded_mag_data[i]).all():
                mag = self.loaded_mag_data[i]
                ax = ax_map["mag"]
                if self.data_changed:
                    ax.plot(time_axis, mag[:, 0], label=f'{label} Mag X')
                    ax.plot(time_axis, mag[:, 1], label=f'{label} Mag Y')
                    ax.plot(time_axis, mag[:, 2], label=f'{label} Mag Z')
                ax.scatter(time_axis[idx], mag[idx, 0], color='green', s=40)
                ax.scatter(time_axis[idx], mag[idx, 1], color='lime', s=40)
                ax.scatter(time_axis[idx], mag[idx, 2], color='darkgreen', s=40)

        # Titles and axis info
        if "acc" in ax_map:
            ax = ax_map["acc"]
            ax.set_ylabel("m/s²")
            #ax.set_title("Acceleration")
            ax.grid(True)
            ax.legend()
            ax.xaxis.set_major_locator(MaxNLocator(nbins=6))
        if "ang" in ax_map:
            ax = ax_map["ang"]
            ax.set_ylabel("deg/s")
            #ax.set_title("Angular Velocity")
            ax.grid(True)
            ax.legend()
            ax.xaxis.set_major_locator(MaxNLocator(nbins=6))
        if "mag" in ax_map:
            ax = ax_map["mag"]
            ax.set_ylabel("a.u.")
            #ax.set_title("Magnetic Field")
            ax.set_xlabel("Timestamp")
            ax.grid(True)
            ax.legend()
            ax.xaxis.set_major_locator(MaxNLocator(nbins=6))

        #Title
        title_parts = []
        if self.acc_checkbox_var.get():
            title_parts.append("Acceleration")
        if self.ang_checkbox_var.get():
            title_parts.append("Angular Velocity")
        if self.mag_checkbox_var.get():
            title_parts.append("Magnetic Field")

        overall_title = " | ".join(title_parts)
        #self.fig_imu.suptitle(overall_title, fontsize=12)
        
        self.fig_imu.suptitle(overall_title, fontsize=10, fontweight='bold')
        self.fig_imu.subplots_adjust(left=0.08, right=0.98, top=0.93, bottom=0.06, hspace=0.35)

        # Draw annotations on top first, then render once
        if self.show_annotations_checkbox_var.get():
            self.draw_viz_annotations()

        self.canvas_imu.draw()

        # After a full redraw, (re-)capture the background for blit-based cursor
        self.setup_imu_cursor()

        self.update_pose_viewer(idx)
        self.update_camera_views(idx)

        self.data_changed =False

     except Exception as e:
        logger.error(f"Error: {e}")

    def draw_viz_annotations(self):
        """Draw annotations on the visualization tab IMU axes only."""
        if not hasattr(self, 'ax_map'):
            return

        _type_colors = {
            "Event": "purple", "Phase Start": "green",
            "Phase End": "red", "Artifact": "orange", "Custom": "royalblue",
        }
        _y_fracs = [0.95, 0.75, 0.55, 0.35, 0.15]

        for ax in self.ax_map.values():
            for artist in list(ax.lines):
                if getattr(artist, '_is_annotation', False):
                    artist.remove()
            for artist in list(ax.texts):
                if getattr(artist, '_is_annotation', False):
                    artist.remove()
            for artist in list(ax.patches):
                if getattr(artist, '_is_annotation', False):
                    artist.remove()

            ymin, ymax = ax.get_ylim()
            y_span = ymax - ymin
            point_idx = 0
            for annot in self.viz_annotations:
                color = _type_colors.get(annot.get('type', ''), 'purple')
                label = annot.get('label', '')
                conf  = annot.get('confidence', 5)
                kind  = annot.get('kind', 'point')
                if kind == 'segment':
                    t_start = annot.get('frame_idx', annot['timestamp'])
                    t_end   = annot.get('frame_end', annot['t_end'])
                    span = ax.axvspan(t_start, t_end, alpha=0.18, color=color, linewidth=0)
                    span._is_annotation = True
                    for tx in (t_start, t_end):
                        bl = ax.axvline(x=tx, color=color, linestyle='-', linewidth=1.0, alpha=0.6)
                        bl._is_annotation = True
                    t_mid = (t_start + t_end) / 2
                    conf_stars = '★' * conf + '☆' * (5 - conf)
                    stxt = ax.text(t_mid, ymin + y_span * 0.85,
                                f" {label}\n{conf_stars} ",
                                color=color, fontsize=7, fontweight='bold',
                                ha='center', va='top', clip_on=True,
                                bbox=dict(boxstyle='round,pad=0.2', facecolor='white',
                                            edgecolor=color, linewidth=0.9, alpha=0.88))
                    stxt._is_annotation = True
                else:
                    ts = annot.get('frame_idx', annot['timestamp'])
                    line = ax.axvline(x=ts, color=color, linestyle='--',
                                    linewidth=1.2, alpha=0.85)
                    line._is_annotation = True
                    y_pos = ymin + _y_fracs[point_idx % len(_y_fracs)] * y_span
                    conf_stars = '★' * conf + '☆' * (5 - conf)
                    txt = ax.text(ts, y_pos, f" {label} {conf_stars} ",
                                color=color, fontsize=7, fontweight='bold',
                                rotation=90, va='top', ha='center', clip_on=True,
                                bbox=dict(boxstyle='round,pad=0.15', facecolor='white',
                                            edgecolor=color, linewidth=0.8, alpha=0.85))
                    txt._is_annotation = True
                    point_idx += 1

        self.canvas_imu.draw_idle()

    def update_camera_views(self, frame_idx):
     try:
        # Camera 1 (Front View) — rendered on a tk.Label via PIL for speed
        if self.loaded_camera1_frames is not None and frame_idx < len(self.loaded_camera1_frames):
            frame = self.loaded_camera1_frames[frame_idx]
            self.show_frame_on_label(self.cam3_label, frame, 'cam3_photo')

        # Camera 2 (Side View) — same fast PIL rendering
        if self.loaded_camera2_frames is not None and frame_idx < len(self.loaded_camera2_frames):
            frame = self.loaded_camera2_frames[frame_idx]
            self.show_frame_on_label(self.cam4_label, frame, 'cam4_photo')

     except Exception as e:
        logger.error(f"Error at frame {frame_idx}: {e}")

    def show_frame_on_label(self, label, frame_rgb, photo_attr):
        """Render a numpy RGB frame onto a tk.Label using PIL — no matplotlib overhead."""
        try:
            w = label.winfo_width()
            h = label.winfo_height()
            if w < 10 or h < 10:
                w, h = 320, 240  # fallback before widget is sized
            img = Image.fromarray(frame_rgb)
            img.thumbnail((w, h), Image.BILINEAR)
            photo = ImageTk.PhotoImage(img)
            label.config(image=photo, text='')
            setattr(self, photo_attr, photo)  # keep a reference to prevent GC
        except Exception as e:
            logger.error(f"show_frame_on_label error: {e}")

    def sync_ann_imu(self, idx):
        """Draw a condensed IMU signal overview on the annotation-tab figure."""
        try:
            if not hasattr(self, 'loaded_acc_data') or not self.loaded_acc_data:
                return

            time_axis = self.timestamps
            active_keys = []
            if self.ann_acc_checkbox_var.get():
                active_keys.append("acc")
            if self.ann_ang_checkbox_var.get():
                active_keys.append("ang")
            if self.ann_mag_checkbox_var.get():
                active_keys.append("mag")
            if not active_keys:
                return

            n = len(active_keys)
            self.ann_fig_imu.clf()
            axes = (self.ann_fig_imu.subplots(n, 1, sharex=True)
                    if n > 1 else [self.ann_fig_imu.add_subplot(111)])
            ax_map = dict(zip(active_keys, axes))

            imu_tickbox_map = {
                0: self.ann_left_thigh_var,
                1: self.ann_left_calf_var,
                2: self.ann_left_foot_var,
                3: self.ann_right_thigh_var,
                4: self.ann_right_calf_var,
                5: self.ann_right_foot_var,
            }
            imu_labels = ['L Thigh', 'L Calf', 'L Foot', 'R Thigh', 'R Calf', 'R Foot']

            for i in range(6):
                if not imu_tickbox_map[i].get():
                    continue
                lbl = imu_labels[i]
                if "acc" in ax_map and not np.isnan(self.loaded_acc_data[i]).all():
                    acc = self.loaded_acc_data[i]
                    ax_map["acc"].plot(time_axis, acc[:, 0], linewidth=0.7)
                    ax_map["acc"].plot(time_axis, acc[:, 1], linewidth=0.7)
                    ax_map["acc"].plot(time_axis, acc[:, 2], linewidth=0.7)
                    ax_map["acc"].scatter(time_axis[idx], acc[idx, 0], color='blue', s=20, zorder=5)
                if "ang" in ax_map and not np.isnan(self.loaded_ang_data[i]).all():
                    ang = self.loaded_ang_data[i]
                    ax_map["ang"].plot(time_axis, ang[:, 0], linewidth=0.7)
                    ax_map["ang"].plot(time_axis, ang[:, 1], linewidth=0.7)
                    ax_map["ang"].plot(time_axis, ang[:, 2], linewidth=0.7)
                    ax_map["ang"].scatter(time_axis[idx], ang[idx, 0], color='orange', s=20, zorder=5)
                if "mag" in ax_map and not np.isnan(self.loaded_mag_data[i]).all():
                    mag = self.loaded_mag_data[i]
                    ax_map["mag"].plot(time_axis, mag[:, 0], linewidth=0.7)
                    ax_map["mag"].plot(time_axis, mag[:, 1], linewidth=0.7)
                    ax_map["mag"].plot(time_axis, mag[:, 2], linewidth=0.7)
                    ax_map["mag"].scatter(time_axis[idx], mag[idx, 0], color='green', s=20, zorder=5)

            # Draw a vertical line at current frame on every subplot
            t_current = time_axis[idx]
            label_map = {"acc": "m/s²", "ang": "deg/s", "mag": "a.u"}
            for key, ax in ax_map.items():
                ax.axvline(t_current, color='red', linewidth=1.2, linestyle='--')
                ax.set_ylabel(label_map.get(key, ""), fontsize=7)
                ax.tick_params(labelsize=6)
                ax.grid(True, linewidth=0.4)
                ax.xaxis.set_major_locator(MaxNLocator(nbins=5))

            self.ann_ax_map = ax_map
            self.ann_fig_imu.subplots_adjust(left=0.12, right=0.98,
                                              top=0.95, bottom=0.08, hspace=0.3)
        
            self.ann_canvas_imu.draw()

        except Exception as e:
            logger.error(f"[Ann IMU sync] {e}")

    def update_pose_viewer(self, frame_idx):
        try:
            self.ax6.clear()
            self.ax6.set_xlabel("X")
            self.ax6.set_ylabel("Y")
            self.ax6.set_zlabel("Z")
            self.ax6.set_xlim3d([-1, 7])
            self.ax6.set_ylim3d([-3.5, 3.5])
            self.ax6.set_zlim3d([-3.5, 3.5])
            self.ax6.set_title("Pose Viewer")
            self.ax6.set_autoscale_on(False)

            self.get_pose()
            self.new_joints = copy.deepcopy(self.joints)

            segment_labels = [ 
                "Left Thigh", "Left Calf", "Left Foot",
                "Right Thigh", "Right Calf", "Right Foot"
            ]
            
            checkbox_vars = [
                self.left_thigh_checkbox_var, self.left_calf_checkbox_var, self.left_foot_checkbox_var,
                self.right_thigh_checkbox_var, self.right_calf_checkbox_var, self.right_foot_checkbox_var
            ]

            if not hasattr(self, 'visual_calibration_reference'):
                self.visual_calibration_reference = {}
            for i in range(6):
                if not checkbox_vars[i].get():
                    continue

                col_q = [f"quat.w({i})", f"quat.x({i})", f"quat.y({i})", f"quat.z({i})"]
                if all(col in self.loaded_imu_data.columns for col in col_q):
                    q = self.loaded_imu_data.loc[frame_idx, col_q].values.astype(float)

                    if np.any(np.isnan(q)) :
                        logger.warning(f"Skipping IMU {i} due to invalid quaternion.")
                        continue  # skip empty quaternions

                    if self.show_calibrated_visual_checkbox_var.get():
                        if i in self.visual_calibration_reference:
                            q = self.multiply_quaternions(self.visual_calibration_reference[i], q)
                        else:
                            logger.warning(f"No calibration reference for IMU {i}")
                            continue  # no calibration available

                    R = self.get_rotation_matrix_quaternions(q)
                    self.rotate_leg_segment(segment_labels[i], R)
                    

            DEFAULT_SETTINGS.plot_body_parts(self.ax6, self.new_joints)
            title = "Pose Viewer (Calibrated)" if self.show_calibrated_visual_checkbox_var.get() else "Pose Viewer"
            self.ax6.set_title(title)
            self.canvas6.draw()   

        except Exception as e:
            logger.error(f"[Pose Viewer] Error updating pose: {e}")

    def get_calibration_data(self):
        self.imu.calibrate()        

    def multiply_quaternions(self, q1, q2):
        # Quaternion multiplication
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        ])

    """def calibrate_quaternions(self):
        calibrated_quaternions = np.empty([self.imu.numOfDevices, 4])
        for dev in range(self.imu.numOfDevices):
            if self.imu.calibration_status[dev]:
                calibrated_quaternions[dev,:] = self.multiply_quaternions(self.imu.calibration_inverse[dev,:], self.imu.quat_data[dev,:])
            else:
                print(f"Device {dev} is not calibrated")
        return calibrated_quaternions"""
        
    # Annotation Methods

    def get_current_annotation_base(self):
        """Return the common fields for any new annotation at the current frame."""
        frame_idx = self.ann_frame_nr_var.get()
        return {
            'timestamp':  float(self.timestamps[frame_idx]),
            'frame_idx':  int(frame_idx),
            'label':      self.annotation_label_var.get().strip() or "Event",
            'type':       self.annotation_type_var.get(),
            'confidence': int(self.annotation_confidence_var.get()),
            'leg_segment':  self.annotation_leg_var.get(),
        }

        # Point annotations

    def add_annotation_at_frame(self):
        """Add a point annotation at the current slider frame."""
        if not hasattr(self, 'loaded_imu_data') or self.loaded_imu_data is None:
            messagebox.showwarning("No Data", "Please load a recording first.")
            return
        annot = self.get_current_annotation_base()
        annot['kind'] = 'point'
        annot['t_end']      = None
        annot['frame_end']  = None
        self.annotations.append(annot)
        self.update_annotation_listbox()
        self.draw_annotations()
        logger.info(f"Point annotation: '{annot['label']}' at t={annot['timestamp']:.3f}s "
                    f"conf={annot['confidence']}")

    # Segment annotations

    def start_segment(self):
        """Mark the current frame as the start of a segment."""
        if not hasattr(self, 'loaded_imu_data') or self.loaded_imu_data is None:
            messagebox.showwarning("No Data", "Please load a recording first.")
            return
        base = self.get_current_annotation_base()
        self.segment_pending_start = base
        self.seg_status_var.set(
            f"▶  Open: '{base['label']}' at t={base['timestamp']:.2f}s — "
            f"now scrub to end point and click  ■ End")
        self.seg_start_btn.config(state='disabled')
        self.seg_end_btn.config(state='normal')
        self.draw_annotations()
        logger.info(f"Segment started: '{base['label']}' at t={base['timestamp']:.3f}s")

    def end_segment(self):
        """Close the open segment at the current frame and save it."""
        if self.segment_pending_start is None:
            return
        base  = self.segment_pending_start
        frame_end = self.ann_frame_nr_var.get()
        t_end     = float(self.timestamps[frame_end])

        if t_end <= base['timestamp']:
            messagebox.showwarning(
                "Invalid Segment",
                "End time must be after start time.\n"
                "Move the slider forward before clicking End Segment.")
            return

        annot = {
            'kind':        'segment',
            'timestamp':   base['timestamp'],    # t_start
            'frame_idx':   base['frame_idx'],    # frame_start (for Jump To)
            'label':       base['label'],
            'type':        base['type'],
            'confidence':  base['confidence'],
            't_end':       t_end,
            'frame_end':   int(frame_end),
            'leg_segment': self.annotation_leg_var.get(),
        }
        self.annotations.append(annot)
        self.segment_pending_start = None
        self.seg_status_var.set("No segment open")
        self.seg_start_btn.config(state='normal')
        self.seg_end_btn.config(state='disabled')
        self.update_annotation_listbox()
        self.draw_annotations()
        duration = t_end - annot['timestamp']
        logger.info(f"Segment closed: '{annot['label']}' "
                    f"t={annot['timestamp']:.3f}s → {t_end:.3f}s ({duration:.3f}s) "
                    f"conf={annot['confidence']}")

    # Drawing 

    def draw_annotations(self):
        """Overlay annotation markers on the annotation-tab IMU axes only."""
        if not hasattr(self, 'ann_ax_map'):
            return

        _type_colors = {
            "Event":       "purple",
            "Phase Start": "green",
            "Phase End":   "red",
            "Artifact":    "orange",
            "Custom":      "royalblue",
        }
        _y_fracs = [0.95, 0.75, 0.55, 0.35, 0.15]

        for ax in self.ann_ax_map.values():
            for artist in list(ax.lines):
                if getattr(artist, '_is_annotation', False):
                    artist.remove()
            for artist in list(ax.texts):
                if getattr(artist, '_is_annotation', False):
                    artist.remove()
            for artist in list(ax.patches):
                if getattr(artist, '_is_annotation', False):
                    artist.remove()

            ymin, ymax = ax.get_ylim()
            y_span = ymax - ymin

            # Pending segment start preview
            if self.segment_pending_start is not None:
                pts = self.segment_pending_start.get('frame_idx', self.segment_pending_start['timestamp'])
                prev = ax.axvline(x=pts, color='green', linestyle=':',
                                linewidth=1.5, alpha=0.7)
                prev._is_annotation = True
                ptxt = ax.text(pts, ymax - y_span * 0.02,
                            f" ▶{self.segment_pending_start['label']}… ",
                            color='green', fontsize=7, rotation=90,
                            va='top', ha='center', clip_on=True,
                            bbox=dict(boxstyle='round,pad=0.1', facecolor='#e8ffe8',
                                        edgecolor='green', linewidth=0.8, alpha=0.9))
                ptxt._is_annotation = True

            point_idx = 0
            for annot in self.annotations:
                color = _type_colors.get(annot.get('type', ''), 'purple')
                label = annot.get('label', '')
                conf  = annot.get('confidence', 5)
                kind  = annot.get('kind', 'point')

                if kind == 'segment':
                    t_start = annot.get('frame_idx', annot['timestamp'])
                    t_end   = annot.get('frame_end', annot['t_end'])
                    span = ax.axvspan(t_start, t_end, alpha=0.18, color=color, linewidth=0)
                    span._is_annotation = True
                    for tx in (t_start, t_end):
                        bl = ax.axvline(x=tx, color=color, linestyle='-',
                                        linewidth=1.0, alpha=0.6)
                        bl._is_annotation = True
                    t_mid = (t_start + t_end) / 2
                    conf_stars = '★' * conf + '☆' * (5 - conf)
                    stxt = ax.text(t_mid, ymin + y_span * 0.85,
                                f" {label}\n{conf_stars} ",
                                color=color, fontsize=7, fontweight='bold',
                                ha='center', va='top', clip_on=True,
                                bbox=dict(boxstyle='round,pad=0.2', facecolor='white',
                                            edgecolor=color, linewidth=0.9, alpha=0.88))
                    stxt._is_annotation = True
                else:
                    ts = annot.get('frame_idx', annot['timestamp'])
                    line = ax.axvline(x=ts, color=color, linestyle='--',
                                    linewidth=1.2, alpha=0.85)
                    line._is_annotation = True
                    y_pos = ymin + _y_fracs[point_idx % len(_y_fracs)] * y_span
                    conf_stars = '★' * conf + '☆' * (5 - conf)
                    txt = ax.text(ts, y_pos,
                                f" {label} {conf_stars} ",
                                color=color, fontsize=7, fontweight='bold',
                                rotation=90, va='top', ha='center', clip_on=True,
                                bbox=dict(boxstyle='round,pad=0.15', facecolor='white',
                                            edgecolor=color, linewidth=0.8, alpha=0.85))
                    txt._is_annotation = True
                    point_idx += 1

        self.ann_canvas_imu.draw_idle()
        self.redraw_ann_timeline()
        # Update count label
        if hasattr(self, 'ann_count_label_var'):
            n = len(self.annotations)
            pts  = sum(1 for a in self.annotations if a.get('kind') == 'point')
            segs = sum(1 for a in self.annotations if a.get('kind') == 'segment')
            self.ann_count_label_var.set(f"{n} total  ({pts} pt, {segs} seg)")

    def duplicate_selected_annotation(self):
            """Create a copy of the selected annotation, inserted right after it."""
            sel = self.annotation_listbox.curselection()
            if not sel:
                messagebox.showinfo("Select Annotation",
                                    "Please select an annotation to duplicate.")
                return

            idx = sel[0]
            new_annot = copy.deepcopy(self.annotations[idx])
            self.annotations.insert(idx + 1, new_annot)

            self.update_annotation_listbox()
            self.draw_annotations()
            self.annotation_listbox.selection_clear(0, tk.END)
            self.annotation_listbox.selection_set(idx + 1)
            self.annotation_listbox.see(idx + 1)
            logger.info(f"Annotation duplicated: '{new_annot.get('label', '')}' "
                        f"(kind={new_annot.get('kind', 'point')})")
            
    def edit_selected_annotation(self):
            """Open a small dialog to edit the selected annotation's fields."""
            sel = self.annotation_listbox.curselection()
            if not sel:
                messagebox.showinfo("Select Annotation",
                                    "Please select an annotation to edit.")
                return

            idx = sel[0]
            annot = self.annotations[idx]
            kind = annot.get('kind', 'point')

            win = tk.Toplevel(self)
            win.title("Edit Annotation")
            win.transient(self)
            win.grab_set()
            win.resizable(False, False)

            frm = ttk.Frame(win, padding=10)
            frm.grid(row=0, column=0, sticky='nsew')

            row = 0

            # Label
            ttk.Label(frm, text="Label:").grid(row=row, column=0, sticky='w', padx=3, pady=3)
            label_var = tk.StringVar(value=annot.get('label', ''))
            ttk.Entry(frm, textvariable=label_var, width=20).grid(row=row, column=1, sticky='ew', padx=3, pady=3)
            row += 1

            # Type
            ttk.Label(frm, text="Type:").grid(row=row, column=0, sticky='w', padx=3, pady=3)
            type_var = tk.StringVar(value=annot.get('type', 'Event'))
            ttk.Combobox(frm, textvariable=type_var, state='readonly',
                        values=["Event", "Phase Start", "Phase End", "Artifact", "Custom"]
                        ).grid(row=row, column=1, sticky='ew', padx=3, pady=3)
            row += 1

           # Leg segment
            ttk.Label(frm, text="Leg:").grid(row=row, column=0, sticky='w', padx=3, pady=3)
            leg_var = tk.StringVar(value=annot.get('leg_segment', ''))
            ttk.Combobox(frm, textvariable=leg_var, state='readonly',
                        values=["", "Left", "Right", "Both"]
                        ).grid(row=row, column=1, sticky='ew', padx=3, pady=3)
            row += 1

            # Confidence
            ttk.Label(frm, text="Confidence:").grid(row=row, column=0, sticky='w', padx=3, pady=3)
            conf_var = tk.IntVar(value=annot.get('confidence', 5))
            tk.Scale(frm, from_=1, to=5, orient='horizontal', variable=conf_var,
                    showvalue=True, length=120, tickinterval=1
                    ).grid(row=row, column=1, sticky='ew', padx=3, pady=3)
            row += 1

            # Frame (start)
            max_frame = int(self.ann_frame_slider['to'])
            start_label = "Frame:" if kind == 'point' else "Start Frame:"
            ttk.Label(frm, text=start_label).grid(row=row, column=0, sticky='w', padx=3, pady=3)
            frame_var = tk.IntVar(value=annot.get('frame_idx', 0))
            ttk.Spinbox(frm, from_=0, to=max_frame, textvariable=frame_var, width=10
                        ).grid(row=row, column=1, sticky='w', padx=3, pady=3)
            row += 1

            # End Frame (segments only)
            end_frame_var = None
            if kind == 'segment':
                ttk.Label(frm, text="End Frame:").grid(row=row, column=0, sticky='w', padx=3, pady=3)
                end_frame_var = tk.IntVar(value=annot.get('frame_end', annot.get('frame_idx', 0)))
                ttk.Spinbox(frm, from_=0, to=max_frame, textvariable=end_frame_var, width=10
                            ).grid(row=row, column=1, sticky='w', padx=3, pady=3)
                row += 1

            # Buttons
            btn_frame = ttk.Frame(frm)
            btn_frame.grid(row=row, column=0, columnspan=2, sticky='ew', pady=(10, 0))
            btn_frame.grid_columnconfigure(0, weight=1)
            btn_frame.grid_columnconfigure(1, weight=1)

            def on_save():
                new_frame_idx = frame_var.get()
                if not (0 <= new_frame_idx <= max_frame):
                    messagebox.showwarning("Invalid Frame", "Frame index is out of range.")
                    return

                if kind == 'segment':
                    new_frame_end = end_frame_var.get()
                    if not (0 <= new_frame_end <= max_frame):
                        messagebox.showwarning("Invalid Frame", "End frame index is out of range.")
                        return
                    if new_frame_end <= new_frame_idx:
                        messagebox.showwarning("Invalid Segment",
                                            "End frame must be after start frame.")
                        return
                    annot['frame_end'] = int(new_frame_end)
                    annot['t_end'] = float(self.timestamps[new_frame_end])

                annot['label'] = label_var.get().strip() or "Event"
                annot['type'] = type_var.get()
                annot['confidence'] = int(conf_var.get())
                annot['frame_idx'] = int(new_frame_idx)
                annot['timestamp'] = float(self.timestamps[new_frame_idx])
                annot['leg_segment'] = leg_var.get()

                self.update_annotation_listbox()
                self.draw_annotations()
                logger.info(f"Annotation edited: '{annot['label']}' "
                            f"(idx={idx}, kind={kind})")
                win.destroy()

            def on_cancel():
                win.destroy()

            ttk.Button(btn_frame, text="Save", command=on_save).grid(row=0, column=0, sticky='ew', padx=3)
            ttk.Button(btn_frame, text="Cancel", command=on_cancel).grid(row=0, column=1, sticky='ew', padx=3)

    #  Listbox / navigation 
    def update_annotation_listbox(self):
        """Refresh the annotation listbox widget."""
        self.annotation_listbox.delete(0, tk.END)
        for i, annot in enumerate(self.annotations):
            kind  = annot.get('kind', 'point')
            conf  = annot.get('confidence', 5)
            label = annot.get('label', '')
            atype = annot.get('type', '')[:6]
            stars = '★' * conf + '☆' * (5 - conf)
            if kind == 'segment':
                dur = annot['t_end'] - annot['timestamp']
                display = (f"[SEG] {label:<14s} {stars}  "
                           f"{annot['timestamp']:7.2f}→{annot['t_end']:7.2f}s ({dur:.2f}s)"
                           f" [{annot.get('leg_segment','')}]")
                self.annotation_listbox.insert(tk.END, display)
                self.annotation_listbox.itemconfig(i, foreground='#8B4513', background='#FFF8EE')
            else:
                display = (f"[PT ] {label:<14s} {stars}  "
                           f"t={annot['timestamp']:8.3f}s  [{atype}]"
                           f" [{annot.get('leg_segment','')}]")
                self.annotation_listbox.insert(tk.END, display)
                self.annotation_listbox.itemconfig(i, foreground='#1a3a6b', background='#EEF4FF')

    def jump_to_selected_annotation(self):
        """Move the frame slider to the start frame of the selected annotation."""
        sel = self.annotation_listbox.curselection()
        if not sel:
            messagebox.showinfo("Select Annotation",
                                "Please select an annotation to jump to.")
            return
        frame_idx = self.annotations[sel[0]]['frame_idx']
        self.ann_frame_nr_var.set(frame_idx)
        self.update_ann_frame_label(frame_idx)
        self.update_annotation_tab_views(frame_idx)

    def delete_selected_annotation(self):
        """Delete the annotation currently selected in the listbox."""
        sel = self.annotation_listbox.curselection()
        if not sel:
            messagebox.showinfo("Select Annotation",
                                "Please select an annotation to delete.")
            return
        removed = self.annotations.pop(sel[0])
        self.update_annotation_listbox()
        self.draw_annotations()
        logger.info(f"Annotation deleted: {removed}")

    def clear_all_annotations(self):
        """Clear all annotations after confirmation."""
        if not self.annotations:
            return
        if messagebox.askyesno("Clear Annotations",
                               "Clear ALL annotations? This cannot be undone."):
            self.annotations.clear()
            self.segment_pending_start = None
            try:
                self.seg_status_var.set("No segment open")
                self.seg_start_btn.config(state='normal')
                self.seg_end_btn.config(state='disabled')
            except Exception:
                pass
            self.update_annotation_listbox()
            self.draw_annotations()
            logger.info("All annotations cleared.")

    # Save / Load 

    def save_annotations_to_json(self):
        
        if not self.annotations:
            messagebox.showinfo("No Annotations", "There are no annotations to save.")
            return
        try:
            sel = self.available_recording_combobox.get()
            if not sel:
                messagebox.showwarning("No Recording", "Please load a recording first.")
                return
            base_path = os.path.splitext(os.path.join(self.load_path, sel))[0]
            json_path = base_path + "_annotations.json"
        except Exception as e:
            messagebox.showerror("Error", f"Could not determine save path:\n{e}")
            return

        import json, datetime
        payload = {
            "recording": sel,
            "saved_at":  datetime.datetime.now().isoformat(timespec='seconds'),
            "annotations": []
        }
        for a in self.annotations:
            payload["annotations"].append({
                "kind":       a.get("kind", "point"),
                "timestamp":  a["timestamp"],
                "frame_idx":  a["frame_idx"],
                "t_end":      a.get("t_end"),
                "frame_end":  a.get("frame_end"),
                "label":      a.get("label", ""),
                "type":       a.get("type", ""),
                "confidence": a.get("confidence", 5),
                "leg_segment": a.get("leg_segment", ""),
            })

        with open(json_path, "w") as f:
            json.dump(payload, f, indent=2)

        messagebox.showinfo("Saved",
                            f"Saved {len(self.annotations)} annotations to:\n"
                            f"{os.path.basename(json_path)}")
        logger.info(f"Annotations saved → {json_path}")

    def load_annotations_from_json(self):
        """Load the annotation JSON that matches the currently selected recording.
        No file dialog — it looks automatically in self.load_path for the file
        named <recording_basename>_annotations.json.
        Also ensures the matching camera videos are loaded."""
        
        try:
            sel = self.available_recording_combobox.get()
            if not sel:
                messagebox.showwarning("No Recording","Please select a recording first.")
                return

            base_path = os.path.splitext(os.path.join(self.load_path, sel))[0]
            json_path = base_path + "_annotations.json"

            if not os.path.exists(json_path):
                messagebox.showinfo("Not Found",
                                    f"No annotation file found for this recording.\n\n"
                                    f"Expected:\n{os.path.basename(json_path)}\n\n"
                                    f"Save your annotations first using "
                                    f"'💾 Save Annotations'.")
                return

            with open(json_path, "r") as f:
                payload = json.load(f)

            self.annotations = []
            for a in payload.get("annotations", []):
                self.annotations.append({
                    "kind":       a.get("kind", "point"),
                    "timestamp":  float(a["timestamp"]),
                    "frame_idx":  int(a["frame_idx"]),
                    "t_end":      float(a["t_end"]) if a.get("t_end") is not None else None,
                    "frame_end":  int(a["frame_end"]) if a.get("frame_end") is not None else None,
                    "label":      a.get("label", ""),
                    "type":       a.get("type", "Event"),
                    "confidence": int(a.get("confidence", 5)),
                    "leg_segment": a.get("leg_segment", ""),
                })

            self.update_annotation_listbox()
            self.data_changed = True
            self.update_visualization_plots(self.frame_nr_var.get())

            # Also load the corresponding camera videos if not already loaded
            cam1_path = base_path + "_camera1.mp4"
            cam2_path = base_path + "_camera2.mp4"
            if os.path.exists(cam1_path) and self.loaded_camera1_frames is None:
                self.loaded_camera1_frames = self.read_video_frames(cam1_path)
                logger.info(f"Auto-loaded camera1 from {cam1_path}")
            if os.path.exists(cam2_path) and self.loaded_camera2_frames is None:
                self.loaded_camera2_frames = self.read_video_frames(cam2_path)
                logger.info(f"Auto-loaded camera2 from {cam2_path}")

            saved_at = payload.get("saved_at", "unknown")
            messagebox.showinfo("Loaded",
                                f"Loaded {len(self.annotations)} annotations\n"
                                f"from: {os.path.basename(json_path)}\n"
                                f"saved at: {saved_at}")
            logger.info(f"Annotations loaded ← {json_path}")

        except Exception as e:
            messagebox.showerror("Error", f"Could not load annotations:\n{e}")
            logger.error(f"Error loading annotations: {e}")

    #End Annotation Methods 

    # WebSocket Server Control Methods
    def toggle_websocket_server(self):
        """Toggle WebSocket server on/off based on checkbox state."""
        if self.websocket_enabled_var.get():
            self.start_websocket_server()
        else:
            self.stop_websocket_server()

    def start_websocket_server(self):
        """Start the WebSocket server."""
        try:
            self.websocket_server = WebSocketServer("localhost", self.websocket_port)
            
            if self.websocket_server.start_server():
                self.websocket_enabled = True
                self.websocket_status_label.config(text="Running", foreground="green")
                self.update_websocket_client_count()
                logger.info(f"WebSocket server started on port {self.websocket_port}")
            else:
                self.websocket_enabled_var.set(False)
                self.websocket_status_label.config(text="Failed to start", foreground="red")
                logger.error("Failed to start WebSocket server")
                
        except Exception as e:
            self.websocket_enabled_var.set(False)
            self.websocket_status_label.config(text="Error", foreground="red")
            logger.error(f"Error starting WebSocket server: {e}")

    def stop_websocket_server(self):
        """Stop the WebSocket server."""
        try:
            if self.websocket_server:
                self.websocket_server.stop_server()
                self.websocket_server = None
                
            self.websocket_enabled = False
            self.websocket_status_label.config(text="Stopped", foreground="red")
            self.websocket_clients_label.config(text="0")
            
            logger.info("WebSocket server stopped")
            
        except Exception as e:
            logger.error(f"Error stopping WebSocket server: {e}")

    def update_websocket_client_count(self):
        """Periodically update the client count display."""
        if self.websocket_server and self.websocket_server.is_running():
            client_count = self.websocket_server.get_client_count()
            self.websocket_clients_label.config(text=str(client_count))
            self.after(2000, self.update_websocket_client_count)

    def send_websocket_data(self, timestamp, quat_data, acc_data, ang_data, mag_data=None):
        """Wrapper to send IMU data via WebSocket if the server is running."""
        if (self.websocket_server and 
            self.websocket_server.is_running() and 
            self.websocket_enabled and 
            hasattr(self, 'imu_ordered_configuration')):
            
            try:
                self.websocket_server.send_imu_data(
                    timestamp,
                    self.imu_ordered_configuration,
                    quat_data,
                    acc_data,
                    ang_data,
                    mag_data
                )
            except Exception as e:
                logger.error(f"Error sending WebSocket data: {e}")



    def toggle_play(self):
        """Toggle playback between Play and Pause."""
        if not hasattr(self, 'loaded_acc_data') or not self.loaded_acc_data:
            logger.warning("No data loaded for playback.")
            return

        if self.is_playing:
            self.is_playing = False
            self.play_button.config(text="▶ Play")
            # Full redraw on pause so scatter/annotations sync correctly
            self.data_changed = True
            self.update_visualization_plots(self.frame_nr_var.get())
            logger.info("Playback paused.")
        else:
            self.is_playing = True
            self.play_button.config(text="⏸ Pause")
            logger.info("Playback started.")
            self.run_play_loop()

    def run_play_loop(self):
        """Video-like play loop: fast camera (PIL) + IMU cursor (blit), no full redraw."""
        if not self.is_playing:
            return

        current_frame = self.frame_nr_var.get()
        max_frame = int(self.select_frame_scrollbar['to'])
        next_frame = current_frame + self.play_step

        if next_frame <= max_frame:
            self.frame_nr_var.set(next_frame)
            self.fast_play_update(next_frame)
            self.after(self.play_speed_ms, self.run_play_loop)
        else:
            self.frame_nr_var.set(max_frame)
            self.fast_play_update(max_frame)
            self.is_playing = False
            self.play_button.config(text="▶ Play")
            # Final full redraw when playback ends
            self.data_changed = True
            self.update_visualization_plots(max_frame)
            logger.info("Playback finished.")

    def fast_play_update(self, frame_idx):
        """
        Lightweight per-frame update during playback:
          - Camera views rendered directly onto tk.Label via PIL (no matplotlib draw)
          - IMU cursor moved via blit (no full figure redraw)
          - Pose viewer redrawn with draw_idle() — annotation tab mirror is skipped
        Annotation tab is NOT synced during play to keep things fast.
        """
        # Camera frames via PIL
        if self.loaded_camera1_frames is not None and frame_idx < len(self.loaded_camera1_frames):
            self.show_frame_on_label(self.cam3_label,
                                      self.loaded_camera1_frames[frame_idx],
                                      'cam3_photo')

        if self.loaded_camera2_frames is not None and frame_idx < len(self.loaded_camera2_frames):
            self.show_frame_on_label(self.cam4_label,
                                      self.loaded_camera2_frames[frame_idx],
                                      'cam4_photo')

        #IMU cursor via blit
        if (self._imu_bg is not None and self._imu_vlines
                and hasattr(self, 'timestamps') and frame_idx < len(self.timestamps)):
            try:
                t = self.timestamps[frame_idx]
                self.canvas_imu.restore_region(self._imu_bg)
                for ax, vline in self._imu_vlines:
                    vline.set_xdata([t, t])
                    ax.draw_artist(vline)
                self.canvas_imu.blit(self.fig_imu.bbox)
                self.canvas_imu.flush_events()
            except Exception as e:
                logger.error(f"IMU blit error: {e}")

        #Pose Viewer (3D skeleton) 
        # Use draw_idle() so Tk coalesces redraws and skips frames it can't keep
        # up with, rather than blocking the event loop.  The annotation tab
        # mirror (ann_ax6 / ann_canvas6) is skipped during play for speed.
        self.fast_pose_update(frame_idx)

    def fast_pose_update(self, frame_idx):
        """
        Redraw the 3D pose viewer for one frame during playback.
        Skips the annotation-tab mirror (ann_ax6/ann_canvas6) for speed.
        Uses draw_idle() so Tk can drop frames it cannot keep up with.
        """
        try:
            self.ax6.clear()
            self.ax6.set_xlabel("X")
            self.ax6.set_ylabel("Y")
            self.ax6.set_zlabel("Z")
            self.ax6.set_xlim3d([-1, 7])
            self.ax6.set_ylim3d([-3.5, 3.5])
            self.ax6.set_zlim3d([-3.5, 3.5])
            self.ax6.set_autoscale_on(False)

            self.get_pose()
            self.new_joints = copy.deepcopy(self.joints)

            segment_labels = [
                "Left Thigh", "Left Calf", "Left Foot",
                "Right Thigh", "Right Calf", "Right Foot"
            ]
            checkbox_vars = [
                self.left_thigh_checkbox_var, self.left_calf_checkbox_var,
                self.left_foot_checkbox_var, self.right_thigh_checkbox_var,
                self.right_calf_checkbox_var, self.right_foot_checkbox_var
            ]

            if not hasattr(self, 'visual_calibration_reference'):
                self.visual_calibration_reference = {}

            for i in range(6):
                if not checkbox_vars[i].get():
                    continue
                col_q = [f"quat.w({i})", f"quat.x({i})", f"quat.y({i})", f"quat.z({i})"]
                if not all(col in self.loaded_imu_data.columns for col in col_q):
                    continue
                q = self.loaded_imu_data.loc[frame_idx, col_q].values.astype(float)
                if np.any(np.isnan(q)):
                    continue
                if self.show_calibrated_visual_checkbox_var.get():
                    if i in self.visual_calibration_reference:
                        q = self.multiply_quaternions(self.visual_calibration_reference[i], q)
                    else:
                        continue
                R = self.get_rotation_matrix_quaternions(q)
                self.rotate_leg_segment(segment_labels[i], R)

            DEFAULT_SETTINGS.plot_body_parts(self.ax6, self.new_joints)
            title = ("Pose Viewer (Calibrated)"
                     if self.show_calibrated_visual_checkbox_var.get()
                     else "Pose Viewer")
            self.ax6.set_title(title)
            self.canvas6.draw_idle()   # non-blocking — Tk skips frames if behind

        except Exception as e:
            logger.error(f"[Pose Viewer fast update] Error: {e}")

    def setup_imu_cursor(self):
        """
        After a full IMU figure draw, plant an animated vertical cursor line on
        every subplot and save the clean background for blit-based updates.
        Call this after canvas_imu.draw() whenever the figure is rebuilt.
        """
        self._imu_vlines = []
        if not hasattr(self, 'ax_map') or not self.ax_map:
            return
        try:
            t0 = (self.timestamps[self.frame_nr_var.get()]
                  if hasattr(self, 'timestamps') and len(self.timestamps) > 0 else 0)
            for ax in self.ax_map.values():
                vline = ax.axvline(t0, color='red', linewidth=1.5,
                                   alpha=0.75, animated=True, zorder=10)
                self._imu_vlines.append((ax, vline))
            self.canvas_imu.draw()          # redraw once to include the (invisible) artists
            self._imu_bg = self.canvas_imu.copy_from_bbox(self.fig_imu.bbox)
        except Exception as e:
            logger.error(f"setup_imu_cursor error: {e}")
            self._imu_vlines = []
            self._imu_bg = None

    def change_playback_speed(self, event=None):
        """Adjust frame step and interval based on the speed combobox selection."""
        speed_map = {
            "0.5x": (50, 1),   # slower:  50 ms, 1 frame  → ~20 FPS equivalent
            "1.0x": (33, 1),   # normal:  33 ms, 1 frame  → ~30 FPS
            "1.5x": (33, 2),   # faster:  33 ms, 2 frames → ~60 FPS equivalent
            "2.0x": (33, 3),   # fast:    33 ms, 3 frames
            "3.0x": (33, 5),   # fastest: 33 ms, 5 frames
        }
        selected_speed = self.speed_combo.get()
        ms, step = speed_map.get(selected_speed, (33, 1))
        self.play_speed_ms = ms
        self.play_step = step
        logger.info(f"Playback speed set to {selected_speed} ({step} frame(s) / {ms} ms)")

# Run the application
if __name__ == "__main__":
    
    try:
        ctypes.windll.winmm.timeBeginPeriod(1)
    except Exception:
        pass
    app = IMURecordingStudio()
    
    app.mainloop()