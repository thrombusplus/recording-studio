import tkinter as tk
from tkinter import ttk, messagebox
from tkinter import filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg,NavigationToolbar2Tk
from matplotlib.ticker import MaxNLocator
import time
from threading import Thread
import threading
from IPython import get_ipython
import numpy as np
import pandas as pd
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
        
        self.latest_frame_cam1 =None
        self.latest_frame_cam2 = None


        self.data_queue = Queue() # Create a queue
        self.use_queue = False

        
        # Thread explanation: #Thread[0] and Thread[1] are used by webcams to stream upon pressing the Start/Stop button
        # Thread[2] will be used when Run/Stop Streaming button is pressed to stream IMU data in the IMU vector view.
        # When Start Recording Button is pressed -> Thread[0] will be used to Collect data from all sensors, Thread[1] will be used to stream webcam 1, 
        # Thread[2] will be used to stream webcam 2, Thread[3] will be used to plot IMU data

        self.thread = [None, None, None, None, None, None] # Stores threads
        self.thread_flag = [False, False, False, False, False, False] # Flag for stopping thread


        self.title("IMU Recording Studio")
        self.geometry("850x900")
        
        # Initate a Tab control
        self.tabControl = ttk.Notebook(self)
        
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

        # IMU Vector Control Frame (κάτω από το plot)
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

        self.imu_pose_selection=ttk.Combobox(frame, state="readonly", values = ["Sitting","Laying"])
        self.imu_pose_selection.set("Sitting")
        self.imu_pose_selection.grid(row=0,column=3, columnspan=1 )

        self.imu_pose_selection=ttk.Combobox(frame, state="readonly", values = ["Sitting","Laying"])
        self.imu_pose_selection.set("Sitting")
        self.imu_pose_selection.grid(row=0,column=3, columnspan=1 )
    

    def create_settings_tab(self):
        # IMU Sensor Status Frame
        self.imu_control_frame = ttk.LabelFrame(self.settings_tab, text="IMU Sensor Control")
        self.imu_control_frame.grid(row=0, column=0, padx=10, pady=10, sticky='nsew')

        # Connect Sensors Button
        self.frame_rate_list= ttk.Combobox(self.imu_control_frame, state="normal", values=["60", "30", "20", "15", "12", "10", "4", "1"])
        self.frame_rate_list.grid(row=0, column=0, columnspan=1)
        self.frame_rate_list.current(0)

        self.connect_button = ttk.Button(self.imu_control_frame, text="Connect Sensors", command=self.conenct_IMU_sensors)
        self.connect_button.grid(row=1, column=0, columnspan=2, pady=10, sticky='nsew')
        
        self.discoconnect_button = ttk.Button(self.imu_control_frame, text="Disconnect Sensors", command=self.disconnect_IMU_sensors)
        self.discoconnect_button.grid(row=2, column=0, columnspan=2, pady=10, sticky='nsew')
        
        self.sync_button = ttk.Button(self.imu_control_frame, text="Sync Sensors", command=self.sync_sensors)
        self.sync_button.grid(row=3, column=0, columnspan=2, pady=10, sticky='nsew') 

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
        self.imu_status_frame.grid(row=2, column=2, padx=10, pady=10, sticky='nsew')

        self.camera_status_frame = ttk.LabelFrame(self.settings_tab, text="Camera Sensor Status")
        self.camera_status_frame.grid(row=0, column=2, padx=10, pady=10, sticky='nsew')              

        # Connect Cameras Button
        self.connect_camera_button = ttk.Button(self.camera_status_frame, text="Read Cameras", command=self.connect_webcams)
        self.connect_camera_button.grid(row=0, column=0, columnspan=1, pady=10)
       
        self.disconnect_camera_button = ttk.Button(self.camera_status_frame, text="Disconnect Cameras", command=self.disconnect_webcams)
        self.disconnect_camera_button.grid(row=0, column=1, columnspan=1, pady=10)   

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

    def create_visualization_tab(self):

        for i in range(8):
            self.visualization_tab.grid_rowconfigure(i, weight=1)
        for j in range(11):
            self.visualization_tab.grid_columnconfigure(j, weight=1)

        # Front View Frame
        visualize_camera_frame_front = ttk.LabelFrame(self.visualization_tab, text="Front View")
        visualize_camera_frame_front.grid(row=4, column=3, columnspan=4,rowspan=4, padx=5, pady=5, sticky='nsew')
        visualize_camera_frame_front.grid_rowconfigure(0, weight=1)
        visualize_camera_frame_front.grid_columnconfigure(0, weight=1)

        self.fig3, self.ax3 = plt.subplots(figsize=(4, 3))
        self.ax3.set_title("Front View")
        self.ax3.axis('off')
        self.canvas3 = FigureCanvasTkAgg(self.fig3, master=visualize_camera_frame_front)
        self.canvas3.draw()
        self.canvas3.get_tk_widget().grid(row=0, column=0, sticky='nsew')

        # Side View Frame
        visualize_camera_frame_side = ttk.LabelFrame(self.visualization_tab, text="Side View")
        visualize_camera_frame_side.grid(row=4, column=7, columnspan=4, rowspan=4, padx=5, pady=5, sticky='nsew')
        visualize_camera_frame_side.grid_rowconfigure(0, weight=1)
        visualize_camera_frame_side.grid_columnconfigure(0, weight=1)

        self.fig4, self.ax4 = plt.subplots(figsize=(4, 3))
        self.ax4.set_title("Side View")
        self.ax4.axis('off')
        self.canvas4 = FigureCanvasTkAgg(self.fig4, master=visualize_camera_frame_side)
        self.canvas4.draw()
        self.canvas4.get_tk_widget().grid(row=0, column=0, sticky='nsew')

        # IMU Data Views Frame
        visualize_imus_frame = ttk.LabelFrame(self.visualization_tab, text="IMU Data Visualization")
        visualize_imus_frame.grid(row=0, column=3, rowspan=4,columnspan=8, padx=5, pady=5, sticky='nsew')

        self.fig_imu, self.ax_imu = plt.subplots(figsize=(12, 6))
        self.fig_imu.tight_layout()
        self.fig_imu.subplots_adjust(hspace=0.3)

        self.ax_imu.set_ylabel("Value")      
        self.ax_imu.set_xlabel("Time")       

        self.canvas_imu = FigureCanvasTkAgg(self.fig_imu, master=visualize_imus_frame)
        self.canvas_imu.draw()
        self.canvas_imu.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        
        # Add toolbar
        self.toolbar = NavigationToolbar2Tk(self.canvas_imu,visualize_imus_frame)
        self.toolbar.update()
        self.toolbar.pack(side=tk.BOTTOM, fill=tk.X)
        self.toolbar.pan()

        # Enable dynamic resizing
        visualize_imus_frame.grid_columnconfigure(0, weight=1)
        visualize_imus_frame.grid_rowconfigure(0, weight=1)

        # Pose Viewer
        recorded_data_imu_vector_viewer_frame = ttk.LabelFrame(self.visualization_tab, text="Pose Viewer")   
        recorded_data_imu_vector_viewer_frame.grid(row=3, column=0,rowspan=5,columnspan=3, padx=5, pady=5, sticky='nsew')
        recorded_data_imu_vector_viewer_frame.grid_rowconfigure(0, weight=1)
        recorded_data_imu_vector_viewer_frame.grid_columnconfigure(0, weight=1)

        self.fig6 = plt.figure(figsize=(4, 6))
        self.ax6 = self.fig6.add_subplot(111, projection='3d')   
        self.ax6.set_title("Pose Viewer")
        self.ax6.axis('off')
        self.canvas6 = FigureCanvasTkAgg(self.fig6, master=recorded_data_imu_vector_viewer_frame)
        self.canvas6.draw()
        self.canvas6.get_tk_widget().grid(row=0, column=0, sticky='nsew')

        # Create data visualization checkboxes
        select_data_frame = ttk.LabelFrame(self.visualization_tab, text="Data Control")
        select_data_frame.grid(row=0, column=0,rowspan=4, columnspan=3, padx=5, pady=5, sticky='nw')

        select_data_frame.config(width=250, height=250)
        select_data_frame.grid_rowconfigure(0, weight=0, minsize=20)
        # select_data_frame.grid_rowconfigure(1, weight=0)
        select_data_frame.grid_columnconfigure(0, weight=0,minsize=100)
        # select_data_frame.grid_columnconfigure(1, weight=1)
        # select_data_frame.grid_columnconfigure(2, weight=1)
        # select_data_frame.grid_columnconfigure(4, weight=1)
        
        # Camera 1 files
        self.camera1_npy_combobox = ttk.Combobox(select_data_frame, state="disabled", values=[])
        self.camera1_npy_combobox.grid(row=7, column=0, columnspan=1, sticky='ew', padx=5)

        # Camera 2 files
        self.camera2_npy_combobox = ttk.Combobox(select_data_frame, state="disabled", values=[])
        self.camera2_npy_combobox.grid(row=8, column=0, columnspan=1, sticky='ew', padx=5)

        # Button for camera files
        load_camera_button = ttk.Button(select_data_frame, text="Load Camera Files", command=self.load_camera_folder)
        load_camera_button.grid(row=7, column=1, columnspan=1, sticky='ew', padx=5, pady=5)

        # Create the Scale bar for scrolling through frames
        self.frame_nr_var = tk.IntVar()
        self.frame_nr_var.set(0)
        self.select_frame_scrollbar = tk.Scale(select_data_frame,from_=0, to=1 ,orient='horizontal', label="Frame Slider", variable=self.frame_nr_var)
        self.select_frame_scrollbar.grid(row=0, column=0, columnspan=4, padx=5, pady=5, sticky='nsew')
        self.select_frame_scrollbar.config(command=self.on_frame_slider_change)

        # Create buttons to increase decrease frame
        decrease_frame_button = ttk.Button(select_data_frame, text="◀", width=2, command=self.decrease_frame_nr_button)
        decrease_frame_button.grid(row=0, column=0, padx=5, pady=5, sticky='sw')
        increase_frame_button = ttk.Button(select_data_frame, text="▶", width=2, command=self.increase_frame_nr_button)
        increase_frame_button.grid(row=0, column=3, padx=5, pady=5, sticky='se')

        self.acc_checkbox_var = tk.BooleanVar()
        self.acc_checkbox_var.set(True)
        self.ang_checkbox_var = tk.BooleanVar()
        self.ang_checkbox_var.set(True)
        self.mag_checkbox_var = tk.BooleanVar()
        self.mag_checkbox_var.set(False)

        self.select_acceleration_tickbox = ttk.Checkbutton(select_data_frame, text="Acceleration", onvalue=True, offvalue=False, variable=self.acc_checkbox_var,command=self.update_visualization_plots)
        self.select_acceleration_tickbox.grid(row=2, column=0, padx=5, pady=5, columnspan=1, sticky='nsew')
        self.select_angular_velocity_tickbox = ttk.Checkbutton(select_data_frame, text="Angular Velocity", onvalue=True, offvalue=False, variable=self.ang_checkbox_var,command=self.update_visualization_plots)
        self.select_angular_velocity_tickbox.grid(row=2, column=1, padx=5, pady=5, columnspan=1, sticky='nsew')
        self.select_magnetic_filed_tickbox = ttk.Checkbutton(select_data_frame, text="Magnetic Field", onvalue=True, offvalue=False, variable = self.mag_checkbox_var,command=self.update_visualization_plots)
        self.select_magnetic_filed_tickbox.grid(row=2, column=2, padx=5, pady=5, columnspan=1,sticky='nsew')

        ttk.Label(select_data_frame, text="Plot Leg Segment:").grid(row=3, column=0, padx=10, pady=10, columnspan=1, sticky='nsew')    
        self.left_thigh_checkbox_var = tk.BooleanVar()
        self.left_thigh_checkbox_var.set(True)
        self.left_calf_checkbox_var = tk.BooleanVar()
        self.left_calf_checkbox_var.set(True)
        self.left_foot_checkbox_var = tk.BooleanVar()
        self.left_foot_checkbox_var.set(True)
        self.right_thigh_checkbox_var = tk.BooleanVar()
        self.right_thigh_checkbox_var.set(True)
        self.right_calf_checkbox_var = tk.BooleanVar()
        self.right_calf_checkbox_var.set(True)
        self.right_foot_checkbox_var = tk.BooleanVar()
        self.right_foot_checkbox_var.set(True)
        
        self.select_left_thigh_tickbox = ttk.Checkbutton(select_data_frame, text="Left Thigh", onvalue=True, offvalue=False, variable=self.left_thigh_checkbox_var)
        self.select_left_thigh_tickbox.grid(row=4, column=0, padx=5, pady=5, columnspan=1, sticky='new')
        self.select_left_calf_tickbox = ttk.Checkbutton(select_data_frame, text="Left Calf", onvalue=True, offvalue=False, variable=self.left_calf_checkbox_var)
        self.select_left_calf_tickbox.grid(row=4, column=1, padx=5, pady=5, columnspan=1, sticky='new')
        self.select_left_foot_tickbox = ttk.Checkbutton(select_data_frame, text="Left Foot", onvalue=True, offvalue=False, variable=self.left_foot_checkbox_var)
        self.select_left_foot_tickbox.grid(row=4, column=2, padx=5, pady=5, columnspan=1, sticky='new')
        self.select_right_thigh_tickbox = ttk.Checkbutton(select_data_frame, text="Right Thigh", onvalue=True, offvalue=False, variable=self.right_thigh_checkbox_var)
        self.select_right_thigh_tickbox.grid(row=5, column=0, padx=5, pady=5, columnspan=1, sticky='new')
        self.select_right_calf_tickbox = ttk.Checkbutton(select_data_frame, text="Right Calf", onvalue=True, offvalue=False, variable=self.right_calf_checkbox_var)
        self.select_right_calf_tickbox.grid(row=5, column=1, padx=5, pady=5, columnspan=1, sticky='new')
        self.select_right_foot_tickbox = ttk.Checkbutton(select_data_frame, text="Right Foot", onvalue=True, offvalue=False, variable=self.right_foot_checkbox_var)
        self.select_right_foot_tickbox.grid(row=5, column=2, padx=5, pady=5, columnspan=1, sticky='new')
        
        #Drop down with available recordings
        self.available_recording_combobox = ttk.Combobox(select_data_frame, state="disabled", values= 'None')
        self.available_recording_combobox.grid(row=6, column=0,padx=5,pady=5, columnspan=1, sticky='new')
        self.available_recording_combobox.config(width=15)
        
        self.available_recording_combobox.bind('<<ComboboxSelected>>', self.on_select_recording)

        load_button = ttk.Button(select_data_frame, text="Load Data", command=self.load_recordings)
        load_button.grid(row=6, column=1, padx=5, pady=5, columnspan=1, sticky='new')
        
        #self.update_plots_button = ttk.Button(select_data_frame, text="Update Plots", command=self.update_visualization_plots)
        #self.update_plots_button.grid(row=6, column=2, padx=10, pady=10, columnspan=1, sticky='new')

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
        print(f"Number of detected cameras: {len(self.idxInitiatedCameras)}")
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
            
            if cam.camera.read()[0]:
                self.InitiatedCameras.append(cam)
                self.idxInitiatedCameras.append(i)
                print(f"Camera {i} is working")
            else:
                print(f"Camera {i} not available")

    # Threads for handling camera 1 and camera 2
    def camera1_thread(self):
        
        try:
            cam_idx = int(self.camera_list_1.get())
            camera_manager = self.InitiatedCameras[cam_idx]
            camera_manager.streaming = True
        except Exception as e:
            print(f"[Camera 1 Thread] Could not initialize camera:{e}")
            return
        
        flag_index =1 if self.recording else 0

        while self.thread_flag[flag_index]:
            try:
                frame_1 = camera_manager.get_frame()
                self.latest_frame_cam1 = frame_1
                #Check if camera is already on recording mode
                if self.recording:
                    pass

            except Exception as e :
                print (f"[Camera Thread] Error: {e}")
                self.latest_frame_cam1 = None

        camera_manager.streaming = False

        time.sleep(0.03)

    def camera2_thread (self):

        try:
            cam_idx =int(self.camera_list_2.get())
            camera_manager = self.InitiatedCameras[cam_idx]
            camera_manager.streaming = True
        except Exception as e:
            print(f"[Camera 2 Thread] Could not initialize camera: {e}")
            return

        flag_index =3 if self.recording else 1

        while self.thread_flag[flag_index]:
            try:
                frame_2 = camera_manager.get_frame()
                self.latest_frame_cam2 = frame_2  

                if self.recording:
                    pass

            except Exception as e:
                print(f"[Camera2 Thread] Error: {e}") 
                self.latest_frame_cam2 = None  

        camera_manager.streaming = False
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
            print("Please select both camera views.")
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
        cam_idx = int(cameranumber)
        print(f"[View] Starting update_view for camera index {cam_idx}")
        
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
        if self.recording:
            print("Cannot start/stop streaming during recording.")

        if self.InitiatedCameras == []:
           print("No cameras connected.")
           return
        
        cam1=int(self.camera_list_1.get())
        cam2=int(self.camera_list_2.get())

        
        if self.InitiatedCameras[cam1].streaming:
           self.stop_streaming_mode()
           self.InitiatedCameras[cam1].streaming = False
           self.InitiatedCameras[cam2].streaming = False
           self.camera_list_1['state'] = 'readonly'
           self.camera_list_2['state'] = 'readonly'
           print("Streaming stopped.")
        else:
           self.InitiatedCameras[cam1].streaming = True
           self.InitiatedCameras[cam2].streaming = True
           self.camera_list_1['state'] = 'disabled'
           self.camera_list_2['state'] = 'disabled'
           self.stream_webcam()
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
       
        if self.recording:
            print("IMU already in recording mode.")
            return

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
               self.thread[i].join(0)
               self.thread[i] = None
               print(f"Thread {i} stopped.")

        self.camera_list_1['state']= 'readonly'
        self.camera_list_2['state']='readonly'

        for cam in self.InitiatedCameras:
            cam.streaming=False

    def stop_threads(self):
        for i in range(6):
        # Skip stopping IMU streaming thread if it’s running separately
          if i == 2 and self.imu_streaming and not self.recording:
            print("[DEBUG] Skipping stop of IMU streaming thread[2]")
            continue

        self.thread_flag[i] = False
        if self.thread[i] is not None:
            self.thread[i].join(0)
            self.thread[i] = None
            print(f" Stopped and cleared thread[{i}]")

    def start_recording(self):
      
      if self.recording:
        print("Recording already in progress.")
        return

      if not self.imu_lock_status:
        print("Configuration must be locked before starting recording.")
        return

      print("Preparing to start recording...")

      while not self.data_queue.empty():
        try:
            self.data_queue.get_nowait()
        except Exception as e:
            print(f"Error while clearing queue: {e}")
            break

      if not self.imu_streaming:
        print("Starting IMU measuring mode")
        self.start_imu_streaming()

    
      self.reset_heading_and_countdown(5)


    def reset_heading_and_countdown(self, seconds_left):
      if seconds_left == 5:  
        try:
            self.imu.reset_heading()
            self.initiate_plot()
            self.imu.get_measurments()
            print("IMU heading reset successfully.")
        except Exception as e:
            print(f"Error resetting IMU heading: {e}")

      if seconds_left > 0:
        print(f"Recording starts in: {seconds_left}...")
        self.after(1000, lambda: self.reset_heading_and_countdown(seconds_left - 1))
      else:
        
        self.start_recording_proper()

    def start_recording_proper(self):
      print("Starting actual recording...")

      self.stop_threads()

      self.use_queue = True
      self.recording = True
    
      self.thread_flag[0] = True
      self.thread[0] = Thread(target=self.camera1_thread)
      self.thread[0].start()
      print("Thread 0  started.")

   
      self.thread_flag[1] = True
      self.thread[1] = Thread(target=self.camera2_thread)
      self.thread[1].start()
      print("Thread 1 started.")
    
      self.thread_flag[3] = True
      self.thread[3] = Thread(target=self.update_imu_plot)
      self.thread[3].start()
      print("Thread 3 started.")

    
      self.thread_flag[4] = True
      self.thread[4] = Thread(target=self.update_camera_figures)
      self.thread[4].start()
      print("Thread 4  started.")

      self.thread_flag[5] = True  # New thread for  data collection
      self.thread[5] = Thread(target=self.data_collection_thread)
      self.thread[5].start()
      print("Thread 5  started.")

      print("Recording Started.")

    def stop_recording(self):
     if not self.recording:
        print("Recording stopped.")
        return

     print("Stopping recording...")

     self.recording = False
     self.use_queue = False

     for i in [0, 1, 3, 4, 5]:
        self.thread_flag[i] = False
        if self.thread[i] is not None:
            self.thread[i].join(0)
            self.thread[i] = None
            print(f"Thread {i} stopped.")

     for camera in self.InitiatedCameras:
        camera.streaming = False

     if self.imu_streaming:
        self.stop_imu_streaming()

     self.imu_streaming = False
     self.thread_flag[2] = False
     self.thread[2] = None

     print("Recording fully stopped.")

     # Save data
     if not hasattr(self, 'selected_data_dir'):
        self.selected_data_dir = FileManager(self.save_directory_field["text"])

     self.selected_data_dir.save_recording(
        self.data_queue,
        self.imu_ordered_configuration,
        self.patients_id_field.get(),
        self.exercises_list.get(),
        self.imu_pose_selection.get() #Sitting or Laying
     )
     print("Recording saved.")

     self.camera_list_1['state'] = 'readonly'
     self.camera_list_2['state'] = 'readonly'

     print("System fully reset: Ready for new streaming or recording.")

    def data_collection_thread(self):
        while self.thread_flag[5]:
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
                   data_entry['imu_acc'] = self.imu.acc_data.copy()
                   data_entry['imu_ang'] = self.imu.gyr_data.copy()
                   data_entry['imu_mag'] = self.imu.mag_data.copy()
                   data_entry['imu_ts'] = self.imu.sensor_timestamp.copy() #Sensor Timestamp

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

    def update_camera_figures(self): # Method used in recording mode.
        while self.thread_flag[4]:
            try:
                if self.latest_frame_cam1 is not None:
                    self.ax1.clear()
                    self.ax1.imshow(self.latest_frame_cam1)
                    self.ax1.axis('off')
                    self.canvas1.draw()
                    print("Camera 1 updated")
                if self.latest_frame_cam2 is not None:
                    self.ax2.clear()
                    self.ax2.imshow(self.latest_frame_cam2)
                    self.ax2.axis('off')
                    self.canvas2.draw()
                    print("Camera 2 updated")
                
                time.sleep(0.01)
            
            except Exception as e:
                print(f"[Thread 4] Error during figure update: {e}")
                

    def stop_imu_streaming(self):
        if self.imu_streaming:
           self.thread_flag[2] = False
           self.imu.stop_measuring_mode()

           if self.thread[2] is not None:
               self.thread[2].join(0)

               self.thread[2]= None

               print("IMU streaming stopped")
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

                time.sleep(1)

                flag_index = 2
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

      current_thread_index = 3 if self.recording else 2

      for _ in range(5):
        self.imu.get_measurments()
        time.sleep(0.01)

      while self.thread_flag[current_thread_index]:
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
        
    def reset_heading(self):
        # self.imu.reset_heading()  
        self.reset_heading_flag = True
        
    # Methods for the Visualization tab
    def decrease_frame_nr_button(self):
        current = self.frame_nr_var.get()
        if current > 0:
            self.frame_nr_var.set(current - 1)
            self.update_visualization_plots(current -1)
        else:
            print("Frame number is already at the minimum value")

    def increase_frame_nr_button(self):
        current = self.frame_nr_var.get()
        max_frame = int(self.select_frame_scrollbar['to'])
        if current < max_frame:
            self.frame_nr_var.set(current + 1)
            self.update_visualization_plots()
        else:
            print("Frame number is already at the maximum value")

    def on_frame_slider_change(self, value):
        current_frame = int(float(value))
        self.frame_nr_var.set(current_frame)
        self.update_visualization_plots(current_frame)  
    
    def load_camera_folder(self):
        folder = filedialog.askdirectory(title="Select Folder with NPY camera files")
        if not folder:
            return

        npy_files = [f for f in os.listdir(folder) if f.endswith(".npy")]
        npy_paths = [os.path.join(folder, f) for f in npy_files]

        if not npy_paths:
            print("No .npy files found.")
            return

        self.camera_npy_files = dict(zip(npy_files, npy_paths))  

        self.camera1_npy_combobox['values'] = npy_files
        self.camera2_npy_combobox['values'] = npy_files
        self.camera1_npy_combobox['state'] = 'readonly'
        self.camera2_npy_combobox['state'] = 'readonly'

        # Ορίζουμε default τιμές
        self.camera1_npy_combobox.set(npy_files[0])
        if len(npy_files) > 1:
            self.camera2_npy_combobox.set(npy_files[1])
        else:
            self.camera2_npy_combobox.set(npy_files[0])

        # σύνδεση event
        self.camera1_npy_combobox.bind("<<ComboboxSelected>>", lambda e: self.load_selected_camera_npy(1))
        self.camera2_npy_combobox.bind("<<ComboboxSelected>>", lambda e: self.load_selected_camera_npy(2))

        # φορτώνουμε τα αρχικά default αρχεία
        self.load_selected_camera_npy(1)
        self.load_selected_camera_npy(2)


    def on_select_camera_npy(self, event):
     selected_file = self.camera_npy_combobox.get()
     self.load_selected_camera_npy(selected_file)

    def load_selected_camera_npy(self, camera_id):
     try:
        if camera_id == 1:
            filename = self.camera1_npy_combobox.get()
        else:
            filename = self.camera2_npy_combobox.get()

        path = self.camera_npy_files.get(filename)
        if not path:
            print(f"No path found for {filename}")
            return

        data = np.load(path, allow_pickle=True)
        if camera_id == 1:
            self.loaded_camera1_frames = data
        else:
            self.loaded_camera2_frames = data

        print(f"Loaded Camera {camera_id} from {path}")
        if hasattr(self, 'timestamps'):
            self.select_frame_scrollbar['to'] = len(data) - 1

        self.update_camera_views(self.frame_nr_var.get())

     except Exception as e:
        print(f"[Load Camera {camera_id}] Error: {e}")



    def load_recordings(self):
        try:
            folder_path = filedialog.askdirectory(title="Select Recordings Folder")
            if not folder_path:
                print("No folder selected.")
                return

            self.selected_data_dir = folder_path

        # Find all CSV files (full filenames)
            csv_files = [f for f in os.listdir(folder_path) if f.endswith('.csv')]
            if not csv_files:
                print("No CSV files found in the selected folder.")
                self.available_recording_combobox['values'] = []
                self.available_recording_combobox['state'] = 'disabled'
                return

            self.available_recording_combobox['values'] = csv_files
            self.available_recording_combobox['state'] = 'readonly'
            self.available_recording_combobox.set(csv_files[0])  # set first

        except Exception as e:
            print(f"[Load Recordings] Error: {e}")

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
        self.setup_visualization_callbacks()

        print(f"Loaded {num_frames} frames, {num_imus} IMUs, for visualization.")

    
    def on_select_recording(self, event):
     try:
        selected_file = self.available_recording_combobox.get()
        if not selected_file:
            print("No file selected in combobox.")
            return

        folder_path = self.selected_data_dir
        csv_path = os.path.join(folder_path, selected_file)
        base_path = os.path.splitext(csv_path)[0]
        cam1_path = base_path + "_camera1.npy"
        cam2_path = base_path + "_camera2.npy"

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
                                self.imu_pose_selection.set("Laying")
                            else:
                                self.imu_pose_selection.set("Sitting")
            except Exception as e:
                print(f"[POSE] Could not read pose info from file: {e}")

            self.timestamps = self.loaded_imu_data["timestamp"].to_numpy()
    
            self.loaded_acc_data = []
            self.loaded_ang_data = []
            self.loaded_mag_data = []

            for imu_index in range(6):
                acc_cols = [f"IMU_{imu_index}_acc_x", f"IMU_{imu_index}_acc_y", f"IMU_{imu_index}_acc_z"]
                ang_cols = [f"IMU_{imu_index}_ang_x", f"IMU_{imu_index}_ang_y", f"IMU_{imu_index}_ang_z"]
                mag_cols = [f"IMU_{imu_index}_mag_x", f"IMU_{imu_index}_mag_y", f"IMU_{imu_index}_mag_z"]

                acc_values = self.loaded_imu_data[acc_cols].to_numpy()
                ang_values = self.loaded_imu_data[ang_cols].to_numpy()
                mag_values = self.loaded_imu_data[mag_cols].to_numpy()

                self.loaded_acc_data.append(acc_values)
                self.loaded_ang_data.append(ang_values)
                self.loaded_mag_data.append(mag_values)
     
            print(f"Loaded IMU data from {csv_path}")
        else:
            print(f"CSV file not found: {csv_path}")
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
            acc_cols = [f"IMU_{imu_index}_acc_x", f"IMU_{imu_index}_acc_y", f"IMU_{imu_index}_acc_z"]

            if all(col in self.loaded_imu_data.columns for col in acc_cols):
                acc_data = self.loaded_imu_data[acc_cols].to_numpy()
                if not np.isnan(acc_data).all():
                    imu_tickbox_map[imu_index].set(True)   # ενεργοποίηση tickbox
                else:
                    imu_tickbox_map[imu_index].set(False)  # απενεργοποίηση
            else:
                imu_tickbox_map[imu_index].set(False)


        # Load camera frames
        if os.path.exists(cam1_path):
            self.loaded_camera1_frames = np.load(cam1_path)
            print(f"Loaded camera1 frames from {cam1_path}")
        else:
            self.loaded_camera1_frames = None
            print(f"Camera1 file not found: {cam1_path}")

        if os.path.exists(cam2_path):
            self.loaded_camera2_frames = np.load(cam2_path)
            print(f"Loaded camera2 frames from {cam2_path}")
        else:
            self.loaded_camera2_frames = None
            print(f"Camera2 file not found: {cam2_path}")

        # Update slider for IMU frames
        if len(self.loaded_imu_data) > 0:
            num_frames = len(self.loaded_imu_data)
            self.select_frame_scrollbar.config(to=num_frames - 1)
            print(f"Updated slider to {num_frames} IMU frames.")

        self.update_visualization_plots()

        print("Finished loading and visualizing data.")

     except Exception as e:
        print(f"[On Select Recording] Error: {e}")

     self.setup_visualization_callbacks()
     self.update_visualization_plots()

    def setup_visualization_callbacks(self):
        self.frame_nr_var.trace_add('write', lambda *args: self.update_visualization_plots())

    def update_visualization_plots(self, frame_idx=None):
        try:
            num_frames = len(self.loaded_acc_data[0]) if self.loaded_acc_data else 0
            if num_frames == 0:
                print("[Visualization Update] No data loaded.")
                return

            idx = self.frame_nr_var.get() if frame_idx is None else frame_idx
            if idx < 0 or idx >= num_frames:
                print("[Visualization Update] Index out of bounds.")
                return

            time_axis = self.timestamps
            self.ax_imu.clear()
            active = False

            imu_tickbox_map = {
                0: self.left_thigh_checkbox_var,
                1: self.left_calf_checkbox_var,
                2: self.left_foot_checkbox_var,
                3: self.right_thigh_checkbox_var,
                4: self.right_calf_checkbox_var,
                5: self.right_foot_checkbox_var
            }

            imu_labels = ['L Thigh', 'L Calf', 'L Foot','R Thigh', 'R Calf', 'R Foot']

            for i in range(len(self.loaded_acc_data)):
                if i not in imu_tickbox_map or not imu_tickbox_map[i].get():
                    continue

                imu_label = imu_labels[i]

                if self.acc_checkbox_var.get() and not np.isnan(self.loaded_acc_data[i]).all():
                    acc_x = [v[0] for v in self.loaded_acc_data[i]]
                    acc_y = [v[1] for v in self.loaded_acc_data[i]]
                    acc_z = [v[2] for v in self.loaded_acc_data[i]]
                    self.ax_imu.plot(time_axis, acc_x, label=f'{imu_label} Acc X')
                    self.ax_imu.plot(time_axis, acc_y, label=f'{imu_label} Acc Y')
                    self.ax_imu.plot(time_axis, acc_z, label=f'{imu_label} Acc Z')
                    self.ax_imu.scatter(self.timestamps[idx], acc_x[idx], color='blue', s=40, zorder=5)
                    self.ax_imu.scatter(self.timestamps[idx], acc_y[idx], color='cyan', s=40, zorder=5)
                    self.ax_imu.scatter(self.timestamps[idx], acc_z[idx], color='navy', s=40, zorder=5)
                    #self.ax_imu.set_ylabel('m/s²')
                    #self.ax_imu.set_title('Acceleration')
                    active = True

                if self.ang_checkbox_var.get() and not np.isnan(self.loaded_ang_data[i]).all():
                    ang_x = [v[0] for v in self.loaded_ang_data[i]]
                    ang_y = [v[1] for v in self.loaded_ang_data[i]]
                    ang_z = [v[2] for v in self.loaded_ang_data[i]]
                    self.ax_imu.plot(time_axis, ang_x, label=f'{imu_label} Ang X')
                    self.ax_imu.plot(time_axis, ang_y, label=f'{imu_label} Ang Y')
                    self.ax_imu.plot(time_axis, ang_z, label=f'{imu_label} Ang Z')
                    self.ax_imu.scatter(self.timestamps[idx], ang_x[idx], color='orange', s=40, zorder=5)
                    self.ax_imu.scatter(self.timestamps[idx], ang_y[idx], color='gold', s=40, zorder=5)
                    self.ax_imu.scatter(self.timestamps[idx], ang_z[idx], color='red', s=40, zorder=5)
                    #self.ax_imu.set_ylabel('rad/s')
                    #self.ax_imu.set_title('Angular Velocity')
                    active = True

                if self.mag_checkbox_var.get() and not np.isnan(self.loaded_mag_data[i]).all():
                    mag_x = [v[0] for v in self.loaded_mag_data[i]]
                    mag_y = [v[1] for v in self.loaded_mag_data[i]]
                    mag_z = [v[2] for v in self.loaded_mag_data[i]]
                    self.ax_imu.plot(time_axis, mag_x, label=f'{imu_label} Mag X')
                    self.ax_imu.plot(time_axis, mag_y, label=f'{imu_label} Mag Y')
                    self.ax_imu.plot(time_axis, mag_z, label=f'{imu_label} Mag Z')
                    self.ax_imu.scatter(self.timestamps[idx], mag_x[idx], color='green', s=40, zorder=5)
                    self.ax_imu.scatter(self.timestamps[idx], mag_y[idx], color='lime', s=40, zorder=5)
                    self.ax_imu.scatter(self.timestamps[idx], mag_z[idx], color='darkgreen', s=40, zorder=5)
                    #self.ax_imu.set_ylabel('μT')
                    #self.ax_imu.set_title('Magnetic Field')
                    active = True

            if not active:
                self.ax_imu.text(0.5, 0.5, "No data selected", ha='center', va='center')

            self.ax_imu.set_xlabel('Timestamp')
            self.ax_imu.grid(True)
            self.ax_imu.legend()
            self.fig_imu.tight_layout()
            self.ax_imu.set_autoscale_on(True)
            self.ax_imu.set_navigate(True)
            self.ax_imu.xaxis.set_major_locator(MaxNLocator(7))

            for label in self.ax_imu.get_xticklabels():
                label.set_rotation(0)
                label.set_horizontalalignment('center')

            self.canvas_imu.draw()

            # Update the Pose Viewer
            self.update_pose_viewer(idx)

            # Update camera view
            self.update_camera_views(idx)

        except Exception as e:
            print(f"[Visualization Update] Error: {e}")

    def update_camera_views(self, frame_idx):
     try:
        # Camera 1 (Front View)
        if self.loaded_camera1_frames is not None and frame_idx < len(self.loaded_camera1_frames):
            self.ax3.clear()
            self.ax3.imshow(self.loaded_camera1_frames[frame_idx])
            self.ax3.axis('off')
            self.canvas3.draw()

        # Camera 2 (Side View)
        if self.loaded_camera2_frames is not None and frame_idx < len(self.loaded_camera2_frames):
            self.ax4.clear()
            self.ax4.imshow(self.loaded_camera2_frames[frame_idx])
            self.ax4.axis('off')
            self.canvas4.draw()

     except Exception as e:
        print(f"[Camera Visualization] Error at frame {frame_idx}: {e}")

    def update_pose_viewer(self, frame_idx):
        try:
            self.ax6.clear()
            self.ax6.set_xlabel("X")
            self.ax6.set_ylabel("Y")
            self.ax6.set_zlabel("Z")
            self.ax6.set_xlim3d([-1, 6])
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

            for i in range(6):
                if not checkbox_vars[i].get():
                    continue

                col_q = [f"IMU_{i}_q0", f"IMU_{i}_q1", f"IMU_{i}_q2", f"IMU_{i}_q3"]
                if all(col in self.loaded_imu_data.columns for col in col_q):
                    q = self.loaded_imu_data.loc[frame_idx, col_q].values.astype(float)
                    R = self.get_rotation_matrix_quaternions(q)
                    self.rotate_leg_segment(segment_labels[i], R)

            DEFAULT_SETTINGS.plot_body_parts(self.ax6, self.new_joints)
            self.canvas6.draw()

        except Exception as e:
            print(f"[Pose Viewer] Error updating pose: {e}")

# Run the application
if __name__ == "__main__":

    app = IMURecordingStudio()
    
    app.mainloop()