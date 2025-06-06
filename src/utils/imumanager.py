from .xdpchandler import *
import matplotlib.pyplot as plt
import numpy as np 
from IPython import get_ipython
from pynput import keyboard
import csv
import time

class IMUManager:
    xdpcHandler = XdpcHandler()

    def __init__(self, parms): # connect sensors
        self.frame_rate = parms    
        self.devices = self.conenct_IMU_sensors()
        self.reset_heading_flag = False # Flag to perfrom heading reset, returns to false afterwards
        self.reset_heading_status = False # If is False then heading is set to Default
        
        self.numOfDevices = len(self.devices.connectedDots())
        self.acc_data = np.empty([self.numOfDevices, 3]) #No sure if should be filled with Nan instead
        self.gyr_data = np.empty([self.numOfDevices, 3])
        self.mag_data = np.empty([self.numOfDevices, 3])
        self.quat_data = np.empty([self.numOfDevices, 4])
        self.calibration_status = np.full([self.numOfDevices,1], False)
        self.calibration_inverse = np.empty([self.numOfDevices, 4])

        self.sensor_timestamp = np.zeros((6))  #for save sensor's timestamps
        
    def conenct_IMU_sensors(self):
        xdpcHandler = XdpcHandler()

        if not xdpcHandler.initialize():
            xdpcHandler.cleanup()
            # exit(-1)

        xdpcHandler.scanForDots()
        if len(xdpcHandler.detectedDots()) == 0:
            print("No Movella DOT device(s) found. Aborting.")
            xdpcHandler.cleanup()
            # exit(-1)

        xdpcHandler.connectDots()

        if len(xdpcHandler.connectedDots()) == 0:
            print("Could not connect to any Movella DOT device(s). Aborting.")
            xdpcHandler.cleanup()
            # exit(-1)

        for device in xdpcHandler.connectedDots():
            filterProfiles = device.getAvailableFilterProfiles()
            print("Available filter profiles:")
            for f in filterProfiles:
                print(f.label())

            print(f"Current profile: {device.onboardFilterProfile().label()}")
            if device.setOnboardFilterProfile("General"):
                print("Successfully set profile to General")
            else:
                print("Setting filter profile failed!")
                
            if device.setOutputRate(self.frame_rate):
                print(f"Successfully set output rate to {self.frame_rate} Hz")
            else:
                print("Setting output rate failed!")
                
        return xdpcHandler
    
    def disconnect_IMU_sensors(self):
        xdpcHandler = self.devices
        # self.start_measuring_mode()
        self.reset_heading_default()
        # self.stop_measuring_mode()
        xdpcHandler.cleanup()
        del xdpcHandler

    def sync_IMU_sensors(self):
        xdpcHandler = self.devices
        manager = xdpcHandler.manager()
        deviceList = xdpcHandler.connectedDots()

        print(f"[SYNC] Preparing to sync {len(deviceList)} devices.")
    
        # Exit from measurement mode
        print("[SYNC] Stopping measurement on all devices...")
        for device in deviceList:
            try:
                device.stopMeasurement()
                print(f"[SYNC] Stopped measurement on {device.bluetoothAddress()}")
            except Exception as e:
                print(f"[SYNC] Could not stop measurement on {device.bluetoothAddress()}: {e}")

        # Stop syncing
        print("[SYNC] Forcing stopSync...")
        try:
            manager.stopSync()
        except Exception as e:
            print(f"[SYNC] stopSync error (ignored): {e}")

        time.sleep(1)  # give SDK time to reset states

        # Statr syncing
        print(f"[SYNC] Starting sync... Root node: {deviceList[-1].bluetoothAddress()}")
        success = manager.startSync(deviceList[-1].bluetoothAddress())

        if success:
            print("[SYNC] Devices synchronized successfully.")
        else:
            print(f"[SYNC] Could not start sync. Reason: {manager.lastResultText()}")
    
        return success

    def start_measuring_mode(self):
        xdpcHandler = self.devices
        print("\nStarting measurement...")
        for device in xdpcHandler.connectedDots():
            if not device.startMeasurement(movelladot_pc_sdk.XsPayloadMode_CustomMode4):
                print(f"Could not put device into measurement mode. Reason: {device.lastResultText()}")
                continue       

    def stop_measuring_mode(self):
        xdpcHandler = self.devices
        print("\nStopping measurement...")
        for device in xdpcHandler.connectedDots():
            if not device.stopMeasurement():
                print("Failed to stop measurement.")
            if not device.disableLogging():
                print("Failed to disable logging.")

    def reset_heading_default(self):
        xdpcHandler = self.devices
        for device in xdpcHandler.connectedDots():
            print(f"\nResetting heading to default for device {device.portInfo().bluetoothAddress()}: ", end="", flush=True)
            if device.resetOrientation(movelladot_pc_sdk.XRM_DefaultAlignment):
                print("OK", end="", flush=True)
                self.reset_heading_status = False
            else:
                print(f"NOK: {device.lastResultText()}", end="", flush=True)
        print("\n", end="", flush=True)

    def reset_heading(self):   
        xdpcHandler = self.devices
        for device in xdpcHandler.connectedDots():
            print(f"\nResetting heading for device {device.portInfo().bluetoothAddress()}: ", end="", flush=True)
            if device.resetOrientation(movelladot_pc_sdk.XRM_Heading):
                print("OK", end="", flush=True)
            else:
                print(f"NOK: {device.lastResultText()}", end="", flush=True)
        print("\n", end="", flush=True) 
        self.reset_heading_status = True
        
    def get_measurments(self):
        dev = 0 #counter for the number of device    
        if self.devices.packetsAvailable():
            for device in self.devices.connectedDots(): #depending on the speed of the for loop, maybe this can take place in parallel 
                # Retrieve a packet
                packet = self.devices.getNextPacket(device.portInfo().bluetoothAddress())
            
                if packet.containsSampleTimeFine():
                    self.sensor_timestamp[dev] = packet.sampleTimeFine()
                else:
                    self.sensor_timestamp[dev] = np.nan
      
                if packet.containsCalibratedData():
                    data = packet.calibratedData()
                    self.acc_data[dev, :] = data.m_acc
                    self.gyr_data[dev, :] = data.m_gyr
                    self.mag_data[dev, :] = data.m_mag

                else:
                    acc = np.empty(3) 
                    acc[:] = np.nan
                    self.acc_data[dev, :] = acc
                    self.gyr_data[dev, :] = acc
                    self.mag_data[dev, :] = acc
                            
                            
                if packet.containsOrientation():
                    self.quat_data[dev, :] = packet.orientationQuaternion()
                    
                else:
                    q = np.empty(4)
                    q[:]=np.nan
                    self.quat_data[dev, :] = q

                dev = dev + 1

            if self.reset_heading_flag:
                self.reset_heading()
                self.reset_heading_flag = False

            if len(self.devices.connectedDots())== 1:
                    self.reshape_measurments()


    def reshape_measurments(self):
        self.acc_data = self.acc_data.reshape(-1, 3)
        self.gyr_data = self.gyr_data.reshape(-1, 3)
        self.mag_data = self.mag_data.reshape(-1, 3)
        self.quat_data =  self.quat_data.reshape(-1, 4)      

    def calibrate(self):
        self.get_quaternions_inverse()
        return print("Inverse Quaternions Saved")


    def get_quaternions_inverse(self):
        for dev in range(len(self.devices.connectedDots())):
            if self.quat_data[dev,:].size != 0:
                squared_sum = ( 
                self.quat_data[dev,0]**2
                + self.quat_data[dev,1]**2
                + self.quat_data[dev,2]**2 
                + self.quat_data[dev,3]**2 )

                self.calibration_inverse[dev,:] = np.array([self.quat_data[dev,0], -self.quat_data[dev,1], -self.quat_data[dev,2], -self.quat_data[dev,3]])/squared_sum
                self.calibration_status[dev] = True
            else:
                print(f"Data of IMU number {dev} are empty.")
        return