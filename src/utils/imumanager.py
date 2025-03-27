from .xdpchandler import *
import matplotlib.pyplot as plt
import numpy as np 
from IPython import get_ipython
from pynput import keyboard
import csv
import time

class IMUManager:
    def __init__(self, parms): # connect sensors
        self.frame_rate = parms    
        self.devices = self.conenct_IMU_sensors()
        self.reset_heading_flag = False # Flag to perfrom heading reset, returns to false afterwards
        self.reset_heading_status = False # If is False then heading is set to Default
        
        numOfDevices = len(self.devices.connectedDots())
        self.acc_data = np.empty([numOfDevices, 3]) #No sure if should be filled with Nan instead
        self.gyr_data = np.empty([numOfDevices, 3])
        self.mag_data = np.empty([numOfDevices, 3])
        self.quat_data = np.empty([numOfDevices, 4])
        
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
        print(f"\nStarting sync for connected devices... Root node: {deviceList[-1].bluetoothAddress()}")
        print("This takes at least 14 seconds")
        if not manager.startSync(deviceList[-1].bluetoothAddress()):
            print(f"Could not start sync. Reason: {manager.lastResultText()}")
            if manager.lastResult() != movelladot_pc_sdk.XRV_SYNC_COULD_NOT_START:
                print("Sync could not be started. Aborting.")
                xdpcHandler.cleanup()

            # If (some) devices are already in sync mode.Disable sync on all devices first.
            manager.stopSync()
            print(f"Retrying start sync after stopping sync")
            if not manager.startSync(deviceList[-1].bluetoothAddress()):
                print(f"Could not start sync. Reason: {manager.lastResultText()}. Aborting.")
                xdpcHandler.cleanup()

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

