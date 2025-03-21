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
        for device in xdpcHandler.connectedDots():
            print(f"\nResetting heading to default for device {device.portInfo().bluetoothAddress()}: ", end="", flush=True)
            if device.resetOrientation(movelladot_pc_sdk.XRM_DefaultAlignment):
                 print("OK", end="", flush=True)
            else:
                print(f"NOK: {device.lastResultText()}", end="", flush=True)
        print("\n", end="", flush=True)

        print("\nStopping measurement...")
        for device in xdpcHandler.connectedDots():
            if not device.stopMeasurement():
             print("Failed to stop measurement.")
            if not device.disableLogging():
                print("Failed to disable logging.")

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
                exit(-1)

            # If (some) devices are already in sync mode.Disable sync on all devices first.
            manager.stopSync()
            print(f"Retrying start sync after stopping sync")
            if not manager.startSync(deviceList[-1].bluetoothAddress()):
                print(f"Could not start sync. Reason: {manager.lastResultText()}. Aborting.")
                xdpcHandler.cleanup()
                exit(-1)  