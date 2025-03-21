from xdpchandler import *
import matplotlib.pyplot as plt
import numpy as np 
from IPython import get_ipython
from pynput import keyboard
import csv
import time

get_ipython().run_line_magic('matplotlib', 'qt')


#Rotation matrix from quaternions as input
def get_rotation_matrix_quaternions(qVector): # returns the rotation matrix from qunernions as input
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


#function to exit data acquisition upon key pressing
def on_press(key):
    global exit_loop
    exit_loop = True
    
exit_loop = False # initialize the variable

#Current time in milliseconds
def currentTime():
    return round(time.time()*1000)


# main function starts here


if __name__ == "__main__":
    xdpcHandler = XdpcHandler()

    if not xdpcHandler.initialize():
        xdpcHandler.cleanup()
        exit(-1)

    xdpcHandler.scanForDots()
    if len(xdpcHandler.detectedDots()) == 0:
        print("No Movella DOT device(s) found. Aborting.")
        xdpcHandler.cleanup()
        exit(-1)

    xdpcHandler.connectDots()

    if len(xdpcHandler.connectedDots()) == 0:
        print("Could not connect to any Movella DOT device(s). Aborting.")
        xdpcHandler.cleanup()
        exit(-1)

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
            
        if device.setOutputRate(60):
            print("Successfully set output rate to 60 Hz")
        else:
            print("Setting output rate failed!")
            

# Set up the logging - we do not need it in our case since we will write data to .csv
# =============================================================================
#         print("Setting quaternion CSV output")
#         device.setLogOptions(movelladot_pc_sdk.XsLogOptions_Quaternion)
# 
#         logFileName = "logfile_" + device.bluetoothAddress().replace(':', '-') + ".csv"
#         print(f"Enable logging to: {logFileName}")
#         if not device.enableLogging(logFileName):
#         print(f"Failed to enable logging. Reason: {device.lastResultText()}")
# 
# =============================================================================
        
    #Initialize the csv files
    sensor1FilePath = 'Subject_1_Exercise_3_Replicate_1_Sensor1.csv'
    sensor2FilePath = 'Subject_1_Exercise_3_Replicate_1_Sensor2.csv'
    
    csvFileSensor1 = open(sensor1FilePath, mode='w', newline='')
    csvFileSensor2 = open(sensor2FilePath, mode='w', newline='')
    
    csvWritter1 = csv.writer(csvFileSensor1)
    csvWritter1.writerow(['Timestamp', 'SensorTimeStamp', 'x_acc', 'y_acc', 'z_acc', 'x_gyro', 'y_gyro', 'z_gyro', 'q0', 'q1', 'q2', 'q3']) #write the column names
    
    
    csvWritter2 = csv.writer(csvFileSensor2)
    csvWritter2.writerow(['Timestamp', 'SensorTimeStamp', 'x_acc', 'y_acc', 'z_acc', 'x_gyro', 'y_gyro', 'z_gyro', 'q0', 'q1', 'q2', 'q3']) #write the column names
    
        
    # create the 3D plot and the axis    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim3d([-2, 2])
    ax.set_ylim3d([-2, 2])
    ax.set_zlim3d([-2, 2])
    ax.set_autoscale_on(False)
    plt.ion()
    plt.show()
    
    # plot the unit vectors - one for the upper limb and one for the lower limb
    unit_vector = np.array([1, 0, 0]) #assuming right positioning of the sensors and x-axis parallel to leg, positive towards the feet
    rotated_vector1 = unit_vector # allocation
    quiver1 = ax.quiver(0, 0, 0, unit_vector[0], unit_vector[1], unit_vector[2], color='blue', label='Vector direction')
    quiver2 = ax.quiver(unit_vector[0], unit_vector[1], unit_vector[2], unit_vector[0], unit_vector[1], unit_vector[2], color='red', label='Vector direction')
         


    #Set what measurments to stream
    print("Putting device into measurement mode.")
    for device in xdpcHandler.connectedDots():
        if not device.startMeasurement(movelladot_pc_sdk.XsPayloadMode_CustomMode4):
            print(f"Could not put device into measurement mode. Reason: {device.lastResultText()}")
            continue
        
    # First printing some headers so we see which data belongs to which device
    s = ""
    for device in xdpcHandler.connectedDots():
        s += f"{device.bluetoothAddress():42}"
    print("%s" % s, flush=True)
     
    orientationResetDone = False
    
     
    # Setup the keyboard input listener
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    print("Start streaming data...")
    print("Press any key to stop  measuring...")
    startTime = movelladot_pc_sdk.XsTimeStamp_nowMs()
    #while movelladot_pc_sdk.XsTimeStamp_nowMs() - startTime <= 20000:
    while not  exit_loop:
        if xdpcHandler.packetsAvailable():
            s = ""
            for device in xdpcHandler.connectedDots(): #depending on the speed of the for loop, maybe this can take place in parallel 
                # Retrieve a packet
                packet = xdpcHandler.getNextPacket(device.portInfo().bluetoothAddress())
                
                if packet.containsCalibratedData():
                    data = packet.calibratedData()
                    acc= data.m_acc
                    gyro = data.m_gyr
                else:
                    acc = np.empty(3) 
                    acc[:] = np.nan
                    gyro = acc
                    
                    
                if packet.containsOrientation():
                    q = packet.orientationQuaternion()

                    rotation_matrix = get_rotation_matrix_quaternions(q)
                    rotated_vector = np.dot(rotation_matrix, unit_vector)
                    
                
                    #update vectors according to current device input and plot
                    if device.portInfo().bluetoothAddress() == 'D4:22:CD:00:A8:0B':
                        rotated_vector1 = rotated_vector
                        quiver1.remove()
                        quiver1 = ax.quiver(0, 0, 0, rotated_vector1[0], rotated_vector1[1], rotated_vector1[2], color='blue', label='Vector direction')
                       
                    elif device.portInfo().bluetoothAddress() == 'D4:22:CD:00:A8:0C':
                        quiver2.remove()
                        quiver2 = ax.quiver(rotated_vector1[0], rotated_vector1[1], rotated_vector1[2], rotated_vector[0], rotated_vector[1], rotated_vector[2], color='red', label='Vector direction')
                
                    plt.pause(0.001)
                 
                else:
                    q = np.empty(3)
                    q[:] = np.nan
                    
                #write to csv
                if orientationResetDone:
                    movelaTime = movelladot_pc_sdk.XsTimeStamp_nowMs() - startTime
                    
                    if device.portInfo().bluetoothAddress() == 'D4:22:CD:00:A8:0B': # device 1
                        csvWritter1.writerow([currentTime(), packet.sampleTimeFine(), acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], q[0], q[1], q[2], q[3]])
                        
                    
                    elif device.portInfo().bluetoothAddress() == 'D4:22:CD:00:A8:0C': #device 2
                        csvWritter2.writerow([currentTime(), packet.sampleTimeFine(), acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], q[0], q[1], q[2], q[3]])
                            
                        
            print("%s\r" % s, end="", flush=True)
            
            if not orientationResetDone and movelladot_pc_sdk.XsTimeStamp_nowMs() - startTime > 5000:
                for device in xdpcHandler.connectedDots():
                    print(f"\nResetting heading for device {device.portInfo().bluetoothAddress()}: ", end="", flush=True)
                    if device.resetOrientation(movelladot_pc_sdk.XRM_Heading):
                        print("OK", end="", flush=True)
                    else:
                        print(f"NOK: {device.lastResultText()}", end="", flush=True)
                print("\n", end="", flush=True)
                orientationResetDone = True

    print("\n-----------------------------------------", end="", flush=True)


    
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
    
    
    csvFileSensor1.close()
    csvFileSensor2.close()
    
     
    