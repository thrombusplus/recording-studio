import cv2
import time
from PIL import Image, ImageTk

class CameraManager:
    def __init__(self, camera_num):
        self.camera = None
        self.open = False
        self.image = None
        self.streaming = False
        self.camNum = int(camera_num)
    
    def open_camera(self): #Method for openning the camera
        if not self.open:
            self.camera = cv2.VideoCapture(self.camNum)
            self.open = True
            self.camera.set(cv2.CAP_PROP_FPS, 30)
            # print(f"Camera {self.camNum} is open")
        else:
            print(f"Camera {self.camNum} is already opened")
        

    def get_frame(self):
        if self.streaming:
            try:
                ret=False
                start_time = time.time()
                timeout = 0.5
                while not ret:
                    ret, frame = self.camera.read() 
                    if time.time() - start_time > timeout:
                        print("timeout reached")
                        return  None            
                self.image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                return self.image
            except Exception as e:
                print(f"Failed to capture a frame from camera {self.camNum}. Error: {e}")
        else:
            print(f"Camera {self.camNum} is not streaming") 
        

    def stop_stream(self):
        if self.open:
            self.camera.release()
            self.open = False
