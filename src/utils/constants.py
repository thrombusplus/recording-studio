import numpy as np
import matplotlib.pyplot as plt

class DEFAULT_SETTINGS: 
    def exercises_list(self):
            list = tuple()
            list.append("Leg raises")
            list.append("Push-ups")

    def max_cameras(): # the maximum number of cameras that can be connected
        return int(5)
        
    def skeleton_pose_laying_joints():
        joints = {
            'head': np.array([0, 6, 0]),
            'neck': np.array([0, 5, 0]),
            'left_shoulder': np.array([-0.5, 5, 0]),
            'right_shoulder': np.array([0.5, 5, 0]),
            'left_elbow' : np.array([-0.8, 4, 0]),
            'right_elbow' : np.array([0.8, 4, 0]),
            'left_wrist' : np.array([-1, 3, 0]),
            'right_wrist' : np.array([1, 3, 0]),
            'hip': np.array([0, 3, 0]),
            'left_hip': np.array([-0.5, 3, 0]),
            'right_hip': np.array([0.5, 3, 0]),
            'left_knee': np.array([-0.5, 1.5, 0]),
            'right_knee': np.array([0.5, 1.5, 0]),
            'left_ankle': np.array([-0.5, 0, 0]),
            'right_ankle': np.array([0.5, 0, 0]),
            'left_toes': np.array([-0.5, 0, 0.3]),
            'right_toes': np.array([0.5, 0, 0.3])
            } 
        return joints
    
    def skeleton_pose_sitting_joints(): #TODO: change the coordinates
        joints = {
            'head': np.array([0, 6, 0]),
            'neck': np.array([0, 5, 0]),
            'left_shoulder': np.array([-0.5, 5, 0]),
            'right_shoulder': np.array([0.5, 5, 0]),
            'left_elbow' : np.array([-0.8, 4, 0]),
            'right_elbow' : np.array([0.8, 4, 0]),
            'left_wrist' : np.array([-1, 3, 0]),
            'right_wrist' : np.array([1, 3, 0]),
            'hip': np.array([0, 3, 0]),
            'left_hip': np.array([-0.5, 3, 0]),
            'right_hip': np.array([0.5, 3, 0]),
            'left_knee': np.array([-0.5, 1.5, 0]),
            'right_knee': np.array([0.5, 1.5, 0]),
            'left_ankle': np.array([-0.5, 0, 0]),
            'right_ankle': np.array([0.5, 0, 0]),
            'left_toes': np.array([-0.5, 0, 0.3]),
            'right_toes': np.array([0.5, 0, 0.3])
            } 
        return joints

    def plot_body_parts(axes, joints):
        
        body_parts = [
            ('head', 'neck'),
            ('neck', 'left_shoulder'), ('neck', 'right_shoulder'),
            ('left_shoulder', 'left_elbow'), ('right_shoulder', 'right_elbow'),
            ('left_elbow', 'left_wrist'), ('right_elbow', 'right_wrist'),
            ('neck', 'hip'),
            ('hip', 'left_hip'), ('hip', 'right_hip'),
            ('left_hip', 'left_knee'), ('right_hip', 'right_knee'),
            ('left_knee', 'left_ankle'), ('right_knee', 'right_ankle'),
            ('left_ankle', 'left_toes'), ('right_ankle', 'right_toes')
            ]
        
    # Plot body parts
        for part in body_parts:
            joint1, joint2 = part
            axes.plot(
                [joints[joint1][0], joints[joint2][0]],
                [joints[joint1][1], joints[joint2][1]],
                [joints[joint1][2], joints[joint2][2]],
                '.-', markersize=2, color='blue'
            )
