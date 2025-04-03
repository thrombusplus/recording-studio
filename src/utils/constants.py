import numpy as np
import matplotlib.pyplot as plt

class DEFAULT_SETTINGS: 
    def exercises_list():
        list = []
        list.append('Leg raises')
        list.append('Push-ups')
        return list

    def max_cameras(): # the maximum number of cameras that can be connected
        return int(5)
        
    def skeleton_pose_laying_joints():
        joints = {
            'Head': np.array([0, 0, 0]),
            'Neck': np.array([1, 0, 0]),
            'Left Shoulder': np.array([1, -0.5, 0]),
            'Right Shoulder': np.array([1, 0.5, 0]),
            'Left Elbow' : np.array([2, -0.8, 0]),
            'Right Elbow' : np.array([2, 0.8, 0]),
            'Left Wrist' : np.array([3, -1, 0]),
            'Right Wrist' : np.array([3, 1, 0]),
            'Hip': np.array([3, 0, 0]),
            'Left Hip': np.array([3, -0.5, 0]),
            'Right Hip': np.array([3, 0.5, 0]),
            'Left Knee': np.array([4.5, -0.5, 0]),
            'Right Knee': np.array([4.5, 0.5, 0]),
            'Left Ankle': np.array([6, -0.5, 0]),
            'Right Ankle': np.array([6, 0.5, 0]),
            'Left Toes': np.array([6, -0.5, 0.2]),
            'Right Toes': np.array([6, 0.5, 0.2])
            } 
        return joints
    
    # def skeleton_pose_sitting_joints(): #TODO: change the coordinates
    #     joints = {
    #         'Head': np.array([0, 6, 0]),
    #         'Neck': np.array([0, 5, 0]),
    #         'Left Shoulder': np.array([-0.5, 5, 0]),
    #         'RightShoulder': np.array([0.5, 5, 0]),
    #         'Left Elbow' : np.array([-0.8, 4, 0]),
    #         'Right Elbow' : np.array([0.8, 4, 0]),
    #         'Left Wrist' : np.array([-1, 3, 0]),
    #         'Right Wrist' : np.array([1, 3, 0]),
    #         'Hip': np.array([0, 3, 0]),
    #         'Left Hip': np.array([-0.5, 3, 0]),
    #         'Right Hip': np.array([0.5, 3, 0]),
    #         'Left Knee': np.array([-0.5, 1.5, 0]),
    #         'Right Knee': np.array([0.5, 1.5, 0]),
    #         'Left Ankle': np.array([-0.5, 0, 0]),
    #         'Right Ankle': np.array([0.5, 0, 0]),
    #         'Left Toes': np.array([-0.5, 0, 0.2]),
    #         'Right Toes': np.array([0.5, 0, 0.2])
    #         } 
    #     return joints

    def plot_body_parts(axes, joints):
        
        body_parts = [
            ('Head', 'Neck'),
            ('Neck', 'Left Shoulder'), ('Neck', 'Right Shoulder'),
            ('Left Shoulder', 'Left Elbow'), ('Right Shoulder', 'Right Elbow'),
            ('Left Elbow', 'Left Wrist'), ('Right Elbow', 'Right Wrist'),
            ('Neck', 'Hip'),
            ('Hip', 'Left Hip'), ('Hip', 'Right Hip'),
            ('Left Hip', 'Left Knee'), ('Right Hip', 'Right Knee'),
            ('Left Knee', 'Left Ankle'), ('Right Knee', 'Right Ankle'),
            ('Left Ankle', 'Left Toes'), ('Right Ankle', 'Right Toes')
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
