import numpy as np
import matplotlib.pyplot as plt

class DEFAULT_SETTINGS: 
    def exercises_list():
        list = []
        
        #Laying
        list.append('Ankle_Pumps')
        list.append('Ankle Rotations')
        list.append('Bed_Supported_Knee_Bends')
        list.append('Abduction_Excercise')
        list.append('Straight_Leg_Raises') # Sitting and Laying option

        #Standing
        list.append('Standing_Knee_raises')
        list.append('Standing_Hip_Abduction')
        list.append('Standing_Hip_Extensions')

        #Sitting
        list.append('Sitting_Supported_Knee_Bends')
        list.append('Sitting_Unsupported_Knee_Bends')

        """list.append('Calf_Pumps')
        list.append('Knee_Extensions')
        list.append('Seated_Marching')
        list.append('Ankle_circles')
        list.append('Heel_Slides')
        list.append('Supine_Leg_Lifts')
        list.append('Side_Leg_Lifts')
        list.append('Clamshells')
        list.append('Glute_Bridges')
        list.append('Leg_Abductions')
        list.append('F_Bicycle_Both_Legs')
        list.append('F_Supine_Bicycle_Both_Legs')
        list.append('F_Bicycle_Reverse_Leg_Left')
        list.append('F_Bicycle_Reverse_Leg_Right')
        list.append('F_Prone_Alternate')
        list.append('F_Prone_Alternate_Knee_Bending')
        list.append('F_Prone_Both_Legs')
        list.append('F_Single_Leg_Jumps_Left')
        list.append('F_Single_Leg_Jumps_Right')
        list.append('F_Supine_Bend_Knee_Ankle_Rotation')
        list.append('F_Supine_Bend_Knee_Ankle_Dorsi_Plantar_Flexion')
        list.append('F_Supine_Bicycle_Inner_Leg')
        list.append('F_Supine_Reverse_Straight_Leg_Left')
        list.append('F_Supine_Reverse_Straight_Leg_Right')
        list.append('F_Supine_Straight_Legs_Ankle_Rotation')
        list.append('F_Supine_Straight_Legs_Ankle_Dorsi_Plantar_Flexion')"""
        
        return list

    def max_cameras(): # the maximum number of cameras that can be connected
        return int(5)
        
    def skeleton_pose_sitting_joints():
        joints = {
            'Head': np.array([0, 0, 3]),
            'Neck': np.array([0, 0, 2]),
            'Right Shoulder': np.array([0, -0.5, 2]),
            'Left Shoulder': np.array([0, 0.5, 2]),
            'Right Elbow' : np.array([0, -0.8, 1]),
            'Left Elbow' : np.array([0, 0.8, 1]),
            'Right Wrist' : np.array([0, -1, 0]),
            'Left Wrist' : np.array([0, 1, 0]),
            'Hip': np.array([0, 0, 0]),
            'Right Hip': np.array([0, -0.5, 0]),
            'Left Hip': np.array([0, 0.5, 0]),
            'Right Knee': np.array([1.5, -0.5, 0]),
            'Left Knee': np.array([1.5, 0.5, 0]),
            'Right Ankle': np.array([1.5, -0.5, -1.5]),
            'Left Ankle': np.array([1.5, 0.5, -1.5]),
            'Right Toes': np.array([1.85, -0.5, -1.5]), #length of foot 0.35
            'Left Toes': np.array([1.85, 0.5, -1.5])
            } 
        return joints
    
    def skeleton_pose_laying_joints(): #TODO: change the coordinates
        joints = {
            'Head': np.array([0, 0, 0]),
            'Neck': np.array([1, 0, 0]),
            'Right Shoulder': np.array([1, -0.5, 0]),
            'Left Shoulder': np.array([1, 0.5, 0]),
            'Right Elbow' : np.array([2, -0.8, 0]),
            'Left Elbow' : np.array([2, 0.8, 0]),
            'Right Wrist' : np.array([3, -1, 0]),
            'Left Wrist' : np.array([3, 1, 0]),
            'Hip': np.array([3, 0, 0]),
            'Right Hip': np.array([3, -0.5, 0]),
            'Left Hip': np.array([3, 0.5, 0]),
            'Right Knee': np.array([4.5, -0.5, 0]),
            'Left Knee': np.array([4.5, 0.5, 0]),
            'Right Ankle': np.array([6, -0.5, 0]),
            'Left Ankle': np.array([6, 0.5, 0]),
            'Right Toes': np.array([6.247, -0.5, 0.247]),
            'Left Toes': np.array([6.247, 0.5, 0.247])
            } 
        return joints

    def skeleton_pose_standing_joints():
        joints = {
            'Head': np.array([0, 0, 3]),
            'Neck': np.array([0, 0, 2]),
            'Right Shoulder': np.array([0, -0.5, 2]),
            'Left Shoulder': np.array([0, 0.5, 2]),
            'Right Elbow': np.array([0, -0.8, 1]),
            'Left Elbow': np.array([0, 0.8, 1]),
            'Right Wrist': np.array([0, -1.0, 0]),
            'Left Wrist': np.array([0, 1.0, 0]),
            'Hip': np.array([0, 0, 0]),
            'Right Hip': np.array([0, -0.5, 0]),
            'Left Hip': np.array([0, 0.5, 0]),
            'Right Knee': np.array([0, -0.5, -1.5]),
            'Left Knee': np.array([0, 0.5, -1.5]),
            'Right Ankle': np.array([0, -0.5, -3.0]),
            'Left Ankle': np.array([0, 0.5, -3.0]),
            'Right Toes': np.array([0.35, -0.5, -3.0]),
            'Left Toes': np.array([0.35, 0.5, -3.0])
        }
        return joints

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
        axes.plot(joints['Head'][0], joints['Head'][1], joints['Head'][2], 'o', markersize=12, color='orange')
        # axes.plot(-0.2, -0.2, 0, 'o', markersize=2, color='red') # Draw the eyes
        # axes.plot(-0.2, 0.2, 0, 'o', markersize=2, color='red')
