import numpy as np
import matplotlib.pyplot as plt

class DEFAULT_SETTINGS: 
    def exercises_list():
        return {
            "Sitting": {
                "Ankle-Pumps":["CL","CR","CB","CA"],
                "Heel-Raises":["CL","CR","CB","CA","IL","IR","IB","IA"],
                "Toe-Taps":["CL","CR","CB","CA","IL","IR","IB","IA"],
                "Ankle-Circles-In":["CL","CR"],
                "Ankle-Circles-Out":["CL","CR"],
                "Knee-Extensions":["CL","CR","CB","CA","IL","IR","IB","IA"],
                "Knee-Marches":["CL","CR","CB","CA","IL","IR","IB","IA"]
            },
            "Standing": {
                "Toe-Raises":["CL","CR","CB","CA","IL","IR","IB","IA"],
                "Calf-Raises":["CL","CR","CB","CA","IL","IR","IB","IA"],
                "Ankle-Pumps":["CL","CR","CB","CA"],
                "Knee-Marches":["CL","CR","CB","CA","IL","IR","IB","IA"],
                "Weight-Shifts":["CA"],
                "Hip-Abductions":["CL","CR","CB","CA","IL","IR","IB","IA"],
                "Mini-Squats" :["CB","IB"]
            },
            "Lying": {
                "Ankle-Pumps": ["CL","CR","CB","CA"],
                "Ankle-Circles-In":["CL", "CR"],
                "Ankle-Circles-Out":["CL","CR"],
                "Heel-Slides":["CL","CR","CB","CA"],
                "Bridging":["CB","IB"],
                "Straight-Leg-Raises":["CL","CR","CA","IL","IR","IA"],
                "Heel-Bridging":["CB","IB"],
                "Supine-Hip-Abduction":["CL","CR","CB","CA","IL","IR","IB","IA"],
                "Supine-Heel-Taps":["CL","CR","CB","CA","IL","IR","IB","IA"],
                "Right-Sided-Hip-Abduction":["CL","IL"],
                "Left-Sided-Hip-Abduction":["CR","IR"]
            }
        }
      
        

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
    
    def skeleton_pose_lying_joints(): #TODO: change the coordinates
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
