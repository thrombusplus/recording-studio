import os
import csv
import numpy as np
import time
import glob


class FileManager:
    def __init__(self, file_path):
        self.file_path = file_path
        self.file = None
       
    def save_recording(self, data_queue, imu_ordered_configuration, patient_id, exercise_name, pose_setting):
        
        # Count existing recordings
        pattern = os.path.join(self.file_path, f"{patient_id}_{exercise_name}_*")
        existing_files = glob.glob(pattern + ".csv")  # assume at least .csv exists per recording
        repeat_number = len(existing_files) + 1
        repeat_str = str(repeat_number).zfill(2)

        timestamp_str = time.strftime('%Y%m%d_%H%M%S')
        filename_base = f"{patient_id}_{exercise_name}_{repeat_str}_{timestamp_str}"

        imu_csv_path = os.path.join(self.file_path, f"{filename_base}.csv")
        imu_txt_path = os.path.join(self.file_path, f"{filename_base}.txt")
        cam1_path = os.path.join(self.file_path, f"{filename_base}_camera1.npy")
        cam2_path = os.path.join(self.file_path, f"{filename_base}_camera2.npy")

        camera1_frames = []
        camera2_frames = []

        with open(imu_csv_path, mode='w', newline='') as csvfile, open(imu_txt_path, mode='w') as txtfile:
            writer = csv.writer(csvfile)

            # Create CSV header
            header = ["timestamp"]
            for i in range(6):  
                header += [f"IMU_{i}_q0", f"IMU_{i}_q1", f"IMU_{i}_q2", f"IMU_{i}_q3"]
                header += [f"IMU_{i}_acc_x", f"IMU_{i}_acc_y", f"IMU_{i}_acc_z"]
                header += [f"IMU_{i}_ang_x", f"IMU_{i}_ang_y", f"IMU_{i}_ang_z"]
                header += [f"IMU_{i}_mag_x", f"IMU_{i}_mag_y", f"IMU_{i}_mag_z"]
            writer.writerow(header)

            # Write txt header (same as CSV but without mag fields)
            txt_header = ["timestamp"]
            for i in range(6):
                txt_header += [f"quat{i}{j}" for j in range(4)]
                txt_header += [f"acc{i}{j}" for j in range(3)]
                txt_header += [f"gyr{i}{j}" for j in range(3)]

            while not data_queue.empty():
                try:
                    data = data_queue.get_nowait()
                    imu_quat = data.get('imu')
                    imu_acc = data.get('imu_acc')
                    imu_ang = data.get('imu_ang')
                    imu_mag = data.get('imu_mag')
                    row = [data['timestamp']]
                    txt_row = [data['timestamp']]

                    for pos_idx in range(6):  # 0: L Thigh, ..., 5: R Foot
                        imu_idx = imu_ordered_configuration[pos_idx] if pos_idx < len(imu_ordered_configuration) else -1

                        # Quaternion
                        if imu_idx != -1 and imu_quat is not None and imu_idx < len(imu_quat):
                            quat_vals = np.round(imu_quat[imu_idx], 4).tolist()
                            row.extend(quat_vals)
                            txt_row.extend(quat_vals)
                        else:
                            row.extend([""] * 4)
                            txt_row.extend([""] * 4)

                        # Accelerometer
                        if imu_idx != -1 and imu_acc is not None and imu_idx < len(imu_acc):
                            acc_vals = np.round(imu_acc[imu_idx], 4).tolist()
                            row.extend(acc_vals)
                            txt_row.extend(acc_vals)
                        else:
                            row.extend([""] * 3)
                            txt_row.extend([""] * 3)

                        # Gyroscope
                        if imu_idx != -1 and imu_ang is not None and imu_idx < len(imu_ang):
                            gyr_vals = np.round(imu_ang[imu_idx], 4).tolist()
                            row.extend(gyr_vals)
                            txt_row.extend(gyr_vals)
                        else:
                            row.extend([""] * 3)
                            txt_row.extend([""] * 3)

                        # Magnetic field (ONLY for CSV)
                        if imu_idx != -1 and imu_mag is not None and imu_idx < len(imu_mag):
                            row.extend(np.round(imu_mag[imu_idx], 4))
                        else:
                            row.extend([""] * 3)

                    writer.writerow(row)
                    txtfile.write(",".join(map(str, txt_row)) + "\n")

                    # Frames
                    if data['frame_cam1'] is not None:
                        camera1_frames.append(data['frame_cam1'])
                    if data['frame_cam2'] is not None:
                        camera2_frames.append(data['frame_cam2'])

                except Exception as e:
                    print(f"[FileManager Save] Error: {e}")

        if camera1_frames:
            np.save(cam1_path, np.array(camera1_frames))
        if camera2_frames:
            np.save(cam2_path, np.array(camera2_frames))
        
        #Pose information
        pose_code = "L" if pose_setting == "Laying" else "S"
        with open(imu_csv_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([f"#POSE={pose_code}"])

        print(f"Saved files: {imu_csv_path}, {imu_txt_path}, {cam1_path}, {cam2_path}")