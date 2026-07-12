import os
import csv
import numpy as np
import time
import glob
import cv2
from src.utils.logger import get_logger

logger = get_logger(__name__) 


# -----Synchronization timestamp export ------
# When SAVE_SYNC_TIMESTAMPS is True, save_recording() additionally writes
# <filename_base>_timestamps.csv: one row per recorded frame with the
# host monotonic timestamps (t_imu, t_cam1, t_cam2, in ns; t_imu is
# captured just when the new.packet arrives) and  about the per-device
# SampleTimeFine sensor clocks (1 MHz device clock, 1 tick = 1 µs).
# This file is only needed to (re)run the camera-IMU synchronization
# for normal data collection.
SAVE_SYNC_TIMESTAMPS = True

class FileManager:
    def __init__(self, file_path):
        self.file_path = file_path
        self.file = None
       
    
    def save_recording(self, data_queue, imu_ordered_configuration, patient_id, exercise_name, pose_setting, label_ID):
        # Count existing recordings
        pattern = os.path.join(self.file_path, f"{patient_id}_{pose_setting}_{exercise_name}_{label_ID}_*.csv")
        existing_files = [f for f in glob.glob(pattern) if not f.endswith("_timestamps.csv")]
        repeat_number = len(existing_files) + 1
        repeat_str = str(repeat_number).zfill(2)

        timestamp_str = time.strftime('%Y%m%d_%H%M%S')
        filename_base = f"{patient_id}_{pose_setting}_{exercise_name}_{label_ID}_{repeat_str}_{timestamp_str}"

        imu_csv_path = os.path.join(self.file_path, f"{filename_base}.csv")
        imu_txt_path = os.path.join(self.file_path, f"{filename_base}.txt")
        cam1_path = os.path.join(self.file_path, f"{filename_base}_camera1.mp4")
        cam2_path = os.path.join(self.file_path, f"{filename_base}_camera2.mp4")
        sync_csv_path = os.path.join(self.file_path, f"{filename_base}_timestamps.csv")

        camera1_frames = []
        camera2_frames = []

        with open(imu_csv_path, mode='w', newline='') as csvfile, \
            open(imu_txt_path, mode='w') as txtfile:

            writer = csv.writer(csvfile)
            syncfile = None
            sync_writer = None

            if SAVE_SYNC_TIMESTAMPS:
                syncfile = open(sync_csv_path, mode='w', newline='')
                sync_writer = csv.writer(syncfile)
                 

            # CSV header
            header = ["timestamp"]  
            for i in range(6):
                header.append(f"IMU_{i}_sensor_ts")
                        
            for i in range(6):  
                header += [f"quat.w({i})", f"quat.x({i})", f"quat.y({i})", f"quat.z({i})"]
                header += [f"acc.x({i})", f"acc.y({i})", f"acc.z({i})"]
                header += [f"ang.x({i})", f"ang.y({i})", f"ang.z({i})"]
                header += [f"mag.x({i})", f"mag.y({i})", f"mag.z({i})"]

            writer.writerow(header)

            # Sync/timestamps file header
            sync_header = ["frame", "t_imu", "t_cam1", "t_cam2"]
            for i in range(6):
                sync_header.append(f"IMU_{i}_sensor_ts")
            if sync_writer:
                sync_writer.writerow(sync_header)

            frame_index = 0

            while not data_queue.empty():
                try:
                    data = data_queue.get_nowait()
                    imu_quat = data.get('imu')
                    imu_acc = data.get('imu_acc')
                    imu_ang = data.get('imu_ang')
                    imu_mag = data.get('imu_mag')
                    unix_ts = int(data['timestamp'] // 1_000_000)
                    imu_ts = data.get('imu_ts') 
                    row = [unix_ts]
                    for pos_idx in range(6):
                        imu_idx = imu_ordered_configuration[pos_idx] if pos_idx < len(imu_ordered_configuration) else -1
                        if imu_idx != -1 and imu_ts is not None and imu_idx < len(imu_ts):
                            row.append(int(imu_ts[imu_idx]))  
                        else:
                            row.append("")  
                    txt_row = [unix_ts]

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

                    # Sync/timestamps row — uses the SAME imu_ordered_configuration
                    # mapping as the main CSV's IMU_i_sensor_ts columns, so
                    # IMU_i_sensor_ts here lines up 1:1 with the main CSV.
                    if sync_writer:
                        sync_row = [
                            frame_index,
                            data.get('t_imu', ""),
                            data.get('t_cam1', ""),
                            data.get('t_cam2', ""),
                        ]
                        for pos_idx in range(6):
                            imu_idx = imu_ordered_configuration[pos_idx] if pos_idx < len(imu_ordered_configuration) else -1
                            if imu_idx != -1 and imu_ts is not None and imu_idx < len(imu_ts):
                                sync_row.append(int(imu_ts[imu_idx]))
                            else:
                                sync_row.append("")
                        sync_writer.writerow(sync_row)

                    # Frames
                    if data['frame_cam1'] is not None:
                        camera1_frames.append(data['frame_cam1'])
                    if data['frame_cam2'] is not None:
                        camera2_frames.append(data['frame_cam2'])

                    frame_index += 1

                except Exception as e:
                    logger.error(f"Error: {e}")

        if camera1_frames:
            height, width, _ = camera1_frames[0].shape
            out1 = cv2.VideoWriter(cam1_path, cv2.VideoWriter_fourcc(*'mp4v'), 30, (width, height))
            for frame in camera1_frames:
                out1.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))  
            out1.release()

        if camera2_frames:
            height, width, _ = camera2_frames[0].shape
            out2 = cv2.VideoWriter(cam2_path, cv2.VideoWriter_fourcc(*'mp4v'), 30, (width, height))
            for frame in camera2_frames:
                out2.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            out2.release()
        
        #Pose information
        if pose_setting == "Lying":
            pose_code ="L"
        elif pose_setting == "Sitting":
            pose_code = "S"
        else:
            pose_code = "ST"   #Standing

        if syncfile:
            syncfile.close()

        with open(imu_csv_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([f"#POSE={pose_code}"])

        SAVED_FILES = [imu_csv_path, imu_txt_path, cam1_path, cam2_path]
        if SAVE_SYNC_TIMESTAMPS:
            SAVED_FILES.append(sync_csv_path)  
        
        logger.info(f"Saved files: {', '.join(SAVED_FILES)}")