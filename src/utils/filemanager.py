import os
import csv
import numpy as np
import time
import glob


class FileManager:
    def __init__(self, file_path):
        self.file_path = file_path
        self.file = None
       
    def save_recording(self, data_queue, imu_ordered_configuration, patient_id, exercise_name):
        
        # Count existing recordings
        pattern = os.path.join(self.file_path, f"{patient_id}_{exercise_name}_*")
        existing_files = glob.glob(pattern + ".csv")  # assume at least .csv exists per recording
        repeat_number = len(existing_files) + 1

        timestamp_str = time.strftime('%Y%m%d_%H%M%S')
        filename_base = f"{patient_id}_{exercise_name}_{repeat_number}_{timestamp_str}"

        imu_csv_path = os.path.join(self.file_path, f"{filename_base}.csv")
        imu_txt_path = os.path.join(self.file_path, f"{filename_base}.txt")
        cam1_path = os.path.join(self.file_path, f"{filename_base}_camera1.npy")
        cam2_path = os.path.join(self.file_path, f"{filename_base}_camera2.npy")

        camera1_frames = []
        camera2_frames = []

        with open(imu_csv_path, mode='w', newline='') as csvfile, open(imu_txt_path, mode='w') as txtfile:
            writer = csv.writer(csvfile)
            writer.writerow(["timestamp"] + [f"IMU_{i}_q{j}" for i in range(6) for j in range(4)])

            while not data_queue.empty():
               try:
                  data = data_queue.get_nowait()
                  imu_ts = data.get('imu_ts')
                  unix_ts = int(data['timestamp'] * 1000)

                  imu_quat = data.get('imu')
                  imu_acc = data.get('imu_acc')

                  # CSV
                  imu_row = [data['timestamp']]
                  for idx in range(6):
                      if imu_quat is not None and idx < imu_quat.shape[0]:
                          imu_row.extend(np.round(imu_quat[idx], 4))
                      else:
                          imu_row.extend([""] * 4)
                  writer.writerow(imu_row)

                  # TXT (new format)
                  if imu_ordered_configuration and isinstance(imu_quat, np.ndarray) and isinstance(imu_acc, np.ndarray):
                      for pos_idx in range(6):  
                          imu_idx = imu_ordered_configuration[pos_idx] if pos_idx < len(imu_ordered_configuration) else -1
                          if imu_idx != -1 and imu_idx < len(imu_quat) and imu_idx < len(imu_acc):
                              # identify leg: left (0) or right (1)
                              leg = 1 if pos_idx in [0, 1, 2] else 0  # 0-2 right, 3-5 left

                              quat = [f"{x:.4f}" for x in imu_quat[imu_idx]]
                              accel = [f"{x:.4f}" for x in imu_acc[imu_idx]]

                              sensor_timestamp = imu_ts[imu_idx]

                              line = f"{unix_ts}|{leg}|\n{sensor_timestamp},{','.join(quat)},{','.join(accel)}\n"
                              txtfile.write(line)

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

        print(f"Saved files: {imu_csv_path}, {imu_txt_path}, {cam1_path}, {cam2_path}")

