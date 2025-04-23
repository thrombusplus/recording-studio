import os
import csv
import numpy as np
import time


class FileManager:
    def __init__(self, file_path):
        self.file_path = file_path
        self.file = None

    def get_subfodlers(self):
        self.subfolder = []
        files = os.listdir(self.file_path)
        for file in files:
            if os.path.isdir(os.path.join(self.file_path, file)):
                self.subfolder.append(file)
        return self.subfolder
    
    def save_recording(self, data_queue, imu_ordered_configuration, patient_id, exercise_name):
        timestamp_str = time.strftime('%Y%m%d_%H%M%S')
        base_path = os.path.join(self.file_path, patient_id, exercise_name, f"session_{timestamp_str}")
        os.makedirs(base_path, exist_ok=True)

        imu_csv_path = os.path.join(base_path, "imu_data.csv")
        imu_txt_path = os.path.join(base_path, "imu_leg_data.txt")

        camera1_frames = []
        camera2_frames = []

        with open(imu_csv_path, mode='w', newline='') as csvfile, open(imu_txt_path, mode='w') as txtfile:
            writer = csv.writer(csvfile)
            writer.writerow(["timestamp"] + [f"IMU_{i}_q{j}" for i in range(6) for j in range(4)])
            txtfile.write("timestamp, Right Thigh, Right Calf, Right Foot, Left Thigh, Left Calf, Left Foot\n")

            while not data_queue.empty():
                try:
                    data = data_queue.get_nowait()
                    ts = data['timestamp']

                    # CSV
                    imu_row = [ts]
                    for idx in range(6):
                        if data['imu'] is not None and idx < data['imu'].shape[0]:
                            imu_row.extend(np.round(data['imu'][idx], 4))
                        else:
                            imu_row.extend([""] * 4)
                    writer.writerow(imu_row)

                    # TXT
                    if imu_ordered_configuration and isinstance(data.get('imu'), np.ndarray):
                        segments = []
                        for pos_idx in range(6):
                            imu_idx = imu_ordered_configuration[pos_idx] if pos_idx < len(imu_ordered_configuration) else -1
                            if imu_idx != -1 and imu_idx < len(data['imu']):
                                q = data['imu'][imu_idx]
                                segments.append(" ".join(f"{x:.4f}" for x in q))
                            else:
                                segments.append("")
                        line = f"{ts:.3f}, " + ", ".join(segments) + "\n"
                        txtfile.write(line)

                    # Frames
                    if data['frame_cam1'] is not None:
                        camera1_frames.append(data['frame_cam1'])
                    if data['frame_cam2'] is not None:
                        camera2_frames.append(data['frame_cam2'])

                except Exception as e:
                    print(f"[FileManager Save] Error: {e}")

        if camera1_frames:
            np.save(os.path.join(base_path, "camera1.npy"), np.array(camera1_frames))
        if camera2_frames:
            np.save(os.path.join(base_path, "camera2.npy"), np.array(camera2_frames))

        print(f"Saved to: {base_path}")