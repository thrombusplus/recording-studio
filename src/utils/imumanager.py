from __future__ import annotations

import numpy as np

from src.utils.logger import get_logger

from .imu_backends.base import IMUDeviceInfo


logger = get_logger(__name__)


class IMUManager:
    _BACKEND_LABELS = {
        "movella": "Movella DOT",
        "websocket": "WebSocket",
    }

    def __init__(self, frame_rate, backend_name="movella", backend_config=None):
        self.frame_rate = frame_rate
        self.backend_name = backend_name
        self.backend_config = backend_config or {}
        self.backend = self._create_backend(backend_name, frame_rate, self.backend_config)
        self.reset_heading_flag = False
        self.reset_heading_status = False
        self.capabilities = self.backend.capabilities
        self.device_infos = self.backend.connect()
        self.numOfDevices = len(self.device_infos)
        self.newPackets = False
        self._reset_buffers(self.numOfDevices)

    @classmethod
    def backend_labels(cls):
        return list(cls._BACKEND_LABELS.values())

    @classmethod
    def backend_name_from_label(cls, label):
        for backend_name, backend_label in cls._BACKEND_LABELS.items():
            if backend_label == label:
                return backend_name
        return "movella"

    def _create_backend(self, backend_name, frame_rate, backend_config):
        if backend_name == "movella":
            from .imu_backends.movella_backend import MovellaBackend

            return MovellaBackend(frame_rate, **backend_config)

        if backend_name == "websocket":
            from .imu_backends.websocket_backend import WebSocketIMUBackend

            return WebSocketIMUBackend(frame_rate, **backend_config)

        raise ValueError(f"Unsupported IMU backend: {backend_name}")

    def _reset_buffers(self, device_count):
        self.acc_data = np.full((device_count, 3), np.nan, dtype=float)
        self.gyr_data = np.full((device_count, 3), np.nan, dtype=float)
        self.mag_data = np.full((device_count, 3), np.nan, dtype=float)
        self.quat_data = np.full((device_count, 4), np.nan, dtype=float)
        self.calibration_status = np.full((device_count, 1), False)
        self.calibration_inverse = np.full((device_count, 4), np.nan, dtype=float)
        self.sensor_timestamp = np.full((device_count,), np.nan, dtype=float)

    def connected_devices(self) -> list[IMUDeviceInfo]:
        self.device_infos = self.backend.connected_devices()
        self.numOfDevices = len(self.device_infos)
        if self.acc_data.shape[0] != self.numOfDevices:
            self._reset_buffers(self.numOfDevices)
        return list(self.device_infos)

    def disconnect_IMU_sensors(self):
        if self.capabilities.can_reset_heading:
            self.reset_heading_default()
        self.backend.disconnect()
        self.device_infos = []
        self.numOfDevices = 0
        self._reset_buffers(0)

    def sync_IMU_sensors(self):
        if not self.capabilities.can_sync:
            logger.warning(f"{self.backend_name} backend does not support sensor sync.")
            return False
        return self.backend.sync()

    def start_measuring_mode(self):
        return self.backend.start_streaming()

    def stop_measuring_mode(self):
        self.backend.stop_streaming()

    def reset_heading_default(self):
        if not self.capabilities.can_reset_heading:
            logger.warning(f"{self.backend_name} backend does not support heading reset.")
            return False

        success = self.backend.reset_heading_default()
        if success:
            self.reset_heading_status = False
        return success

    def reset_heading(self):
        if not self.capabilities.can_reset_heading:
            logger.warning(f"{self.backend_name} backend does not support heading reset.")
            return False

        success = self.backend.reset_heading()
        if success:
            self.reset_heading_status = True
        return success

    def get_measurments(self):
        snapshot = self.backend.get_snapshot()
        if snapshot is None:
            return

        device_count = snapshot.quat_data.shape[0]
        if device_count != self.numOfDevices:
            self.numOfDevices = device_count
            self._reset_buffers(device_count)

        self.quat_data = snapshot.quat_data
        self.acc_data = snapshot.acc_data
        self.gyr_data = snapshot.gyr_data
        self.mag_data = snapshot.mag_data
        self.sensor_timestamp = snapshot.sensor_timestamp
        self.newPackets = True

        if self.reset_heading_flag:
            self.reset_heading()
            self.reset_heading_flag = False

    def reshape_measurments(self):
        self.acc_data = self.acc_data.reshape(-1, 3)
        self.gyr_data = self.gyr_data.reshape(-1, 3)
        self.mag_data = self.mag_data.reshape(-1, 3)
        self.quat_data = self.quat_data.reshape(-1, 4)

    def calibrate(self):
        self.get_quaternions_inverse()
        return logger.info("Inverse Quaternions Saved")

    def get_quaternions_inverse(self):
        for dev in range(self.numOfDevices):
            if not np.any(np.isnan(self.quat_data[dev, :])):
                squared_sum = (
                    self.quat_data[dev, 0] ** 2
                    + self.quat_data[dev, 1] ** 2
                    + self.quat_data[dev, 2] ** 2
                    + self.quat_data[dev, 3] ** 2
                )

                self.calibration_inverse[dev, :] = np.array(
                    [
                        self.quat_data[dev, 0],
                        -self.quat_data[dev, 1],
                        -self.quat_data[dev, 2],
                        -self.quat_data[dev, 3],
                    ]
                ) / squared_sum
                self.calibration_status[dev] = True
                logger.debug(f"IMU {dev} reference quaternion: {self.quat_data[dev, :]}")
            else:
                logger.warning(f"Data of IMU number {dev} are empty.")
        return
