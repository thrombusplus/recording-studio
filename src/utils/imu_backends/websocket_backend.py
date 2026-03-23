from __future__ import annotations

import asyncio
import json
import threading
from typing import Any

import numpy as np
import websockets

from src.utils.logger import get_logger

from .base import IMUBackend, IMUCapabilities, IMUDeviceInfo, IMUSnapshot


logger = get_logger(__name__)


class WebSocketIMUBackend(IMUBackend):
    name = "websocket"
    display_name = "WebSocket"
    capabilities = IMUCapabilities(
        can_sync=False,
        can_reset_heading=False,
        has_battery_level=False,
    )

    def __init__(self, frame_rate: int, **config):
        super().__init__(frame_rate, **config)
        self.url = config.get("url", "ws://localhost:8186")
        self.device_count = max(int(config.get("device_count", 6)), 1)
        self._device_infos = self._build_device_infos(self.device_count)
        self._connected = False
        self._stop_event = threading.Event()
        self._thread = None
        self._loop = None
        self._websocket = None
        self._lock = threading.Lock()
        self._latest_snapshot = None
        self._snapshot_version = 0
        self._last_consumed_version = 0

    def connect(self) -> list[IMUDeviceInfo]:
        self._connected = True
        return list(self._device_infos)

    def disconnect(self) -> None:
        self.stop_streaming()
        self._connected = False

    def connected_devices(self) -> list[IMUDeviceInfo]:
        return list(self._device_infos) if self._connected else []

    def start_streaming(self) -> bool:
        if not self._connected:
            self.connect()

        if self._thread and self._thread.is_alive():
            logger.info("WebSocket IMU backend is already streaming.")
            return True

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        logger.info(f"Starting WebSocket IMU backend at {self.url}")
        return True

    def stop_streaming(self) -> None:
        self._stop_event.set()

        if self._loop and self._loop.is_running() and self._websocket is not None:
            try:
                future = asyncio.run_coroutine_threadsafe(
                    self._websocket.close(),
                    self._loop,
                )
                future.result(timeout=2)
            except Exception:
                logger.debug("WebSocket backend close signal timed out.")

        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3)
            if self._thread.is_alive():
                logger.warning("WebSocket IMU backend thread did not terminate cleanly.")

        self._thread = None
        self._loop = None
        self._websocket = None

    def get_snapshot(self) -> IMUSnapshot | None:
        with self._lock:
            if self._latest_snapshot is None or self._snapshot_version == self._last_consumed_version:
                return None

            snapshot = IMUSnapshot(
                quat_data=self._latest_snapshot.quat_data.copy(),
                acc_data=self._latest_snapshot.acc_data.copy(),
                gyr_data=self._latest_snapshot.gyr_data.copy(),
                mag_data=self._latest_snapshot.mag_data.copy(),
                sensor_timestamp=self._latest_snapshot.sensor_timestamp.copy(),
            )
            self._last_consumed_version = self._snapshot_version
            return snapshot

    def _run_loop(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._receive_loop())
        finally:
            self._websocket = None
            self._loop.run_until_complete(self._loop.shutdown_asyncgens())
            self._loop.close()
            self._loop = None

    async def _receive_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                async with websockets.connect(self.url) as websocket:
                    self._websocket = websocket
                    logger.info(f"Connected to IMU websocket source at {self.url}")
                    async for message in websocket:
                        if self._stop_event.is_set():
                            break

                        snapshot = self._parse_message(message)
                        if snapshot is None:
                            continue

                        with self._lock:
                            self._latest_snapshot = snapshot
                            self._snapshot_version += 1
            except Exception as exc:
                if self._stop_event.is_set():
                    break

                logger.warning(
                    f"WebSocket IMU backend connection error at {self.url}: {exc}"
                )
                await asyncio.sleep(1)

    def _parse_message(self, message: Any) -> IMUSnapshot | None:
        if isinstance(message, bytes):
            message = message.decode("utf-8")

        if not isinstance(message, str):
            logger.warning("WebSocket IMU backend received a non-text message.")
            return None

        payload = message.strip()
        if not payload:
            return None

        try:
            if payload.startswith("{"):
                return self._parse_json_payload(json.loads(payload))
            return self._parse_csv_payload(payload)
        except Exception as exc:
            logger.warning(f"Failed to parse websocket IMU payload: {exc}")
            return None

    def _parse_json_payload(self, payload: dict[str, Any]) -> IMUSnapshot:
        if "devices" in payload:
            return self._parse_devices_payload(payload["devices"])
        return self._parse_matrix_payload(payload)

    def _parse_devices_payload(self, devices: list[dict[str, Any]]) -> IMUSnapshot:
        quat_data = np.full((self.device_count, 4), np.nan, dtype=float)
        acc_data = np.full((self.device_count, 3), np.nan, dtype=float)
        gyr_data = np.full((self.device_count, 3), np.nan, dtype=float)
        mag_data = np.full((self.device_count, 3), np.nan, dtype=float)
        sensor_timestamp = np.full((self.device_count,), np.nan, dtype=float)

        for index, device in enumerate(devices[: self.device_count]):
            quat_data[index] = self._vector(device.get("quat") or device.get("quaternion"), 4)
            acc_data[index] = self._vector(device.get("acc") or device.get("accelerometer"), 3)
            gyr_data[index] = self._vector(
                device.get("gyr") or device.get("gyro") or device.get("ang"),
                3,
            )
            mag_data[index] = self._vector(device.get("mag") or device.get("magnetometer"), 3)
            sensor_timestamp[index] = self._scalar(
                device.get("sensor_timestamp") or device.get("sensor_ts") or device.get("timestamp")
            )

        return IMUSnapshot(
            quat_data=quat_data,
            acc_data=acc_data,
            gyr_data=gyr_data,
            mag_data=mag_data,
            sensor_timestamp=sensor_timestamp,
        )

    def _parse_matrix_payload(self, payload: dict[str, Any]) -> IMUSnapshot:
        quat_data = self._matrix(
            payload.get("quat") or payload.get("quaternions"),
            self.device_count,
            4,
        )
        acc_data = self._matrix(
            payload.get("acc") or payload.get("accelerometer"),
            self.device_count,
            3,
        )
        gyr_data = self._matrix(
            payload.get("gyr") or payload.get("gyro") or payload.get("ang"),
            self.device_count,
            3,
        )
        mag_data = self._matrix(
            payload.get("mag") or payload.get("magnetometer"),
            self.device_count,
            3,
        )

        raw_sensor_timestamps = (
            payload.get("sensor_timestamp")
            or payload.get("sensor_ts")
            or payload.get("timestamps")
        )
        sensor_timestamp = np.full((self.device_count,), np.nan, dtype=float)
        if raw_sensor_timestamps is not None:
            for index, value in enumerate(list(raw_sensor_timestamps)[: self.device_count]):
                sensor_timestamp[index] = self._scalar(value)

        return IMUSnapshot(
            quat_data=quat_data,
            acc_data=acc_data,
            gyr_data=gyr_data,
            mag_data=mag_data,
            sensor_timestamp=sensor_timestamp,
        )

    def _parse_csv_payload(self, payload: str) -> IMUSnapshot | None:
        raw_parts = [part.strip() for part in payload.split(",") if part.strip() != ""]
        if not raw_parts:
            return None

        values = []
        for part in raw_parts:
            try:
                values.append(float(part))
            except ValueError:
                logger.warning(f"Skipping non-numeric websocket IMU value: {part}")
                return None

        device_values = values[1:] if len(values) > self.device_count * 10 else values
        expected_values = self.device_count * 10

        if len(device_values) < expected_values:
            device_values.extend([np.nan] * (expected_values - len(device_values)))
        elif len(device_values) > expected_values:
            device_values = device_values[:expected_values]

        sensor_matrix = np.array(device_values, dtype=float).reshape(self.device_count, 10)
        return IMUSnapshot(
            quat_data=sensor_matrix[:, :4],
            acc_data=sensor_matrix[:, 4:7],
            gyr_data=sensor_matrix[:, 7:10],
            mag_data=np.full((self.device_count, 3), np.nan, dtype=float),
            sensor_timestamp=np.full((self.device_count,), np.nan, dtype=float),
        )

    @staticmethod
    def _scalar(value: Any) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return np.nan

    @staticmethod
    def _vector(values: Any, expected_length: int) -> np.ndarray:
        vector = np.full((expected_length,), np.nan, dtype=float)
        if values is None:
            return vector

        for index, value in enumerate(list(values)[:expected_length]):
            vector[index] = WebSocketIMUBackend._scalar(value)
        return vector

    @staticmethod
    def _matrix(values: Any, rows: int, cols: int) -> np.ndarray:
        matrix = np.full((rows, cols), np.nan, dtype=float)
        if values is None:
            return matrix

        for row_index, row in enumerate(list(values)[:rows]):
            matrix[row_index] = WebSocketIMUBackend._vector(row, cols)
        return matrix

    @staticmethod
    def _build_device_infos(device_count: int) -> list[IMUDeviceInfo]:
        return [
            IMUDeviceInfo(identifier=f"ws-{index}", tag=f"WS IMU {index + 1}")
            for index in range(device_count)
        ]
