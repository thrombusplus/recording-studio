from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(slots=True)
class IMUDeviceInfo:
    identifier: str
    tag: str
    battery_level: Optional[int] = None


@dataclass(frozen=True, slots=True)
class IMUCapabilities:
    can_sync: bool = False
    can_reset_heading: bool = False
    has_battery_level: bool = False


@dataclass(slots=True)
class IMUSnapshot:
    quat_data: np.ndarray
    acc_data: np.ndarray
    gyr_data: np.ndarray
    mag_data: np.ndarray
    sensor_timestamp: np.ndarray


class IMUBackend(ABC):
    name = "base"
    display_name = "Base"
    capabilities = IMUCapabilities()

    def __init__(self, frame_rate: int, **config):
        self.frame_rate = frame_rate
        self.config = config

    @abstractmethod
    def connect(self) -> list[IMUDeviceInfo]:
        """Create or discover IMU devices and return their metadata."""

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect and clean up backend resources."""

    @abstractmethod
    def connected_devices(self) -> list[IMUDeviceInfo]:
        """Return metadata for currently connected devices."""

    @abstractmethod
    def start_streaming(self) -> bool:
        """Start live data streaming."""

    @abstractmethod
    def stop_streaming(self) -> None:
        """Stop live data streaming."""

    @abstractmethod
    def get_snapshot(self) -> Optional[IMUSnapshot]:
        """Return the latest snapshot if a new one is available."""

    def sync(self) -> bool:
        return False

    def reset_heading(self) -> bool:
        return False

    def reset_heading_default(self) -> bool:
        return False
