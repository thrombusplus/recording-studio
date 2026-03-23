from __future__ import annotations

import numpy as np

from src.utils.logger import get_logger

from .base import IMUBackend, IMUCapabilities, IMUDeviceInfo, IMUSnapshot


logger = get_logger(__name__)


class MovellaBackend(IMUBackend):
    name = "movella"
    display_name = "Movella DOT"
    capabilities = IMUCapabilities(
        can_sync=True,
        can_reset_heading=True,
        has_battery_level=True,
    )

    def __init__(self, frame_rate: int, **config):
        super().__init__(frame_rate, **config)
        self._handler = None
        self._sdk = None
        self._device_infos: list[IMUDeviceInfo] = []

    def connect(self) -> list[IMUDeviceInfo]:
        from src.utils.xdpchandler import XdpcHandler, movelladot_pc_sdk

        self._sdk = movelladot_pc_sdk
        self._handler = XdpcHandler()

        if not self._handler.initialize():
            self._handler.cleanup()
            self._handler = None
            return []

        self._handler.scanForDots()
        if len(self._handler.detectedDots()) == 0:
            logger.warning("No Movella DOT device(s) found. Aborting.")
            self._handler.cleanup()
            self._handler = None
            return []

        self._handler.connectDots()
        if len(self._handler.connectedDots()) == 0:
            logger.warning("Could not connect to any Movella DOT device(s). Aborting.")
            self._handler.cleanup()
            self._handler = None
            return []

        for device in self._handler.connectedDots():
            filter_profiles = device.getAvailableFilterProfiles()
            logger.info("Available filter profiles:")
            for filter_profile in filter_profiles:
                logger.debug(filter_profile.label())

            logger.info(f"Current profile: {device.onboardFilterProfile().label()}")
            if device.setOnboardFilterProfile("General"):
                logger.info("Successfully set profile to General")
            else:
                logger.error("Setting filter profile failed!")

            if device.setOutputRate(self.frame_rate):
                logger.info(f"Successfully set output rate to {self.frame_rate} Hz")
            else:
                logger.error("Setting output rate failed!")

        self._device_infos = self._build_device_infos()
        return list(self._device_infos)

    def disconnect(self) -> None:
        if self._handler is None:
            return

        self._handler.cleanup()
        self._handler = None
        self._device_infos = []

    def connected_devices(self) -> list[IMUDeviceInfo]:
        if self._handler is None:
            return []

        self._device_infos = self._build_device_infos()
        return list(self._device_infos)

    def start_streaming(self) -> bool:
        if self._handler is None or self._sdk is None:
            logger.warning("Movella backend is not connected.")
            return False

        logger.info("Starting measurement...")
        success = False
        for device in self._handler.connectedDots():
            if not device.startMeasurement(self._sdk.XsPayloadMode_CustomMode4):
                logger.warning(
                    f"Could not put device into measurement mode. Reason: {device.lastResultText()}"
                )
                continue
            success = True
        return success

    def stop_streaming(self) -> None:
        if self._handler is None:
            return

        logger.info("Stopping measurement...")
        for device in self._handler.connectedDots():
            if not device.stopMeasurement():
                logger.error("Failed to stop measurement.")
            if not device.disableLogging():
                logger.error("Failed to disable logging.")

    def sync(self) -> bool:
        if self._handler is None:
            logger.warning("Movella backend is not connected.")
            return False

        manager = self._handler.manager()
        device_list = self._handler.connectedDots()
        logger.info(f"Preparing to sync {len(device_list)} devices.")

        logger.info("Stopping measurement on all devices...")
        for device in device_list:
            try:
                device.stopMeasurement()
                logger.info(f"Stopped measurement on {device.bluetoothAddress()}")
            except Exception as exc:
                logger.error(
                    f"Could not stop measurement on {device.bluetoothAddress()}: {exc}"
                )

        logger.info("Forcing stopSync...")
        try:
            manager.stopSync()
        except Exception as exc:
            logger.warning(f"stopSync error (ignored): {exc}")

        if not device_list:
            logger.warning("No connected Movella devices available for syncing.")
            return False

        root_node = device_list[-1].bluetoothAddress()
        logger.info(f"Starting sync... Root node: {root_node}")
        success = manager.startSync(root_node)

        if success:
            logger.info("Devices synchronized successfully.")
        else:
            logger.warning(f"Could not start sync. Reason: {manager.lastResultText()}")

        return success

    def reset_heading(self) -> bool:
        if self._handler is None or self._sdk is None:
            return False

        success = False
        for device in self._handler.connectedDots():
            logger.info(
                f"Resetting heading for device {device.portInfo().bluetoothAddress()}: "
            )
            if device.resetOrientation(self._sdk.XRM_Heading):
                logger.info("OK")
                success = True
            else:
                logger.warning(f"NOK: {device.lastResultText()}")
        return success

    def reset_heading_default(self) -> bool:
        if self._handler is None or self._sdk is None:
            return False

        success = False
        for device in self._handler.connectedDots():
            logger.info(
                f"Resetting heading to default for device {device.portInfo().bluetoothAddress()}: "
            )
            if device.resetOrientation(self._sdk.XRM_DefaultAlignment):
                logger.info("OK")
                success = True
            else:
                logger.warning(f"NOK: {device.lastResultText()}")
        return success

    def get_snapshot(self) -> IMUSnapshot | None:
        if self._handler is None or not self._handler.packetsAvailable():
            return None

        devices = self._handler.connectedDots()
        device_count = len(devices)
        acc_data = np.full((device_count, 3), np.nan, dtype=float)
        gyr_data = np.full((device_count, 3), np.nan, dtype=float)
        mag_data = np.full((device_count, 3), np.nan, dtype=float)
        quat_data = np.full((device_count, 4), np.nan, dtype=float)
        sensor_timestamp = np.full((device_count,), np.nan, dtype=float)

        for index, device in enumerate(devices):
            packet = self._handler.getNextPacket(device.portInfo().bluetoothAddress())
            if packet is None:
                continue

            if packet.containsSampleTimeFine():
                sensor_timestamp[index] = packet.sampleTimeFine()

            if packet.containsCalibratedData():
                data = packet.calibratedData()
                acc_data[index, :] = data.m_acc
                gyr_data[index, :] = data.m_gyr
                mag_data[index, :] = data.m_mag

            if packet.containsOrientation():
                quat_data[index, :] = packet.orientationQuaternion()

        return IMUSnapshot(
            quat_data=quat_data,
            acc_data=acc_data,
            gyr_data=gyr_data,
            mag_data=mag_data,
            sensor_timestamp=sensor_timestamp,
        )

    def _build_device_infos(self) -> list[IMUDeviceInfo]:
        if self._handler is None:
            return []

        device_infos = []
        for device in self._handler.connectedDots():
            device_infos.append(
                IMUDeviceInfo(
                    identifier=device.portInfo().bluetoothAddress(),
                    tag=device.deviceTagName(),
                    battery_level=device.batteryLevel(),
                )
            )
        return device_infos
