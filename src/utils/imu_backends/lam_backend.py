import socket
import struct
import threading
import json
import time
import math
from typing import List, Dict, Callable, Optional

import numpy as np
from zeroconf import Zeroconf, ServiceBrowser

from src.utils.logger import get_logger

from .base import IMUBackend, IMUCapabilities, IMUDeviceInfo, IMUSnapshot


logger = get_logger(__name__)


class LAMDiscoveredDevice:
    def __init__(self, hostname: str, ip_address: str, port: int):
        self.hostname = hostname
        self.ip_address = ip_address
        self.port = port

class IMUServiceListener:
    def __init__(self):
        self.devices = []

    def remove_service(self, zeroconf, type, name):
        pass

    def update_service(self, zeroconf, type, name):
        pass

    def add_service(self, zeroconf, type, name):
        info = zeroconf.get_service_info(type, name)
        if info and info.parsed_addresses():
            ip_address = info.parsed_addresses()[0]
            self.devices.append(LAMDiscoveredDevice(
                hostname=info.server,
                ip_address=ip_address,
                port=info.port
            ))

class LAM:
    HEARTBEAT_INTERVAL = 2.0  # Seconds

    def __init__(self):
        self._udp_socket = None
        self._is_listening = False
        self._listen_thread = None
        self._heartbeat_thread = None
        self._remote_ip = ""
        self._remote_port = 5000
        
        self.status_json = ""
        self.status = {}
        self.last_status_update = 0
        
        # Callbacks
        self.on_status_updated: Callable[[str], None] = None
        self.on_data_received: Callable[[Dict], None] = None

    @staticmethod
    def discover_devices(service_type: str = "_imu._udp.local.", timeout: float = 3.0) -> List[LAMDiscoveredDevice]:
        zeroconf = Zeroconf()
        listener = IMUServiceListener()
        browser = ServiceBrowser(zeroconf, service_type, listener)
        time.sleep(timeout)
        zeroconf.close()
        return listener.devices

    def initialize(self, remote_ip: str, remote_port: int, local_port: int = 0):
        self._remote_ip = remote_ip
        self._remote_port = remote_port
        
        self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_socket.bind(('', local_port))
        
        self.start_listening()
        self.send_heartbeat()
        
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()

    def _heartbeat_loop(self):
        while self._is_listening:
            self.send_heartbeat()
            current_time = time.time()
            if self.last_status_update > 0 and (current_time - self.last_status_update) > 4:
                if self.on_status_updated:
                    self.on_status_updated("Disconnected")
                self.stop_listening()
                break
            time.sleep(self.HEARTBEAT_INTERVAL)

    def send_heartbeat(self):
        self.send_command("ping")
        self.get_status()

    def start_listening(self):
        if self._is_listening:
            return
        self._is_listening = True
        self._listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._listen_thread.start()

    def stop_listening(self):
        self._is_listening = False
        if self._udp_socket:
            self._udp_socket.close()
            self._udp_socket = None

    def send_command(self, cmd: str):
        if self._udp_socket:
            self._udp_socket.sendto(cmd.encode('utf-8'), (self._remote_ip, self._remote_port))

    def send_initialize(self): self.send_command("initialize")
    def connect(self, port_arg: str): self.send_command(f"connect:{port_arg}")
    def set_sampling_rate(self, rate: int): self.send_command(f"setSamplingRate:{rate}")
    def start_streaming(self): self.send_command("startStreaming")
    def stop_streaming(self): self.send_command("stopStreaming")
    def get_status(self): self.send_command("getStatus")
    def reset_imu(self): self.send_command("reset")
    def disconnect(self): self.send_command("disconnect")

    def _listen_loop(self):
        while self._is_listening:
            try:
                data, _ = self._udp_socket.recvfrom(4096)
                self._handle_data_received(data)
            except Exception as e:
                if self._is_listening:
                    pass # Handle socket closure or timeout

    def _handle_data_received(self, data: bytes):
        logger.debug(f"Received {len(data)} bytes")
        
        if len(data) >= 4:
            sync = struct.unpack('<I', data[0:4])[0]
            logger.debug(f"First 4 bytes as uint32: 0x{sync:08X}")
            if sync == 0xDEADBEEF:
                logger.debug("Recognized as binary 0xDEADBEEF format")
                self._parse_binary_payload(data)
                return

        # If not binary 0xDEADBEEF, treat as JSON string
        try:
            msg = data.decode('utf-8')
            logger.debug(f"Treating as JSON: {msg[:100]}")
            self.status_json = msg
            self.status = json.loads(msg)
            self.last_status_update = time.time()
            if self.on_status_updated:
                self.on_status_updated(msg)
        except json.JSONDecodeError:
            logger.debug(f"Failed to parse as JSON, hex: {data[:32].hex()}")

    def _parse_binary_payload(self, data: bytes):
        """Parse LAM binary payload.
        
        The device sends multiple packets per frame. Each packet contains:
        - Main Header (48 bytes) - shared info
        - One SAGA block - one sensor's data
        
        We accumulate SAGAs until all are received, then emit the combined payload.
        """
        if len(data) < 48:
            logger.warning(f"Binary data too short: {len(data)} bytes")
            return

        # Parse Main Header
        sync = struct.unpack('<I', data[0:4])[0]
        total_packs = data[4]
        pack_no = data[5]
        samples = data[6]
        samples_temp = data[7]
        timestamp_ms = struct.unpack('<Q', data[8:16])[0]
        config_main = data[16:28]
        trans_int_ms = struct.unpack('<H', data[28:30])[0]
        fs = data[30]
        fs_temp = data[31]
        saga_map = list(data[32:42])
        
        logger.debug(f"Binary payload: total_packs={total_packs}, pack_no={pack_no}, samples={samples}, saga_map={saga_map}")

        main_head = {
            "SYNC": sync,
            "Total_packs": total_packs,
            "Pack_no": pack_no,
            "Samples": samples,
            "Samples_temp": samples_temp,
            "Timestamp_ms": timestamp_ms,
            "CONFIG": config_main,
            "Trans_int_ms": trans_int_ms,
            "Fs": fs,
            "Fs_temp": fs_temp,
            "SAGA_MAP": saga_map
        }

        # On first packet of frame, initialize accumulation state
        if pack_no == 1:
            self._frame_accum_sagas = [None] * 10
            self._frame_accum_main_head = main_head.copy()
            self._frame_accum_remaining = sum(saga_map)
            self._frame_accum_timestamp = timestamp_ms
            logger.debug(f"Started frame accumulation, remaining SAGAs: {self._frame_accum_remaining}")

        # Parse SAGA from byte 48 (single SAGA per packet)
        if len(data) < 52:
            logger.warning("Data too short for SAGA header")
            return
            
        saga_sync = struct.unpack('<I', data[48:52])[0]
        if saga_sync != 0xDEADBEEF:
            logger.warning(f"Invalid SAGA sync: expected 0xDEADBEEF, got {hex(saga_sync)}")
            return

        # Parse SAGA header
        saga_length = struct.unpack('<H', data[52:54])[0]
        saga_no = struct.unpack('<H', data[54:56])[0]
        
        logger.debug(f"Parsed SAGA: no={saga_no}, length={saga_length}")
        
        # Extract SAGA data
        saga_data = data[48:48+saga_length]
        parsed_saga = self._parse_lam_data(saga_data, samples, samples_temp, main_head)
        
        # Store in accumulation buffer
        self._frame_accum_sagas[saga_no] = parsed_saga
        self._frame_accum_remaining -= 1
        
        logger.debug(f"SAGA {saga_no} stored, remaining: {self._frame_accum_remaining}")

        # When all SAGAs received, emit payload
        if self._frame_accum_remaining == 0:
            logger.debug("All SAGAs received, emitting payload")
            payload = {
                "MainHead": self._frame_accum_main_head,
                "Sagas": self._frame_accum_sagas,
                "Timestamp_ms": self._frame_accum_timestamp
            }
            if self.on_data_received:
                self.on_data_received(payload)

    def _parse_lam_data(self, saga_data: bytes, smp: int, smp_t: int, main_head: Dict) -> Dict:
        """Parse SAGA data matching C# implementation.
        
        Config bits: 0=ACC, 1=GYR, 2=MAG, 3=QUAT, 4=Temp, 5=PPG, 6=Test
        """
        sync = struct.unpack('<I', saga_data[0:4])[0]
        length = struct.unpack('<H', saga_data[4:6])[0]
        saga_no = struct.unpack('<H', saga_data[6:8])[0]
        config = saga_data[8:20]
        empty = struct.unpack('<I', saga_data[20:24])[0]
        
        body_length = length - 24
        float_count = body_length // 4
        body = struct.unpack(f'<{float_count}f', saga_data[24:24+body_length])
        
        parsed_saga = {
            "SAGA_no": saga_no, 
            "SYNC": sync,
            "LENGTH": length,
            "CONFIG": config,
            "Empty": empty,
            "Data": {}
        }
        cnt = 0
        
        # Parse sensor data based on config bits (matching C# implementation)
        if config[0] == 1:  # ACC
            parsed_saga["Data"]["ACC_X"] = list(body[cnt:cnt+smp]); cnt += smp
            parsed_saga["Data"]["ACC_Y"] = list(body[cnt:cnt+smp]); cnt += smp
            parsed_saga["Data"]["ACC_Z"] = list(body[cnt:cnt+smp]); cnt += smp
            
        if config[1] == 1:  # GYR
            parsed_saga["Data"]["GYR_X"] = list(body[cnt:cnt+smp]); cnt += smp
            parsed_saga["Data"]["GYR_Y"] = list(body[cnt:cnt+smp]); cnt += smp
            parsed_saga["Data"]["GYR_Z"] = list(body[cnt:cnt+smp]); cnt += smp
            
        if config[2] == 1:  # MAG
            parsed_saga["Data"]["MAG_X"] = list(body[cnt:cnt+smp]); cnt += smp
            parsed_saga["Data"]["MAG_Y"] = list(body[cnt:cnt+smp]); cnt += smp
            parsed_saga["Data"]["MAG_Z"] = list(body[cnt:cnt+smp]); cnt += smp
            
        if config[3] == 1:  # QUAT (W, X, Y, Z order like C#)
            parsed_saga["Data"]["QUAT_W"] = list(body[cnt:cnt+smp]); cnt += smp
            parsed_saga["Data"]["QUAT_X"] = list(body[cnt:cnt+smp]); cnt += smp
            parsed_saga["Data"]["QUAT_Y"] = list(body[cnt:cnt+smp]); cnt += smp
            parsed_saga["Data"]["QUAT_Z"] = list(body[cnt:cnt+smp]); cnt += smp
            
        if config[4] == 1:  # Temperature (matching C# implementation)
            parsed_saga["Data"]["T_AMBIENT"] = list(body[cnt:cnt+smp_t]); cnt += smp_t
            parsed_saga["Data"]["T_BODY"] = list(body[cnt:cnt+smp_t]); cnt += smp_t
            
        if config[5] == 1:  # PPG (IR, R, G, B)
            parsed_saga["Data"]["PPG_IR"] = list(body[cnt:cnt+smp]); cnt += smp
            parsed_saga["Data"]["PPG_R"] = list(body[cnt:cnt+smp]); cnt += smp
            parsed_saga["Data"]["PPG_G"] = list(body[cnt:cnt+smp]); cnt += smp
            parsed_saga["Data"]["PPG_B"] = list(body[cnt:cnt+smp]); cnt += smp
            
        if config[6] == 1:  # Test data
            parsed_saga["Data"]["TEST1"] = list(body[cnt:cnt+smp]); cnt += smp
            parsed_saga["Data"]["TEST2"] = list(body[cnt:cnt+smp]); cnt += smp
        
        return parsed_saga


class LAMBackend(IMUBackend):
    """IMU Backend for LAM (Local Area Motion) IMU sensors.
    
    This backend supports LAM IMU sensors via UDP protocol with mDNS discovery.
    Default sensor order: {7, 6, 5, 2, 1, 0}
    """
    
    name = "lam"
    display_name = "LAM IMU"
    capabilities = IMUCapabilities(
        can_sync=False,
        can_reset_heading=True,
        has_battery_level=False,
    )
    
    # Default sensor order for LAM IMU body tracking:
    DefaultSensorUnitIds = [1, 2, 3, 6, 7, 8]
    
    def __init__(self, frame_rate: int, **config):
        super().__init__(frame_rate, **config)
        self._lam = LAM()
        self._connected = False
        self._streaming = False
        self._discovering = False
        self._discovery_complete = False
        self._latest_payload = None
        self._payload_lock = threading.Lock()
        self._discovery_thread = None
        
        # Configuration options
        self.ip_address = config.get("ip_address", None)
        self.port = int(config.get("port", 5000))
        self.device_count = int(config.get("device_count", 6))
        self.discovery_timeout = float(config.get("discovery_timeout", 3.0))
        self.local_port = int(config.get("local_port", 0))
        
        # Quaternion reference management (matching C# implementation)
        self._reset_zero = False
        self._zero_yaw = False
        self._qref = [(1.0, 0.0, 0.0, 0.0)] * 10  # Identity quaternions
        self._qreal = [(1.0, 0.0, 0.0, 0.0)] * 10
        self._qyaw_zer = [(1.0, 0.0, 0.0, 0.0)] * 10
        
        # Build device infos based on sensor order (initially empty until discovery)
        self._device_infos: list[IMUDeviceInfo] = []
        self._discovered_devices: list[LAMDiscoveredDevice] = []
        
        # Set up data callback
        self._lam.on_data_received = self._handle_data
    
    def set_zero(self):
        """Set zero flag for orientation reset (matches C# Set_zero())."""
        self._reset_zero = True
        
    def set_zero_yaw(self):
        """Set zero yaw flag for yaw reset (matches C# Zero_yaw)."""
        self._zero_yaw = True
        
    def _normalize_quaternion(self, q):
        """Normalize quaternion."""
        w, x, y, z = q
        norm = (w*w + x*x + y*y + z*z) ** 0.5
        if norm > 1e-6:
            return (w/norm, x/norm, y/norm, z/norm)
        return (1.0, 0.0, 0.0, 0.0)  # Return identity if degenerate
    
    def _quaternion_multiply(self, q1, q2):
        """Multiply two quaternions q1 * q2."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return (
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        )
    
    def _quaternion_conjugate(self, q):
        """Get quaternion conjugate."""
        w, x, y, z = q
        return (w, -x, -y, -z)
    
    def _get_yaw_from_quaternion(self, q):
        """Extract yaw angle from quaternion (radians)."""
        w, x, y, z = self._normalize_quaternion(q)
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        num = 2.0 * (w*z + x*y)
        den = 1.0 - 2.0 * (y*y + z*z)
        return math.atan2(num, den)
    
    def _create_yaw_quaternion(self, yaw):
        """Create quaternion for rotation about Z axis by yaw radians."""
        half_yaw = yaw / 2.0
        return (math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw))
    
    def _reset_orientation(self, quaternions):
        """Apply quaternion reset/orientation correction.
        
        Matches C# ResetOrientation() method.
        """
        corrected = []
        for i, q in enumerate(quaternions):
            if q is None:
                corrected.append(None)
                continue
                
            # Normalize input quaternion
            q_normalized = self._normalize_quaternion(q)
            
            # Store in Qreal
            self._qreal[i] = q_normalized
            
            if self._zero_yaw:
                # Extract yaw and create reference quaternion
                yaw = self._get_yaw_from_quaternion(q_normalized)
                q_yaw = self._create_yaw_quaternion(yaw)
                q_yaw_conj = self._quaternion_conjugate(q_yaw)
                self._qref[i] = q_yaw_conj  # Qref * Qreal removes yaw
                
            if self._reset_zero:
                # Reset full reference to make corrected orientation identity
                self._qref[i] = self._quaternion_conjugate(q_normalized)
            
            # Apply correction: Qcorrected = Qref * Qreal
            q_corrected = self._quaternion_multiply(self._qref[i], q_normalized)
            q_corrected = self._normalize_quaternion(q_corrected)
            corrected.append(q_corrected)
        
        # Reset flags
        self._zero_yaw = False
        self._reset_zero = False
        
        return corrected
    
    def discover_devices_async(self, timeout: float = None, callback: Callable = None) -> None:
        """Start asynchronous device discovery.
        
        Args:
            timeout: Discovery timeout in seconds (uses config default if None)
            callback: Optional callback function called when discovery completes
        """
        if self._discovering:
            logger.warning("LAM discovery already in progress")
            return
        
        if timeout is None:
            timeout = self.discovery_timeout
        
        self._discovering = True
        self._discovery_complete = False
        self._discovered_devices = []
        
        def _discovery_worker():
            try:
                logger.info(f"Starting LAM device discovery (timeout: {timeout}s)...")
                self._discovered_devices = LAM.discover_devices(timeout=timeout)
                logger.info(f"LAM discovery complete: found {len(self._discovered_devices)} devices")
            except Exception as e:
                logger.error(f"LAM discovery error: {e}")
                self._discovered_devices = []
            finally:
                self._discovering = False
                self._discovery_complete = True
                if callback:
                    callback(self._discovered_devices)
        
        self._discovery_thread = threading.Thread(target=_discovery_worker, daemon=True)
        self._discovery_thread.start()
    
    def is_discovering(self) -> bool:
        """Check if discovery is in progress."""
        return self._discovering
    
    def is_discovery_complete(self) -> bool:
        """Check if discovery has completed (success or failure)."""
        return self._discovery_complete
    
    def get_discovered_devices(self) -> list:
        """Get list of discovered LAM devices."""
        return list(self._discovered_devices)
    
    def cancel_discovery(self) -> None:
        """Cancel ongoing discovery."""
        if self._discovering:
            logger.info("Cancelling LAM discovery...")
            # Note: Zeroconf discovery can't be easily cancelled, but we can stop tracking
            self._discovering = False
    
    def _build_device_infos(self) -> list[IMUDeviceInfo]:
        """Build device info list based on sensor order."""
        device_infos = []
        for i, sensor_id in enumerate(self.DefaultSensorUnitIds[:self.device_count]):
            device_infos.append(
                IMUDeviceInfo(
                    identifier=f"LAM_SENSOR_{sensor_id}",
                    tag=f"Sensor {sensor_id}",
                    battery_level=None
                )
            )
        return device_infos
    
    def _handle_data(self, payload: Dict):
        """Callback for LAM data reception."""
        with self._payload_lock:
            self._latest_payload = payload
    
    def connect(self) -> list[IMUDeviceInfo]:
        """Connect to LAM IMU devices via discovery or direct IP."""
        if self._connected:
            logger.info("LAM backend already connected.")
            return list(self._device_infos)
        
        # Try to use discovered devices first
        if self._discovered_devices:
            self.ip_address = self._discovered_devices[0].ip_address
            self.port = self._discovered_devices[0].port
            logger.info(f"Using discovered LAM device at {self.ip_address}:{self.port}")
        elif self.ip_address is None:
            # If no IP specified and no discoveries, connection should fail
            logger.warning("Cannot connect: No LAM devices discovered and no IP specified.")
            return []
        
        # Initialize LAM connection
        try:
            self._lam.initialize(self.ip_address, self.port, self.local_port)
            self._connected = True
            logger.info(f"LAM backend connected to {self.ip_address}:{self.port}")
            
            # Build device infos for connected devices
            self._device_infos = self._build_device_infos()
        except Exception as e:
            logger.error(f"Failed to connect to LAM backend: {e}")
            self._connected = False
            self._device_infos = []
            return []
        
        return list(self._device_infos)
    
    def disconnect(self) -> None:
        """Disconnect from LAM IMU devices."""
        if not self._connected:
            return
        
        self.stop_streaming()
        self._lam.stop_listening()
        self._connected = False
        logger.info("LAM backend disconnected.")
    
    def connected_devices(self) -> list[IMUDeviceInfo]:
        """Return list of connected devices."""
        return list(self._device_infos) if self._connected else []
    
    def start_streaming(self) -> bool:
        """Start streaming data from LAM IMU."""
        if not self._connected:
            success = self.connect()
            if not success:
                return False
        
        if self._streaming:
            logger.info("LAM backend already streaming.")
            return True
        
        try:
            # Start listening for data
            self._lam.start_listening()
            
            # Initialize and start streaming
            self._lam.send_initialize()
            time.sleep(0.5)  # Allow initialization
            self._lam.set_sampling_rate(int(self.frame_rate))
            time.sleep(0.1)
            self._lam.start_streaming()
            
            self._streaming = True
            logger.info(f"LAM backend started streaming at {self.frame_rate} Hz")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start LAM streaming: {e}")
            return False
    
    def stop_streaming(self) -> None:
        """Stop streaming data from LAM IMU."""
        if not self._streaming:
            return
        
        try:
            self._lam.stop_streaming()
            logger.info("LAM backend stopped streaming")
        except Exception as e:
            logger.error(f"Error stopping LAM streaming: {e}")
        finally:
            self._streaming = False
    
    def sync(self) -> bool:
        """Sync LAM IMU sensors (not supported)."""
        logger.warning("LAM backend does not support sensor sync.")
        return False
    
    def reset_heading(self) -> bool:
        """Reset heading of all LAM IMU sensors (full reset).
        
        Matches C# Reset_zero = true behavior.
        """
        if not self._connected:
            logger.warning("Cannot reset heading: LAM backend not connected.")
            return False
        
        try:
            # Set full reset flag (matches C# Reset_zero = true)
            self.set_zero()
            # Send reset command to device
            self._lam.reset_imu()
            logger.info("LAM heading reset command sent (full reset)")
            return True
        except Exception as e:
            logger.error(f"Failed to reset LAM heading: {e}")
            return False
    
    def reset_heading_default(self) -> bool:
        """Reset heading to default alignment with yaw reset.
        
        Matches C# Zero_yaw = true behavior.
        """
        if not self._connected:
            logger.warning("Cannot reset heading: LAM backend not connected.")
            return False
        
        try:
            # Set yaw reset flag (matches C# Zero_yaw = true)
            self.set_zero_yaw()
            logger.info("LAM heading reset command sent (yaw reset)")
            return True
        except Exception as e:
            logger.error(f"Failed to reset LAM heading: {e}")
            return False
    
    def get_snapshot(self) -> Optional[IMUSnapshot]:
        """Get latest IMU snapshot from LAM data."""
        if not self._streaming:
            return None
        
        with self._payload_lock:
            if self._latest_payload is None:
                return None
            
            try:
                return self._parse_lam_payload(self._latest_payload)
            except Exception as e:
                logger.error(f"Error parsing LAM payload: {e}")
                return None
    
    def _parse_lam_payload(self, payload: Dict) -> IMUSnapshot:
        """Parse LAM payload into IMUSnapshot format.
        
        Handles multi-sensor data and maps to sensor order.
        Matches C# Queue_Package and ResetOrientation logic.
        """
        main_head = payload.get("MainHead", {})
        sagas = payload.get("Sagas", [])
        
        # Determine device count from configuration
        device_count = self.device_count
        samples = main_head.get("Samples", 1)
        saga_map = main_head.get("SAGA_MAP", [])
        
        # Initialize arrays
        acc_data = np.full((device_count, 3), np.nan, dtype=float)
        gyr_data = np.full((device_count, 3), np.nan, dtype=float)
        mag_data = np.full((device_count, 3), np.nan, dtype=float)
        quat_data = np.full((device_count, 4), np.nan, dtype=float)
        sensor_timestamp = np.full((device_count,), np.nan, dtype=float)
        
        # Parse SAGA data
        for saga in sagas:
            if saga is None:
                continue
            
            saga_no = saga.get("SAGA_no", -1)
            data = saga.get("Data", {})
            
            # Map SAGA number to device index based on sensor order
            try:
                device_index = self.DefaultSensorUnitIds.index(saga_no)
                if device_index >= device_count:
                    continue
            except ValueError:
                logger.warning(f"Unknown SAGA number: {saga_no}")
                continue
            
            # Get timestamp for this sample (matches C#: MainHead.Timestamp_ms + n * 20)
            base_timestamp_ms = main_head.get("Timestamp_ms", 0)
            sample_timestamp_ms = base_timestamp_ms + (samples - 1) * 20  # Use last sample
            sensor_timestamp[device_index] = sample_timestamp_ms / 1000.0  # Convert to seconds
            
            # Extract ACC data (matching C# XYZ to YZX transformation)
            if "ACC_X" in data and "ACC_Y" in data and "ACC_Z" in data:
                acc_x = data["ACC_X"]
                acc_y = data["ACC_Y"] 
                acc_z = data["ACC_Z"]
                if acc_x and acc_y and acc_z:
                    # C# code shows XYZ -> YZX transformation
                    acc_data[device_index, 0] = acc_y[-1]  # Y -> X
                    acc_data[device_index, 1] = acc_z[-1]  # Z -> Y  
                    acc_data[device_index, 2] = acc_x[-1]  # X -> Z
            
            # Extract GYR data (matching C# XYZ -> YZX transformation)
            if "GYR_X" in data and "GYR_Y" in data and "GYR_Z" in data:
                gyr_x = data["GYR_X"]
                gyr_y = data["GYR_Y"]
                gyr_z = data["GYR_Z"]
                if gyr_x and gyr_y and gyr_z:
                    # C# code shows XYZ -> YZX transformation
                    gyr_data[device_index, 0] = gyr_y[-1]  # Y -> X
                    gyr_data[device_index, 1] = gyr_z[-1]  # Z -> Y
                    gyr_data[device_index, 2] = gyr_x[-1]   # X -> Z
            
            # Extract MAG data (matching C# XYZ -> YZX transformation)
            if "MAG_X" in data and "MAG_Y" in data and "MAG_Z" in data:
                mag_x = data["MAG_X"]
                mag_y = data["MAG_Y"]
                mag_z = data["MAG_Z"]
                if mag_x and mag_y and mag_z:
                    # C# code shows XYZ -> YZX transformation
                    mag_data[device_index, 0] = mag_y[-1]  # Y -> X
                    mag_data[device_index, 1] = mag_z[-1]  # Z -> Y
                    mag_data[device_index, 2] = mag_x[-1]  # X -> Z
            
            # Extract QUAT data (matching C# WXYZ format)
            if "QUAT_W" in data and "QUAT_X" in data and "QUAT_Y" in data and "QUAT_Z" in data:
                quat_w = data["QUAT_W"]
                quat_x = data["QUAT_X"]
                quat_y = data["QUAT_Y"]
                quat_z = data["QUAT_Z"]
                if quat_w and quat_x and quat_y and quat_z:
                    # Store as WXYZ format for quaternion processing
                    raw_quat = (quat_w[-1], quat_x[-1], quat_y[-1], quat_z[-1])
                    quat_data[device_index, :] = raw_quat
        
        # Apply quaternion reset/orientation if needed (matches C# ResetOrientation)
        if self._reset_zero or self._zero_yaw:
            for i in range(device_count):
                if not np.any(np.isnan(quat_data[i, :])):
                    # Convert to tuple for quaternion processing
                    q_tuple = tuple(quat_data[i, :])
                    corrected_quats = self._reset_orientation([q_tuple])
                    if corrected_quats[0] is not None:
                        quat_data[i, :] = corrected_quats[0]
        
        return IMUSnapshot(
            quat_data=quat_data,
            acc_data=acc_data,
            gyr_data=gyr_data,
            mag_data=mag_data,
            sensor_timestamp=sensor_timestamp,
        )