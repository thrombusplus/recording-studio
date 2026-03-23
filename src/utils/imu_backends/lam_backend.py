import socket
import struct
import threading
import json
import time
from typing import List, Dict, Callable
from zeroconf import Zeroconf, ServiceBrowser

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
        if len(data) >= 4:
            sync = struct.unpack('<I', data[0:4])[0]
            if sync == 0xDEADBEEF:
                self._parse_binary_payload(data)
                return

        # If not binary 0xDEADBEEF, treat as JSON string
        try:
            msg = data.decode('utf-8')
            self.status_json = msg
            self.status = json.loads(msg)
            self.last_status_update = time.time()
            if self.on_status_updated:
                self.on_status_updated(msg)
        except json.JSONDecodeError:
            pass

    def _parse_binary_payload(self, data: bytes):
        if len(data) < 48:
            return

        # Parse Main Header
        total_packs, pack_no, samples, samples_temp = struct.unpack('<BBBB', data[4:8])
        timestamp_ms = struct.unpack('<Q', data[8:16])[0]
        config_main = data[16:28]
        trans_int_ms = struct.unpack('<H', data[28:30])[0]
        fs, fs_temp = struct.unpack('<BB', data[30:32])
        saga_map = list(data[32:42])
        
        main_head = {
            "Total_packs": total_packs, "Pack_no": pack_no, "Samples": samples,
            "Samples_temp": samples_temp, "Timestamp_ms": timestamp_ms,
            "Trans_int_ms": trans_int_ms, "Fs": fs, "Fs_temp": fs_temp
        }

        if pack_no == 1:
            self.tmp_saga_map = saga_map.copy()

        cursor = 48
        lam_data_array = [None] * 10

        for i in range(10):
            if hasattr(self, 'tmp_saga_map') and self.tmp_saga_map[i] == 1:
                if cursor + 24 > len(data):
                    break
                    
                saga_sync = struct.unpack('<I', data[cursor:cursor+4])[0]
                if saga_sync != 0xDEADBEEF:
                    return
                
                saga_length = struct.unpack('<H', data[cursor+4:cursor+6])[0]
                saga_data = data[cursor:cursor+saga_length]
                lam_data_array[i] = self._parse_lam_data(saga_data, samples, samples_temp)
                cursor += saga_length
                self.tmp_saga_map[i] = 0

                if sum(self.tmp_saga_map) == 0:
                    payload = {"MainHead": main_head, "Sagas": lam_data_array}
                    if self.on_data_received:
                        self.on_data_received(payload)

    def _parse_lam_data(self, saga_data: bytes, smp: int, smp_t: int) -> Dict:
        length = struct.unpack('<H', saga_data[4:6])[0]
        saga_no = struct.unpack('<H', saga_data[6:8])[0]
        config = saga_data[8:20]
        
        body_length = length - 24
        float_count = body_length // 4
        body = struct.unpack(f'<{float_count}f', saga_data[24:24+body_length])
        
        parsed_saga = {"SAGA_no": saga_no, "Data": {}}
        cnt = 0
        
        if config[0] == 1:
            parsed_saga["Data"]["ACC_X"] = body[cnt:cnt+smp]; cnt += smp
            parsed_saga["Data"]["ACC_Y"] = body[cnt:cnt+smp]; cnt += smp
            parsed_saga["Data"]["ACC_Z"] = body[cnt:cnt+smp]; cnt += smp
        if config[1] == 1:
            parsed_saga["Data"]["GYR_X"] = body[cnt:cnt+smp]; cnt += smp
            parsed_saga["Data"]["GYR_Y"] = body[cnt:cnt+smp]; cnt += smp
            parsed_saga["Data"]["GYR_Z"] = body[cnt:cnt+smp]; cnt += smp
        if config[2] == 1:
            parsed_saga["Data"]["MAG_X"] = body[cnt:cnt+smp]; cnt += smp
            parsed_saga["Data"]["MAG_Y"] = body[cnt:cnt+smp]; cnt += smp
            parsed_saga["Data"]["MAG_Z"] = body[cnt:cnt+smp]; cnt += smp
        if config[3] == 1:
            parsed_saga["Data"]["QUAT_W"] = body[cnt:cnt+smp]; cnt += smp
            parsed_saga["Data"]["QUAT_X"] = body[cnt:cnt+smp]; cnt += smp
            parsed_saga["Data"]["QUAT_Y"] = body[cnt:cnt+smp]; cnt += smp
            parsed_saga["Data"]["QUAT_Z"] = body[cnt:cnt+smp]; cnt += smp
            
        # Additional parsing mapped corresponding to the C# implementation (Temp, Ppg, Test) must be appended here if required.
        
        return parsed_saga