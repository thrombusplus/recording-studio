import asyncio
import websockets
import threading
import numpy as np
import logging

logger = logging.getLogger(__name__)

class WebSocketServer:
    def __init__(self, host="localhost", port=8185):
        self.host = host
        self.port = port
        self.server = None
        self.clients = set()
        self.loop = None
        self.thread = None

    async def _register(self, websocket):
        """Registers a new client and handles their connection state."""
        self.clients.add(websocket)
        logger.info(f"WebSocket client connected: {websocket.remote_address}")
        try:
            await websocket.wait_closed()
        finally:
            self.clients.remove(websocket)
            logger.info(f"WebSocket client disconnected: {websocket.remote_address}")

    async def _broadcast(self, message):
        """Sends a message to all connected clients."""
        if self.clients:
            await asyncio.wait([client.send(message) for client in self.clients])

    def broadcast(self, message):
        """A thread-safe method to call the async broadcast function."""
        if self.loop and self.loop.is_running():
            asyncio.run_coroutine_threadsafe(self._broadcast(message), self.loop)

    def _run_server(self):
        """Runs the asyncio event loop and the server in the thread."""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        async def start_server_and_run():
            """A coroutine to start the server and log its running state."""
            # Start the WebSocket server
            self.server = await websockets.serve(
                self._register, self.host, self.port
            )
            # Log that the server is successfully running
            logger.info(f"WebSocket server is listening on {self.server.sockets[0].getsockname()}")

        # Schedule the server startup coroutine to run on the event loop
        self.loop.create_task(start_server_and_run())

        try:
            # Run the event loop until stop() is called. This is a blocking call.
            self.loop.run_forever()
        finally:
            # This cleanup code runs after loop.stop() is called from another thread
            if hasattr(self, 'server') and self.server:
                self.server.close()
                # Wait for the server to finish closing
                self.loop.run_until_complete(self.server.wait_closed())
            
            # Final loop cleanup
            self.loop.run_until_complete(self.loop.shutdown_asyncgens())
            self.loop.close()
            logger.info("WebSocket server loop has been closed.")

    def start_server(self):
        """Starts the server in a new daemon thread."""
        if self.thread and self.thread.is_alive():
            logger.warning("WebSocket server is already running.")
            return False
        try:
            self.thread = threading.Thread(target=self._run_server, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            logger.error(f"Failed to start WebSocket server thread: {e}")
            return False

    def stop_server(self):
        """Stops the server and joins the thread."""
        if self.loop and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.thread:
            self.thread.join(timeout=2)
            if self.thread.is_alive():
                 logger.warning("WebSocket server thread did not terminate cleanly.")
        self.thread = None
        self.server = None
        self.loop = None
        logger.info("WebSocket server stopped.")

    def is_running(self):
        """Checks if the server thread is active."""
        return self.thread is not None and self.thread.is_alive()

    def get_client_count(self):
        """Returns the number of currently connected clients."""
        return len(self.clients)

    def send_imu_data(self, timestamp, imu_config, quat_data, acc_data, ang_data, mag_data=None):
        """
        Formats and sends IMU data to connected clients in a comma-separated text format..
        """
        if not self.is_running() or not self.clients:
            return
            
        try:
            # Timestamp in milliseconds
            message_parts = [str(int(timestamp * 1000))]
            
            # Iterate through the 6 possible body parts
            for i in range(6):
                device_idx = imu_config[i] if i < len(imu_config) else -1
                
                # Default values for unassigned sensors or invalid data
                q = [1.0, 0.0, 0.0, 0.0]
                a = [0.0, 0.0, 0.0]
                g = [0.0, 0.0, 0.0]
                
                if device_idx != -1:
                    # Safely access data for the configured device index
                    if quat_data is not None and device_idx < quat_data.shape[0] and not np.any(np.isnan(quat_data[device_idx])):
                        q = quat_data[device_idx]
                    if acc_data is not None and device_idx < acc_data.shape[0] and not np.any(np.isnan(acc_data[device_idx])):
                        a = acc_data[device_idx]
                    if ang_data is not None and device_idx < ang_data.shape[0] and not np.any(np.isnan(ang_data[device_idx])):
                        g = ang_data[device_idx]

                # Flatten and format all 10 values (quat, acc, gyr) - magnetometer excluded
                sensor_data = np.concatenate([q, a, g])
                message_parts.extend([f"{val:.4f}" for val in sensor_data])

            message = ",".join(message_parts)
            self.broadcast(message)

        except Exception as e:
            logger.error(f"Error formatting or sending WebSocket IMU data: {e}")
