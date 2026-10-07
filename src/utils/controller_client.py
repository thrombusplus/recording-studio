import asyncio
import json
import logging
import queue
import threading
import time

import websockets

logger = logging.getLogger(__name__)

# Address of the controller server. If the main PC changes, update this
DEFAULT_CONTROLLER_URL = "ws://127.0.0.1:8000/ws"


class ControllerClient:
    RECONNECT_DELAY = 3  # seconds between reconnection attempts

    def __init__(self, url=DEFAULT_CONTROLLER_URL):
        self.url = url
        self.events = queue.Queue()
        self._thread = None
        self._loop = None
        self._task = None
        self._stop_requested = threading.Event()

    # public API (called from the GUI thread)

    def start(self):
        """Start the background connection thread. Returns False if already running."""
        if self.is_running():
            return False
        self._stop_requested.clear()
        self._thread = threading.Thread(
            target=self._thread_main, name="ControllerClient", daemon=True
        )
        self._thread.start()
        return True

    def stop(self):
        """Close the connection and stop reconnecting."""
        self._stop_requested.set()
        loop, task = self._loop, self._task
        if loop is not None and task is not None and not loop.is_closed():
            try:
                loop.call_soon_threadsafe(task.cancel)
            except RuntimeError:
                pass  # loop already closed
        if self._thread is not None:
            self._thread.join(timeout=3)
            if self._thread.is_alive():
                logger.warning("Controller client thread did not terminate cleanly.")
        self._thread = None

    def is_running(self):
        return self._thread is not None and self._thread.is_alive()

    # background thread

    def _thread_main(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._task = self._loop.create_task(self._listen_forever())
        try:
            self._loop.run_until_complete(self._task)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            self.events.put(("log", f"Controller client crashed: {e}"))
        finally:
            self._loop.close()
            self._loop = None
            self._task = None
            self.events.put(("status", "disconnected", "Stopped"))

    async def _listen_forever(self):
        """Connect, listen for commands, and reconnect automatically if the server drops."""
        while not self._stop_requested.is_set():
            self.events.put(("status", "connecting", self.url))
            try:
                async with websockets.connect(self.url, open_timeout=5) as ws:
                    self.events.put(("status", "connected", self.url))
                    async for message in ws:
                        # Timestamp immediately, before any other processing,
                        # so the log shows when the command actually arrived.
                        self._handle_message(message, time.time())
            except asyncio.CancelledError:
                raise
            except Exception as e:
                self.events.put(("status", "disconnected", str(e) or type(e).__name__))

            if not self._stop_requested.is_set():
                await asyncio.sleep(self.RECONNECT_DELAY)

    def _handle_message(self, message, receive_time):
        try:
            data = json.loads(message)
            command = str(data.get("command", "")).strip().lower()
        except Exception as e:
            self.events.put(("log", f"Invalid controller message ({e}): {message!r}"))
            return

        if command in ("start", "stop"):
            self.events.put(("command", command, receive_time))
        else:
            self.events.put(("log", f"Controller sent unknown command: {message!r}"))