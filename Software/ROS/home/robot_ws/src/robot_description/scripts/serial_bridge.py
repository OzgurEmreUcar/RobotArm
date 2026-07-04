#!/usr/bin/env python3
"""
Serial communication bridge to the Arduino-based low-level motor controller.

Responsibilities:
  - Opening / retrying the serial connection.
  - Reading and parsing status lines from the Arduino
    (ACK, DON, READY, DROP, DBG).
  - Flow-controlled sending of motion segments, respecting the Arduino's
    finite onboard command buffer (ARDUINO_BUF_MAX slots).

This module has no ROS dependency on purpose: it only knows about serial
I/O and command framing, so it can be tested or reused independently of
the ROS2 node.
"""

import queue
import threading
import time

import serial

from constants import ARDUINO_BUF_MAX


class SerialBridge:
    """Owns the serial link and command flow-control with the Arduino firmware."""

    def __init__(self, logger, cancel_flag: threading.Event):
        """
        Args:
            logger: A logger exposing .info()/.warn()/.error() (e.g. ROS node logger).
            cancel_flag: Shared event; when set, sending/queueing is suspended.
        """
        self._logger = logger
        self._cancel_flag = cancel_flag

        self.ser = None
        self._ser_lock = threading.Lock()

        self._buf_lock = threading.Lock()
        self._buf_free = ARDUINO_BUF_MAX

        # Segments waiting to be sent, and positions waiting to be acknowledged.
        self._send_queue = queue.Queue()
        self._ack_queue = queue.Queue()

        # Callback: fn(pos_rad: list[float] | None) -> None
        # Invoked whenever the Arduino reports a completed motion segment ("DON").
        self.on_segment_done = None

        # Callback: fn() -> None
        # Invoked whenever the Arduino reports a dropped command ("DROP").
        self.on_buffer_drop = None

    @property
    def is_connected(self) -> bool:
        return self.ser is not None

    # ------------------------------------------------------------------ #
    # Connection management
    # ------------------------------------------------------------------ #
    def connect_async(self, port: str, baud: int, retries: int = 5) -> None:
        """Kick off a background thread that attempts to open the serial port."""
        threading.Thread(
            target=self._connect, args=(port, baud, retries), daemon=True
        ).start()

    def _connect(self, port: str, baud: int, retries: int) -> None:
        for attempt in range(1, retries + 1):
            try:
                with self._ser_lock:
                    self.ser = serial.Serial(port, baud, timeout=0.05)
                    time.sleep(2.0)
                    self.ser.reset_input_buffer()
                self._logger.info(f'Arduino connected successfully -> {port}')
                return
            except Exception as e:
                self._logger.warn(f'Connection attempt {attempt}/{retries} failed: {e}')
                time.sleep(1.0)
        self._logger.error('Failed to reach the Arduino port!')

    def close(self) -> None:
        """Close the serial port, if open."""
        if self.ser:
            self.ser.close()

    # ------------------------------------------------------------------ #
    # Thread lifecycle
    # ------------------------------------------------------------------ #
    def start(self) -> None:
        """Start the background reader and sender threads."""
        threading.Thread(target=self._reader_loop, daemon=True).start()
        threading.Thread(target=self._sender_loop, daemon=True).start()

    # ------------------------------------------------------------------ #
    # Low-level I/O
    # ------------------------------------------------------------------ #
    def send_cmd(self, cmd_str: str) -> bool:
        """Write a raw command string to the serial port."""
        with self._ser_lock:
            if not self.ser:
                return False
            try:
                self.ser.write(cmd_str.encode('utf-8'))
                return True
            except serial.SerialException:
                self.ser = None
                return False

    def send_control(self, cmd: str, retries: int = 3) -> bool:
        """Send a control command (e.g. FREEZE/ENGAGE/ABORT), retrying on failure."""
        for _ in range(retries):
            if self.send_cmd(f'<{cmd}>\n'):
                return True
            time.sleep(0.1)
        return False

    # ------------------------------------------------------------------ #
    # Motion segment queueing
    # ------------------------------------------------------------------ #
    def enqueue_segment(self, deltas, dur_ms, pos_rad) -> None:
        """Queue a motion segment (per-joint step deltas + duration) for sending."""
        self._send_queue.put((deltas, dur_ms, pos_rad))

    def reset_buffer_state(self) -> None:
        """Reset the flow-control counter to the Arduino's full buffer capacity."""
        with self._buf_lock:
            self._buf_free = ARDUINO_BUF_MAX

    def clear_send_queue(self) -> None:
        """Drop any pending outgoing motion segments."""
        with self._send_queue.mutex:
            self._send_queue.queue.clear()

    def clear_queues(self) -> None:
        """Drop pending outgoing segments AND pending position acknowledgements."""
        self.clear_send_queue()
        with self._ack_queue.mutex:
            self._ack_queue.queue.clear()

    # ------------------------------------------------------------------ #
    # Background loops
    # ------------------------------------------------------------------ #
    def _reader_loop(self) -> None:
        """Continuously read and dispatch status lines coming from the Arduino."""
        while True:
            try:
                in_waiting = 0
                with self._ser_lock:
                    if self.ser:
                        in_waiting = self.ser.in_waiting

                if in_waiting == 0:
                    time.sleep(0.001)
                    continue

                line = ""
                with self._ser_lock:
                    if self.ser:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                if not line:
                    continue

                self._handle_line(line)

            except Exception as e:
                self._logger.error(f'Serial reader error: {e}')
                time.sleep(0.05)

    def _handle_line(self, line: str) -> None:
        """Parse and react to a single status line received from the Arduino."""
        if line == 'ACK':
            with self._buf_lock:
                self._buf_free = min(self._buf_free + 1, ARDUINO_BUF_MAX)

        elif line == 'DON':
            pos_rad = None
            try:
                pos_rad = list(self._ack_queue.get_nowait())
            except queue.Empty:
                pass
            if self.on_segment_done:
                self.on_segment_done(pos_rad)

        elif line == 'READY':
            self.reset_buffer_state()
            with self._ack_queue.mutex:
                self._ack_queue.queue.clear()

        elif line == 'DROP':
            self._logger.warn('!!! ARDUINO BUFFER DROP !!!')
            if self.on_buffer_drop:
                self.on_buffer_drop()

        elif line.startswith('DBG,'):
            # Skip heavy debug lines from the Arduino to save CPU time.
            pass

    def _sender_loop(self) -> None:
        """Send queued motion segments, respecting the Arduino's buffer capacity."""
        while True:
            try:
                deltas, dur_ms, pos_rad = self._send_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if self._cancel_flag.is_set():
                continue

            # Wait for a free slot in the Arduino's command buffer.
            while True:
                if self._cancel_flag.is_set():
                    break
                with self._buf_lock:
                    if self._buf_free > 0:
                        self._buf_free -= 1
                        break
                time.sleep(0.001)

            if self._cancel_flag.is_set():
                continue

            packet = f"<{','.join(map(str, deltas))},{dur_ms}>\n"
            self._ack_queue.put(pos_rad)
            self.send_cmd(packet)
