"""Blocking UDP receiver with timeout, matching the C++ receiver behavior."""

from __future__ import annotations

import socket
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class UdpDatagram:
    data: bytes
    address: Tuple[str, int]


class UdpReceiver:
    def __init__(self, host: str = "0.0.0.0", port: int = 8080, buffer_size: int = 2048) -> None:
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        self._socket: Optional[socket.socket] = None

    def start(self) -> None:
        if self._socket is not None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        self._socket = sock

    def stop(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def receive_once(self, timeout_ms: int = 100) -> Optional[UdpDatagram]:
        if self._socket is None:
            raise RuntimeError("UdpReceiver.start() must be called before receive_once()")
        self._socket.settimeout(max(timeout_ms, 0) / 1000.0)
        try:
            data, address = self._socket.recvfrom(self.buffer_size)
        except socket.timeout:
            return None
        return UdpDatagram(data=data, address=address)

    def receive_latest(self, timeout_ms: int = 0, max_packets: int = 64) -> tuple[Optional[UdpDatagram], int]:
        """Return the newest datagram available and drop older queued packets.

        VR senders often run faster than the IK/visualization loop. Reading only
        one packet per frame lets the OS socket queue build up and makes the
        robot follow stale hand poses. This method drains the queue up to
        ``max_packets`` and returns the last packet read.
        """

        if self._socket is None:
            raise RuntimeError("UdpReceiver.start() must be called before receive_latest()")
        latest = self.receive_once(timeout_ms)
        if latest is None:
            return None, 0

        count = 1
        old_timeout = self._socket.gettimeout()
        self._socket.settimeout(0.0)
        try:
            while count < max(1, max_packets):
                try:
                    data, address = self._socket.recvfrom(self.buffer_size)
                except (BlockingIOError, socket.timeout):
                    break
                latest = UdpDatagram(data=data, address=address)
                count += 1
        finally:
            self._socket.settimeout(old_timeout)
        return latest, count

    def __enter__(self) -> "UdpReceiver":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()
