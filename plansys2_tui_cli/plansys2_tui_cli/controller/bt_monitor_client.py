# Copyright 2026 Juan Carlos Manzanares Serrano
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Client of the BehaviorTree.CPP monitoring protocol (groot2_protocol.h, MIT license)."""

from __future__ import annotations

import random
import struct
from typing import Dict, Optional, Tuple

PROTOCOL_ID = 2
REQUEST_FULLTREE = b'T'
REQUEST_STATUS = b'S'

REQUEST_HEADER_SIZE = 6
REPLY_HEADER_SIZE = REQUEST_HEADER_SIZE + 16


def parse_address(text: str, default_host: str = 'localhost') -> Tuple[str, int]:
    """Return (host, port) from '1820', 'host:1820' or 'tcp://host:1820'."""
    text = text.strip()
    if text.startswith('tcp://'):
        text = text[len('tcp://'):]
    host, sep, port = text.rpartition(':')
    if not sep:
        host = default_host
    if not port.isdigit() or not 0 < int(port) < 65536:
        raise ValueError(f'no valid port in "{text}"')
    return host or default_host, int(port)


class BTMonitorError(Exception):
    """Failed request to a tree publisher."""


def encode_request(request_type: bytes, unique_id: Optional[int] = None) -> bytes:
    """Encode a request header."""
    if unique_id is None:
        unique_id = random.getrandbits(32)
    return struct.pack('<BcI', PROTOCOL_ID, request_type, unique_id)


def decode_reply_header(header: bytes) -> Tuple[int, bytes, int, bytes]:
    """Return (protocol, type, unique id, tree id) from a reply header."""
    if len(header) != REPLY_HEADER_SIZE:
        raise BTMonitorError(f'reply header of {len(header)} bytes, not {REPLY_HEADER_SIZE}')
    protocol, request_type, unique_id = struct.unpack_from('<BcI', header)
    return protocol, request_type, unique_id, bytes(header[REQUEST_HEADER_SIZE:])


def decode_status(buffer: bytes) -> Dict[int, int]:
    """Return {uid: status} from a status buffer."""
    if len(buffer) % 3:
        raise BTMonitorError(f'status buffer of {len(buffer)} bytes, not a multiple of 3')
    return dict(struct.iter_unpack('<HB', buffer))


def check_reply(frames, request: bytes) -> Tuple[bytes, bytes]:
    """Return (tree id, payload) from a reply."""
    if len(frames) >= 2 and frames[0] == b'error':
        raise BTMonitorError(frames[1].decode(errors='replace'))
    if len(frames) < 2:
        raise BTMonitorError(f'reply of {len(frames)} frames, not 2')
    protocol, request_type, unique_id, tree_id = decode_reply_header(frames[0])
    _, sent_type, sent_id = struct.unpack('<BcI', request)
    if protocol != PROTOCOL_ID:
        raise BTMonitorError(f'protocol {protocol}, this client speaks {PROTOCOL_ID}')
    if request_type != sent_type or unique_id != sent_id:
        raise BTMonitorError('reply to another request')
    return tree_id, frames[1]


class BTMonitorClient:
    """ZMQ client of a tree publisher."""

    def __init__(self, host: str = 'localhost', port: int = 1800, timeout: float = 0.5):
        import zmq

        self._zmq = zmq
        self.address = f'tcp://{host}:{port}'
        self._timeout_ms = int(timeout * 1000)
        self._context = zmq.Context.instance()
        self._socket = None

    def _connect(self):
        socket = self._context.socket(self._zmq.REQ)
        socket.setsockopt(self._zmq.LINGER, 0)
        socket.setsockopt(self._zmq.RCVTIMEO, self._timeout_ms)
        socket.setsockopt(self._zmq.SNDTIMEO, self._timeout_ms)
        socket.connect(self.address)
        self._socket = socket

    def _request(self, request_type: bytes) -> Tuple[bytes, bytes]:
        if self._socket is None:
            self._connect()
        request = encode_request(request_type)
        try:
            self._socket.send(request)
            frames = self._socket.recv_multipart()
        except self._zmq.ZMQError as error:
            self.close()
            raise BTMonitorError(f'no answer from {self.address}') from error
        return check_reply(frames, request)

    def fetch_tree(self) -> Tuple[bytes, str]:
        """Return (tree id, tree XML)."""
        tree_id, payload = self._request(REQUEST_FULLTREE)
        return tree_id, payload.decode(errors='replace')

    def fetch_status(self) -> Tuple[bytes, Dict[int, int]]:
        """Return (tree id, {uid: status})."""
        tree_id, payload = self._request(REQUEST_STATUS)
        return tree_id, decode_status(payload)

    def close(self) -> None:
        """Close the socket."""
        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None
