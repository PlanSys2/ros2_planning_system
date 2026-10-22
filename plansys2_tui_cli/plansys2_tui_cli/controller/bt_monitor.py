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

"""Background monitor of a behavior tree publisher."""

from __future__ import annotations

from dataclasses import dataclass, field
import threading
from typing import Callable, Dict, Optional

from .bt_monitor_client import BTMonitorClient, BTMonitorError
from .bt_tree import BTNode, parse_tree


@dataclass
class BTSnapshot:
    """State of the monitored tree."""

    address: str
    connected: bool = False
    error: str = ''
    tree_id: bytes = b''
    root: Optional[BTNode] = None
    new_tree: bool = False
    statuses: Dict[int, int] = field(default_factory=dict)


class BTMonitor:
    """Poll a tree publisher in a thread."""

    def __init__(self, host: str, port: int, callback: Callable[[BTSnapshot], None],
                 period: float = 0.2, client_factory: Callable = BTMonitorClient):
        self._host = host
        self._port = port
        self._callback = callback
        self._period = period
        self._client_factory = client_factory
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._tree_id = b''
        self._root: Optional[BTNode] = None
        self._statuses: Dict[int, int] = {}

    @property
    def address(self) -> str:
        """Return the address of the publisher."""
        return f'tcp://{self._host}:{self._port}'

    def start(self) -> None:
        """Start polling."""
        self._thread.start()

    def stop(self) -> None:
        """Stop polling."""
        self._stop.set()
        self._thread.join(timeout=1.0)

    def poll(self, client) -> BTSnapshot:
        """Poll the publisher once."""
        snapshot = BTSnapshot(self.address)
        try:
            status_id, statuses = client.fetch_status()
            if status_id != self._tree_id or self._root is None:
                tree_id, xml = client.fetch_tree()
                root = parse_tree(xml)
                self._tree_id, self._root = tree_id, root
                snapshot.new_tree = True
                status_id, statuses = client.fetch_status()
            snapshot.connected = status_id == self._tree_id
            if snapshot.connected:
                self._statuses = statuses
        except BTMonitorError as error:
            snapshot.error = str(error)
        except Exception as error:
            snapshot.error = f'{type(error).__name__}: {error}'
        snapshot.tree_id, snapshot.root = self._tree_id, self._root
        snapshot.statuses = self._statuses
        return snapshot

    def _run(self) -> None:
        try:
            client = self._client_factory(self._host, self._port)
        except ImportError:
            self._callback(BTSnapshot(
                self.address, error='pyzmq is not installed (python3-zmq): no Behavior tree'))
            return
        while not self._stop.is_set():
            self._callback(self.poll(client))
            self._stop.wait(self._period)
        client.close()
