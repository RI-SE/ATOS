"""This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
"""

"""ROS side of the map view: collects the test origin, scenario trajectories and live
object poses, and converts them to WGS84 for the browser.

Data sources (all in the ATOS local frame, see geo.py):
- /atos/get_test_origin       GeoPose of the local frame origin (EsminiAdapter)
- /atos/get_object_ids        object ids and names (OpenScenarioGateway)
- /atos/object_<id>/path      nav_msgs/Path, transient local (EsminiAdapter/ObjectControl)
- /atos/object_monitor        atos_interfaces/Monitor for all objects (ObjectControl)
- /atos/connected_object_ids  ids of connected objects, every second (ObjectControl)
- /atos/state_change          ObjectControl state transitions
"""

import math
import threading
import time

import rclpy
from atos_interfaces.msg import Monitor, ObjectIdArray, StateChange
from atos_interfaces.srv import GetObjectIds, GetTestOrigin
from nav_msgs.msg import Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from .geo import LocalFrame, decimate, yaw_from_quaternion

# ObjectControl states as returned by GetObjectControlState / StateChange (OBCState_t).
STATE_IDLE = 1
STATE_INITIALIZED = 2

ORIGIN_TIMEOUT_S = 5.0
IDS_TIMEOUT_S = 5.0
CONNECTED_IDS_MAX_AGE_S = 3.0
MONITOR_STALE_S = 2.0
MONITOR_FORGET_S = 10.0

PATH_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
LATEST_QOS = QoSProfile(depth=1)


class MapDataSource:
    """Thread-safe store of map data, fed by ROS callbacks and read by the UI."""

    def __init__(self, node: Node, get_control_state=lambda: None) -> None:
        self.node = node
        self._get_control_state = get_control_state
        self._lock = threading.Lock()

        self.frame: LocalFrame | None = None
        self.status = "Waiting for a scenario to be initialized..."
        self.object_names: dict[int, str] = {}
        # Raw trajectories in the local frame, per object id: (xs, ys).
        self._raw_paths: dict[int, tuple[list[float], list[float]]] = {}
        self._geo_paths: dict[int, list[list[float]]] = {}
        self.traj_version = 0
        self._monitors: dict[int, dict] = {}
        self._connected_ids: set[int] = set()
        self._connected_ids_stamp = 0.0

        self._loading = False
        self._needs_load = True
        self._path_subs: dict[int, object] = {}

        # A separate callback group, so the control panel's blocking state polling
        # cannot stall map updates (and the callbacks below run one at a time).
        self._cb_group = cb = MutuallyExclusiveCallbackGroup()
        self._ids_client = node.create_client(
            GetObjectIds, "/atos/get_object_ids", callback_group=cb
        )
        self._origin_client = node.create_client(
            GetTestOrigin, "/atos/get_test_origin", callback_group=cb
        )
        node.create_subscription(
            StateChange, "/atos/state_change", self._on_state_change,
            QoSProfile(depth=10), callback_group=cb,
        )
        node.create_subscription(
            Monitor, "/atos/object_monitor", self._on_monitor, LATEST_QOS, callback_group=cb
        )
        node.create_subscription(
            ObjectIdArray, "/atos/connected_object_ids", self._on_connected_ids,
            QoSProfile(depth=10), callback_group=cb,
        )
        node.create_timer(1.0, self._maintain, callback_group=cb)

    # ------------------------------------------------------------------ loading

    def request_reload(self) -> None:
        with self._lock:
            self._needs_load = True

    def _maintain(self) -> None:
        """Start a (re)load once ObjectControl has initialized a scenario.

        get_test_origin busy-waits inside EsminiAdapter until an origin is set, which
        only happens while a scenario is loaded, so it is only called from INITIALIZED on.
        """
        state = self._get_control_state()
        with self._lock:
            if self._loading or not self._needs_load:
                return
            if state is None or state < STATE_INITIALIZED:
                return
            self._loading = True
            self._needs_load = False
        self._load_object_ids()

    def _on_state_change(self, msg: StateChange) -> None:
        if msg.prev_state == STATE_IDLE and msg.current_state == STATE_INITIALIZED:
            self.request_reload()

    def _finish_load(self, status: str) -> None:
        with self._lock:
            self._loading = False
            self.status = status
        self.node.get_logger().info(f"Map view: {status}")

    def _call_with_timeout(self, client, request, timeout_s, on_result, what) -> None:
        if not client.service_is_ready():
            self._finish_load(f"{what} service is not available")
            with self._lock:
                self._needs_load = True  # try again on the next tick
            return
        future = client.call_async(request)
        timer = None

        def stop_timer():
            if timer is not None and not timer.is_canceled():
                timer.cancel()
                self.node.destroy_timer(timer)

        def on_timeout():
            stop_timer()
            if not future.done():
                client.remove_pending_request(future)
                self._finish_load(f"Timed out waiting for {what}")

        def on_done(fut):
            stop_timer()
            if fut.cancelled():
                return
            try:
                result = fut.result()
            except Exception as e:  # noqa: BLE001 - report any service failure
                self._finish_load(f"{what} failed: {e!r}")
                return
            on_result(result)

        timer = self.node.create_timer(timeout_s, on_timeout, callback_group=self._cb_group)
        future.add_done_callback(on_done)

    def _load_object_ids(self) -> None:
        self._call_with_timeout(
            self._ids_client, GetObjectIds.Request(), IDS_TIMEOUT_S, self._on_object_ids, "object ids"
        )

    def _on_object_ids(self, result) -> None:
        names = {}
        for index, object_id in enumerate(result.ids):
            name = result.names[index] if index < len(result.names) else ""
            names[int(object_id)] = name or f"Object {object_id}"
        with self._lock:
            self.object_names = names
        self._call_with_timeout(
            self._origin_client, GetTestOrigin.Request(), ORIGIN_TIMEOUT_S, self._on_origin, "test origin"
        )

    def _on_origin(self, result) -> None:
        position = result.origin.position
        lat, lon = position.latitude, position.longitude
        if not result.success or not (math.isfinite(lat) and math.isfinite(lon)):
            self._finish_load("Scenario has no geo-reference, cannot place overlays")
            return
        try:
            frame = LocalFrame(lat, lon)
        except ValueError as e:
            self._finish_load(str(e))
            return
        with self._lock:
            frame_changed = self.frame is None or (
                (self.frame.origin_lat, self.frame.origin_lon) != (lat, lon)
            )
            self.frame = frame
            ids = set(self.object_names)
            for object_id in list(self._raw_paths):
                if object_id not in ids:
                    del self._raw_paths[object_id]
            self._geo_paths = {
                object_id: self._convert_path(frame, *raw)
                for object_id, raw in self._raw_paths.items()
            }
            if frame_changed:
                self._monitors.clear()
            self.traj_version += 1
        self._update_path_subscriptions(ids)
        self._finish_load(f"Origin {lat:.7f}, {lon:.7f}")

    def _update_path_subscriptions(self, ids: set[int]) -> None:
        for object_id in list(self._path_subs):
            if object_id not in ids:
                self.node.destroy_subscription(self._path_subs.pop(object_id))
        for object_id in ids:
            if object_id not in self._path_subs:
                self._path_subs[object_id] = self.node.create_subscription(
                    Path,
                    f"/atos/object_{object_id}/path",
                    lambda msg, object_id=object_id: self._on_path(object_id, msg),
                    PATH_QOS,
                    callback_group=self._cb_group,
                )

    # ---------------------------------------------------------------- callbacks

    @staticmethod
    def _convert_path(frame: LocalFrame, xs, ys) -> list[list[float]]:
        if not xs:
            return []
        keep = decimate(xs, ys)
        lons, lats = frame.to_lonlat([xs[i] for i in keep], [ys[i] for i in keep])
        return [[round(float(lat), 9), round(float(lon), 9)] for lat, lon in zip(lats, lons)]

    def _on_path(self, object_id: int, msg: Path) -> None:
        xs = [pose.pose.position.x for pose in msg.poses]
        ys = [pose.pose.position.y for pose in msg.poses]
        with self._lock:
            frame = self.frame
        geo = self._convert_path(frame, xs, ys) if frame is not None else None
        with self._lock:
            self._raw_paths[object_id] = (xs, ys)
            if geo is not None and frame is self.frame:
                self._geo_paths[object_id] = geo
                self.traj_version += 1

    def _on_monitor(self, msg: Monitor) -> None:
        pose = msg.pose.pose
        q = pose.orientation
        entry = {
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw": yaw_from_quaternion(q.x, q.y, q.z, q.w),
            "speed": math.hypot(msg.velocity.twist.linear.x, msg.velocity.twist.linear.y),
            "state": int(msg.object_state.state),
            "received": time.monotonic(),
        }
        with self._lock:
            self._monitors[int(msg.atos_header.object_id)] = entry

    def _on_connected_ids(self, msg: ObjectIdArray) -> None:
        with self._lock:
            self._connected_ids = {int(i) for i in msg.ids}
            self._connected_ids_stamp = time.monotonic()

    # ------------------------------------------------------------------ for the UI

    def snapshot(self, known_traj_version: int | None) -> dict:
        """Data for one browser update.

        Trajectories are only included when they changed since known_traj_version.
        Monitor poses are converted here, at the UI rate, not at the MONR rate.
        """
        now = time.monotonic()
        with self._lock:
            frame = self.frame
            names = dict(self.object_names)
            connected_fresh = now - self._connected_ids_stamp < CONNECTED_IDS_MAX_AGE_S
            connected = set(self._connected_ids) if connected_fresh else set()
            for object_id in [i for i, m in self._monitors.items() if now - m["received"] > MONITOR_FORGET_S]:
                del self._monitors[object_id]
            monitors = {i: dict(m) for i, m in self._monitors.items()}
            payload = {
                "status": self.status,
                "origin": [frame.origin_lat, frame.origin_lon] if frame else None,
                "traj_version": self.traj_version,
            }
            if known_traj_version != self.traj_version:
                payload["trajectories"] = [
                    {"id": object_id, "name": names.get(object_id, f"Object {object_id}"), "latlngs": latlngs}
                    for object_id, latlngs in sorted(self._geo_paths.items())
                    if latlngs
                ]

        objects = []
        if frame is not None and monitors:
            # Show connected objects; if the connected list is not being published,
            # fall back to any object that reports monitor data.
            ids = sorted(i for i in monitors if i in connected or not connected_fresh)
            if ids:
                lats, lons, bearings = frame.poses_to_geo(
                    [monitors[i]["x"] for i in ids],
                    [monitors[i]["y"] for i in ids],
                    [monitors[i]["yaw"] for i in ids],
                )
                for k, object_id in enumerate(ids):
                    m = monitors[object_id]
                    objects.append(
                        {
                            "id": object_id,
                            "name": names.get(object_id, f"Object {object_id}"),
                            "lat": round(float(lats[k]), 9),
                            "lon": round(float(lons[k]), 9),
                            "bearing": round(float(bearings[k]), 2),
                            "speed": round(m["speed"], 2),
                            "stale": now - m["received"] > MONITOR_STALE_S,
                        }
                    )
        payload["objects"] = objects
        return payload

    @property
    def object_ids(self) -> list[int]:
        with self._lock:
            return sorted(set(self.object_names) | set(self._geo_paths))
