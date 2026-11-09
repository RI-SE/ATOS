"""This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
"""

"""MapDataSource tests with a fake rclpy node (no ROS graph needed)."""

import math
from types import SimpleNamespace as NS

import pytest

from atos_gui.mapview import map_data
from atos_gui.mapview.geo import LocalFrame
from atos_gui.mapview.map_data import MapDataSource

ORIGIN = (57.7775, 12.7814)


class FakeFuture:
    def __init__(self):
        self._done, self._result, self._callbacks = False, None, []

    def done(self):
        return self._done

    def cancelled(self):
        return False

    def result(self):
        return self._result

    def add_done_callback(self, cb):
        self._callbacks.append(cb)

    def set_result(self, result):
        self._done, self._result = True, result
        for cb in self._callbacks:
            cb(self)


class FakeClient:
    def __init__(self, name):
        self.name, self.calls, self.removed, self.ready = name, [], [], True

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = FakeFuture()
        self.calls.append(future)
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)


class FakeTimer:
    def __init__(self, period, cb):
        self.period, self.cb, self.canceled = period, cb, False

    def cancel(self):
        self.canceled = True

    def is_canceled(self):
        return self.canceled


class FakeNode:
    def __init__(self):
        self.clients, self.subs, self.timers, self.destroyed_subs = {}, {}, [], []

    def create_client(self, _type, name, callback_group=None):
        self.clients[name] = FakeClient(name)
        return self.clients[name]

    def create_subscription(self, _type, topic, cb, qos, callback_group=None):
        self.subs[topic] = cb
        return topic

    def destroy_subscription(self, sub):
        self.destroyed_subs.append(sub)
        self.subs.pop(sub, None)

    def create_timer(self, period, cb, callback_group=None):
        timer = FakeTimer(period, cb)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer):
        timer.canceled = True

    def get_logger(self):
        return NS(info=lambda *_: None, warning=lambda *_: None)


def pose(x, y, yaw):
    return NS(
        position=NS(x=x, y=y, z=0.0),
        orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
    )


def path_msg(points):
    return NS(poses=[NS(pose=pose(x, y, 0.0)) for x, y in points])


def monitor_msg(object_id, x, y, yaw, vx=3.0, vy=4.0):
    return NS(
        atos_header=NS(object_id=object_id),
        pose=NS(pose=pose(x, y, yaw)),
        velocity=NS(twist=NS(linear=NS(x=vx, y=vy, z=0.0))),
        object_state=NS(state=4),
    )


def origin_result(lat, lon, success=True):
    return NS(success=success, origin=NS(position=NS(latitude=lat, longitude=lon, altitude=0.0)))


@pytest.fixture
def setup():
    node = FakeNode()
    state = {"value": 1}
    data = MapDataSource(node, get_control_state=lambda: state["value"])
    maintain = node.timers[0].cb
    return node, state, data, maintain


def load(node, state, maintain, ids=(1, 2), names=("Car", "Truck")):
    state["value"] = 2
    maintain()
    node.clients["/atos/get_object_ids"].calls[-1].set_result(NS(ids=list(ids), names=list(names)))
    node.clients["/atos/get_test_origin"].calls[-1].set_result(origin_result(*ORIGIN))


def test_waits_for_initialized_before_calling_services(setup):
    node, state, data, maintain = setup
    maintain()
    assert node.clients["/atos/get_object_ids"].calls == []
    assert node.clients["/atos/get_test_origin"].calls == []


def test_load_sequence_and_snapshot(setup):
    node, state, data, maintain = setup
    load(node, state, maintain)
    assert data.frame is not None and data.status.startswith("Origin")
    assert "/atos/object_1/path" in node.subs and "/atos/object_2/path" in node.subs

    node.subs["/atos/object_1/path"](path_msg([(0, 0), (10, 0), (10, 10)]))
    node.subs["/atos/object_monitor"](monitor_msg(1, 10.0, 0.0, math.pi / 2))
    node.subs["/atos/connected_object_ids"](NS(ids=[1]))

    snap = data.snapshot(None)
    assert snap["origin"] == list(ORIGIN)
    (traj,) = snap["trajectories"]
    assert traj["id"] == 1 and traj["name"] == "Car" and len(traj["latlngs"]) == 3
    assert traj["latlngs"][0] == pytest.approx(list(ORIGIN), abs=1e-9)

    (obj,) = snap["objects"]
    frame = LocalFrame(*ORIGIN)
    lats, lons, bearings = frame.poses_to_geo([10.0], [0.0], [math.pi / 2])
    assert obj["lat"] == pytest.approx(lats[0], abs=1e-8)
    assert obj["lon"] == pytest.approx(lons[0], abs=1e-8)
    assert obj["bearing"] == pytest.approx(bearings[0], abs=0.01)
    assert obj["bearing"] == pytest.approx(0.0, abs=0.01) or obj["bearing"] == pytest.approx(360.0, abs=0.01)
    assert obj["speed"] == pytest.approx(5.0)
    assert obj["stale"] is False

    # Unchanged trajectories are not resent.
    assert "trajectories" not in data.snapshot(snap["traj_version"])


def test_path_received_before_origin_is_converted_later(setup):
    node, state, data, maintain = setup
    state["value"] = 2
    maintain()
    node.clients["/atos/get_object_ids"].calls[-1].set_result(NS(ids=[1], names=["Car"]))
    assert data.frame is None
    # Origin not answered yet; a (re)subscribed path arrives via a previous subscription.
    data._on_path(1, path_msg([(0, 0), (5, 5)]))
    assert data.snapshot(None)["trajectories"] == []
    node.clients["/atos/get_test_origin"].calls[-1].set_result(origin_result(*ORIGIN))
    assert len(data.snapshot(None)["trajectories"][0]["latlngs"]) == 2


def test_only_connected_objects_shown_and_stale_flag(setup, monkeypatch):
    node, state, data, maintain = setup
    load(node, state, maintain)
    now = [1000.0]
    monkeypatch.setattr(map_data.time, "monotonic", lambda: now[0])
    node.subs["/atos/object_monitor"](monitor_msg(1, 0.0, 0.0, 0.0))
    node.subs["/atos/object_monitor"](monitor_msg(2, 5.0, 0.0, 0.0))
    node.subs["/atos/connected_object_ids"](NS(ids=[2]))
    assert [o["id"] for o in data.snapshot(None)["objects"]] == [2]

    # Connected list not published any more: fall back to all objects with data.
    now[0] += map_data.CONNECTED_IDS_MAX_AGE_S + 0.1
    objects = data.snapshot(None)["objects"]
    assert [o["id"] for o in objects] == [1, 2]
    assert all(o["stale"] for o in objects)  # older than MONITOR_STALE_S

    now[0] += map_data.MONITOR_FORGET_S
    assert data.snapshot(None)["objects"] == []


def test_origin_timeout_reports_and_retries(setup):
    node, state, data, maintain = setup
    state["value"] = 2
    maintain()
    node.clients["/atos/get_object_ids"].calls[-1].set_result(NS(ids=[1], names=["Car"]))
    origin_client = node.clients["/atos/get_test_origin"]
    timeout_timer = node.timers[-1]
    assert timeout_timer.period == map_data.ORIGIN_TIMEOUT_S
    timeout_timer.cb()
    assert origin_client.removed == [origin_client.calls[-1]]
    assert "Timed out" in data.status and data.frame is None

    # A new scenario init triggers another attempt.
    node.subs["/atos/state_change"](NS(prev_state=1, current_state=2))
    maintain()
    assert len(node.clients["/atos/get_object_ids"].calls) == 2


def test_service_unavailable_retries_next_tick(setup):
    node, state, data, maintain = setup
    node.clients["/atos/get_object_ids"].ready = False
    state["value"] = 2
    maintain()
    assert "not available" in data.status
    node.clients["/atos/get_object_ids"].ready = True
    maintain()
    assert len(node.clients["/atos/get_object_ids"].calls) == 1


def test_reinit_with_fewer_objects_drops_subscriptions(setup):
    node, state, data, maintain = setup
    load(node, state, maintain)
    node.subs["/atos/object_2/path"](path_msg([(0, 0), (1, 1)]))
    node.subs["/atos/state_change"](NS(prev_state=1, current_state=2))
    load(node, state, maintain, ids=(1,), names=("Car",))
    assert "/atos/object_2/path" in node.destroyed_subs
    assert [t["id"] for t in data.snapshot(None)["trajectories"]] == []
    assert data.object_ids == [1]


def test_missing_georeference(setup):
    node, state, data, maintain = setup
    state["value"] = 2
    maintain()
    node.clients["/atos/get_object_ids"].calls[-1].set_result(NS(ids=[1], names=["Car"]))
    node.clients["/atos/get_test_origin"].calls[-1].set_result(origin_result(float("nan"), 0.0))
    assert data.frame is None and "geo-reference" in data.status
