"""This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
"""

import math

import numpy as np
import pytest
from pyproj import Transformer

from atos_gui.mapview.geo import WGS84, LocalFrame, decimate, yaw_from_quaternion

# Origin of conf/odr/GaragePlanHighRes.xodr (AstaZero).
ORIGIN_LAT = 57.7775
ORIGIN_LON = 12.7814


@pytest.fixture
def frame():
    return LocalFrame(ORIGIN_LAT, ORIGIN_LON)


def angle_diff_deg(a, b):
    return (np.asarray(a) - np.asarray(b) + 180.0) % 360.0 - 180.0


def test_origin_maps_to_origin(frame):
    lons, lats = frame.to_lonlat([0.0], [0.0])
    assert lats[0] == pytest.approx(ORIGIN_LAT, abs=1e-12)
    assert lons[0] == pytest.approx(ORIGIN_LON, abs=1e-12)


def test_matches_esmini_projection_round_trip(frame):
    # Forward with the exact definition EsminiAdapter uses, inverse with ours.
    esmini = Transformer.from_crs(
        "EPSG:4326",
        f"+proj=tmerc +lat_0={ORIGIN_LAT} +lon_0={ORIGIN_LON} +datum=WGS84 +units=m",
        always_xy=True,
    )
    xs = np.array([-2000.0, -150.0, 0.0, 300.0, 2000.0])
    ys = np.array([1500.0, -2000.0, 0.0, 42.0, -2000.0])
    lons, lats = frame.to_lonlat(xs, ys)
    xs_back, ys_back = esmini.transform(lons, lats)
    np.testing.assert_allclose(xs_back, xs, atol=1e-3)
    np.testing.assert_allclose(ys_back, ys, atol=1e-3)


def test_axes_point_east_and_north(frame):
    lons, lats = frame.to_lonlat([100.0, 0.0], [0.0, 100.0])
    assert lons[0] > ORIGIN_LON and lats[0] == pytest.approx(ORIGIN_LAT, abs=1e-6)
    assert lats[1] > ORIGIN_LAT and lons[1] == pytest.approx(ORIGIN_LON, abs=1e-12)
    # Distances are true metres (tmerc scale factor is 1 on the central meridian).
    _, _, d_east = WGS84.inv(ORIGIN_LON, ORIGIN_LAT, lons[0], lats[0])
    _, _, d_north = WGS84.inv(ORIGIN_LON, ORIGIN_LAT, lons[1], lats[1])
    assert d_east == pytest.approx(100.0, abs=1e-3)
    assert d_north == pytest.approx(100.0, abs=1e-6)


@pytest.mark.parametrize(
    "yaw, bearing",
    [
        (0.0, 90.0),  # ENU east
        (math.pi / 2, 0.0),  # north
        (math.pi, 270.0),  # west
        (-math.pi / 2, 180.0),  # south
        (3 * math.pi / 2, 180.0),  # [0, 2pi) as stored by ATOS
        (math.radians(30), 60.0),
    ],
)
def test_yaw_to_bearing_on_central_meridian(frame, yaw, bearing):
    _, _, bearings = frame.poses_to_geo([0.0], [0.0], [yaw])
    assert angle_diff_deg(bearings[0], bearing) == pytest.approx(0.0, abs=1e-6)
    assert 0.0 <= bearings[0] < 360.0


def test_meridian_convergence_sign(frame):
    # Grid north east of the central meridian (northern hemisphere) is rotated
    # clockwise from true north by about dlon * sin(lat); west of it, anticlockwise.
    _, _, bearings = frame.poses_to_geo([2000.0, -2000.0], [0.0, 0.0], [math.pi / 2] * 2)
    lons, _ = frame.to_lonlat([2000.0], [0.0])
    expected = (lons[0] - ORIGIN_LON) * math.sin(math.radians(ORIGIN_LAT))
    assert expected > 0.02  # big enough for the test to mean something
    assert angle_diff_deg(bearings[0], expected) == pytest.approx(0.0, abs=1e-3)
    assert angle_diff_deg(bearings[1], -expected) == pytest.approx(0.0, abs=1e-3)


def test_heading_follows_drawn_trajectory(frame):
    # A marker heading along a straight local segment must point along the segment
    # as drawn on the map, anywhere in a +-2 km test area.
    rng = np.random.default_rng(0)
    for _ in range(50):
        x0, y0 = rng.uniform(-2000, 2000, 2)
        yaw = rng.uniform(0, 2 * math.pi)
        x1, y1 = x0 + 20 * math.cos(yaw), y0 + 20 * math.sin(yaw)
        lats, lons, bearings = frame.poses_to_geo([x0], [y0], [yaw])
        lon1, lat1 = frame.to_lonlat([x1], [y1])
        azimuth, _, _ = WGS84.inv(lons[0], lats[0], lon1[0], lat1[0])
        assert angle_diff_deg(bearings[0], azimuth) == pytest.approx(0.0, abs=1e-3)


def test_differs_from_flat_earth_gnss_topics_as_expected(frame):
    # llhOffsetMeters (CRSTransformation.cpp), used for gnss_path / gnss_fix, is a
    # flat-earth step on a sphere of radius a. It should agree to within ~0.3%.
    a = 6378137.0
    for x, y in [(1000.0, 0.0), (0.0, 1000.0), (-700.0, 700.0)]:
        flat_lat = ORIGIN_LAT + y / a * 180 / math.pi
        flat_lon = ORIGIN_LON + x / a * 180 / math.pi / math.cos(math.radians(ORIGIN_LAT))
        lons, lats = frame.to_lonlat([x], [y])
        _, _, error = WGS84.inv(flat_lon, flat_lat, lons[0], lats[0])
        assert error < 0.003 * math.hypot(x, y)


def test_yaw_from_quaternion():
    for yaw in [0.0, 0.3, math.pi / 2, 3.0, -2.0]:
        q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        assert yaw_from_quaternion(*q) == pytest.approx(math.atan2(math.sin(yaw), math.cos(yaw)))
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == 0.0


def test_decimate_keeps_endpoints_and_spacing():
    xs = np.linspace(0, 10, 1001)  # 1 cm spacing
    ys = np.zeros_like(xs)
    keep = decimate(xs, ys, 0.1)
    assert keep[0] == 0 and keep[-1] == len(xs) - 1
    assert np.all(np.diff(xs[keep[:-1]]) >= 0.1 - 1e-9)
    assert len(keep) < 110


def test_invalid_origin_rejected():
    with pytest.raises(ValueError):
        LocalFrame(123.0, 0.0)
