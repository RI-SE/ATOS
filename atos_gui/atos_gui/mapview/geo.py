"""This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
"""

"""Conversions from the ATOS local test frame to WGS84 for the map view.

ATOS' local frame (see EsminiAdapter, esminiadapter.cpp) is a transverse Mercator
projection centred on the test origin:

    +proj=tmerc +lat_0=<origin lat> +lon_0=<origin lon> +datum=WGS84 +units=m

x is grid east, y is grid north (metres), and headings are ENU yaw: counter-clockwise
from +x. Using the same PROJ definition here makes the inverse exact, unlike the
flat-earth approximation (llhOffsetMeters) behind the gnss_path / gnss_fix topics.

Leaflet's Web Mercator display is conformal and always north-up, so a marker rotated
clockwise on screen by the true geodesic bearing points the right way.
"""

import math
import threading

import numpy as np
from pyproj import Geod, Transformer

WGS84 = Geod(ellps="WGS84")

# Length of the step used to turn a heading into a geodesic bearing. The tmerc
# scale/curvature over one metre is far below anything visible on a map.
_HEADING_STEP_M = 1.0


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about z) of a quaternion, in radians, CCW from +x."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class LocalFrame:
    """The ATOS local frame for one test origin."""

    def __init__(self, origin_lat: float, origin_lon: float) -> None:
        if not (-90.0 <= origin_lat <= 90.0 and -180.0 <= origin_lon <= 180.0):
            raise ValueError(f"Invalid origin {origin_lat}, {origin_lon}")
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.proj_string = (
            f"+proj=tmerc +lat_0={origin_lat!r} +lon_0={origin_lon!r} "
            "+datum=WGS84 +units=m"
        )
        self._to_wgs84 = Transformer.from_crs(
            self.proj_string, "EPSG:4326", always_xy=True
        )
        self._from_wgs84 = Transformer.from_crs(
            "EPSG:4326", self.proj_string, always_xy=True
        )
        # pyproj Transformer objects must not be used from several threads at once.
        self._lock = threading.Lock()

    def to_lonlat(self, xs, ys) -> tuple[np.ndarray, np.ndarray]:
        """Local x/y (metres) to WGS84 longitude/latitude (degrees)."""
        return self._transform(self._to_wgs84, xs, ys)

    def from_lonlat(self, lons, lats) -> tuple[np.ndarray, np.ndarray]:
        """WGS84 longitude/latitude (degrees) to local x/y (metres)."""
        return self._transform(self._from_wgs84, lons, lats)

    def _transform(self, transformer: Transformer, a, b) -> tuple[np.ndarray, np.ndarray]:
        a = np.asarray(a, dtype=float)
        b = np.asarray(b, dtype=float)
        with self._lock:
            if a.size == 1:
                # pyproj's single-point fast path calls float() on its inputs, which NumPy
                # deprecates for size-1 arrays (pyproj 3.7, the last for Python 3.10).
                p, q = transformer.transform(float(a.item()), float(b.item()))
                return np.full(a.shape, p), np.full(b.shape, q)
            p, q = transformer.transform(a, b)
        return np.asarray(p), np.asarray(q)

    def poses_to_geo(self, xs, ys, yaws) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Local poses to (lat, lon, bearing).

        The bearing is in degrees clockwise from true north, in [0, 360). It is the
        geodesic azimuth from each position to a point one step ahead along the
        heading, so it includes the meridian convergence exactly.
        """
        xs = np.atleast_1d(np.asarray(xs, dtype=float))
        ys = np.atleast_1d(np.asarray(ys, dtype=float))
        yaws = np.atleast_1d(np.asarray(yaws, dtype=float))
        lons, lats = self.to_lonlat(xs, ys)
        lons_ahead, lats_ahead = self.to_lonlat(
            xs + _HEADING_STEP_M * np.cos(yaws), ys + _HEADING_STEP_M * np.sin(yaws)
        )
        if lons.size == 1:
            # Same single-point fast path as in _transform.
            azimuth, _, _ = WGS84.inv(lons.item(), lats.item(), lons_ahead.item(), lats_ahead.item())
            azimuths = np.full(lons.shape, azimuth)
        else:
            azimuths, _, _ = WGS84.inv(lons, lats, lons_ahead, lats_ahead)
        bearings = np.mod(np.asarray(azimuths, dtype=float), 360.0)
        return lats, lons, bearings


def decimate(xs, ys, min_spacing_m: float = 0.1) -> np.ndarray:
    """Indices of points at least min_spacing_m apart (first and last always kept).

    Trajectories from esmini are densely sampled; dropping points closer than a few
    centimetres keeps the browser payload small without visibly changing the line.
    """
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    n = len(xs)
    if n <= 2:
        return np.arange(n)
    keep = [0]
    last_x, last_y = xs[0], ys[0]
    limit = min_spacing_m * min_spacing_m
    for i in range(1, n - 1):
        dx = xs[i] - last_x
        dy = ys[i] - last_y
        if dx * dx + dy * dy >= limit:
            keep.append(i)
            last_x, last_y = xs[i], ys[i]
    keep.append(n - 1)
    return np.asarray(keep)
