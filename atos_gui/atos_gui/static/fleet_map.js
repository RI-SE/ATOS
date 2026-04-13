(function() {
  function toRad(value) {
    return value * Math.PI / 180.0;
  }

  // Constants aligned with atos/common/util.c:
  // EARTH_EQUATOR_RADIUS_M = 6378137.0 and INVERSE_FLATTENING = 298.257223563
  function vincentyDistanceMeters(lat1, lon1, lat2, lon2) {
    const a = 6378137.0;
    const f = 1.0 / 298.257223563;
    const b = (1.0 - f) * a;

    const phi1 = toRad(lat1);
    const phi2 = toRad(lat2);
    const L = toRad(lon2 - lon1);

    const U1 = Math.atan((1.0 - f) * Math.tan(phi1));
    const U2 = Math.atan((1.0 - f) * Math.tan(phi2));
    const sinU1 = Math.sin(U1);
    const cosU1 = Math.cos(U1);
    const sinU2 = Math.sin(U2);
    const cosU2 = Math.cos(U2);

    let lambda = L;
    let lambdaPrev = 0.0;
    let iterLimit = 100;
    let sinSigma = 0.0;
    let cosSigma = 0.0;
    let sigma = 0.0;
    let sinAlpha = 0.0;
    let cosSqAlpha = 0.0;
    let cos2SigmaM = 0.0;

    while (iterLimit > 0) {
      iterLimit -= 1;
      const sinLambda = Math.sin(lambda);
      const cosLambda = Math.cos(lambda);
      sinSigma = Math.sqrt(
        (cosU2 * sinLambda) * (cosU2 * sinLambda) +
        (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda) *
          (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda)
      );

      if (sinSigma === 0.0) {
        return 0.0;
      }

      cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda;
      sigma = Math.atan2(sinSigma, cosSigma);
      sinAlpha = (cosU1 * cosU2 * sinLambda) / sinSigma;
      cosSqAlpha = 1.0 - sinAlpha * sinAlpha;
      if (cosSqAlpha !== 0.0) {
        cos2SigmaM = cosSigma - 2.0 * sinU1 * sinU2 / cosSqAlpha;
      } else {
        cos2SigmaM = 0.0;
      }

      const C = f / 16.0 * cosSqAlpha * (4.0 + f * (4.0 - 3.0 * cosSqAlpha));
      lambdaPrev = lambda;
      lambda = L + (1.0 - C) * f * sinAlpha *
        (sigma + C * sinSigma *
          (cos2SigmaM + C * cosSigma * (-1.0 + 2.0 * cos2SigmaM * cos2SigmaM)));

      if (Math.abs(lambda - lambdaPrev) < 1e-12) {
        break;
      }
    }

    if (iterLimit <= 0) {
      return Number.NaN;
    }

    const uSq = cosSqAlpha * (a * a - b * b) / (b * b);
    const A = 1.0 + uSq / 16384.0 *
      (4096.0 + uSq * (-768.0 + uSq * (320.0 - 175.0 * uSq)));
    const B = uSq / 1024.0 *
      (256.0 + uSq * (-128.0 + uSq * (74.0 - 47.0 * uSq)));
    const deltaSigma = B * sinSigma *
      (cos2SigmaM + B / 4.0 *
        (cosSigma * (-1.0 + 2.0 * cos2SigmaM * cos2SigmaM) -
         B / 6.0 * cos2SigmaM * (-3.0 + 4.0 * sinSigma * sinSigma) *
           (-3.0 + 4.0 * cos2SigmaM * cos2SigmaM)));

    return b * A * (sigma - deltaSigma);
  }

  function findPathCoordinates(geojson) {
    if (!geojson || !Array.isArray(geojson.features)) {
      return [];
    }

    const preferred = geojson.features.find(function(feature) {
      return feature && feature.id === "PathSection" &&
        feature.geometry && feature.geometry.type === "LineString";
    });
    if (preferred && preferred.geometry && preferred.geometry.coordinates) {
      return preferred.geometry.coordinates;
    }

    const fallback = geojson.features.find(function(feature) {
      return feature && feature.geometry && feature.geometry.type === "LineString";
    });
    if (fallback && fallback.geometry && fallback.geometry.coordinates) {
      return fallback.geometry.coordinates;
    }

    return [];
  }

  function computeTotalLengthMeters(coords) {
    if (!Array.isArray(coords) || coords.length < 2) {
      return 0.0;
    }

    let total = 0.0;
    for (let index = 1; index < coords.length; index += 1) {
      const previous = coords[index - 1];
      const current = coords[index];
      const distance = vincentyDistanceMeters(previous[1], previous[0], current[1], current[0]);
      if (Number.isFinite(distance)) {
        total += distance;
      }
    }
    return total;
  }

  function normalizeTruckStates(trucks) {
    if (!Array.isArray(trucks)) {
      return [];
    }
    return trucks.filter(function(item) {
      return item &&
        Number.isFinite(Number(item.lat)) &&
        Number.isFinite(Number(item.lon));
    });
  }

  function renderSvg(container, coords, trucks) {
    const width = Math.max(container.clientWidth, 700);
    const height = Math.max(container.clientHeight, 420);
    const padding = 30;

    let minLon = Infinity;
    let maxLon = -Infinity;
    let minLat = Infinity;
    let maxLat = -Infinity;

    coords.forEach(function(item) {
      const lon = item[0];
      const lat = item[1];
      if (lon < minLon) minLon = lon;
      if (lon > maxLon) maxLon = lon;
      if (lat < minLat) minLat = lat;
      if (lat > maxLat) maxLat = lat;
    });

    const lonSpan = Math.max(maxLon - minLon, 1e-9);
    const latSpan = Math.max(maxLat - minLat, 1e-9);

    function toSvgXY(lon, lat) {
      const x = padding + ((lon - minLon) / lonSpan) * (width - 2 * padding);
      const y = height - padding - ((lat - minLat) / latSpan) * (height - 2 * padding);
      return {x: x, y: y};
    }

    const linePoints = coords.map(function(item) {
      const p = toSvgXY(item[0], item[1]);
      return p.x.toFixed(2) + "," + p.y.toFixed(2);
    }).join(" ");

    const totalMeters = computeTotalLengthMeters(coords);
    const totalKm = totalMeters / 1000.0;

    const truckCircles = trucks.map(function(item) {
      const p = toSvgXY(Number(item.lon), Number(item.lat));
      const uid = String(item.uid || "truck");
      const speedKmh = Number(item.speed_kmh || 0);
      const courseDeg = Number(item.course_deg || 0);
      const color = item.tcp_connected ? "#dc2626" : "#6b7280";
      return (
        "<g>" +
          "<circle cx='" + p.x.toFixed(2) + "' cy='" + p.y.toFixed(2) + "' r='5' fill='" + color + "' />" +
          "<text x='" + (p.x + 8).toFixed(2) + "' y='" + (p.y - 8).toFixed(2) + "' " +
            "font-size='12' font-family='sans-serif' fill='#111827'>" +
            uid + " " + speedKmh.toFixed(1) + " km/h @" + courseDeg.toFixed(0) + "°" +
          "</text>" +
        "</g>"
      );
    }).join("");

    container.innerHTML =
      "<svg width='100%' height='" + height + "' viewBox='0 0 " + width + " " + height + "' " +
      "style='background:#f8fafc;border:1px solid #cbd5e1;border-radius:8px;'>" +
      "<polyline fill='none' stroke='#0ea5e9' stroke-width='3' points='" + linePoints + "'/>" +
      truckCircles +
      "</svg>" +
      "<div style='padding-top:8px;font-family:sans-serif;font-size:13px;'>" +
      "<div><b>Path points:</b> " + coords.length + "</div>" +
      "<div><b>Total length (Vincenty):</b> " + totalMeters.toFixed(2) + " m</div>" +
      "<div><b>Total length:</b> " + totalKm.toFixed(3) + " km</div>" +
      "<div><b>Live trucks:</b> " + trucks.length + "</div>" +
      "</div>";
  }

  function renderInternal(containerId) {
    const container = document.getElementById(containerId);
    if (!container) {
      return;
    }
    const state = window.__fleetRoadMapState[containerId];
    if (!state) {
      return;
    }

    const coords = findPathCoordinates(state.geojson);
    if (coords.length === 0) {
      container.innerHTML = "<div style='padding:12px;color:#b91c1c;font-weight:600;'>No LineString found in geojson.</div>";
      return;
    }
    renderSvg(container, coords, normalizeTruckStates(state.trucks));
  }

  window.__fleetRoadMapState = window.__fleetRoadMapState || {};

  window.renderFleetRoadMap = function(containerId, geojson, trucks) {
    window.__fleetRoadMapState[containerId] = {
      geojson: geojson,
      trucks: normalizeTruckStates(trucks),
    };
    renderInternal(containerId);
  };

  window.updateFleetTruckStates = function(containerId, trucks) {
    if (!window.__fleetRoadMapState[containerId]) {
      return;
    }
    window.__fleetRoadMapState[containerId].trucks = normalizeTruckStates(trucks);
    renderInternal(containerId);
  };
})();
