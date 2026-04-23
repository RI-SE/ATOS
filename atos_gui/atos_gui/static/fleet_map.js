(function() {
  function toRad(value) {
    return value * Math.PI / 180.0;
  }

  function escapeHtml(value) {
    return String(value || "")
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/\"/g, "&quot;")
      .replace(/'/g, '&#39;');
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
    }).map(function(item) {
      const normalized = Object.assign({}, item);
      normalized.uid = String(item.uid || "truck");
      normalized.path_name = item.path_name ? String(item.path_name) : "";
      normalized.path_index = Number.isFinite(Number(item.path_index)) ? Number(item.path_index) : -1;
      return normalized;
    }).sort(function(a, b) {
      return a.uid.localeCompare(b.uid, undefined, { sensitivity: "base", numeric: true });
    });
  }

  function selectPathName(state) {
    const trucks = state.trucks || [];
    const counts = {};
    trucks.forEach(function(item) {
      const pathName = item.path_name || state.defaultPathName;
      if (!pathName) {
        return;
      }
      counts[pathName] = (counts[pathName] || 0) + 1;
    });

    let bestName = state.defaultPathName;
    let bestCount = -1;
    Object.keys(counts).forEach(function(name) {
      if (counts[name] > bestCount) {
        bestCount = counts[name];
        bestName = name;
      }
    });

    if (!bestName || !state.pathsByName[bestName]) {
      const available = Object.keys(state.pathsByName);
      if (available.length > 0) {
        bestName = available[0];
      }
    }
    return bestName;
  }

  function computeAheadDistanceMap(trucks, pathLengthMeters, selectedPathName) {
    const result = {};
    const candidates = trucks
      .filter(function(item) {
        const itemPath = item.path_name || selectedPathName;
        return itemPath === selectedPathName && Number.isFinite(Number(item.distance_m));
      })
      .map(function(item) {
        return {
          uid: String(item.uid || "truck"),
          distance_m: Number(item.distance_m),
        };
      })
      .sort(function(a, b) { return a.distance_m - b.distance_m; });

    if (candidates.length < 2) {
      return result;
    }

    for (let i = 0; i < candidates.length; i += 1) {
      const current = candidates[i];
      const next = candidates[(i + 1) % candidates.length];
      let gap = next.distance_m - current.distance_m;
      if (gap <= 0 && pathLengthMeters > 0) {
        gap += pathLengthMeters;
      }
      if (gap > 0.01) {
        result[current.uid] = gap;
      }
    }
    return result;
  }

  function renderSvg(container, coords, trucks, selectedPathName, pathsByName) {
    const width = 600;
    const height = 650;
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

    const filteredTrucks = trucks.filter(function(item) {
      const itemPath = item.path_name || selectedPathName;
      return itemPath === selectedPathName;
    });

    const aheadDistanceMap = computeAheadDistanceMap(filteredTrucks, totalMeters, selectedPathName);

    const truckCircles = filteredTrucks.map(function(item) {
      const p = toSvgXY(Number(item.lon), Number(item.lat));
      const uid = String(item.uid || "truck");
      const speedKmh = Number.isFinite(Number(item.speed_mps))
        ? Number(item.speed_mps) * 3.6
        : Number(item.speed_kmh || 0);
      const courseDeg = Number(item.course_deg || 0);
      const pathIndex = Number(item.path_index || -1);
      const color = item.tcp_connected ? "#dc2626" : "#6b7280";
      const ahead = aheadDistanceMap[uid];
      const aheadText =
        Number.isFinite(ahead)
          ? (" next: " + ahead.toFixed(1) + " m")
          : " next: -";
      return (
        "<g>" +
          "<circle cx='" + p.x.toFixed(2) + "' cy='" + p.y.toFixed(2) + "' r='5' fill='" + color + "' />" +
          "<text x='" + (p.x + 8).toFixed(2) + "' y='" + (p.y - 8).toFixed(2) + "' " +
            "font-size='12' font-family='sans-serif' fill='#111827'>" +
            uid + " [idx " + pathIndex + "] " + speedKmh.toFixed(1) + " km/h @" + courseDeg.toFixed(0) + "°" + aheadText +
          "</text>" +
        "</g>"
      );
    }).join("");

    const truckRows = filteredTrucks.map(function(item) {
      const uid = String(item.uid || "truck");
      const speedKmh = Number.isFinite(Number(item.speed_mps))
        ? Number(item.speed_mps) * 3.6
        : Number(item.speed_kmh || 0);
      const courseDeg = Number(item.course_deg || 0);
      const pathIndex = Number(item.path_index || -1);
      const lastCotMessage = String(item.last_cot_message || "-");
      const lastTcpCommand = String(item.last_tcp_command || "-");
      const tcpWarning = String(item.last_tcp_warning || "");
      const ahead = aheadDistanceMap[uid];
      const aheadCell =
        Number.isFinite(ahead)
          ? ahead.toFixed(1) + " m"
          : "-";
      return (
        "<tr>" +
          "<td style='padding:2px 8px 2px 0;'>" + uid + "</td>" +
          "<td style='padding:2px 8px 2px 0;'>" + pathIndex + "</td>" +
          "<td style='padding:2px 8px 2px 0;'>" + speedKmh.toFixed(1) + " km/h</td>" +
          "<td style='padding:2px 8px 2px 0;'>" + courseDeg.toFixed(0) + "°</td>" +
          "<td style='padding:2px 8px 2px 0;'><b>" + aheadCell + "</b></td>" +
          "<td style='padding:2px 0;max-width:760px;'>" +
            "<div><b>Latest CoT</b></div>" +
            "<pre style='margin:2px 0 0 0;max-height:120px;overflow:auto;white-space:pre-wrap;word-break:break-word;font-family:monospace;font-size:12px;border:1px solid #d1d5db;border-radius:6px;padding:6px;background:#f8fafc;'>" + escapeHtml(lastCotMessage) + "</pre>" +
            "<div style='padding-top:4px;'><b>Last TCP</b></div>" +
            "<pre style='margin:2px 0 0 0;max-height:120px;overflow:auto;white-space:pre-wrap;word-break:break-word;font-family:monospace;font-size:12px;border:1px solid #d1d5db;border-radius:6px;padding:6px;background:#f8fafc;'>" + escapeHtml(lastTcpCommand) + "</pre>" +
            (tcpWarning ? "<div style='padding-top:4px;color:#b91c1c;font-weight:600;'>TCP warning: " + escapeHtml(tcpWarning) + "</div>" : "") +
          "</td>" +
        "</tr>"
      );
    }).join("");

    const allPathNames = Object.keys(pathsByName).sort();
    const pathSummary = allPathNames.map(function(pathName) {
      const count = trucks.filter(function(t) {
        const tPath = t.path_name || selectedPathName;
        return tPath === pathName;
      }).length;
      return "<li><code>" + pathName + "</code>: " + count + " truck(s)</li>";
    }).join("");

    container.innerHTML =
      "<div style='display:flex;gap:12px;align-items:flex-start;flex-wrap:wrap;'>" +
      "<div style='flex:0 0 auto;width:600px;'>" +
      "<svg width='" + width + "' height='" + height + "' viewBox='0 0 " + width + " " + height + "' " +
      "style='background:#f8fafc;border:1px solid #cbd5e1;border-radius:8px;'>" +
      "<polyline fill='none' stroke='#0ea5e9' stroke-width='3' points='" + linePoints + "'/>" +
      truckCircles +
      "</svg>" +
      "</div>" +
      "<div style='flex:1 1 420px;min-width:420px;padding:2px 4px;font-family:sans-serif;font-size:13px;'>" +
      "<div><b>Selected path:</b> <code>" + selectedPathName + "</code></div>" +
      "<div><b>Path points:</b> " + coords.length + "</div>" +
      "<div><b>Total length (Vincenty):</b> " + totalMeters.toFixed(2) + " m</div>" +
      "<div><b>Total length:</b> " + totalKm.toFixed(3) + " km</div>" +
      "<div><b>Live trucks on selected path:</b> " + filteredTrucks.length + "</div>" +
      "<div style='padding-top:6px;'><b>Available paths in payload</b></div>" +
      "<ul style='margin:4px 0 8px 16px;'>" + pathSummary + "</ul>" +
      "<div style='padding-top:6px;'><b>Distance To Next Truck Ahead</b></div>" +
      "<table style='font-family:sans-serif;font-size:13px;border-collapse:collapse;'>" +
      "<thead><tr><th style='text-align:left;padding:2px 8px 2px 0;'>Truck</th>" +
      "<th style='text-align:left;padding:2px 8px 2px 0;'>Path idx</th>" +
      "<th style='text-align:left;padding:2px 8px 2px 0;'>Speed</th>" +
      "<th style='text-align:left;padding:2px 8px 2px 0;'>Course</th>" +
      "<th style='text-align:left;padding:2px 8px 2px 0;'>Next ahead</th>" +
      "<th style='text-align:left;padding:2px 0;'>Latest CoT / Last TCP</th></tr></thead>" +
      "<tbody>" + truckRows + "</tbody></table>" +
      "</div>" +
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

    const selectedPathName = selectPathName(state);
    const geojson = state.pathsByName[selectedPathName];
    const coords = findPathCoordinates(geojson);

    if (!selectedPathName || coords.length === 0) {
      container.innerHTML = "<div style='padding:12px;color:#b91c1c;font-weight:600;'>No valid path found in payload.</div>";
      return;
    }

    renderSvg(container, coords, state.trucks, selectedPathName, state.pathsByName);
  }

  function normalizePayload(payload) {
    const normalized = payload || {};
    normalized.default_path_name = String(normalized.default_path_name || "");
    normalized.paths = normalized.paths && typeof normalized.paths === "object" ? normalized.paths : {};
    normalized.trucks = normalizeTruckStates(normalized.trucks || []);
    return normalized;
  }

  window.__fleetRoadMapState = window.__fleetRoadMapState || {};

  window.renderFleetRoadMap = function(containerId, payload) {
    const p = normalizePayload(payload);
    window.__fleetRoadMapState[containerId] = {
      defaultPathName: p.default_path_name,
      pathsByName: p.paths,
      trucks: p.trucks,
    };
    renderInternal(containerId);
  };

  window.updateFleetRoadMap = function(containerId, payload) {
    const p = normalizePayload(payload);
    window.__fleetRoadMapState[containerId] = {
      defaultPathName: p.default_path_name,
      pathsByName: p.paths,
      trucks: p.trucks,
    };
    renderInternal(containerId);
  };
})();
