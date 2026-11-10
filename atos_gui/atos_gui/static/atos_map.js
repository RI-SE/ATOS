/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Map view overlays for the ATOS control panel. Works on the Leaflet map created
 * by NiceGUI's ui.leaflet (reached through getElement(id).map, Leaflet as window.L).
 *
 * All positions arrive as WGS84 lat/lon and headings as geodesic bearings
 * (degrees clockwise from true north), already converted from the ATOS local
 * frame in Python (atos_gui/mapview/geo.py). Web Mercator is conformal and the
 * map is north-up, so rotating a north-pointing icon clockwise by the bearing is
 * geographically correct.
 */
(function () {
  "use strict";

  // v2: width is stored as a percentage of the available width. The old key held
  // absolute pixels, which also captured window shrinks and pinned the map small.
  const SIZE_STORAGE_KEY = "atosMapSize2";

  const BASE_LAYERS = {
    satellite: {
      url: "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
      options: {
        maxNativeZoom: 19,
        maxZoom: 22,
        attribution:
          "Tiles &copy; Esri &mdash; Source: Esri, Maxar, Earthstar Geographics, and the GIS User Community",
      },
    },
    osm: {
      url: "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
      options: {
        maxNativeZoom: 19,
        maxZoom: 22,
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
      },
    },
  };

  // Every shape points north (up) in a 100x100 box centred on the object position.
  const OUTLINE = "#111";
  const SHAPES = {
    arrow: (c) =>
      `<path d="M50 4 L88 92 L50 72 L12 92 Z" fill="${c}" stroke="${OUTLINE}" stroke-width="5" stroke-linejoin="round"/>`,
    chevron: (c) =>
      `<path d="M14 80 L50 16 L86 80" fill="none" stroke="${OUTLINE}" stroke-width="26" stroke-linecap="round" stroke-linejoin="round"/>` +
      `<path d="M14 80 L50 16 L86 80" fill="none" stroke="${c}" stroke-width="15" stroke-linecap="round" stroke-linejoin="round"/>`,
    vehicle: (c) =>
      `<rect x="27" y="5" width="46" height="90" rx="12" fill="${c}" stroke="${OUTLINE}" stroke-width="5"/>` +
      `<path d="M33 30 Q50 21 67 30 L64 43 Q50 37 36 43 Z" fill="${OUTLINE}" opacity="0.75"/>` +
      `<rect x="34" y="76" width="32" height="10" rx="3" fill="${OUTLINE}" opacity="0.45"/>`,
    circle: (c) =>
      `<line x1="50" y1="50" x2="50" y2="-22" stroke="${OUTLINE}" stroke-width="13" stroke-linecap="round"/>` +
      `<line x1="50" y1="50" x2="50" y2="-22" stroke="${c}" stroke-width="6" stroke-linecap="round"/>` +
      `<circle cx="50" cy="50" r="26" fill="${c}" stroke="${OUTLINE}" stroke-width="5"/>`,
  };
  const STALE_COLOR = "#9e9e9e";

  // Esri imagery depth varies by location (e.g. zoom 18 at AstaZero); deeper tiles are a
  // "Map data not yet available" placeholder. The tilemap service says where imagery exists.
  const ESRI_TILEMAP = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tilemap";
  const IMAGERY_PROBE_ZOOMS = [19, 18, 17, 16, 15, 14, 13, 12];
  const IMAGERY_PROBE_TIMEOUT_MS = 3000;
  const FIT_FALLBACK_MAX_ZOOM = 18; // satellite, while/if the probe gives no answer
  const FIT_PADDING_PX = 40;

  function dashArray(dash, weight) {
    const w = Math.max(1, weight);
    switch (dash) {
      case "dashed":
        return `${3 * w} ${2 * w}`;
      case "dotted":
        return `1 ${2 * w}`;
      case "dashdot":
        return `${4 * w} ${2 * w} 1 ${2 * w}`;
      default:
        return null;
    }
  }

  function escapeHtml(value) {
    return String(value ?? "")
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;");
  }

  function storageGet(key) {
    try {
      return JSON.parse(window.localStorage.getItem(key));
    } catch (e) {
      return null;
    }
  }

  function storageSet(key, value) {
    try {
      window.localStorage.setItem(key, JSON.stringify(value));
    } catch (e) {
      /* private mode or blocked storage: size is just not remembered */
    }
  }

  function getState(elementId) {
    const comp = typeof getElement === "function" ? getElement(elementId) : null;
    if (!comp || !comp.map || !window.L) return null;
    if (!comp._atos) {
      comp._atos = {
        map: comp.map,
        style: null,
        base: null,
        baseKey: null,
        trajectories: {}, // id -> {group, line, start, end, name}
        objects: {}, // id -> {marker, shapeKey, size, color}
        lastObjects: [],
        fitted: false,
        centeredOnOrigin: false,
        follow: null,
        origin: null, // [lat, lon] of the scenario's local frame origin
        originMarker: null,
        imageryMaxZoom: null, // deepest Esri zoom with imagery over the scenario
        generation: 0, // bumped on reset, so late probe results are ignored
      };
      watchResize(comp);
      // Composite the map as one layer. Otherwise Chrome snaps each tile to device pixels
      // on its own, and on HiDPI screens a map at a fractional page offset shows hairline
      // gaps between tile rows.
      comp.$el.style.willChange = "transform";
    }
    return comp._atos;
  }

  // Only a drag of the resize handle is remembered. Window resizes just reflow the
  // map: the width stays a percentage, so the map grows again with the window.
  function watchResize(comp) {
    const wrapper = comp.$el.parentElement;
    if (!wrapper) return;
    const saved = storageGet(SIZE_STORAGE_KEY);
    if (saved && saved.h > 0) wrapper.style.height = `${saved.h}px`;
    if (saved && saved.wPct > 0) wrapper.style.width = `${Math.min(100, saved.wPct)}%`;

    const observer = new ResizeObserver(() => comp.map.invalidateSize({ pan: false }));
    observer.observe(wrapper);

    // The resize grip belongs to the wrapper itself; the map covers everything else.
    const onPointerDown = (ev) => {
      if (ev.target !== wrapper) return;
      const startW = wrapper.offsetWidth;
      const startH = wrapper.offsetHeight;
      window.addEventListener(
        "pointerup",
        () => {
          const w = wrapper.offsetWidth;
          const h = wrapper.offsetHeight;
          if (w === startW && h === startH) return;
          const avail = wrapper.parentElement ? wrapper.parentElement.clientWidth : 0;
          const wPct = avail > 0 ? Math.min(100, (w / avail) * 100) : 100;
          // Replace the browser's pixel width with a relative one.
          wrapper.style.width = `${wPct}%`;
          storageSet(SIZE_STORAGE_KEY, { wPct, h });
        },
        { once: true },
      );
    };
    wrapper.addEventListener("pointerdown", onPointerDown);
    comp.map.on("unload", () => {
      observer.disconnect();
      wrapper.removeEventListener("pointerdown", onPointerDown);
    });
  }

  function applyBaseLayer(state) {
    const style = state.style;
    let key = style.base;
    let def = BASE_LAYERS[key];
    if (key === "custom") {
      def = style.customUrl
        ? { url: style.customUrl, options: { maxZoom: 22, maxNativeZoom: style.customMaxNativeZoom || 19 } }
        : BASE_LAYERS.satellite;
      key = `custom:${style.customUrl}`;
    }
    if (!def) {
      def = BASE_LAYERS.satellite;
      key = "satellite";
    }
    if (state.baseKey === key) return;
    if (state.base) state.map.removeLayer(state.base);
    const options = Object.assign({}, def.options);
    if (key === "satellite" && state.imageryMaxZoom) options.maxNativeZoom = state.imageryMaxZoom;
    state.base = window.L.tileLayer(def.url, options).addTo(state.map);
    state.base.bringToBack();
    state.baseKey = key;
  }

  /** Deepest zoom the active base layer has real tiles for; fits never go closer. */
  function nativeMaxZoom(state) {
    if (state.baseKey === "satellite") return state.imageryMaxZoom || FIT_FALLBACK_MAX_ZOOM;
    return (state.base && state.base.options.maxNativeZoom) || 19;
  }

  async function tileHasImagery(z, x, y) {
    const response = await fetch(`${ESRI_TILEMAP}/${z}/${y}/${x}/1/1`);
    if (!response.ok) throw new Error(`tilemap HTTP ${response.status}`);
    const json = await response.json();
    return Array.isArray(json.data) && json.data[0] === 1;
  }

  /** Deepest zoom where the scenario's centre and corners all have Esri imagery. */
  async function probeImageryMaxZoom(map, bounds) {
    const points = [bounds.getCenter(), bounds.getNorthWest(), bounds.getNorthEast(), bounds.getSouthWest(), bounds.getSouthEast()];
    for (const z of IMAGERY_PROBE_ZOOMS) {
      const tiles = new Map();
      for (const p of points) {
        const t = map.project(p, z).divideBy(256).floor();
        tiles.set(`${t.x},${t.y}`, t);
      }
      const results = await Promise.all([...tiles.values()].map((t) => tileHasImagery(z, t.x, t.y)));
      if (results.every(Boolean)) return z;
    }
    return IMAGERY_PROBE_ZOOMS[IMAGERY_PROBE_ZOOMS.length - 1];
  }

  /**
   * Find the imagery depth over `bounds` and stop the satellite layer from requesting
   * deeper tiles: past it Leaflet scales up the deepest real tiles instead. Resolves
   * (possibly without a result) within IMAGERY_PROBE_TIMEOUT_MS.
   */
  function updateImageryMaxZoom(state, bounds) {
    const generation = state.generation;
    const probe = probeImageryMaxZoom(state.map, bounds)
      .then((z) => {
        if (generation !== state.generation) return;
        state.imageryMaxZoom = z;
        if (state.baseKey === "satellite" && state.base && state.base.options.maxNativeZoom !== z) {
          state.base.options.maxNativeZoom = z;
          state.base.redraw();
        }
      })
      .catch(() => {
        /* offline or blocked: keep the defaults */
      });
    const timeout = new Promise((resolve) => setTimeout(resolve, IMAGERY_PROBE_TIMEOUT_MS));
    return Promise.race([probe, timeout]);
  }

  function colorFor(state, id) {
    const colors = (state.style && state.style.colors) || {};
    return colors[id] || colors[String(id)] || "#ffd400";
  }

  function styleTrajectory(state, id, traj) {
    const s = state.style.trajectory;
    const color = colorFor(state, id);
    traj.line.setStyle({
      color,
      weight: s.width,
      opacity: s.opacity,
      dashArray: dashArray(s.dash, s.width),
      lineCap: s.dash === "dotted" ? "round" : "butt",
    });
    const r = Math.max(3, s.width + 2);
    traj.start.setStyle({ radius: r, color: OUTLINE, weight: 2, fillColor: color, fillOpacity: 1, opacity: s.opacity });
    traj.start.setRadius(r);
    traj.end.setStyle({ radius: r, color, weight: Math.max(2, s.width / 2), fillOpacity: 0, opacity: s.opacity });
    traj.end.setRadius(r);
  }

  function setTrajectories(state, trajectories) {
    const L = window.L;
    const seen = new Set();
    for (const t of trajectories) {
      seen.add(String(t.id));
      let traj = state.trajectories[t.id];
      if (!traj) {
        const line = L.polyline(t.latlngs, { interactive: true });
        const start = L.circleMarker(t.latlngs[0], { interactive: false });
        const end = L.circleMarker(t.latlngs[t.latlngs.length - 1], { interactive: false });
        const group = L.layerGroup([line, start, end]).addTo(state.map);
        traj = { group, line, start, end };
        state.trajectories[t.id] = traj;
      } else {
        traj.line.setLatLngs(t.latlngs);
        traj.start.setLatLng(t.latlngs[0]);
        traj.end.setLatLng(t.latlngs[t.latlngs.length - 1]);
      }
      traj.name = t.name;
      traj.line.bindTooltip(escapeHtml(t.name), { sticky: true });
      styleTrajectory(state, t.id, traj);
    }
    for (const id of Object.keys(state.trajectories)) {
      if (!seen.has(String(id))) {
        state.map.removeLayer(state.trajectories[id].group);
        delete state.trajectories[id];
      }
    }
    if (!state.fitted && trajectories.length > 0) {
      state.fitted = true;
      const generation = state.generation;
      const bounds = scenarioBounds(state);
      updateImageryMaxZoom(state, bounds).then(() => {
        if (generation === state.generation) fitScenario(state);
      });
    }
  }

  function markerIcon(state, color, size) {
    const shape = SHAPES[state.style.marker.shape] || SHAPES.arrow;
    const html =
      `<div class="atos-obj-rot" style="width:${size}px;height:${size}px;transform-origin:50% 50%;">` +
      `<svg viewBox="0 0 100 100" width="${size}" height="${size}" style="overflow:visible;filter:drop-shadow(0 0 2px rgba(0,0,0,.6))">` +
      shape(color) +
      "</svg></div>";
    return window.L.divIcon({ className: "atos-obj-icon", html, iconSize: [size, size], iconAnchor: [size / 2, size / 2] });
  }

  function tooltipText(o) {
    return `${escapeHtml(o.name)}<br>${o.speed.toFixed(1)} m/s, heading ${Math.round(o.bearing)}&deg;` +
      (o.stale ? "<br><i>no recent data</i>" : "");
  }

  function setObjects(state, objects) {
    const L = window.L;
    const ms = state.style.marker;
    const seen = new Set();
    for (const o of objects) {
      seen.add(String(o.id));
      const color = o.stale ? STALE_COLOR : colorFor(state, o.id);
      const shapeKey = `${ms.shape}|${ms.size}|${color}|${ms.labels}`;
      let obj = state.objects[o.id];
      if (!obj) {
        const marker = L.marker([o.lat, o.lon], { keyboard: false, zIndexOffset: 1000 }).addTo(state.map);
        obj = { marker, shapeKey: null };
        state.objects[o.id] = obj;
      } else {
        obj.marker.setLatLng([o.lat, o.lon]);
      }
      if (obj.shapeKey !== shapeKey) {
        obj.marker.setIcon(markerIcon(state, color, ms.size));
        obj.marker.unbindTooltip();
        obj.marker.bindTooltip("", {
          permanent: !!ms.labels,
          direction: "right",
          offset: [ms.size / 2, 0],
          className: "atos-obj-label",
        });
        obj.shapeKey = shapeKey;
      }
      obj.marker.setOpacity(o.stale ? 0.7 : 1);
      obj.marker.setTooltipContent(ms.labels ? escapeHtml(o.name) : tooltipText(o));
      const el = obj.marker.getElement();
      const rot = el && el.querySelector(".atos-obj-rot");
      if (rot) rot.style.transform = `rotate(${o.bearing}deg)`;
    }
    for (const id of Object.keys(state.objects)) {
      if (!seen.has(String(id))) {
        state.map.removeLayer(state.objects[id].marker);
        delete state.objects[id];
      }
    }
    state.lastObjects = objects;
    if (state.follow !== null && state.follow !== undefined) {
      const target = objects.find((o) => String(o.id) === String(state.follow));
      if (target) state.map.panTo([target.lat, target.lon], { animate: false });
    }
  }

  // Crosshair at the local frame origin with its x (east) and y (north) axes.
  const ORIGIN_ICON_SIZE = 56;
  function originIcon() {
    const n = ORIGIN_ICON_SIZE;
    const c = n / 2;
    const axis = (x2, y2, color) =>
      `<line x1="${c}" y1="${c}" x2="${x2}" y2="${y2}" stroke="${OUTLINE}" stroke-width="5" stroke-linecap="round"/>` +
      `<line x1="${c}" y1="${c}" x2="${x2}" y2="${y2}" stroke="${color}" stroke-width="2.5" stroke-linecap="round"/>`;
    const html =
      `<svg viewBox="0 0 ${n} ${n}" width="${n}" height="${n}" style="overflow:visible;filter:drop-shadow(0 0 2px rgba(0,0,0,.6))">` +
      axis(n - 4, c, "#ff3b3b") +
      axis(c, 4, "#7cfc00") +
      `<path d="M${n - 4} ${c} l-7 -4 v8 z" fill="#ff3b3b" stroke="${OUTLINE}" stroke-width="1"/>` +
      `<path d="M${c} 4 l-4 7 h8 z" fill="#7cfc00" stroke="${OUTLINE}" stroke-width="1"/>` +
      `<text x="${n - 6}" y="${c + 13}" font-size="11" font-weight="700" fill="#fff" stroke="${OUTLINE}" stroke-width="3" paint-order="stroke" text-anchor="middle" font-family="sans-serif">x</text>` +
      `<text x="${c + 11}" y="11" font-size="11" font-weight="700" fill="#fff" stroke="${OUTLINE}" stroke-width="3" paint-order="stroke" text-anchor="middle" font-family="sans-serif">y</text>` +
      `<circle cx="${c}" cy="${c}" r="7" fill="none" stroke="${OUTLINE}" stroke-width="4"/>` +
      `<circle cx="${c}" cy="${c}" r="7" fill="none" stroke="#fff" stroke-width="2"/>` +
      `<circle cx="${c}" cy="${c}" r="1.8" fill="#fff" stroke="${OUTLINE}" stroke-width="1"/>` +
      `</svg>`;
    return window.L.divIcon({ className: "atos-obj-icon", html, iconSize: [n, n], iconAnchor: [c, c] });
  }

  /** Show, move or remove the origin marker to match the setting and the latest origin. */
  function syncOrigin(state) {
    const show = !!(state.style && state.style.showOrigin) && !!state.origin;
    if (!show) {
      if (state.originMarker) state.map.removeLayer(state.originMarker);
      state.originMarker = null;
      return;
    }
    const [lat, lon] = state.origin;
    if (!state.originMarker) {
      state.originMarker = window.L.marker([lat, lon], { icon: originIcon(), keyboard: false, zIndexOffset: 500 }).addTo(
        state.map,
      );
      state.originMarker.bindTooltip("", { direction: "top", offset: [0, -ORIGIN_ICON_SIZE / 2] });
    } else {
      state.originMarker.setLatLng([lat, lon]);
    }
    state.originMarker.setTooltipContent(`Scenario origin<br>${lat.toFixed(7)}, ${lon.toFixed(7)}`);
  }

  function scenarioBounds(state) {
    let bounds = null;
    for (const traj of Object.values(state.trajectories)) {
      const b = traj.line.getBounds();
      if (b.isValid()) bounds = bounds ? bounds.extend(b) : window.L.latLngBounds(b.getSouthWest(), b.getNorthEast());
    }
    for (const obj of Object.values(state.objects)) {
      const ll = obj.marker.getLatLng();
      bounds = bounds ? bounds.extend(ll) : window.L.latLngBounds(ll, ll);
    }
    return bounds;
  }

  function fitScenario(state) {
    const bounds = scenarioBounds(state);
    if (bounds) {
      state.map.fitBounds(bounds, { padding: [FIT_PADDING_PX, FIT_PADDING_PX], maxZoom: nativeMaxZoom(state) });
    }
    return !!bounds;
  }

  function injectCss() {
    if (document.getElementById("atos-map-css")) return;
    const css = document.createElement("style");
    css.id = "atos-map-css";
    css.textContent =
      ".atos-obj-icon{background:transparent;border:none;}" +
      ".atos-obj-label{font-weight:600;}";
    document.head.appendChild(css);
  }

  window.atosMap = {
    /** Apply all user style settings; safe to call repeatedly. Returns false until the map exists. */
    setStyle(elementId, style) {
      const state = getState(elementId);
      if (!state) return false;
      injectCss();
      state.style = style;
      applyBaseLayer(state);
      for (const [id, traj] of Object.entries(state.trajectories)) styleTrajectory(state, id, traj);
      for (const obj of Object.values(state.objects)) obj.shapeKey = null; // rebuild icons
      setObjects(state, state.lastObjects);
      syncOrigin(state);
      return true;
    },

    /** Apply one data snapshot from Python (MapDataSource.snapshot). */
    update(elementId, payload) {
      const state = getState(elementId);
      if (!state || !state.style) return false;
      if (payload.origin && !state.centeredOnOrigin) {
        state.centeredOnOrigin = true;
        if (!state.fitted) state.map.setView(payload.origin, Math.min(17, nativeMaxZoom(state)));
      }
      const origin = payload.origin || null;
      if (String(origin) !== String(state.origin)) {
        state.origin = origin;
        syncOrigin(state);
      }
      if (payload.trajectories) setTrajectories(state, payload.trajectories);
      if (payload.objects) setObjects(state, payload.objects);
      return true;
    },

    fit(elementId) {
      const state = getState(elementId);
      return state ? fitScenario(state) : false;
    },

    follow(elementId, objectId) {
      const state = getState(elementId);
      if (!state) return false;
      state.follow = objectId === "" ? null : objectId;
      if (state.follow !== null) setObjects(state, state.lastObjects);
      return true;
    },

    /** Forget overlays, e.g. after a new scenario was initialized. */
    reset(elementId) {
      const state = getState(elementId);
      if (!state) return false;
      for (const traj of Object.values(state.trajectories)) state.map.removeLayer(traj.group);
      for (const obj of Object.values(state.objects)) state.map.removeLayer(obj.marker);
      state.trajectories = {};
      state.objects = {};
      state.lastObjects = [];
      state.origin = null;
      syncOrigin(state);
      state.fitted = false;
      state.centeredOnOrigin = false;
      state.imageryMaxZoom = null; // the new scenario may be somewhere else
      state.generation += 1;
      return true;
    },
  };
})();
