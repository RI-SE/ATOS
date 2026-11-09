"""This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
"""

"""NiceGUI part of the map view: a resizable satellite map with scenario
trajectories and live, heading-rotated object markers, plus style settings.

The overlays are drawn by static/atos_map.js; this module creates the ui.leaflet
element, pushes MapDataSource snapshots to it at UI_RATE_S, and keeps the user's
style settings (persisted in app.storage.general).
"""

import copy
import json

from nicegui import app, ui

from .map_data import MapDataSource

STATIC_ROUTE = "/atos_gui_static"
MAP_JS_VERSION = "20261105-2"
UI_RATE_S = 0.1
SETTINGS_KEY = "atos_map_settings"

# Bright colours that stay readable on aerial imagery.
PALETTE = ["#ffd400", "#00e5ff", "#ff4fd8", "#7cfc00", "#ff8c00", "#ffffff", "#ff3b3b", "#9b8cff"]

DEFAULT_SETTINGS = {
    "trajectory": {"width": 3, "dash": "solid", "opacity": 0.9},
    "marker": {"shape": "arrow", "size": 32, "labels": True},
    "base": "satellite",
    "customUrl": "",
    "colorOverrides": {},
}

DASH_OPTIONS = {"solid": "Solid", "dashed": "Dashed", "dotted": "Dotted", "dashdot": "Dash-dot"}
SHAPE_OPTIONS = {"arrow": "Arrow", "chevron": "Chevron", "vehicle": "Vehicle", "circle": "Circle + heading"}
BASE_OPTIONS = {"satellite": "Satellite (Esri World Imagery)", "osm": "OpenStreetMap", "custom": "Custom tile URL"}


def load_settings() -> dict:
    settings = copy.deepcopy(DEFAULT_SETTINGS)
    stored = app.storage.general.get(SETTINGS_KEY) or {}
    for key, value in stored.items():
        if isinstance(value, dict) and isinstance(settings.get(key), dict):
            settings[key].update(value)
        elif key in settings:
            settings[key] = value
    return settings


def save_settings(settings: dict) -> None:
    app.storage.general[SETTINGS_KEY] = copy.deepcopy(settings)


class MapView:
    """One map view instance, created inside a page function (one per browser tab)."""

    def __init__(self, data: MapDataSource) -> None:
        self.data = data
        self.settings = load_settings()
        self._sent_traj_version = None
        self._sent_origin = None
        self._style_sent = False
        self._known_ids: list[int] = []
        self._color_inputs: dict[int, ui.color_input] = {}

        ui.add_head_html(f'<script src="{STATIC_ROUTE}/atos_map.js?v={MAP_JS_VERSION}"></script>')

        with ui.card().classes("w-full"):
            with ui.row().classes("w-full items-center gap-2"):
                ui.label("Map").classes("text-lg font-medium")
                self.status_label = ui.label().classes("text-sm text-grey-7")
                ui.space()
                self.follow_select = ui.select(
                    {"": "Follow: off"}, value="", on_change=self._on_follow
                ).props("dense outlined options-dense").classes("w-44")
                ui.button("Fit to scenario", icon="fit_screen", on_click=self._fit).props("flat dense")

            with ui.row().classes("w-full items-start gap-4 md:flex-nowrap"):
                # Drag the bottom-right corner to resize; atos_map.js reflows the map
                # and remembers the size in the browser.
                with ui.element("div").style(
                    "resize: both; overflow: hidden; width: 100%; max-width: 100%;"
                    "height: 60vh; min-height: 250px; min-width: 280px;"
                    "border-radius: 8px; border: 1px solid rgba(128,128,128,.4);"
                ):
                    self.map = ui.leaflet(
                        center=(0.0, 0.0), zoom=2, options={"maxZoom": 22}
                    ).style("width: 100%; height: 100%;")
                    # Drop NiceGUI's default OSM layer; base layers are managed in JS.
                    # Not initialized yet, so clearing the Python-side list is enough.
                    self.map.layers.clear()

                with ui.column().classes("w-full md:w-72 shrink-0"):
                    self._build_settings()

        ui.timer(UI_RATE_S, self._tick)

    # ---------------------------------------------------------------- settings

    def _build_settings(self) -> None:
        s = self.settings
        with ui.expansion("Map settings", icon="tune").classes("w-full").props("dense"):
            ui.label("Trajectory").classes("text-sm font-medium")
            ui.label().bind_text_from(s["trajectory"], "width", lambda v: f"Width: {v} px").classes("text-xs")
            ui.slider(min=1, max=12, step=1, value=s["trajectory"]["width"],
                      on_change=lambda e: self._set("trajectory", "width", e.value))
            ui.select(DASH_OPTIONS, label="Line style", value=s["trajectory"]["dash"],
                      on_change=lambda e: self._set("trajectory", "dash", e.value)).classes("w-full")
            ui.label().bind_text_from(s["trajectory"], "opacity", lambda v: f"Opacity: {round(v * 100)} %").classes("text-xs")
            ui.slider(min=0.1, max=1.0, step=0.05, value=s["trajectory"]["opacity"],
                      on_change=lambda e: self._set("trajectory", "opacity", e.value))

            ui.separator()
            ui.label("Object marker").classes("text-sm font-medium")
            ui.label().bind_text_from(s["marker"], "size", lambda v: f"Size: {v} px").classes("text-xs")
            ui.slider(min=12, max=96, step=2, value=s["marker"]["size"],
                      on_change=lambda e: self._set("marker", "size", e.value))
            ui.select(SHAPE_OPTIONS, label="Marker style", value=s["marker"]["shape"],
                      on_change=lambda e: self._set("marker", "shape", e.value)).classes("w-full")
            ui.switch("Show labels", value=s["marker"]["labels"],
                      on_change=lambda e: self._set("marker", "labels", e.value))

            ui.separator()
            ui.label("Object colours").classes("text-sm font-medium")
            self.colors_column = ui.column().classes("w-full gap-1")
            ui.button("Reset colours", on_click=self._reset_colors).props("flat dense size=sm")

            ui.separator()
            ui.label("Base map").classes("text-sm font-medium")
            ui.select(BASE_OPTIONS, value=s["base"],
                      on_change=lambda e: self._set(None, "base", e.value)).classes("w-full")
            custom_url = ui.input(
                "Custom tile URL", value=s["customUrl"], placeholder="https://host/{z}/{x}/{y}.png"
            ).classes("w-full").bind_visibility_from(s, "base", lambda v: v == "custom")
            # Apply on blur/enter only, so typing does not reload tiles per keystroke.
            for event in ("blur", "keydown.enter"):
                custom_url.on(event, lambda: self._set(None, "customUrl", (custom_url.value or "").strip()))
            ui.label("Satellite imagery: Esri World Imagery (no API key, attribution required).").classes(
                "text-xs text-grey-7")

    def _set(self, group: str | None, key: str, value) -> None:
        target = self.settings[group] if group else self.settings
        if target.get(key) == value:
            return
        target[key] = value
        save_settings(self.settings)
        self._push_style()

    def _object_colors(self) -> dict[str, str]:
        overrides = self.settings["colorOverrides"]
        return {
            str(object_id): overrides.get(str(object_id)) or PALETTE[index % len(PALETTE)]
            for index, object_id in enumerate(self._known_ids)
        }

    def _set_color(self, object_id: int, color: str) -> None:
        if not color:
            return
        overrides = self.settings["colorOverrides"]
        if overrides.get(str(object_id)) == color:
            return
        overrides[str(object_id)] = color
        save_settings(self.settings)
        self._push_style()

    def _reset_colors(self) -> None:
        self.settings["colorOverrides"] = {}
        save_settings(self.settings)
        colors = self._object_colors()
        for object_id, color_input in self._color_inputs.items():
            color_input.value = colors.get(str(object_id), "")
        self._push_style()

    def _rebuild_object_controls(self, names: dict[int, str]) -> None:
        colors = self._object_colors()
        self.colors_column.clear()
        self._color_inputs = {}
        with self.colors_column:
            if not self._known_ids:
                ui.label("No objects yet").classes("text-xs text-grey-7")
            for object_id in self._known_ids:
                self._color_inputs[object_id] = ui.color_input(
                    names.get(object_id, f"Object {object_id}"),
                    value=colors[str(object_id)],
                    on_change=lambda e, object_id=object_id: self._set_color(object_id, e.value),
                ).props("dense").classes("w-full")
        options = {"": "Follow: off"}
        options.update({str(i): f"Follow: {names.get(i, i)}" for i in self._known_ids})
        self.follow_select.set_options(options, value=self.follow_select.value if self.follow_select.value in options else "")

    def _style_payload(self) -> dict:
        s = self.settings
        return {
            "trajectory": s["trajectory"],
            "marker": s["marker"],
            "base": s["base"],
            "customUrl": s["customUrl"],
            "colors": self._object_colors(),
        }

    def _js(self, code: str) -> None:
        ui.run_javascript(f"window.atosMap ? {code} : false")

    def _push_style(self) -> None:
        if self.map.is_initialized:
            self._js(f"window.atosMap.setStyle({self.map.id}, {json.dumps(self._style_payload())})")

    # ------------------------------------------------------------------ updates

    def _fit(self) -> None:
        self._js(f"window.atosMap.fit({self.map.id})")

    def _on_follow(self, e) -> None:
        self._js(f"window.atosMap.follow({self.map.id}, {json.dumps(e.value or '')})")

    async def _tick(self) -> None:
        if not self.map.is_initialized:
            return
        if not self._style_sent:
            # Leaflet and atos_map.js load asynchronously; retry until both are there.
            ok = await ui.run_javascript(
                f"window.atosMap ? window.atosMap.setStyle({self.map.id}, {json.dumps(self._style_payload())}) : false"
            )
            self._style_sent = bool(ok)
            if not self._style_sent:
                return

        payload = self.data.snapshot(self._sent_traj_version)
        self.status_label.text = payload["status"]

        ids = self.data.object_ids
        if ids != self._known_ids:
            self._known_ids = ids
            self._rebuild_object_controls(dict(self.data.object_names))
            self._push_style()

        if payload["origin"] != self._sent_origin:
            if self._sent_origin is not None:
                self._js(f"window.atosMap.reset({self.map.id})")
            self._sent_origin = payload["origin"]
            payload["trajectories"] = payload.get("trajectories") or self.data.snapshot(None)["trajectories"]

        self._js(f"window.atosMap.update({self.map.id}, {json.dumps(payload)})")
        self._sent_traj_version = payload["traj_version"]
