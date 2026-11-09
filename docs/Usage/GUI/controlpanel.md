# Control Panel
Control Panel is used to run ATOS without any further visualisation. In order to start Control Panel, start ATOS and then open your web browser and go to `ATOS_host_address:3000`, where `ATOS_host_address` is the address to the server that ATOS is hosted on. If you run ATOS on your own computer and want to use Control Panel on the same computer, you would go to `localhost:3000`.

## Requirements
The GUI is built with [NiceGUI](https://nicegui.io). The installation script installs NiceGUI 4.x on Python 3.11 or newer (ROS 2 Jazzy and newer) and NiceGUI 3.19.x on Python 3.10 (ROS 2 Humble). The GUI is not available on ROS 2 Foxy (Python 3.8); ATOS still launches there, without the GUI.

## Running over HTTPS/TLS
To connect to a remote ATOS server, you have to accept the following certificates in your browser: `https://hostname.of.atos:3443`, `https://hostname.of.atos:9090`.
You can now follow the steps above to connect to the remote server either using the standalone Control Panel, or as a panel in [FoxGlove Studio](./foxglove.md).

## Features

When opening Control Panel there will be a set of buttons, A state value determined by [ObjectControl](../Modules/ObjectControl.md), and the connection status to the atos gui server. The buttons are:

- **Init** - Initialise a test, loads things such as the scenario and which objects to use.
- **Connect** - Connects to the objects that were initialised.
- **Arm** - Arms the objects, meaning that they are ready to start the test.
- **Start** - Starts the test.
- **Reset Test** - Reset the loaded test
- **All clear** - Clears the test.
- **Disarm** - Disarms the objects, meaning that they are no longer ready to start the test.
- **Abort** - Stops the test, for instance if something goes wrong or the test needs to be stopped for some reason.

## Map view
Below the buttons, a satellite map shows the trajectories of the loaded scenario and, once objects are connected, a marker for each object, rotated by its reported heading.

- **When data appears**: trajectories and the test origin are loaded once ObjectControl reaches `INITIALIZED` (after **Init**), and reloaded on each new Init. Object markers are shown for the objects in `/atos/connected_object_ids`. A marker turns grey when no monitor data has arrived for 2 seconds.
- **Geographic accuracy**: ATOS works in a local transverse Mercator frame centred on the test origin (from the OpenDRIVE `<geoReference>`). The map converts trajectories (`/atos/object_<id>/path`) and monitor poses (`/atos/object_monitor`) with that same projection, using PROJ via `pyproj`, so overlays are exact. Headings, which are counter-clockwise from east in the local frame, are converted to true bearings, including meridian convergence. If the scenario has no geo-reference, the map says so and draws nothing.
- **Resizing**: drag the bottom-right corner of the map. The browser remembers the size.
- **Settings** (*Map settings*, saved on the ATOS host):
    - Trajectory width, line style (solid, dashed, dotted, dash-dot) and opacity.
    - Marker size and style (arrow, chevron, vehicle, circle with heading line), and whether labels are shown.
    - A colour per object.
- **Base map**: satellite imagery is Esri World Imagery, which needs no API key (attribution is shown on the map). OpenStreetMap or a custom tile URL (e.g. a local tile server for sites without internet) can be selected instead. Overlays are drawn even if no tiles load. How deep Esri's imagery goes varies by location (zoom 18 at AstaZero, for example). After each Init the map asks Esri's tile-map service where imagery exists over the scenario. Fitting never zooms closer than that, and when you zoom in further by hand, the deepest real imagery is scaled up instead of showing "Map data not yet available" tiles.
- **Fit to scenario / Follow**: zoom to all trajectories, or keep one object centred.

In relative-kinematics (anchor) mode, the map still shows everything in the test origin frame.