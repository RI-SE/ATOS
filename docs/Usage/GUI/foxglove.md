# Foxglove Studio
If you want to run ATOS with extra visualisation you can use [Foxglove Studio](https://app.foxglove.dev/). 

## Connect to ATOS
# Secure mode
If you are running foxbridge in secure mode, i.e. `ros2 launch atos atos.launch.py insecure:=False``, you need to accept the websocket certificate. Do this by going to https://ATOS_host_address:8765 in your browser (chrome) and accepting the self-signed certificate, which is automatically generated on startup of ATOS.
In order to connect to ATOS in Foxglove Studio, ATOS first needs to be running. Then, in Foxglove Studio, go to `Data source -> New connection -> Open connection -> FoxGlove Websocket` and enter `wss://ATOS_host_address:8765`.
# Insecure mode
If you are running foxbridge in insecure mode, i.e. `ros2 launch atos atos.launch.py insecure:=True``, you can connect to ATOS in Foxglove Studio by going to `Data source -> New connection -> Open connection -> FoxGlove Websocket` and enter `ws://ATOS_host_address:8765`.

!!! note
    The browser version of Foxglove at app.foxglove.dev is served over HTTPS, and browsers only allow it to open unencrypted `ws://` connections to `localhost`. If ATOS runs on another machine, use secure mode (`wss://`) or the Foxglove desktop app.

## Extensions
In order for everything to be displayed correctly, some extensions needs to be added. The extensions for ATOS are found [here](https://github.com/RI-SE/ATOS/tree/dev/plugins/foxglove) with the file type `.foxe`. In Foxglove Studio, go to `Extensions` and then drag and drop the extensions.

## Changing layout
The premade layouts for ATOS are located [here](https://github.com/RI-SE/ATOS/tree/dev/plugins/foxglove). To add a layout in Foxglove Studio, go to `Layouts -> Import layout` and select a layout for ATOS.

## Panels in Foxglove Studio
The panels used in layouts for ATOS are Map and 3D. To change settings for the panels, press `Settings` that are located at the top right of each panel.

### Map - This is a panel that displays a map with objects and their paths.
 If you want a custom map, in settings change `Tile layer` to custom and enter the address to your tile server. In the settings menu you can also turn on or off which objects and paths that should be displayed, and also change their colours.

### 3D - This is a panel for 3D representation of a scenario. Objects, paths, and site scans can be visualised here. 
In the settings menu, the objects, paths, and site scans can be turned on or off by pressing the eye icon next the topic. In the settings menu you can also change size, colour, and shape of the points.