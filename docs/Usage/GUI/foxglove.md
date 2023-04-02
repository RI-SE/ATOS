# Foxglove Studio
If you want to run ATOS with more visualisation you can use [Foxglove Studio](https://github.com/foxglove/studio). To get started, visit [studio.foxglove.dev](https://studio.foxglove.dev).

#### Connect to ATOS
In order to connect to ATOS in Foxglove Studio, ATOS first needs to be running. Then, in Foxglove Studio, go to `Data source -> New connection -> Open connection -> Rosbridge` and enter `wss://ATOS_host_address:9090`.

#### Extensions
In order for everything to be displayed correctly, some extensions needs to be added. The extensions for ATOS are found [here](https://github.com/RI-SE/ATOS/tree/dev/plugins/foxglove) with the file type `.foxe`. In Foxglove Studio, go to `Extensions` and then drag and drop the extensions.

#### Changing layout
The premade layouts for ATOS are located [here](https://github.com/RI-SE/ATOS/tree/dev/plugins/foxglove). To add a layout in Foxglove Studio, go to `Layouts -> Import layout` and select one of the layouts for ATOS.

#### Panels in Foxglove Studio
The panels used in layouts for ATOS are Simple Control, Map, and 3D. To change settings for the panels, press `Settings` that are located at the top right of each panel.

Simple Control - This is a panel for Simple Control as described previously. It has the following settings:

- `ATOS hostname` - The host address of ATOS.
- `ATOS port` - The port to connect to.
- `Protocol` - Either `https` or `http`

Map - This is a panel that displays a map with objects and their paths. If you want a custom map, in settings change `Tile layer` to custom and enter the address to your tile server. In the settings you can also turn on or off which objects and paths that should be displayed, and also change their colours.

3D - This is a panel for 3D representation of a scenario. Objects, paths, and site scans can be visualised here. In the settings menu, the objects, paths, and site scans can be turned on or off. They can also change size, colour, and shape.