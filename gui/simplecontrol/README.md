# Simple Control GUI
TODO: add pictures.


Simple Control is a GUI for use with the ATOS server. It is used to control the stages of the test by issuing commands to the ATOS server.


## Local usage
To use the gui locally on the same computer as ATOS, simply start ATOS and go to http://localhost:3000.

### Connecting with https
By default, a self-signed certificate is generated for you at startup. This certificate/key-pair is used to encrypt https as well as websockets traffic. 

Go to https://localhost:3443 and accept the certificate warning to connect to the gui.

## Integration with foxglove

Prerequisites: google chrome or chromium.

Simple Control can be used as a panel in foxglove studio.
To use the web application follow these steps: go to https://studio.foxglove.dev and click the plus-symbol. Chose "open connection" -> "Rosbridge", enter wss://localhost:9090 and click "open".

Go to the Layouts tab and click "import layout". Choose a layout from the plugins/foxglove/ directory in the ATOS repository. Drag-and-drop the .foxe files from the same directory into the web browser.

You are now ready to use the ATOS GUI in foxglove studio.

## Connecting to a remote ATOS server

To connect to a remote ATOS server, you have to accept the following certificates in your browser: https://hostname.of.atos:3443, https://hostname.of.atos:9090.

You can now follow the steps above to connect to the remote server either using the standalone Simple Control, or as a panel in foxglove studio.

## FAQs

TODO