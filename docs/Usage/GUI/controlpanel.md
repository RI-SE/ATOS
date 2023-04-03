# Control Panel
Control Panel is used to run ATOS without any further visualisation. In order to start Control Panel, start ATOS and then open your web browser and go to `ATOS_host_address:3000`, where `ATOS_host_address` is the address to the server that ATOS is hosted on. If you run ATOS on your own computer and want to use Control Panel on the same computer, you would go to `localhost:3000`.

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