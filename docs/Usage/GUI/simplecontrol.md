# Simple Control
Simple Control is used to run ATOS without any further visualisation. In order to start Simple Control, start ATOS and then open your web browser and go to `ATOS_host_address:3000`, where `ATOS_host_address` is the address to the server that ATOS is hosted on. If you run ATOS on your own computer and want to use Simple Control on the same computer, you would go to `localhost:3000`.

When opening Simple Control there will be a set of buttons, [ObjectControl](../Modules/ObjectControl.md) state, and connection status to ATOS. The buttons are:

- **Init** - Initialise a test, loads things such as the scenario and which objects to use.
- **Connect** - Connects to the objects that were initialised.
- **Arm** - Arms the objects, meaning that they are ready to start the test.
- **Start** - Starts the test.
- **Reset Test** - Reset the loaded test
- **All clear** - Clears the test.
- **Disarm** - Disarms the objects, meaning that they are no longer ready to start the test.
- **Abort** - Stops the test, for instance if something goes wrong or the test needs to be stopped for some reason.