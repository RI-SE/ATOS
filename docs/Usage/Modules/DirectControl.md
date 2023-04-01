# DirectControl
A module for sending control signals to test equipment.
## About the module
This module is used for inputting control signals, for example steering and throttle, into ATOS. An application could be to control equipment via a game controller or to pass data from other systems directly to objects. The signal data is not transmitted directly to the objects since other modules are responsible for ensuring correct test object state before sending such data. Instead, the data is re-sent on a ROS2 topic.
