# About ATOS

Originally, ATOS was developed as part of several research projects funded by the Swedish Agency for Innovation, Vinnova. The main objective of these projects was to establish a unique platform for testing and research on automated vehicles and associated communication.  

As of April 2023, ATOS has been released as Open Source. ATOS is based on ROS 2 and is implemented mostly in C++. ATOS Features a module structure to seamlessly enable the addition of new features as modules, with respect to the unknown prerequisites for future testing needs. 

The core functionality of ATOS implements the standard ISO 22133, and its main task involves test object monitoring and control for active safety and automated vehicle testing in complex and realistic scenarios. 

 ATOS is part of an ecosystem of Open-Source software: 

* ISO 22133, library containing message (de)serialization 
* IsoObject, implementation of an ISO 22133 test object 
* ATOS, implementation of an ISO 22133-compliant control center 

**Vision**: The vision of ATOS is to facilitate cutting-edge scenario-based AV-testing as an essential part of the V&V toolchain of most major OEMs. 

**Purpose**: ATOS is being developed to provide researchers and industry actors with software that tackles the current and future need for performing complex scenario testing with test equipment from multiple vendors. Complex scenario testing involves mixed-reality and simulated testing with virtual test participants, testing with multiple participants, and high precision test cases, test cases involving advanced connectivity, as well as any combination thereof. 

Today, ATOS contains functionality for: 

*  Sending and receiving ISO 22133-messages to/from test objects, including trajectories (waypoints), heartbeat messages, state-machine altering messages, and more. 
* Monitoring the current state of test objects. 
* Controlling the state of a test through a graphical user interface. 
* Executing scenarios specified in OpenSCENARIO/DRIVE, made possible through integration with the simulator esmini.  
* Communicating with simulators via Open Simulation Interface. 
* Sending V2X messages to test objects. 
* Generating “back-to-start" trajectories, allowing test objects to return to their initial position, enabling a higher degree of automation. 
* Enabling direct control of test objects.  
* Time-synchronized logging features that detail the position of each test object during the test. 
* Pointcloud integration for visualization. 

The scope of ATOS development going forward includes but is not limited to: 
* Increased interoperability with simulation software. 
* Increased testing dynamism and support for ASAM OpenDRIVE/SCENARIO. 
* Control over infrastructure components such as V2X or cellular infrastructure. 
* Interfacing with the outside world through external data to emulate traffic message brokers (Interchange nodes/C-roads). 
* Improved and simplified user experience. 
* Adhering to various safety standards.  