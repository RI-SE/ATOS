Last edited 15/10- 2019

Contents
==============================
[v.0.4.0 December Server-Snapshot](#December-release-v040---serversnapshot)

[v.0.3.0 Trigger and Action](#maestro-v030-trigger-and-action)

[v.0.2.0 July Server-snapshot](#july-release-v020---serversnapshot)

[v.0.1.0 Drottning Kristina](drottning-kristina-v010)

December release v0.4.0 - ServerSnapshot  
==============================

This version is compatible with Maestro-Tools v0.4.0 and Util v0.4.0

New Features
-------

1. Supervisor now is its own c++ module.
2. Supervisor now sends disarm if any object is not within a short distance to its starting position and somewhat aligned to the trajectory
3. Supervisor now sends disconnect if any trajectory files do not match its standards (right now, its standards only state that the files should be parseable)
4. Supervisor now sends abort if an object exits a permitted geofence or enters a forbidden one

Enhancements 
-------
1. Implemented testing utility which allows starting an executable and polling its status from Python
2. Implemented test which starts and kills core executable, failing if one or more modules crash before the killing commences
3. Implemented testing utility which emulates external MSCP communication to system control
4. Implemented testing utility which checks availability of ports
5. Implemented testing utility which opens and parses a trajectory file
6. Implemented test which connects via MSCP and sends initializeScenario, after which it disconnects. It fails if the MSCP connection fails or the server crashes
7. Implemented test which steps through the "normal" test procedure i.e. connect MSCP, upload file, init, connect, arm, start, abort, reset


Fixed bugs
-------
- MQ priorities were wrong: highest priority was lowest. This has been remedied.
- Fixed error in C-ITS control DENM timestamps
- Fixed mismatch with ISO HEAB, MONR, STRT, TRAJ based on 4a workshop

Maestro v0.3.0 Trigger and Action
==============================

This version is compatible with Maestro-Tools v0.3.0 and Util v0.3.0

New Features
-------

1. Server is now able to send and handle basic Trigger and Action messages. More specifically EXAC and ACCM.
2. Brake detection algorithm for creating an internal trigger event
3. C-ITS control initial implementation of CAM and DENM messages for demo


Enhancements 
-------
The location for all files necessary to run the server has can now be defined. Defaults to ~/.maestro


Fixed bugs
-------
Uploading .traj files from the GUC no longer results in a corrupted file with POST messages at the end.  


July release v0.2.0 - ServerSnapshot  
==============================

This version is compatible with the July release of Maestro-Tools and the July release of util 

New Features
-------

1. Geofencing has been enabled. Server checks if an object is located inside or outside a given geofence and prints this to terminal.
2. MQbus defined in the util repo is now beeing used instead of the old message


"Enhancements" 
-------
Latidude spelling error has been changed to Latitude


Fixed bugs
-------
Server no longer creates Junk files in root directory

Drottning Kristina v0.1.0 
==============================

This version is compatible with the Drottning Kristina release of Maestro-Tools 

New Features
-------

1. Server now has a new and improved logger
2. Server has an UDP connection to the GUC which is used to send MONR

Enhancements 
-------
iCommReceive now has UTC timestamp

