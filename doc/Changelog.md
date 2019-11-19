Last edited 15/10- 2019

Contents
==============================
[v.0.4.0 November Server-Snapshot](#november-release-v040---serversnapshot)

[v.0.3.0 Trigger and Action](#maestro-v030-trigger-and-action)

[v.0.2.0 July Server-snapshot](#july-release-v020---serversnapshot)

[v.0.1.0 Drottning Kristina](drottning-kristina-v010)

July release v0.2.0 - ServerSnapshot  
==============================

This version is compatible with Maestro-Tools v0.4.0 and Util v0.4.0

New Features
-------

1. Supervisor now is its own c++ module.

Enhancements 
-------
The location for all files necessary to run the server has can now be defined. Defaults to ~/.maestro


Fixed bugs
-------
MQ priorities was wrong, Highest priority was lowest. This has been remedied.

Maestro v0.3.0 Trigger and Action
==============================

This version is compatible with Maestro-Tools v0.3.0 and Util v0.3.0

New Features
-------

1. Server is now able to send and handle basic Trigger and Action messages. More specifically EXAC and ACCM.


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

