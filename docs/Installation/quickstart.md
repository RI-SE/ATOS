#### Prerequisites
Make sure you have installed ATOS by following the instructions in the [Installation](installation.md) section.

-------------------------

# Quick start

This section will walk you through the steps of starting ATOS, connecting it with a virtual test object and visualizing the scenario in Foxglove studio.

* Start ATOS by running either one of the docker commands (or with "ros2 launch ..." if you have installed it locally). In this guide, the docker compose command is used.

        
        /<git_repo_path>/ATOS $ docker compose up
        (or if you are not in the docker group)
        /<git_repo_path>/ATOS $ sudo -E docker compose up

    !!! Warning
        Running the default docker compose command will run network traffic insecurely between ATOS and Foxglove. See the [GUI](../Usage/GUI/foxglove.md) documentation on how to enable secure connections. 
       

* Make sure the folder ~/.astazero/ATOS/ was created on the host machine. Since it was created by docker you will need to change the owner to your own user. Run the following command on the host machine:

       
        sudo chown -R $USER:$USER ~/.astazero/ATOS/
        

* Start a virtual test object by running 

        docker run -it --rm --network host -v ~/.astazero/ATOS/:/root/.astazero/ATOS/ --name iso_object_demo astazero/iso_object_demo:latest

  This docker command creates a virtual object which will respond to control messages from ATOS. You can find the source code for the _ISO\_objectDemo_ at the [isoObject repo](https://github.com/RI-SE/isoObject).

* Open Chrome and go to [http://localhost:8080](http://localhost:8080).

* Press Open connection -> Foxglove WebSocket. Enter WebSocket URL ws://localhost:8765. Press Open. You should now see ROS topics in the left panel named "Topics". 

* Install the foxglove ATOS extensions from the ATOS repo. Do this by dragging the .foxe files from the folder /{repo_path}/ATOS/plugins/foxglove/ into the Foxglove studio browser window.

* Press the buttons Init and Connect in the control panel pane. You should now see several trajectories and an object appear in the 3D view.

* Now press Arm and Start. The object should now follow one of the trajectories.