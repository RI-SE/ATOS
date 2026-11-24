#### Prerequisites
Make sure you have [docker](https://docs.docker.com/engine/install/) installed and have downloaded the [ATOS repo](https://github.com/RI-SE/ATOS). See the [Installation](installation.md) section for other ways of running ATOS.

-------------------------

# Quick start

This section will walk you through the steps of starting ATOS in docker, connecting it with a virtual test object and running a scenario from the ATOS GUI.

* Start ATOS by running either one of the docker commands (or with "ros2 launch ..." if you have installed it locally). In this guide, the docker compose command is used.

        
        /<git_repo_path>/ATOS $ docker compose -f docker-compose-bridge.yml up
        (or if you are not in the docker group)
        /<git_repo_path>/ATOS $ sudo -E docker compose -f docker-compose-bridge.yml up
       

* Make sure the folder ~/.astazero/ATOS/ was created on the host machine. If it was created by docker during the first boot up, you might need to change the owner to your own local user to make changes. Run the following command on the host machine:

       
        sudo chown -R $USER:$USER ~/.astazero/ATOS/
        

* (Optional) Configure the scenario execution by opening the params.yaml file at ~/.astazero/ATOS/conf/params.yaml, which ATOS creates on its first start. Under the parameters for the node open_scenario_gateway, make sure that open_scenario_file and active_object_names are set correctly. The default configuration works without changes: it uses the GaragePlanScenario and connects to one object ("1") reachable on hostname isoObject (the IP/hostname is configured in ~/.astazero/ATOS/Catalogs/Vehicles/VehicleCatalog.xosc). Restart ATOS after changing the configuration.

* Start a virtual test object by running 

        docker run -it --rm --hostname isoObject --network atos_atos-net --name isoObject astazero/iso_object_demo:latest

  This docker command creates a virtual object which will respond to control messages from ATOS. You can find the source code for the _ISO\_objectDemo_ at the [isoObject repo](https://github.com/RI-SE/isoObject).

* Open Chrome and go to the ATOS control panel at [http://localhost:8420/control](http://localhost:8420/control).

* Press the buttons Init and Connect. You should now see trajectories and an object appear in the map view.

* Now press Arm and Start. The object should follow one of the trajectories.
