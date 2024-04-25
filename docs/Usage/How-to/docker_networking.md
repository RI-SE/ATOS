# ATOS and Docker networks

## Running isoObject inside docker containers
Starting ATOS with docker compose will create a network named `atos_atos-net`. This network is used to connect the other containers that ATOS uses. Like foxglove-studio and the isoObject demo for example.

If you wish to run several isoObject demo containers, you can do so by connecting them to the `atos_atos-net` network and make sure they have unique hostnames. You can then use these hostnames in the VehicleCatalog.xosc file in the IP property field. like this 

```xml 
    <Vehicle 1 ...>
        <Properties>
            <Property name="ip" value="isoObjectA"/>
        </Properties>
    </Vehicle>
```

Then when starting an isoObject container, you can do so by running the following command:

```bash
docker run -it --rm --hostname isoObjectA --network atos_atos-net astazero/iso_object_demo:latest
```

This works because the `atos_atos-net` network is a user defined bridge network, which means that they can resolve the hostnames of the containers connected on that network.

## Running the isoObject on the host machine
If you wish to run the isoObject on the host machine, you can do so by specifying the hostname of the host machine in the VehicleCatalog file. Like this

```xml 
    <Vehicle ...>
        <Properties>
            <Property name="ip" value="host.docker.internal"/>
        </Properties>
    </Vehicle>
```

From the docker compose file, the host gateway address is added by these lines

```yaml
    extra_hosts:
      - "host.docker.internal:host-gateway"
```