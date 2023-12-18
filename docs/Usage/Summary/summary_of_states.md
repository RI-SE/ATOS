# Summary of ATOS test-states

Once ATOS is running, the control-panel GUI can be used to issue commands to ATOS. Certain commands are legal only in specific states. States and the legal transitions between states are described in the standard [ISO22133](https://www.iso.org/standard/78970.html).

A normal sequence of events is described in the following table:    

| Step | Action | Description | Expected Response |
|------|------|------|------|
| 1 | Initialize the server | Press the `Init` button. | Scenario is loaded and trajectories are generated for each test object. If previously in `Idle` state, ATOS transitions to state: `Initialized` |
| 2 | Connect to all objects. | Press the `Connect` button. | The server will now try to connect to all objects in the scenario by establishing IP-connections. ATOS transitions to the state `Connected`.
| 3 | Arm the scenario. | Press the `Arm` button. |  ATOS transitions to state `Armed` to indicate that all objects are armed and ready to start. | 
| 4 | Start the scenario. | Press the `Start` button. | The test starts and ATOS transitions to state: `Running`. This means that the test is live, and both physical and virtual test participants, will start moving according to the scenario. |
| 5 | Abort the scenario. | Press the `Abort` button. | ATOS Aborts the scenario and transitions to state `Aborting`. |
| 6 | Reset the system. | Press the `Clear Abort` button. | ATOS transitions to the state `Connected`. |

