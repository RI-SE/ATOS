# JournalControl
A module for generating output logs for the user.

## ROS parameters
The following ROS parameters can be set in the params.yaml file:

```yaml
atos:
  journal_control:
    ros__parameters:
      scenario_name: "" # Optional, name of the journal_file
```
## About the module
This module takes data recorded by other modules via the `JournalRecord` functions and merges it into a single ordered file. This data is separate from that which is logged via `RCLCPP_INFO` etc. which is more aimed at developers using the system. The journal data is intended to be used for data relevant to the test execution e.g. position data or triggered events.

1. At test arm, JournalControl creates references / "bookmarks" in each other module's journal which are used to keep track of which line marked the point at which the test was armed.

2. Once a test is completed JournalControl creates an end bookmark in each journal.

3. Weaves together a contiguous record of what happened in the test, based on the content between the start and end bookmarks in each module's journal.

4. The record is outputted as a file intended to be downloaded by the system user.
