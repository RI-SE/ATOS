#  Release Procedure

A new release procedure is started once a milestone has been accomplished and a new one has been set. Within the release procedure there are two phases; a development phase and a release phase. The development phase is focused on devloping and implementing new features which are needed to achive the milestone, while the release phase focues on refining and testing Maestro such that it can be released as a stable version. The release phase is 4 weeks long and during this period and before this period starts, the amount of features are locked. After this feature freeze, no new features are allowed to be included in the planned release. This is illustrated in the picture below:

![A timeline for the release of a software update](timeline_procedure.png)

## Definition of a Release

A release is the process of moving code from the `dev` branch to the `master` branch and creating a release on the [Maestro Github repository](https://github.com/RI-SE/Maestro/releases). A release should contain the software needed to successfully run Maestro on a computer.

# Development Phase

```
feature -> dev
```
Testing should be performed and documented through a lightweight protocol:
* Complete a normal test with one virtual test object
* Complete a normal test with two virtual test objects
* Complete a aborting test with two virtual test objects


# Release Phase
```
dev -> master
(dev -> chronos) testing requirement on this merge might not be as extensive
```
A set number of test cases should be performed and documented. These tests are conducted with the use of real objects (for example using the RC-car or any other ISO-22133-1-compatible test object).

The following documentation should be updated during the release phase:
* Maestro Documentation (Including system description, function description, etc.)
* User Guide
* Test documentation which contains
    - Test description
    - Test result
    - Test result responsible person (Normally the test executor)

The release documentation on github should be created containing:
* Version number #
* Change log (what has changed since last release?)
* Compatibility
  - GUC version number #
  - Virtual Object version number #
  - util version number #
* Known bugs with classification and error description.
    - Severity: minor/major.
    - Possible workaround.

