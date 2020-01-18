CyPhyHouseExperiments
=====================

[![License](https://img.shields.io/github/license/cyphyhouse/CyPhyHouseExperiments)](LICENSE)

CyPhyHouseExperiments repository is for executable scripts for reproducing
experiment results in our research papers.
These scripts can be used in [Intelligent Robotics Lab][url-irl] in UIUC as well
as in our Gazebo based simulation engine, [Cymulator][url-cym].
This repository also contains the glue code that integrates all components in
the [CyPhyHouse][url-cph] project.

[url-cph]: https://cyphyhouse.github.io/
[url-irl]: https://robotics.illinois.edu/robotics-facilities/
[url-cym]: https://github.com/cyphyhouse/Cymulator


Website and Documentation
-------------------------

CyPhyHouseExperiments is part of the CyPhyHouse project and currently requires
setting up hardware devices and installing software on each device.
Please visit following websites for detail usages.

Broad overview of CyPhyHouse project is available at:

  https://cyphyhouse.github.io/

Or you can find the documentation at:

  https://cyphyhouse.rtfd.io/


License
-------

CyPhyHouseExperiments is licensed under the terms of the NCSA License (see the file
[LICENSE](LICENSE)).


Run experiment scripts
======================

Assume the Koord compiler and middle-ware are already installed.

Usage
-----

Change the working directory to `experiments`.


### Apps without Motion module

To execute a Koord program that does not require the Motion module, simply use
the `experiment.sh` script under `experiments/`.
For example, run the following command to execute `addnums` with 1 simulated
agent
```bash
$ ./experiment.sh apps/addnum.koord configs/no_motion_1_drone.global.yml
```


### Apps with Motion module

To execute a Koord program that uses the Motion module, we need to first
instantiate simulated devices in our Cymulator.

**TODO:** The configuration file `simconfig_5_drones` for Cymulator is not well
designed. It should be merged into the same global configuration file.

```bash
rosrun rosrun cym_gazebo cymulate.py simconfig_5_drones.yml
```

Similarly, we then use `experiment.sh` to start the experiment.
For example, run the following command to execute `lineform` with 5 simulated
agents
```bash
$ ./experiment.sh apps/lineform.koord configs/motion_5_drones.global.yml
```
