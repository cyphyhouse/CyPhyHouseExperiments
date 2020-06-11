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




Installation
============

1.  Inside the src directory of a catkin_ws clone the following repos:
  - `$ git clone https://github.com/cyphyhouse/CyPyHous3.git --branch for-cymulator`
	- `$ git clone https://github.com/cyphyhouse/Cymulator`
2. Run `$ catkin_make` inside your catkin_ws directory
3.  Inside the `CyPyHous3` folder run `$ pip install -e ./` 
4. `$ source devel/setup.bash` from you`catkin_ws` directory to source the new environment variables  
5. In any directory clone the following repo: `$ git clone https://github.com/cyphyhouse/CyPhyHouseExperiments`
6.   Download the latest version of the [koord langauge](https://github.com/cyphyhouse/KoordLanguage/releases)
7.  In a separate terminal: ` $ rosrun cym_gazebo cymulate.py scenes/cym_5_drones.yml`  (make sure that this terminal contains the sourced environment variables from step 4)
8.  Run a koord experiment:
	- ` ./experiment.sh apps/lineform.koord configs/motion_5_drones.global.yml`
	- `$ ./experiment_no_koord.sh app_py/follow_path.krd.py configs/motion_5_drones.global.yml`(if the above fails, note this is without koord, and probably means you have the wrong version of Java)
  
Installation Help
-----------------
If the installation fails please make sure the following packages are installed:
- `ros-kinetic-gazebo9-ros-pkgs`
- `libyaml-cpp-dev`
-  Java version 1.11 (needed for using koord language)
- `netifaces`
- `defusedxml`


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

**TODO:** The configuration file `cym_5_drones` for Cymulator is not well
designed. It should be merged into the same global configuration file.

```bash
rosrun cym_gazebo cymulate.py scenes/cym_5_drones.yml
```

Similarly, we then use `experiment.sh` to start the experiment.
For example, run the following command to execute `lineform` with 5 simulated
agents
```bash
$ ./experiment.sh apps/lineform.koord configs/motion_5_drones.global.yml
```
