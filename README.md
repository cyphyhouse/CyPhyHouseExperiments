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

0. Download ROS Kinetic and create a catkin_ws
	- [ROS Kinetic Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu)
	- [Creating a catkin_ws](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
1.  Inside the `src` directory of your catkin_ws clone the following repos:
 	- `$ git clone https://github.com/cyphyhouse/CyPyHous3.git --branch for-cymulator`
	- `$ git clone https://github.com/cyphyhouse/Cymulator`
	- `$ git clone https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor`
	- `$ git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo`
2. Run `$ catkin_make` inside your `catkin_ws` directory
3. Inside the `CyPyHous3` folder run `$ pip install -e ./` 
4. `$ source devel/setup.bash` from your `catkin_ws` directory to source the new environment variables  
5. In any directory (does not need to be in the catkin_ws) clone the following repo: `$ git clone https://github.com/cyphyhouse/CyPhyHouseExperiments`
6. Download the latest version of the [koord langauge](https://github.com/cyphyhouse/KoordLanguage/releases)
	- place `koord-0.1-jar-with-dependencies.jar` inside `CyPhyHouseExperiments/experiments_koord` folder

  
Installation Help
-----------------
If the installation fails please make sure the following packages are installed:
- `ros-kinetic-gazebo9-ros-pkgs`
- `ros-kinetic-gazebo9-ros` 
- `ros-kinetic-gazebo9-ros-control`
- `ros-kinetic-gazebo9-plugins` 
- `ros-kinetic-ackermann-msgs` 
- `ros-kinetic-geographic-msgs` 
- `ros-kinetic-serial`
- `ros-kinetic-ros-control ros-kinetic-ros-controllers`
- `ros-kinetic-hector-localization ros-kinetic-hector-models`
- `ros-kinetic-geometry2 ros-kinetic-robot`
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
the `experiment.sh` script under `experiments_koord/`.
For example, run the following command to execute `addnums` with 1 simulated
agent
```bash
$ ./experiment.sh app_krd/addnum.krd configs/no_motion_1_drone.global.yml
```


### Apps with Motion module

To execute a Koord program that uses the Motion module, we need to first
instantiate simulated devices in our Cymulator.

```bash
rosrun cym_gazebo cymulate.py scenes/cym_5_drones.yml
```

Similarly, we then go to `experiments_koord/` and use `experiment.sh` to start the experiment.
For example, run the following command to execute `lineform` with 5 simulated
agents
```bash
$ ./experiment.sh app_krd/lineform.krd configs/motion_5_drones.global.yml
```
