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


Cite CyPhyHouse
---------------

### Plain Text

R. Ghosh et al., "CyPhyHouse: A programming, simulation, and deployment toolchain for heterogeneous distributed coordination",
*2020 IEEE International Conference on Robotics and Automation (ICRA)*, Paris, France, 2020, pp. 6654-6660,
doi: 10.1109/ICRA40945.2020.9196513.


### BibTex
```BibTeX
@INPROCEEDINGS{CyPhyHouse,
  author={Ritwika Ghosh and Joao P. Jansch-Porto and Chiao Hsieh and 
          Amelia Gosse and Minghao Jiang and Hebron Taylor and Peter Du and
          Sayan Mitra and Geir Dullerud},
  booktitle={2020 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={CyPhyHouse: A programming, simulation, and deployment toolchain for heterogeneous distributed coordination},
  year={2020},
  volume={},
  number={},
  pages={6654-6660},
  doi={10.1109/ICRA40945.2020.9196513}
}
```


License
-------

CyPhyHouseExperiments is licensed under the terms of the NCSA License (see the file
[LICENSE](LICENSE)).


Installation
============

1. Follow the instruction at https://github.com/cyphyhouse/Cymulator to install Cymulator.
   We assume the catkin workspace is under `catkin_ws`.

   - (Optional) Follow the instruction at https://github.com/chaolmu/gazebo_models_worlds_collection to download and install
     additional Gazebo simulation worlds for better looking scenes.

2. Clone this CyPhyHouseExperiments repository under any directory (does not need to be in the `catkin_ws`):
   ```
   git clone https://github.com/cyphyhouse/CyPhyHouseExperiments
   ```

Install Packages for Koord Experiments
--------------------------------------

Following steps are required only for using Koord language compiler and middleware to run Koord experiments.
You can skip these steps if you want to run other experiments such as UTM experiments,
and check the specific directory for more instructions. 

3. Download the latest version of the [koord langauge](https://github.com/cyphyhouse/KoordLanguage/releases)
   - place `koord-0.1-jar-with-dependencies.jar` inside `CyPhyHouseExperiments/experiments_koord` folder

4. Clone and install CyPyHous3 middleware under any directory (does not need to be in `catkin_ws`).
   ```shell
   git clone https://github.com/cyphyhouse/CyPyHous3.git --branch for-cymulator  # Clone the repo with the for-cymulator branch
   pip3 install --user -e CyPyHous3/
   ```

  
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
-  Java version 1.11 (needed for using Koord language)
- `netifaces`
- `defusedxml`


Run experiment scripts for Koord applications
=============================================

Assume the Koord compiler and middle-ware are already installed.

Change the working directory to `experiments_koord`.


### Apps with Motion module

To execute a Koord program that uses the Motion module, we need to first
instantiate simulated devices in our Cymulator.

```bash
source catkin_ws/devel/setup.bash  # catkin_ws is your workspace for catkin
rosrun cym_gazebo cymulate.py ../scenes/cym_5_drones.yml
```

We then use `experiment.sh` to start the experiment in another terminal.
For example, run the following command to execute `lineform` with 5 simulated
agents in another terminal.

```bash
source catkin_ws/devel/setup.bash  # catkin_ws is your workspace for catkin
./experiment.sh app_krd/lineform.krd configs/motion_5_drones.global.yml
```

**More applications and experiment instructions can be found under the `experiments_koord` folder.**
