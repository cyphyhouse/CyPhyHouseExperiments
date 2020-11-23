# Experiments with Koord Applications

The main experiment script `experiment.sh` is to run a given Koord application
in the Gazebo simulation world. 

To execute a Koord program that uses the Motion module, we need to first
instantiate simulated drones in our Cymulator.

```bash
source catkin_ws/devel/setup.bash  # catkin_ws is your workspace for catkin
rosrun cym_gazebo cymulate.py ../scenes/cym_5_drones.yml
```

We then use `experiment.sh` to start the experiment.
For example, run the following command to execute `lineform` with 5 simulated
agents in another terminal.

```bash
source catkin_ws/devel/setup.bash  # catkin_ws is your workspace for catkin
./experiment.sh app_krd/lineform.krd configs/motion_5_drones.global.yml
```

Note that the selected `*.global.yml` files should specify the same number and types of devices as specified in
`scenes/*.yml` and instantiated in the Gazebo simulation world. 


## Experiments with Compiled Koord Code in Python  

Additionally, `experiment_skip_koord.sh` is to run the experiments directly using the compiled Koord code in Python.
We provide several compiled Koord code under `app_krd_py`. Some are modified to show visual markers in Gazebo
to better illustrate the progress of current simulation run.  

For example, we similarly instantiate simulated drones in warehouse world using our Cymulator.
```bash
source catkin_ws/devel/setup.bash  # catkin_ws is your workspace for catkin
rosrun cym_gazebo cymulate.py ../scenes/warehouse-5_drones.yml
```

We then use `experiment_skip_koord.sh` to start the experiment.
Run the following command to run Distributed Delivery with 5 simulated
agents in another terminal.
```bash
source catkin_ws/devel/setup.bash  # catkin_ws is your workspace for catkin
./experiment.sh app_krd/dist_delivery_w_markers.krd.py configs/motion_5_drones.global.yml
```

The Gazebo simulation looks like the GIF below.
![Warehouse Delivery](/docs/warehouse-delivery-sim.gif)
 