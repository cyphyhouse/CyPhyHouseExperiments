#################################################
Platform-Dependent Analysis on Motion using DryVR
#################################################

#. Collect raw traces recorded as ROS bags from Gazebo simulation.

   * A raw trace currently consists of at lease a sequence of timestamped waypoints and another sequence of timestamped
     states.
   * The sequence of timestamped waypoints indicates when each waypoint is sent to the vehicle.
     We send a new waypoint only after the previous waypoint is reached.
     Reaching a waypoint is determined by staying within a distance to the waypoint for a short duration tolerance.
     (The distance and duration tolerance for Hector Quadrotor are defined by its builtin Pose action server.)
   * The sequence of timestamped states is periodically sampled *independent of the waypoints*.
     It contains the ground truth state provided by Gazebo.
     A state includes position, orientation, linear velocity and angular velocity of the vehicle model.

#. Generate traces for DryVR from a raw ROS bag trace and store as a Python pickle file.

   * The sequence of timestamped states is divided into segments based on the timestamp of each waypoint.
     We **assume** a segment is the behavior of the black-box dynamical system under the mode specified by the waypoint,
     and the initial state is the first sampled state *at or after* the timestamp of the waypoint.
     The trace segment for this waypoint is the sequence of sampled states starting from the initial state of this waypoint,
     and ending at the state before the initial state of the next waypoint.
   * The output for each bag file is a sequence of pairs.
     Each pair consists of one stamped waypoint and one trace segment of stamped states.
   * The output is stored as a pickle file to remove dependency on ROS packages for the next steps.

#. Compute reachtube using DryVR

    * TODO


******
Usages
******

Due to the large size of the binary files, we provide only a small example here on GitHub.
Our collected ROS bag files and generated Python pickle files are available at
`our Google Drive <https://drive.google.com/drive/folders/1cJUD-M4f6GuaEvkgcd8e1Cj_yukn4DEF?usp=sharing>`_.

#. Collect ROS bags.
   To customize the simulation, look into the arguments for ``test_hector_quad_random_waypoints.launch``
   in our ``cym_device`` package.

   .. code-block:: shell

      ./collect_rosbag_trace.sh

#. Convert ROS bags to Python pickle files.
   Given a input file named ``file.bag``, it generates an output file named ``file.bag.pickle``.

   .. code-block:: shell

      ./convert_to_pickle.py bags/*.bag

#. Generate reachtube. TODO

   .. code-block:: shell

      ./compute_reachtube.py bags/*.pickle


.. note::

    Currently all scripts under this folder are only for Hector Quadrotor model in Gazebo simulation.
    In particular, we use the builtin PID-based position controller with the following PID parameter values.

    .. code-block:: yaml

        position:
            type: hector_quadrotor_controllers/PositionController
            x: {p: 2.0, i: 1.0, d: 1.0}
            y: {p: 2.0, i: 1.0, d: 1.0}
            z: {p: 2.0, i: 1.0, d: 0.5}
            yaw: {p: 2.0, i: 1.0, d: 1.0}

    See https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor for Hector Quadrotor,
    and check ``hector_quadrotor_controllers/params/controller.yaml`` for configuring PID values.
