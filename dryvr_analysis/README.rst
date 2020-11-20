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

#. Generate traces for DryVR from a raw ROS bag trace

   * The sequence of timestamped states is divided into segments based on the timestamp of each waypoint.
     We **assume** a segment is the behavior of the black-box dynamical system under the mode specified by the waypoint,
     and the initial state is the last sampled state at or before the timestamp of the waypoint.
   * TODO


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
