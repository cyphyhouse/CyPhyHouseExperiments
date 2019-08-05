import io
import unittest

from cph.konfig import Konfig


class TestGenLocalConfig(unittest.TestCase):
    def setUp(self) -> None:
        pass

    def test_validate_global_config_empty(self):
        # TODO
        test_cfg = io.StringIO("")
        konf = Konfig(test_cfg)
        self.assertFalse(konf.validate_global_config())

    def test_validate_global_config_pass(self):
        test_cfg = io.StringIO("""
leader_pid: 1
mutex_handler: BaseMutexHandler
agents:
    - pid: 1
      plist: [2002, 2003]  # TODO use pid instead to refer to other agents
      on_device: drone1
      motion_automaton: MoatTestDrone
devices:
    drone1:
      bot_type: QUAD
      ip: 127.0.0.1
      port: 2001
      ros_node_prefix: 'quad_wp_node'
      queue_size: 1
      waypoint_topic:
          topic: 'waypoint'
          type: PoseStamped  # geometry_msgs/PoseStamped
      reached_topic:
          topic: 'reached'
          type: String  # std_msgs/String
      positioning_topic:
          topic:  '/vrpn_client_node/'  # TODO '"/vrpn_client_node/" + vicon_obj + "/pose"'
          type: PoseStamped  # geometry_msgs/PoseStamped
      planner: SimplePlanner
      motion_automata: [MoatTestDrone]
        """)
        # TODO
        konf = Konfig(test_cfg)
        self.assertTrue(konf.validate_global_config())

    def test_gen_local_config(self):
        # TODO
        pass


if __name__ == '__main__':
    unittest.main()

