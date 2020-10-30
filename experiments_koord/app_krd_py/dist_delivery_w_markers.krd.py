import numpy as np

import rospy
from cym_gazebo import marker_builder
from cym_marker.msg import Marker

from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.deconflict import clear_path
from src.motion.rectobs import RectObs
from src.motion.cylobs import CylObs
from src.objects.udt import Task
from src.motion.pos_types import pos3d, Pos

MARKER_Z = 0.05


class TaskApp(AgentThread):

    TASKS = [
        Task(pos3d(pos[0], pos[1], pos[2]), i, False, None)
        for i, pos in enumerate([
            # Drones' tasks
            (+3.0,  1.8, 4.0),
            (-3.0,  0.2, 2.6),
            (+2.5,  6.3, 4.0),
            (-2.5, -4.5, 2.6),
            (+3.0, -4.5, 4.0),
            (-1.5,  6.3, 1.3),
            (-3.0,  1.8, 2.6),
            (+3.0,  0.2, 1.3),
            (-2.5,  6.3, 2.6),
            (-2.5, -6.0, 2.6),
            (+3.0, -6.0, 4.0),
        ])
    ]
    SHELVE_SCALE = np.array([5.4, 2.0, 6.0])

    OBSTACLES = [
        CylObs(Pos(np.array([3.8239, 3.403029, 0])), radius=0.5, height=1.6),
        RectObs(Pos(np.array([1.0, 3.4, 3.0])), SHELVE_SCALE),
        RectObs(Pos(np.array([-4.13394, 3.4, 3.0])), SHELVE_SCALE),
        RectObs(Pos(np.array([1.0, 4.7, 3.0])), SHELVE_SCALE),
        RectObs(Pos(np.array([-4.13394, 4.7, 3.0])), SHELVE_SCALE),
        RectObs(Pos(np.array([1.0, -1.5, 3.0])), SHELVE_SCALE),
        RectObs(Pos(np.array([-4.13394, -1.5, 3.0])), SHELVE_SCALE),
        RectObs(Pos(np.array([1.0, -2.8, 3.0])), SHELVE_SCALE),
        RectObs(Pos(np.array([-4.13394, -2.8, 3.0])), SHELVE_SCALE),
        RectObs(Pos(np.array([1.0, -7.8, 3.0])), SHELVE_SCALE),
        RectObs(Pos(np.array([-4.13394, -7.8, 3.0])), SHELVE_SCALE),
        RectObs(Pos(np.array([1.0, -9.1, 3.0])), SHELVE_SCALE),
        RectObs(Pos(np.array([-4.13394, -9.1, 3.0])), SHELVE_SCALE),
        RectObs(Pos(np.array([-8.34545, 0.0, 1.6])), np.array([0.4, 7.8, 3.2])),
        RectObs(Pos(np.array([-3.5, 9.0, 1.6])), np.array([7.8, 0.4, 3.2]))
    ]

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(TaskApp, self).__init__(agent_config, moat_config)
        self._pub_marker = rospy.Publisher("/cym_marker", Marker,
                                           queue_size=10,
                                           latch=True)
        if self.agent_gvh.is_leader:
            self.init_all_task_markers()
            self.init_all_obstacle_markers()

        self.moat.planner.min_rand = -10.0
        self.moat.planner.max_rand = 10.0
        self.moat.planner.min_zrand = 0.0
        self.moat.planner.max_zrand = 4.5
        self.moat.planner.expand_dis = 3.0
        self.moat.planner.max_iter = 200

    def initialize_vars(self):
        self.initialize_lock('pick_route')
        self.agent_gvh.create_aw_var('tasks', list, TaskApp.TASKS)
        self.agent_gvh.create_ar_var('route', list, [self.moat.position])
        self.locals['my_task'] = None
        self.locals['test_route'] = None
        self.locals['doing'] = False
        self.locals['tasks'] = []
        self.locals['obstacles'] = self.OBSTACLES

    def loop_body(self):
        if not self.locals['doing']:
            if sum([int(a.assigned) for a in self.read_from_shared('tasks', None)]) == len(
                    self.read_from_shared('tasks', None)):
                self.trystop()
                return

            if self.lock('pick_route'):
                self.locals['tasks'] = self.read_from_shared('tasks', None)
                print("Agent", self.pid(), "at", self.moat.position,
                      "has lock. Remaining tasks:",
                      [t.id for t in self.locals['tasks'] if not t.assigned])
                for i in range(len(self.locals['tasks'])):
                    if not self.locals['tasks'][i].assigned:
                        self.locals['my_task'] = self.locals['tasks'][i]
                        self.locals['test_route'] = self.moat.planner.find_path(self.moat.position,
                                                                                self.locals['my_task'].location,
                                                                                self.locals['obstacles'])
                        if clear_path([path for path in
                                       [self.read_from_shared('route', pid) for pid in range(self.num_agents())]],
                                      self.locals['test_route'], self.pid(), tolerance=1.0):
                            self.locals['doing'] = True
                            self.locals['my_task'].assign(self.pid())
                            self.assign_task_marker()  # Add a visual marker in simulator
                            self.locals['tasks'][i] = self.locals['my_task']
                            self.agent_gvh.put('tasks', self.locals['tasks'])
                            self.agent_gvh.put('route', self.locals['test_route'], self.pid())
                            print("Agent", self.pid(), "is going to task", i, "at", self.locals['my_task'].location)
                            self.moat.follow_path(self.locals['test_route'])
                        else:
                            self.agent_gvh.put('route', [self.moat.position],
                                               self.pid())
                            self.locals['my_task'] = None
                            self.locals['doing'] = False
                            continue
                        break
                if not self.locals['doing']:
                    print("Agent", self.pid(), "didnt find a clear path")
                self.unlock('pick_route')
                rospy.sleep(0.05)
        else:
            if self.moat.reached:
                if self.locals['my_task'] is not None:
                    self.finish_task_marker()
                    self.locals['my_task'] = None
                self.locals['doing'] = False
                rospy.sleep(1.0)  # Wait at the task for a while
                return

    def assign_task_marker(self):
        marker = add_target_marker(self.locals['my_task'])
        self._pub_marker.publish(marker)

    def finish_task_marker(self):
        marker = add_reached_marker(self.locals['my_task'])
        self._pub_marker.publish(marker)

    def init_all_task_markers(self):
        for task in TaskApp.TASKS:
            rospy.sleep(0.4)
            marker = add_init_marker(task)
            if marker:
                self._pub_marker.publish(marker)

    def init_all_obstacle_markers(self):
        for i, obs in enumerate(self.OBSTACLES):
            rospy.sleep(0.4)
            marker = add_obstacle_marker("obstacle", i, obs)
            if marker:
                self._pub_marker.publish(marker)


def task_marker_builder(task: Task):
    pos = task.location

    if pos.z == 0:  # Ground tasks
        builder = marker_builder.PutCylinder()
        builder.scale.z = 0.01
        builder.pose.position.z = pos.z + MARKER_Z
    else:
        builder = marker_builder.PutSphere()
        builder.scale.z = 1.0
        builder.pose.position.z = pos.z

    builder.ns = "tasks"
    builder.id = task.id
    builder.pose.position.x = pos.x
    builder.pose.position.y = pos.y
    builder.scale.x = 1.0
    builder.scale.y = 1.0
    return builder


def add_obstacle_marker(ns: str, m_id: int, obs):
    if isinstance(obs, RectObs):
        builder = marker_builder.PutBox()
        builder.scale.x = obs.size[0]
        builder.scale.y = obs.size[1]
        builder.scale.z = obs.size[2]
    elif isinstance(obs, CylObs):
        builder = marker_builder.PutCylinder()
        builder.scale.x = obs.diameter
        builder.scale.y = obs.diameter
        builder.scale.z = obs.size[2]
    else:
        return None
    builder.ns = ns
    builder.id = m_id
    builder.pose.position.x = obs.position.x
    builder.pose.position.y = obs.position.y
    builder.pose.position.z = obs.position.z

    builder.use_material("Gazebo/YellowTransparent")
    return builder.build()


def add_init_marker(task: Task):
    builder = task_marker_builder(task)
    builder.use_material("Gazebo/BlackTransparent")
    return builder.build()


def add_target_marker(task: Task):
    builder = task_marker_builder(task)
    builder.use_material("Gazebo/RedTransparent")
    return builder.build()


def add_reached_marker(task: Task):
    builder = task_marker_builder(task)
    builder.use_material("Gazebo/GreenTransparent")
    return builder.build()

