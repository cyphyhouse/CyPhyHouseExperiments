import rospy
from cym_gazebo import marker_builder
from cym_marker.msg import Marker

from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.deconflict import clear_path
from src.objects.udt import Task
from src.motion.pos_types import Pos


class TaskApp(AgentThread):

    tasks = [
        Task(Pos((4*pos[0], 4*pos[1], pos[2])), i, False, None)
        for i, pos in enumerate([
            (1.75, 1.75, 0),
            (0, 2, 0),
            (-2, 2, 0),
            (-2, -2, 0),
            (0.25, -2, 1),
            (2, 0, 0),
            (1.75, -1.75, 0),
            (2, 1.75, 1),
            (0.25, 2, 1.25),
            (-2, 0.25, 1.25),
            (0, 0.25, 1.5),
            (1, -1, 1),
            (0, -2, 0),
            (-2, 0, 0),
            (-1.75, 2, 1.5),
            (-2, -1.75, 1.15),
        ])
    ]

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(TaskApp, self).__init__(agent_config, moat_config)
        self._pub_marker =rospy.Publisher("/cym_marker", Marker,
                                           queue_size=10,
                                           latch=True)
        if self.agent_gvh.is_leader:
            self.init_all_task_markers()

    def initialize_vars(self):
        self.initialize_lock('pick_route')
        self.agent_gvh.create_aw_var('tasks', list, TaskApp.tasks)
        self.agent_gvh.create_ar_var('route', list, [self.moat.position])
        self.locals['my_task'] = None
        self.locals['test_route'] = None
        self.locals['doing'] = False
        self.locals['tasks'] = []
        self.locals['obstacles'] = []

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
                                                                                          self.locals[
                                                                                              'my_task'].location,
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
        my_task = self.locals['my_task']
        task_loc = my_task.location
        marker = add_target_marker('tasks', my_task.id, (task_loc.x, task_loc.y, 0.0))
        self._pub_marker.publish(marker)

    def finish_task_marker(self):
        my_task = self.locals['my_task']
        task_loc = my_task.location
        marker = add_reached_marker('tasks', my_task.id, (task_loc.x, task_loc.y, 0.0))
        self._pub_marker.publish(marker)

    def init_all_task_markers(self):
        for task in TaskApp.tasks:
            task_loc = task.location
            marker = add_init_marker('tasks', task.id, (task_loc.x, task_loc.y, 0.0))
            rospy.sleep(0.4)
            self._pub_marker.publish(marker)


def pos_marker_builder(ns: str, m_id: int, pos):
    builder = marker_builder.PutCylinder()
    builder.ns = ns
    builder.id = m_id
    builder.pose.position.x = pos[0]
    builder.pose.position.y = pos[1]
    builder.pose.position.z = pos[2]
    builder.scale.z = 0.01
    return builder


def add_init_marker(ns: str, m_id: int, pos):
    builder = pos_marker_builder(ns, m_id, pos)
    builder.use_material("Gazebo/BlackTransparent")
    return builder.build()


def add_target_marker(ns: str, m_id: int, pos):
    builder = pos_marker_builder(ns, m_id, pos)
    builder.use_material("Gazebo/RedTransparent")
    return builder.build()


def add_reached_marker(ns: str, m_id: int, pos):
    builder = pos_marker_builder(ns, m_id, pos)
    builder.use_material("Gazebo/GreenTransparent")
    return builder.build()

