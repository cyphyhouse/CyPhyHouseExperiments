import math
from scipy.spatial.transform import Rotation

from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.pos_types import pos3d


class ShapeForm(AgentThread):

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(ShapeForm, self).__init__(agent_config, moat_config)

    def initialize_vars(self):
        self.locals['i'] = 0
        self.locals['pattern'] = 0
        self.agent_gvh.create_ar_var('mypos', type(pos3d), self.moat.position)

    def loop_body(self):
        self.locals['i'] += 1
        if self.locals['i'] % 50 == 0:
            self.locals['pattern'] += 1

        if self.num_agents() not in [x**2 for x in range(1, 6)]:
            self.trystop()  # Avoid non square number of agents
            return
        self.write_to_shared('mypos', self.pid(), self.moat.position)
        if self.pid() in get_corners(self.num_agents()):
            self.moat.goTo(get_corner_pos(self.pid(), self.num_agents(), self.locals['pattern']))
        else:
            self.moat.goTo(
                mid_pt_of_list(
                    [self.read_from_shared('mypos', i) for i in get_nbrs(self.pid(), self.num_agents())]
                ))


def mid_pt_of_list(poslist):
    return pos3d(sum([p.x for p in poslist]) / len(poslist), sum([p.y for p in poslist]) / len(poslist),
                 sum([p.z for p in poslist]) / len(poslist))


def get_corners(n):
    assert math.sqrt(n).is_integer()

    w = int(math.sqrt(n))
    corner_coord = [(0, 0), (0, w-1), (w-1, 0), (w-1, w-1)]
    corners = set([x*w + y for x, y in corner_coord])
    return corners


def get_corner_pos(pid, num_agents, pattern: int = 1):
    if pattern == 0:
        space = 5
        rot = 0
    elif pattern == 1:
        space = 2.5
        rot = 0
    elif pattern == 2:
        space = 5
        rot = 60
    elif pattern == 3:
        space = 5
        rot = 120
    elif pattern == 4:
        space = 5
        rot = 180
    else:
        space = 3
        rot = 180

    r = Rotation.from_euler('z', rot, degrees=True)

    assert math.sqrt(num_agents).is_integer()
    w = int(math.sqrt(num_agents))
    x, y = (pid // w, pid % w)

    vec = pos3d((x - (w-1) / 2)*space, (y - (w-1) / 2)*space, 10.0)
    rot_vec_arr = r.apply(vec.mk_arr())

    return pos3d(-67.0, 0.0, 0.0) + pos3d(*rot_vec_arr)


def get_nbrs(pid, num_agents):
    assert math.sqrt(num_agents).is_integer()

    w = int(math.sqrt(num_agents))
    x, y = (pid // w, pid % w)

    if x == 0 or x == w-1:
        all_nbrs = [(x, y-1), (x, y+1)]
    elif y == 0 or y == w-1:
        all_nbrs = [(x-1, y), (x+1, y)]
    else:
        all_nbrs = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]

    prop_nbrs = [(nx, ny) for nx, ny in all_nbrs if 0 <= nx < w and 0 <= ny < w]
    return set([nx*w + ny for nx, ny in prop_nbrs])
