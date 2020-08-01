from src.harness.agentThread import AgentThread


class Follow_points(AgentThread):

    def __init__(self, config, motion_config):
        super(Follow_points, self).__init__(config, motion_config)


    def initialize_vars(self):
        self.locals = {}
        self.locals['tries'] = 1
        self.locals['p'] = 10 - 3 * self.pid()


        # self.waypoints = [[(20,14,1), (23,0,1), (16,-16,1), (-4,-4,1)],
        #                   [(14.5,-16,2), (26,0,2), (20,14,2), (7,8,2)],
        #                   [(2,2,1.5), (-2,-2,1.5), (-2,2,1.5), (0,0,1.5)]]

        self.waypoints = [[(19,13,1), (23,0,1), (16,-16,1), (-4,-4,1)],
                          [(18,13.5,2), (26,0,2), (20,-17,2), (-4,-7,2)],
                          [(16,14.5,3), (29,0,3), (22,-17,3), (-7,-7,3)],]

        self.waypoint_num = 0

        self.locals[self.pid()] = self.waypoints[self.pid()]

    def loop_body(self):
        if self.locals['tries'] == 1:
            x = self.locals[self.pid()][self.waypoint_num][0]
            y = self.locals[self.pid()][self.waypoint_num][1]
            z = self.locals[self.pid()][self.waypoint_num][2]
            self.write_to_actuator('Motion.target', self.pos3d(x, y, z))
            self.locals['tries'] += 1
            self.waypoint_num += 1
            return
        if self.locals['tries'] < 5 and self.read_from_sensor('Motion.reached'):
            x = self.locals[self.pid()][self.waypoint_num][0]
            y = self.locals[self.pid()][self.waypoint_num][1]
            z = self.locals[self.pid()][self.waypoint_num][2]
            self.write_to_actuator('Motion.target', self.pos3d(x, y, z))
            self.locals['tries'] += 1
            self.waypoint_num += 1
            return
        if self.locals['tries'] == 4 and self.read_from_sensor('Motion.reached'):
            self.trystop()
            return

        # if self.locals['tries'] == 1:
        #     self.write_to_actuator('Motion.target', self.pos3d( -self.locals['p'], self.locals['p'], 1.0))
        #     self.locals['tries'] = 2
        #     return
        # if self.locals['tries'] == 2 and self.read_from_sensor('Motion.reached'):
        #     self.write_to_actuator('Motion.target', self.pos3d(self.locals['p'], self.locals['p'], 1.0))
        #     self.locals['tries'] = 3
        #     return
        # if self.locals['tries'] == 3 and self.read_from_sensor('Motion.reached'):
        #     self.write_to_actuator('Motion.target', self.pos3d(self.locals['p'],  -self.locals['p'], 1.0))
        #     self.locals['tries'] = 4
        #     return
        # if self.locals['tries'] == 4 and self.read_from_sensor('Motion.reached'):
        #     self.write_to_actuator('Motion.target', self.pos3d( -self.locals['p'],  -self.locals['p'], 1.0))
        #     self.locals['tries'] = 5
        #     return
        # if self.locals['tries'] == 5 and self.read_from_sensor('Motion.reached'):
        #     self.trystop()
        #     return
